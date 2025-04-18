#include "canlib.h"

#include <wpinet/EventLoopRunner.h>
#include <wpinet/uv/Poll.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstring>

#define NUM_CAN_BUSES 5

namespace mrc {

struct CanBus {
    int socketHandle{-1};
    unsigned busId{0};
    std::function<void(uint8_t, const canfd_frame&)> upcall;
    std::weak_ptr<wpi::uv::Poll> poller;

    ~CanBus() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    bool Init(wpi::uv::Loop& loop, uint8_t bus, std::function<void(uint8_t, const canfd_frame&)> up);
    bool UpdateFilters(const std::vector<can_filter>& filters);
};

bool CanBus::Init(wpi::uv::Loop& loop, uint8_t bus, std::function<void(uint8_t, const canfd_frame&)> up) {
    busId = bus;
    upcall = std::move(up);

    socketHandle =
        socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

    if (socketHandle == -1) {
        return false;
    }

    // Disable all filters
    if (setsockopt(socketHandle, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0) == -1) {
        return false;
    }

    ifreq ifr;
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "can%u", busId);

    if (ioctl(socketHandle, SIOCGIFINDEX, &ifr) == -1) {
        return false;
    }

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketHandle, reinterpret_cast<const sockaddr*>(&addr),
             sizeof(addr)) == -1) {
        return false;
    }

    auto poll = wpi::uv::Poll::Create(loop, socketHandle);

    poll->pollEvent.connect([this](int mask) {
        if (mask & UV_READABLE) {
            canfd_frame frame;
            int rVal = read(socketHandle, &frame, sizeof(frame));

            if (rVal != CAN_MTU && rVal != CANFD_MTU) {
                // TODO Error handling, do we need to reopen the socket?
                return;
            }

            if (frame.can_id & CAN_ERR_FLAG) {
                // Do nothing if this is an error frame
                return;
            }

            if (rVal == CANFD_MTU) {
                frame.flags = CANFD_FDF;
            }

            upcall(busId, frame);
        }
        if (mask & UV_WRITABLE) {
        }
    });

    poll->Start(UV_READABLE);

    return true;
}

bool CanBus::UpdateFilters(const std::vector<can_filter>& filters) {
    return setsockopt(socketHandle, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                      filters.size()) != -1;
}

struct CanLib::Impl {
    std::array<CanBus, NUM_CAN_BUSES> buses;
    wpi::EventLoopRunner loopRunner;

    std::vector<MaskFilterCallback> filters;

    bool Init();
    bool AddCallback(std::span<MaskFilterCallback> filters);

    void DataCallback(uint8_t bus, const canfd_frame& frame);
};

void CanLib::Impl::DataCallback(uint8_t bus, const canfd_frame& frame) {
    for (auto&& f : filters) {
        if ((frame.can_id & f.mask) == (f.filter & f.mask)) {
            f.callback(bus, frame);
        }
    }
}

bool CanLib::Impl::Init() {
    bool success = true;
    loopRunner.ExecSync([this, &success](wpi::uv::Loop& loop) {
        for (size_t i = 0; i < buses.size(); i++) {
            if (!buses[i].Init(loop, static_cast<uint8_t>(i), [this](uint8_t bus, const canfd_frame& frame) {
                DataCallback(bus, frame);
            })) {
                success = false;
            }
        }
    });
    return success;
}

bool CanLib::Impl::AddCallback(std::span<MaskFilterCallback> filters) {
    bool retVal = true;
    loopRunner.ExecSync(
        [this, &retVal, newFilters = filters](wpi::uv::Loop& loop) {
            for (auto&& f : newFilters) {
                this->filters.emplace_back(std::move(f));
            }
            // Update all buses
            std::vector<can_filter> canFilters;
            for (auto&& f : this->filters) {
                can_filter filter = {
                    .can_id = f.filter,
                    .can_mask = f.mask,
                };
                canFilters.emplace_back(filter);
            }
            for (auto&& bus : buses) {
                if (!bus.UpdateFilters(canFilters)) {
                    retVal = false;
                }
            }
        });
    return retVal;
}

CanLib::CanLib() = default;
CanLib::~CanLib() = default;

bool CanLib::Init() {
    pImpl = std::make_unique<Impl>();
    if (!pImpl->Init()) {
        pImpl = nullptr;
        return false;
    }
    return true;
}

bool CanLib::AddCallback(std::span<MaskFilterCallback> filters) {
    return pImpl->AddCallback(filters);
}

void CanLib::Stop() {
    pImpl = nullptr;
}
}  // namespace mrc