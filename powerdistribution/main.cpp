#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
#include <signal.h>
#endif
#include <stdio.h>

#include <linux/can.h>

#include "version.h"
#include "canlib.h"

#include "networktables/NetworkTableInstance.h"
#include "networktables/RawTopic.h"
#include "networktables/IntegerTopic.h"

#define NUM_CAN_BUSES 2

static constexpr uint32_t deviceTypeMask = 0x3F000000;
static constexpr uint32_t powerDistributionFilter = 0x08000000;

struct CanState {
    int socketHandle{-1};
    nt::IntegerPublisher deviceIdPublisher;
    std::array<nt::RawPublisher, 4> framePublishers;
    unsigned busId{0};

    ~CanState() {
        if (socketHandle != -1) {
            close(socketHandle);
        }
    }

    void handleCanFrame(const canfd_frame& frame);
    void handlePowerFrame(const canfd_frame& frame);
    void start(const nt::NetworkTableInstance& ntInst);
};

void CanState::handleCanFrame(const canfd_frame& frame) {
    // Can't support FD frames
    if (frame.flags & CANFD_FDF) {
        return;
    }

    // Looking for Device Type 8 or 9.
    // That will tell us what we're handling
    uint32_t maskedDeviceType = frame.can_id & deviceTypeMask;

    if (maskedDeviceType == powerDistributionFilter) {
        handlePowerFrame(frame);
    }
}

void CanState::handlePowerFrame(const canfd_frame& frame) {
    uint16_t apiId = (frame.can_id >> 6) & 0x3FF;

    int frameNum = 0;
    uint32_t deviceId = frame.can_id & 0x1FFF003F;

    if (frame.can_id & 0x10000) {
        // Rev Frame
        if (apiId < 0x60 || apiId > 0x63) {
            // Not valid
            return;
        }

        frameNum = apiId - 0x60;
    } else {
        // CTRE frame
        if (apiId == 0x5D) {
            // Special case
            frameNum = 3;
        } else if (apiId < 0x50 || apiId > 0x52) {
            // Not valid
            return;
        } else {
            frameNum = apiId - 0x50;
        }
    }

    deviceIdPublisher.Set(deviceId);

    std::span<const uint8_t> frameSpan = {
        reinterpret_cast<const uint8_t*>(frame.data), frame.len};

    if (frameNum < 0 || frameNum >= static_cast<int>(framePublishers.size())) {
        printf("BUGBUG logic error invalid frame number\n");
        return;
    }

    framePublishers[frameNum].Set(frameSpan);
}

void CanState::start(const nt::NetworkTableInstance& ntInst) {
    nt::PubSubOptions options;
    options.sendAll = true;
    options.keepDuplicates = true;
    options.periodic = 0.005;

    auto busIdStr = std::to_string(busId);

    for (size_t i = 0; i < framePublishers.size(); i++) {
        auto iStr = std::to_string(i);
        framePublishers[i] =
            ntInst.GetRawTopic("/pd/" + busIdStr + "/frame" + iStr)
                .Publish("pd", options);
    }

    deviceIdPublisher =
        ntInst.GetIntegerTopic("/pd/" + busIdStr + "/deviceid").Publish();
}

bool SetupPowerDistribution(std::span<CanState> states, const nt::NetworkTableInstance& inst, mrc::CanLib& canLib) {
    for (auto&& s : states) {
        s.start(inst);
    }

    mrc::MaskFilterCallback cb {
        .mask = 0x1FFE0000 | CAN_EFF_FLAG,
        .filter = 0x08040000 | CAN_EFF_FLAG,
        .callback = [states](uint8_t bus, const canfd_frame frame) {
            states[bus].handleCanFrame(frame);
        },
    };
    return canLib.AddCallback({&cb, 1});
}

int main() {
    printf("Starting PowerDistributionDaemon\n");
    printf("\tBuild Hash: %s\n", MRC_GetGitHash());
    printf("\tBuild Timestamp: %s\n", MRC_GetBuildTimestamp());

#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGTERM);
    sigaddset(&signal_set, SIGINT);
    sigprocmask(SIG_BLOCK, &signal_set, nullptr);
#endif

    std::array<CanState, NUM_CAN_BUSES> states;

    auto ntInst = nt::NetworkTableInstance::Create();
    ntInst.SetServer({"localhost"}, 6810);
    ntInst.StartClient("PowerDistributionDaemon");

    mrc::CanLib canLib;

    if (!canLib.Init()) {
        return -1;
    }

    if (!SetupPowerDistribution(states, ntInst, canLib)) {
        return -1;
    }

    {
#if defined(__linux__) && defined(MRC_DAEMON_BUILD)
        int sig = 0;
        sigwait(&signal_set, &sig);
#else
        (void)getchar();
#endif
    }
    canLib.Stop();
    ntInst.StopClient();
    nt::NetworkTableInstance::Destroy(ntInst);

    return 0;
}
