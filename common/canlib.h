#pragma once

#include <functional>
#include <span>
#include <memory>

struct canfd_frame;

namespace mrc {
struct MaskFilterCallback {
    uint32_t mask;
    uint32_t filter;
    std::function<void(uint8_t, const canfd_frame&)> callback;
};

class CanLib {
    public:
    CanLib();
    ~CanLib();
    CanLib(const CanLib&) = delete;
    CanLib(const CanLib&&) = delete;
    const CanLib& operator=(const CanLib&) = delete;
    const CanLib& operator=(const CanLib&&) = delete;
    
    bool Init();

    // Do not call stop from the loop
    void Stop();

    bool AddCallback(std::span<MaskFilterCallback> filters);

    private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};
}