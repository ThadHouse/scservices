#include "MotorNtState.h"

#include "wpi/nt/NetworkTableInstance.hpp"

#define PERCENTAGE_MODE 0
#define VOLTAGE_MODE 1
#define POSITION_PID_MODE 2
#define VELOCITY_PID_MODE 3
#define FOLLOWER_MODE 4

using namespace eh;

void MotorNtState::SetEncoder(double positionRaw, double velocityRaw) {
    double reversed = reversedSubscriber.Get(false) ? -1.0 : 1.0;
    double distancePerCount = distancePerCountSubscriber.Get(0);
    if (distancePerCount == 0) {
        distancePerCount = 1;
    }
    lastEncoderPosition = positionRaw * reversed * distancePerCount;
    // TODO does this need to be scaled
    lastEncoderVelocity = velocityRaw * reversed * distancePerCount;

    encoderPublisher.Set(lastEncoderPosition);
    velocityPublisher.Set(lastEncoderVelocity);
}

MotorNtState::PowerResult MotorNtState::ComputeMotorPower(
    double batteryVoltage) {
    double reversed = reversedSubscriber.Get(false) ? -1.0 : 1.0;
    if (batteryVoltage == 0) {
        return {
            .power = 0.0,
            .followerIndex = -1,
            .reverseFollower = false,
        };
    }
    double setpoint = setpointSubscriber.Get(0);
    switch (modeSubscriber.Get(PERCENTAGE_MODE)) {
        case VOLTAGE_MODE:
            return {
                .power = (setpoint / batteryVoltage) * reversed,
                .followerIndex = -1,
                .reverseFollower = false,
            };

        case POSITION_PID_MODE:
            return {
                .power = (positionPid.Compute(setpoint, lastEncoderPosition) /
                          batteryVoltage) *
                         reversed,
                .followerIndex = -1,
                .reverseFollower = false,
            };

        case VELOCITY_PID_MODE:
            return {
                .power = (velocityPid.Compute(setpoint, lastEncoderVelocity) /
                          batteryVoltage) *
                         reversed,
                .followerIndex = -1,
                .reverseFollower = false,
            };

        case FOLLOWER_MODE: {
            int intSetpoint = static_cast<int>(setpoint);
            return {
                .power = 0.0,
                .followerIndex = intSetpoint % 4,
                .reverseFollower = intSetpoint >= 4,
            };
        }

        default:
            return {.power = setpoint * reversed,
                    .followerIndex = -1,
                    .reverseFollower = false};
    }
}

void MotorNtState::Initialize(const wpi::nt::NetworkTableInstance& instance,
                              int motorNum, const std::string& busIdStr,
                              wpi::nt::PubSubOptions options) {
    auto motorNumStr = std::to_string(motorNum);
    encoderPublisher = instance
                           .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                           motorNumStr + "/encoder")
                           .Publish(options);
    velocityPublisher = instance
                            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                            motorNumStr + "/encoderVelocity")
                            .Publish(options);
    currentPublisher = instance
                           .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                           motorNumStr + "/current")
                           .Publish(options);
    setpointSubscriber = instance
                             .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" +
                                             motorNumStr + "/setpoint")
                             .Subscribe(0, options);
    floatOn0Subscriber = instance
                             .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                              motorNumStr + "/floatOn0")
                             .Subscribe(false, options);
    enabledSubscriber = instance
                            .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                             motorNumStr + "/enabled")
                            .Subscribe(false, options);

    modeSubscriber = instance
                         .GetIntegerTopic("/rhsp/" + busIdStr + "/motor" +
                                          motorNumStr + "/mode")
                         .Subscribe(0, options);

    reversedSubscriber = instance
                             .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" +
                                              motorNumStr + "/reversed")
                             .Subscribe(false, options);

    distancePerCountSubscriber =
        instance
            .GetDoubleTopic("/rhsp/" + busIdStr + "/motor" + motorNumStr +
                            "/distancePerCount")
            .Subscribe(0, options);

    resetEncoderSubscriber =
        instance
            .GetBooleanTopic("/rhsp/" + busIdStr + "/motor" + motorNumStr +
                             "/resetEncoder")
            .Subscribe(false, options);

    velocityPid.Initialize(instance, motorNumStr, busIdStr, options);
    positionPid.Initialize(instance, motorNumStr, busIdStr, options);
}
