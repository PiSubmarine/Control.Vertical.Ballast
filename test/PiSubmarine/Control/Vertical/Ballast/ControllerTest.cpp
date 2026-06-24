#include <chrono>
#include <cmath>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "PiSubmarine/Ballast/Api/IControllerMock.h"
#include "PiSubmarine/Ballast/Telemetry/Api/IProviderMock.h"
#include "PiSubmarine/Control/Vertical/Ballast/Controller.h"
#include "PiSubmarine/Depth/Telemetry/Api/IProviderMock.h"
#include "PiSubmarine/Error/Api/MakeError.h"

namespace PiSubmarine::Control::Vertical::Ballast
{
    using ::testing::Return;
    using ::testing::StrictMock;
    using ::testing::Truly;

    namespace
    {
        auto BallastFillNear(const double expected)
        {
            return Truly([expected](const ::PiSubmarine::Ballast::BallastFillFraction value)
            {
                return std::abs(static_cast<double>(value) - expected) < 1e-6;
            });
        }
    }

    TEST(ControllerTest, ForwardsDirectBallastPositionCommand)
    {
        StrictMock<::PiSubmarine::Ballast::Api::IControllerMock> ballastController;
        StrictMock<::PiSubmarine::Ballast::Telemetry::Api::IProviderMock> ballastTelemetryProvider;
        StrictMock<::PiSubmarine::Depth::Telemetry::Api::IProviderMock> depthProvider;
        Controller controller{ballastController, ballastTelemetryProvider, depthProvider};

        EXPECT_CALL(
            ballastController,
            SetTargetPosition(::PiSubmarine::Ballast::BallastFillFraction{0.6}))
            .WillOnce(Return(Error::Api::Result<void>{}));

        const auto result = controller.SetTarget(Api::Command::SetBallastPositionTo(
            ::PiSubmarine::Ballast::BallastFillFraction{0.6}));

        EXPECT_TRUE(result.has_value());
    }

    TEST(ControllerTest, KeepCurrentCapturesCurrentDepthAndCurrentBallastTarget)
    {
        StrictMock<::PiSubmarine::Ballast::Api::IControllerMock> ballastController;
        StrictMock<::PiSubmarine::Ballast::Telemetry::Api::IProviderMock> ballastTelemetryProvider;
        StrictMock<::PiSubmarine::Depth::Telemetry::Api::IProviderMock> depthProvider;
        Controller controller{
            ballastController,
            ballastTelemetryProvider,
            depthProvider,
            Controller::Config{
                .ProportionalGain = 0.2,
                .IntegralGainPerSecond = 0.05,
                .DerivativeGainSeconds = 0.0,
                .IntegralLimitMetersSeconds = 50.0,
                .DepthDeadband = 0.05_m,
                .MaximumBallastCorrection = NormalizedFraction{0.5}}};

        EXPECT_CALL(depthProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Depth::Telemetry::Api::State>{
                ::PiSubmarine::Depth::Telemetry::Api::State{.Depth = 4.0_m}}));
        EXPECT_CALL(ballastTelemetryProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Ballast::Telemetry::Api::State>{
                ::PiSubmarine::Ballast::Telemetry::Api::State{
                    .Position = ::PiSubmarine::Ballast::BallastFillFraction{0.55}}}));

        ASSERT_TRUE(controller.SetTarget(Api::Command::KeepCurrentValue()).has_value());

        EXPECT_CALL(depthProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Depth::Telemetry::Api::State>{
                ::PiSubmarine::Depth::Telemetry::Api::State{.Depth = 4.01_m}}));
        EXPECT_CALL(ballastController, SetTargetPosition(BallastFillNear(0.55)))
            .WillOnce(Return(Error::Api::Result<void>{}));

        controller.Tick(std::chrono::seconds{1}, std::chrono::milliseconds{100});
    }

    TEST(ControllerTest, SetDepthTargetCommandsMoreBallastWhenDroneIsTooShallow)
    {
        StrictMock<::PiSubmarine::Ballast::Api::IControllerMock> ballastController;
        StrictMock<::PiSubmarine::Ballast::Telemetry::Api::IProviderMock> ballastTelemetryProvider;
        StrictMock<::PiSubmarine::Depth::Telemetry::Api::IProviderMock> depthProvider;
        Controller controller{
            ballastController,
            ballastTelemetryProvider,
            depthProvider,
            Controller::Config{
                .ProportionalGain = 0.2,
                .IntegralGainPerSecond = 0.0,
                .DerivativeGainSeconds = 0.0,
                .IntegralLimitMetersSeconds = 50.0,
                .DepthDeadband = 0.05_m,
                .MaximumBallastCorrection = NormalizedFraction{0.5}}};

        EXPECT_CALL(ballastTelemetryProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Ballast::Telemetry::Api::State>{
                ::PiSubmarine::Ballast::Telemetry::Api::State{
                    .Position = ::PiSubmarine::Ballast::BallastFillFraction{0.4}}}));

        ASSERT_TRUE(controller.SetTarget(Api::Command::SetDepthTargetTo(5.0_m)).has_value());

        EXPECT_CALL(depthProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Depth::Telemetry::Api::State>{
                ::PiSubmarine::Depth::Telemetry::Api::State{.Depth = 4.0_m}}));
        EXPECT_CALL(ballastController, SetTargetPosition(BallastFillNear(0.6)))
            .WillOnce(Return(Error::Api::Result<void>{}));

        controller.Tick(std::chrono::seconds{1}, std::chrono::seconds{1});
    }

    TEST(ControllerTest, SetDepthTargetCommandsLessBallastWhenDroneIsTooDeep)
    {
        StrictMock<::PiSubmarine::Ballast::Api::IControllerMock> ballastController;
        StrictMock<::PiSubmarine::Ballast::Telemetry::Api::IProviderMock> ballastTelemetryProvider;
        StrictMock<::PiSubmarine::Depth::Telemetry::Api::IProviderMock> depthProvider;
        Controller controller{
            ballastController,
            ballastTelemetryProvider,
            depthProvider,
            Controller::Config{
                .ProportionalGain = 0.2,
                .IntegralGainPerSecond = 0.0,
                .DerivativeGainSeconds = 0.0,
                .IntegralLimitMetersSeconds = 50.0,
                .DepthDeadband = 0.05_m,
                .MaximumBallastCorrection = NormalizedFraction{0.5}}};

        EXPECT_CALL(ballastTelemetryProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Ballast::Telemetry::Api::State>{
                ::PiSubmarine::Ballast::Telemetry::Api::State{
                    .Position = ::PiSubmarine::Ballast::BallastFillFraction{0.6}}}));

        ASSERT_TRUE(controller.SetTarget(Api::Command::SetDepthTargetTo(4.0_m)).has_value());

        EXPECT_CALL(depthProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Depth::Telemetry::Api::State>{
                ::PiSubmarine::Depth::Telemetry::Api::State{.Depth = 5.0_m}}));
        EXPECT_CALL(ballastController, SetTargetPosition(BallastFillNear(0.4)))
            .WillOnce(Return(Error::Api::Result<void>{}));

        controller.Tick(std::chrono::seconds{1}, std::chrono::seconds{1});
    }

    TEST(ControllerTest, MissingTelemetryTriggersFailSafeToEmptyBallast)
    {
        StrictMock<::PiSubmarine::Ballast::Api::IControllerMock> ballastController;
        StrictMock<::PiSubmarine::Ballast::Telemetry::Api::IProviderMock> ballastTelemetryProvider;
        StrictMock<::PiSubmarine::Depth::Telemetry::Api::IProviderMock> depthProvider;
        Controller controller{ballastController, ballastTelemetryProvider, depthProvider};

        EXPECT_CALL(ballastTelemetryProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Ballast::Telemetry::Api::State>{
                ::PiSubmarine::Ballast::Telemetry::Api::State{
                    .Position = ::PiSubmarine::Ballast::BallastFillFraction{0.5}}}));

        ASSERT_TRUE(controller.SetTarget(Api::Command::SetDepthTargetTo(3.0_m)).has_value());

        EXPECT_CALL(depthProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Depth::Telemetry::Api::State>{
                std::unexpected(Error::Api::MakeError(Error::Api::ErrorCondition::CommunicationError))}));
        EXPECT_CALL(ballastController, SetTargetPosition(BallastFillNear(0.0)))
            .WillOnce(Return(Error::Api::Result<void>{}));

        controller.Tick(std::chrono::seconds{1}, std::chrono::milliseconds{100});
    }

    TEST(ControllerTest, MissingDepthDuringKeepCurrentTriggersFailSafeToEmptyBallast)
    {
        StrictMock<::PiSubmarine::Ballast::Api::IControllerMock> ballastController;
        StrictMock<::PiSubmarine::Ballast::Telemetry::Api::IProviderMock> ballastTelemetryProvider;
        StrictMock<::PiSubmarine::Depth::Telemetry::Api::IProviderMock> depthProvider;
        Controller controller{ballastController, ballastTelemetryProvider, depthProvider};

        EXPECT_CALL(depthProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Depth::Telemetry::Api::State>{
                std::unexpected(Error::Api::MakeError(Error::Api::ErrorCondition::CommunicationError))}));
        EXPECT_CALL(ballastController, SetTargetPosition(BallastFillNear(0.0)))
            .WillOnce(Return(Error::Api::Result<void>{}));

        const auto result = controller.SetTarget(Api::Command::KeepCurrentValue());

        EXPECT_FALSE(result.has_value());
    }

    TEST(ControllerTest, DerivativeGainDefaultsToPiBehaviorWhenZero)
    {
        StrictMock<::PiSubmarine::Ballast::Api::IControllerMock> ballastController;
        StrictMock<::PiSubmarine::Ballast::Telemetry::Api::IProviderMock> ballastTelemetryProvider;
        StrictMock<::PiSubmarine::Depth::Telemetry::Api::IProviderMock> depthProvider;
        Controller controller{
            ballastController,
            ballastTelemetryProvider,
            depthProvider,
            Controller::Config{
                .ProportionalGain = 0.1,
                .IntegralGainPerSecond = 0.0,
                .DerivativeGainSeconds = 0.0,
                .IntegralLimitMetersSeconds = 50.0,
                .DepthDeadband = 0.01_m,
                .MaximumBallastCorrection = NormalizedFraction{0.5}}};

        EXPECT_CALL(ballastTelemetryProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Ballast::Telemetry::Api::State>{
                ::PiSubmarine::Ballast::Telemetry::Api::State{
                    .Position = ::PiSubmarine::Ballast::BallastFillFraction{0.5}}}));
        ASSERT_TRUE(controller.SetTarget(Api::Command::SetDepthTargetTo(5.0_m)).has_value());

        EXPECT_CALL(depthProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Depth::Telemetry::Api::State>{
                ::PiSubmarine::Depth::Telemetry::Api::State{.Depth = 4.0_m}}));
        EXPECT_CALL(ballastController, SetTargetPosition(BallastFillNear(0.6)))
            .WillOnce(Return(Error::Api::Result<void>{}));
        controller.Tick(std::chrono::seconds{1}, std::chrono::seconds{1});

        EXPECT_CALL(depthProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Depth::Telemetry::Api::State>{
                ::PiSubmarine::Depth::Telemetry::Api::State{.Depth = 4.2_m}}));
        EXPECT_CALL(ballastController, SetTargetPosition(BallastFillNear(0.58)))
            .WillOnce(Return(Error::Api::Result<void>{}));
        controller.Tick(std::chrono::seconds{2}, std::chrono::seconds{1});
    }

    TEST(ControllerTest, MissingBallastTelemetryDuringDepthModeSetupTriggersFailSafe)
    {
        StrictMock<::PiSubmarine::Ballast::Api::IControllerMock> ballastController;
        StrictMock<::PiSubmarine::Ballast::Telemetry::Api::IProviderMock> ballastTelemetryProvider;
        StrictMock<::PiSubmarine::Depth::Telemetry::Api::IProviderMock> depthProvider;
        Controller controller{ballastController, ballastTelemetryProvider, depthProvider};

        EXPECT_CALL(ballastTelemetryProvider, GetState())
            .WillOnce(Return(Error::Api::Result<::PiSubmarine::Ballast::Telemetry::Api::State>{
                std::unexpected(Error::Api::MakeError(Error::Api::ErrorCondition::CommunicationError))}));
        EXPECT_CALL(ballastController, SetTargetPosition(BallastFillNear(0.0)))
            .WillOnce(Return(Error::Api::Result<void>{}));

        const auto result = controller.SetTarget(Api::Command::SetDepthTargetTo(3.0_m));

        EXPECT_FALSE(result.has_value());
    }
}
