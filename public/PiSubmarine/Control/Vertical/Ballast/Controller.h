#pragma once

#include <chrono>
#include <optional>

#include "PiSubmarine/Ballast/Api/IController.h"
#include "PiSubmarine/Control/Vertical/Api/IController.h"
#include "PiSubmarine/Depth/Telemetry/Api/IProvider.h"
#include "PiSubmarine/Meters.h"
#include "PiSubmarine/NormalizedFraction.h"
#include "PiSubmarine/Time/ITickable.h"

namespace PiSubmarine::Control::Vertical::Ballast
{
    class Controller final
        : public Api::IController
        , public Time::ITickable
    {
    public:
        struct Config
        {
            double ProportionalGain = 0.2;
            double IntegralGainPerSecond = 0.05;
            double DerivativeGainSeconds = 0.0;
            double IntegralLimitMetersSeconds = 50.0;
            Meters DepthDeadband = 0.05_m;
            NormalizedFraction MaximumBallastCorrection = NormalizedFraction{0.5};
            ::PiSubmarine::Ballast::BallastFillFraction InitialBallastFill =
                ::PiSubmarine::Ballast::BallastFillFraction::Empty();
            ::PiSubmarine::Ballast::BallastFillFraction InitialEquilibriumBallastFill =
                ::PiSubmarine::Ballast::BallastFillFraction{NormalizedFraction{0.5}};
        };

        Controller(
            ::PiSubmarine::Ballast::Api::IController& ballastController,
            ::PiSubmarine::Depth::Telemetry::Api::IProvider& depthProvider,
            const Config& config) noexcept;

        Controller(
            ::PiSubmarine::Ballast::Api::IController& ballastController,
            ::PiSubmarine::Depth::Telemetry::Api::IProvider& depthProvider) noexcept;

        [[nodiscard]] Error::Api::Result<void> SetTarget(const Api::Command& target) override;
        void Tick(const std::chrono::nanoseconds& uptime, const std::chrono::nanoseconds& deltaTime) override;

    private:
        enum class Mode
        {
            KeepCurrent,
            SetBallastPosition,
            SetDepthTarget
        };

        [[nodiscard]] Error::Api::Result<void> ConfigureDepthMode(Meters targetDepth, Mode mode);
        [[nodiscard]] Error::Api::Result<void> FailSafeToSurface();
        [[nodiscard]] Error::Api::Result<void> TickDepthControl(const std::chrono::nanoseconds& deltaTime);
        [[nodiscard]] static double ClampSymmetric(double value, double limit) noexcept;

        ::PiSubmarine::Ballast::Api::IController& m_BallastController;
        ::PiSubmarine::Depth::Telemetry::Api::IProvider& m_DepthProvider;
        Config m_Config;
        Mode m_Mode = Mode::SetBallastPosition;
        Meters m_TargetDepth = 0.0_m;
        ::PiSubmarine::Ballast::BallastFillFraction m_BallastBias;
        double m_IntegralError = 0.0;
        std::optional<double> m_PreviousDepthError;
    };
}
