#include "PiSubmarine/Control/Vertical/Ballast/Controller.h"

#include <algorithm>
#include <cmath>

#include "PiSubmarine/Error/Api/MakeError.h"

namespace PiSubmarine::Control::Vertical::Ballast
{
    Controller::Controller(
        ::PiSubmarine::Ballast::Api::IController& ballastController,
        ::PiSubmarine::Depth::Telemetry::Api::IProvider& depthProvider,
        const Config& config) noexcept
        : m_BallastController(ballastController)
        , m_DepthProvider(depthProvider)
        , m_Config(config)
    {
    }

    Error::Api::Result<void> Controller::SetTarget(const Api::Command& target)
    {
        if (const auto* keepCurrent = target.TryGet<Api::Command::KeepCurrent>())
        {
            static_cast<void>(keepCurrent);
            const auto depthState = m_DepthProvider.GetState();
            if (!depthState.has_value() || !depthState->Depth.has_value())
            {
                const auto failSafeResult = FailSafeToSurface();
                if (!failSafeResult.has_value())
                {
                    return failSafeResult;
                }

                return depthState.has_value()
                    ? Error::Api::Result<void>{std::unexpected(
                        Error::Api::MakeError(Error::Api::ErrorCondition::NotReady))}
                    : Error::Api::Result<void>{std::unexpected(depthState.error())};
            }

            return ConfigureDepthMode(*depthState->Depth, Mode::KeepCurrent);
        }

        if (const auto* setBallastPosition = target.TryGet<Api::Command::SetBallastPosition>())
        {
            m_Mode = Mode::SetBallastPosition;
            m_IntegralError = 0.0;
            m_PreviousDepthError.reset();
            m_BallastBias = setBallastPosition->Position;
            return m_BallastController.SetTargetPosition(setBallastPosition->Position);
        }

        if (const auto* setDepthTarget = target.TryGet<Api::Command::SetDepthTarget>())
        {
            return ConfigureDepthMode(setDepthTarget->Depth, Mode::SetDepthTarget);
        }

        return {};
    }

    void Controller::Tick(const std::chrono::nanoseconds&, const std::chrono::nanoseconds& deltaTime)
    {
        if (m_Mode == Mode::SetBallastPosition)
        {
            return;
        }

        static_cast<void>(TickDepthControl(deltaTime));
    }

    Error::Api::Result<void> Controller::ConfigureDepthMode(const Meters targetDepth, const Mode mode)
    {
        const auto ballastTargetResult = m_BallastController.GetTargetPosition();
        if (!ballastTargetResult.has_value())
        {
            const auto failSafeResult = FailSafeToSurface();
            if (!failSafeResult.has_value())
            {
                return failSafeResult;
            }

            return std::unexpected(ballastTargetResult.error());
        }

        m_Mode = mode;
        m_TargetDepth = targetDepth;
        m_BallastBias = *ballastTargetResult;
        m_IntegralError = 0.0;
        m_PreviousDepthError.reset();
        return {};
    }

    Error::Api::Result<void> Controller::FailSafeToSurface()
    {
        m_Mode = Mode::SetBallastPosition;
        m_BallastBias = ::PiSubmarine::Ballast::BallastFillFraction::Empty();
        m_IntegralError = 0.0;
        m_PreviousDepthError.reset();
        return m_BallastController.SetTargetPosition(::PiSubmarine::Ballast::BallastFillFraction::Empty());
    }

    Error::Api::Result<void> Controller::TickDepthControl(const std::chrono::nanoseconds& deltaTime)
    {
        const auto deltaSeconds = std::chrono::duration<double>(deltaTime).count();
        if (deltaSeconds <= 0.0)
        {
            return {};
        }

        const auto depthState = m_DepthProvider.GetState();
        if (!depthState.has_value() || !depthState->Depth.has_value())
        {
            const auto failSafeResult = FailSafeToSurface();
            if (!failSafeResult.has_value())
            {
                return failSafeResult;
            }

            return depthState.has_value()
                ? Error::Api::Result<void>{std::unexpected(
                    Error::Api::MakeError(Error::Api::ErrorCondition::NotReady))}
                : Error::Api::Result<void>{std::unexpected(depthState.error())};
        }

        const auto depthError = m_TargetDepth.Value - depthState->Depth->Value;

        if (std::abs(depthError) <= m_Config.DepthDeadband.Value)
        {
            m_IntegralError = 0.0;
            m_PreviousDepthError = depthError;
            return m_BallastController.SetTargetPosition(m_BallastBias);
        }

        m_IntegralError = ClampSymmetric(
            m_IntegralError + (depthError * deltaSeconds),
            m_Config.IntegralLimitMetersSeconds);

        const auto derivativeErrorPerSecond = m_PreviousDepthError.has_value()
            ? (depthError - *m_PreviousDepthError) / deltaSeconds
            : 0.0;
        m_PreviousDepthError = depthError;

        const auto correction = (m_Config.ProportionalGain * depthError) +
            (m_Config.IntegralGainPerSecond * m_IntegralError) +
            (m_Config.DerivativeGainSeconds * derivativeErrorPerSecond);
        const auto clampedCorrection = ClampSymmetric(
            correction,
            static_cast<double>(m_Config.MaximumBallastCorrection));
        const auto commandedBallastPosition = std::clamp(
            static_cast<double>(m_BallastBias) + clampedCorrection,
            0.0,
            1.0);

        return m_BallastController.SetTargetPosition(
            ::PiSubmarine::Ballast::BallastFillFraction{commandedBallastPosition});
    }

    double Controller::ClampSymmetric(const double value, const double limit) noexcept
    {
        return std::clamp(value, -limit, limit);
    }
}
