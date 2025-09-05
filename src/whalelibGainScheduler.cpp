#include "whalelibGainScheduler.hpp"
#include "lemlib/chassis/whalelibchassis.hpp"
#include "lemlib/chassis/chassis.hpp"
#include <cmath>
#include <functional>

namespace lemlib {

    // Create default ControllerSettings for initialization
const ControllerSettings DEFAULT_LINEAR = {0, 0, 0, 0, 0, 0, 0, 0, 0};
const ControllerSettings DEFAULT_ANGULAR = {0, 0, 0, 0, 0, 0, 0, 0, 0};

WhaleGainScheduler::WhaleGainScheduler(Chassis& chassis) 
    : Whale(chassis),
      farLinearGains(10.0, 0.0, 3.0),
      mediumLinearGains(6.0, 0.5, 2.0),
      closeLinearGains(2.0, 0.1, 1.0),
      farAngularGains(6.0, 0.0, 1.5),
      mediumAngularGains(4.0, 0.2, 1.0),
      closeAngularGains(2.0, 0.1, 0.5),
      linearFarThreshold(36.0f),
      linearMediumThreshold(24.0f),
      angularFarThreshold(90.0f),
      angularMediumThreshold(45.0f),
      currentLinearGains(0, 0, 0),
      currentAngularGains(0, 0, 0),
      originalConfig(DEFAULT_LINEAR, DEFAULT_ANGULAR), // Initialize with default settings
      gainsModified(false)
{
    // Store the current configuration
    originalConfig = Whale::getCurrentProfile();
}

// Add these methods anywhere in the lemlib namespace in your CPP file:

void WhaleGainScheduler::setLinearGainScheduler(const PIDGains& farGains, const PIDGains& mediumGains, 
                                               const PIDGains& closeGains, float farThreshold, 
                                               float mediumThreshold) {
    farLinearGains = farGains;
    mediumLinearGains = mediumGains;
    closeLinearGains = closeGains;
    linearFarThreshold = farThreshold;
    linearMediumThreshold = mediumThreshold;
}

void WhaleGainScheduler::setAngularGainScheduler(const PIDGains& farGains, const PIDGains& mediumGains, 
                                                const PIDGains& closeGains, float farThreshold, 
                                                float mediumThreshold) {
    farAngularGains = farGains;
    mediumAngularGains = mediumGains;
    closeAngularGains = closeGains;
    angularFarThreshold = farThreshold;
    angularMediumThreshold = mediumThreshold;
}

void WhaleGainScheduler::setLinearInterpolationFunction(std::function<PIDGains(float distance, const Pose& current, const Pose& target)> func) {
    linearInterpolationFunc = func;
}

void WhaleGainScheduler::setAngularInterpolationFunction(std::function<PIDGains(float error, const Pose& current, const Pose& target)> func) {
    angularInterpolationFunc = func;
}

PIDGains WhaleGainScheduler::getCurrentLinearGains() const {
    return currentLinearGains;
}

PIDGains WhaleGainScheduler::getCurrentAngularGains() const {
    return currentAngularGains;
}

void WhaleGainScheduler::resetGainScheduler() {
    if (gainsModified) {
        Whale::revertToProfile();
        gainsModified = false;
    }
}


void WhaleGainScheduler::moveToPointWithScheduler(float x, float y, int timeout, 
                                                bool forwards, float maxSpeed, 
                                                float minSpeed, float earlyExitRange) {
    Pose currentPose = getPose();
    
    float distance = std::sqrt(std::pow(x - currentPose.x, 2) + std::pow(y - currentPose.y, 2));
    
    PIDGains linearGains;
    if (linearInterpolationFunc) {
        linearGains = linearInterpolationFunc(distance, currentPose, {x, y, 0});
    } else {
        linearGains = calculateLinearGains(distance);
    }
    
    // Get angular settings from current profile and convert to PIDGains for consistency
    ControllerSettings angularSettings = Whale::getCurrentProfile().angular;
    PIDGains angularGains(angularSettings.kP, angularSettings.kI, angularSettings.kD);
    
    applyGainScheduler(linearGains, angularGains);
    
    Whale::moveToPoint(x, y, timeout, false, {
        .forwards = forwards,
        .maxSpeed = maxSpeed,
        .minSpeed = minSpeed,
        .earlyExitRange = earlyExitRange
    });
}

void WhaleGainScheduler::moveToPoseWithScheduler(float x, float y, float theta, int timeout, 
                                               bool forwards, float maxSpeed, 
                                               float minSpeed, float lead, 
                                               float horizontalDrift) {
    Pose currentPose = getPose();
    
    float distance = std::sqrt(std::pow(x - currentPose.x, 2) + std::pow(y - currentPose.y, 2));
    
    float headingError = std::abs(theta - currentPose.theta);
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    headingError = std::abs(headingError);
    
    PIDGains linearGains, angularGains;
    
    if (linearInterpolationFunc) {
        linearGains = linearInterpolationFunc(distance, currentPose, {x, y, theta});
    } else {
        linearGains = calculateLinearGains(distance);
    }
    
    if (angularInterpolationFunc) {
        angularGains = angularInterpolationFunc(headingError, currentPose, {x, y, theta});
    } else {
        angularGains = calculateAngularGains(headingError);
    }
    
    applyGainScheduler(linearGains, angularGains);
    
    Whale::moveToPose(x, y, theta, timeout, false, {
        .forwards = forwards,
        .horizontalDrift = horizontalDrift,
        .lead = lead,
        .maxSpeed = maxSpeed,
        .minSpeed = minSpeed
    });
}

void WhaleGainScheduler::turnToPointWithScheduler(float x, float y, int timeout, 
                                                float maxSpeed, float minSpeed, 
                                                float earlyExitRange) {
    Pose currentPose = getPose();
    
    float targetHeading = std::atan2(y - currentPose.y, x - currentPose.x) * 180 / M_PI;
    float headingError = std::abs(targetHeading - currentPose.theta);
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    headingError = std::abs(headingError);
    
    PIDGains angularGains;
    if (angularInterpolationFunc) {
        angularGains = angularInterpolationFunc(headingError, currentPose, {x, y, 0});
    } else {
        angularGains = calculateAngularGains(headingError);
    }
    
    // Get linear settings from current profile and convert to PIDGains for consistency
    ControllerSettings linearSettings = Whale::getCurrentProfile().linear;
    PIDGains linearGains(linearSettings.kP, linearSettings.kI, linearSettings.kD);
    
    applyGainScheduler(linearGains, angularGains);
    
    Whale::turnToPoint(x, y, timeout, false, {
        .maxSpeed = static_cast<int>(maxSpeed),
        .minSpeed = static_cast<int>(minSpeed),
        .earlyExitRange = earlyExitRange
    });
}

void WhaleGainScheduler::turnToHeadingWithScheduler(float targetHeading, int timeout, 
                                                  float maxSpeed, float minSpeed, 
                                                  AngularDirection direction) {
    Pose currentPose = getPose();
    
    float headingError = std::abs(targetHeading - currentPose.theta);
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    headingError = std::abs(headingError);
    
    PIDGains angularGains;
    if (angularInterpolationFunc) {
        angularGains = angularInterpolationFunc(headingError, currentPose, {0, 0, targetHeading});
    } else {
        angularGains = calculateAngularGains(headingError);
    }
    
    // Get linear settings from current profile and convert to PIDGains for consistency
    ControllerSettings linearSettings = Whale::getCurrentProfile().linear;
    PIDGains linearGains(linearSettings.kP, linearSettings.kI, linearSettings.kD);
    
    applyGainScheduler(linearGains, angularGains);
    
    Whale::turnToHeading(targetHeading, timeout, false, {
        .direction = direction,
        .maxSpeed = static_cast<int>(maxSpeed),
        .minSpeed = static_cast<int>(minSpeed)
    });
}

// ... (other configuration methods remain the same) ...

PIDGains WhaleGainScheduler::interpolatePID(const PIDGains& gains1, const PIDGains& gains2, float t) const {
    PIDGains result;
    result.kp = gains1.kp * (1 - t) + gains2.kp * t;
    result.ki = gains1.ki * (1 - t) + gains2.ki * t;
    result.kd = gains1.kd * (1 - t) + gains2.kd * t;
    return result;
}

PIDGains WhaleGainScheduler::calculateLinearGains(float distance) const {
    if (distance > linearFarThreshold) {
        return farLinearGains;
    } else if (distance > linearMediumThreshold) {
        float t = (distance - linearMediumThreshold) / (linearFarThreshold - linearMediumThreshold);
        return interpolatePID(mediumLinearGains, farLinearGains, t);
    } else if (distance > 0) {
        float t = distance / linearMediumThreshold;
        return interpolatePID(closeLinearGains, mediumLinearGains, t);
    } else {
        return closeLinearGains;
    }
}

PIDGains WhaleGainScheduler::calculateAngularGains(float error) const {
    if (error > angularFarThreshold) {
        return farAngularGains;
    } else if (error > angularMediumThreshold) {
        float t = (error - angularMediumThreshold) / (angularFarThreshold - angularMediumThreshold);
        return interpolatePID(mediumAngularGains, farAngularGains, t);
    } else if (error > 0) {
        float t = error / angularMediumThreshold;
        return interpolatePID(closeAngularGains, mediumAngularGains, t);
    } else {
        return closeAngularGains;
    }
}

ControllerSettings WhaleGainScheduler::convertPidGainsToControllerSettings(const PIDGains& pidGains, const ControllerSettings& templateSettings) const {
    ControllerSettings settings = templateSettings;
    settings.kP = pidGains.kp;
    settings.kI = pidGains.ki;
    settings.kD = pidGains.kd;
    // Other settings are already copied from templateSettings
    return settings;
}

void WhaleGainScheduler::applyGainScheduler(const PIDGains& linear, const PIDGains& angular) {
    if (!gainsModified) {
        originalConfig = Whale::getCurrentProfile();
    }
    
    // Convert PIDGains to ControllerSettings using the helper function
    ControllerSettings linearSettings = convertPidGainsToControllerSettings(linear, originalConfig.linear);
    ControllerSettings angularSettings = convertPidGainsToControllerSettings(angular, originalConfig.angular);
    
    Whale::setManualPID(linearSettings, angularSettings);
    gainsModified = true;
    
    currentLinearGains = linear;
    currentAngularGains = angular;
}

} // namespace lemlib