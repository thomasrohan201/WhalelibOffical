#include "lemlib/chassis/whalelibchassis.hpp"
#include "lemlib/chassis/chassis.hpp"
#include <cmath>

namespace lemlib {

    // Create default ControllerSettings for initialization
const ControllerSettings DEFAULT_LINEAR = {0, 0, 0, 0, 0, 0, 0, 0, 0};
const ControllerSettings DEFAULT_ANGULAR = {0, 0, 0, 0, 0, 0, 0, 0, 0};

Whale::Whale(Chassis& chassis) 
    : chassis(chassis),
      fastProfile(DEFAULT_LINEAR, DEFAULT_ANGULAR),
      accurateProfile(DEFAULT_LINEAR, DEFAULT_ANGULAR),
      currentProfile(DEFAULT_LINEAR, DEFAULT_ANGULAR),
      customProfile(DEFAULT_LINEAR, DEFAULT_ANGULAR),
      usingCustomProfile(false),
      manualMode(false) {}

void Whale::gotoPoint(float x, float y, int timeout,
                     bool forwards, float maxSpeed, float minSpeed) {
    chassis.turnToPoint(x, y, timeout, {
        .forwards = forwards,
        .maxSpeed = static_cast<int>(maxSpeed)
    });
    
    chassis.moveToPoint(x, y, timeout, {
        .forwards = forwards,
        .maxSpeed = maxSpeed,
        .minSpeed = minSpeed
    });
}

// Utility functions
void Whale::cancelMotion() { chassis.cancelMotion(); }
void Whale::waitUntilDone() { chassis.waitUntilDone(); }
void Whale::waitUntil(float dist) { chassis.waitUntil(dist); }
void Whale::calibrate(bool calibrateIMU) { chassis.calibrate(calibrateIMU); }
void Whale::setPose(float x, float y, float theta) { chassis.setPose(x, y, theta); }
Pose Whale::getPose() { return chassis.getPose(); }
void Whale::setBrakeMode(pros::motor_brake_mode_e mode) { chassis.setBrakeMode(mode); }

// Profile management
void Whale::setMovementProfiles(const MovementConfig& fastProfile, 
                              const MovementConfig& accurateProfile) {
    this->fastProfile = fastProfile;
    this->accurateProfile = accurateProfile;
    this->currentProfile = fastProfile;
    applyControllerSettings(fastProfile.linear, fastProfile.angular);
}

void Whale::setMovementProfiles(const ControllerSettings& fastLinear,
                              const ControllerSettings& fastAngular,
                              const ControllerSettings& accurateLinear,
                              const ControllerSettings& accurateAngular) {
    this->fastProfile = MovementConfig(fastLinear, fastAngular);
    this->accurateProfile = MovementConfig(accurateLinear, accurateAngular);
    this->currentProfile = fastProfile;
    applyControllerSettings(fastLinear, fastAngular);
}

// Movement functions with accuracy parameter
void Whale::moveToPoint(float x, float y, int timeout, bool accurate,
                      const MoveToPointParams& params) {
    setProfile(accurate);
    chassis.moveToPoint(x, y, timeout, params);
}

void Whale::turnToPoint(float x, float y, int timeout, bool accurate,
                      const TurnToPointParams& params) {
    setProfile(accurate);
    chassis.turnToPoint(x, y, timeout, params);
}

void Whale::turnToHeading(float heading, int timeout, bool accurate,
                        const TurnToHeadingParams& params) {
    setProfile(accurate);
    chassis.turnToHeading(heading, timeout, params);
}

void Whale::swingToPoint(float x, float y, int timeout, DriveSide lockedSide,
                        bool accurate, const SwingToPointParams& params) {
    setProfile(accurate);
    chassis.swingToPoint(x, y, lockedSide, timeout, params);
}

void Whale::swingToHeading(float heading, int timeout, DriveSide lockedSide,
                            bool accurate, const SwingToHeadingParams& params) {
    setProfile(accurate);
    chassis.swingToHeading(heading, lockedSide, timeout, params);
}


void Whale::moveToPose(float x, float y, float heading, int timeout, bool accurate,
                     const MoveToPoseParams& params) {
    setProfile(accurate);
    chassis.moveToPose(x, y, heading, timeout, params);
}

// Manual PID control
void Whale::setManualPID(const ControllerSettings& linear, const ControllerSettings& angular) {
    manualMode = true;
    applyControllerSettings(linear, angular);
}

void Whale::revertToProfile() {
    manualMode = false;
    applyControllerSettings(currentProfile.linear, currentProfile.angular);
}

// Profile management
void Whale::setCustomProfile(const MovementConfig& profile) {
    customProfile = profile;
    currentProfile = profile;
    usingCustomProfile = true;
    applyControllerSettings(profile.linear, profile.angular);
}

void Whale::resetToDefaultProfile() {
    usingCustomProfile = false;
    currentProfile = fastProfile;
    applyControllerSettings(fastProfile.linear, fastProfile.angular);
}

void Whale::moveForward(float inches, int timeout, bool accurate,
                       const MoveToPointParams& params) {
    Pose current = getPose();
    
    // For 0° = North coordinate system:
    float targetX = current.x + inches * sin(current.theta * M_PI / 180.0);
    float targetY = current.y + inches * cos(current.theta * M_PI / 180.0);
    
    printf("CURRENT: X=%.2f, Y=%.2f, Theta=%.2f deg\n", current.x, current.y, current.theta);
    printf("TARGET:  X=%.2f, Y=%.2f\n", targetX, targetY);
    
    moveToPoint(targetX, targetY, timeout, accurate, params);
}

void Whale::moveBackward(float inches, int timeout, bool accurate,
                        const MoveToPointParams& params) {
    Pose current = getPose();
    
    // Use NEGATIVE inches to calculate point BEHIND the robot
    float targetX = current.x + (-inches) * sin(current.theta * M_PI / 180.0);
    float targetY = current.y + (-inches) * cos(current.theta * M_PI / 180.0);
    
    printf("CURRENT: X=%.2f, Y=%.2f, Theta=%.2f deg\n", current.x, current.y, current.theta);
    printf("TARGET:  X=%.2f, Y=%.2f (BEHIND)\n", targetX, targetY);
    
    MoveToPointParams reverseParams = params;
    reverseParams.forwards = false; // Also force reverse driving
    
    moveToPoint(targetX, targetY, timeout, accurate, reverseParams);
}


MovementConfig Whale::getCurrentProfile() const { return currentProfile; }
MovementConfig Whale::getFastProfile() const { return fastProfile; }
MovementConfig Whale::getAccurateProfile() const { return accurateProfile; }

// Helper functions
void Whale::applyControllerSettings(const ControllerSettings& linear, 
                                  const ControllerSettings& angular) {
    // Copy controller settings to chassis
    chassis.linearController.kP = linear.kP;
    chassis.linearController.kI = linear.kI;
    chassis.linearController.kD = linear.kD;
    chassis.linearController.windupRange = linear.windupRange;
    chassis.linearController.smallError = linear.smallError;
    chassis.linearController.smallErrorTimeout = linear.smallErrorTimeout;
    chassis.linearController.largeError = linear.largeError;
    chassis.linearController.largeErrorTimeout = linear.largeErrorTimeout;
    chassis.linearController.slew = linear.slew;
    
    chassis.angularController.kP = angular.kP;
    chassis.angularController.kI = angular.kI;
    chassis.angularController.kD = angular.kD;
    chassis.angularController.windupRange = angular.windupRange;
    chassis.angularController.smallError = angular.smallError;
    chassis.angularController.smallErrorTimeout = angular.smallErrorTimeout;
    chassis.angularController.largeError = angular.largeError;
    chassis.angularController.largeErrorTimeout = angular.largeErrorTimeout;
    chassis.angularController.slew = angular.slew;
}

void Whale::setProfile(bool accurate) {
    if (manualMode || usingCustomProfile) return;
    
    if (accurate) {
        currentProfile = accurateProfile;
        applyControllerSettings(accurateProfile.linear, accurateProfile.angular);
    } else {
        currentProfile = fastProfile;
        applyControllerSettings(fastProfile.linear, fastProfile.angular);
    }
}

} // namespace lemlib