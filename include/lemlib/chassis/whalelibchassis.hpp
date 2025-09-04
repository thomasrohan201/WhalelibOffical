#pragma once

#include "lemlib/chassis/chassis.hpp"

namespace lemlib {

/**
 * @brief Movement configuration structure for PID profiles and speed limits
 */
struct MovementConfig {
    ControllerSettings linear; /**< Linear controller settings */
    ControllerSettings angular; /**< Angular controller settings */

    
    /**
     * @brief Construct a new Movement Config object
     * @param lin Linear controller settings
     * @param ang Angular controller settings

     */
    MovementConfig(const ControllerSettings& lin, const ControllerSettings& ang,
                  float maxSpd = 127.0f, float minSpd = 0.0f)
        : linear(lin), angular(ang) {}
};

/**
 * @brief Whale movement controller with enhanced PID profile management
 */
class Whale {
public:
    /**
     * @brief Construct a new Whale movement controller
     * @param chassis Reference to the chassis object
     */
    Whale(Chassis& chassis);

    // Basic movement functions
    void gotoPoint(float x, float y, int timeout, 
                  bool forwards = true, float maxSpeed = 127, float minSpeed = 0);

    // Utility functions
    void cancelMotion();
    void waitUntilDone();
    void waitUntil(float dist);
    void calibrate(bool calibrateIMU = true);
    void setPose(float x, float y, float theta);
    Pose getPose();
    void setBrakeMode(pros::motor_brake_mode_e mode);

    // Profile-based movement functions with accuracy parameter
    void moveToPoint(float x, float y, int timeout, bool accurate = false,
                    const MoveToPointParams& params = {});
    void turnToPoint(float x, float y, int timeout, bool accurate = false,
                    const TurnToPointParams& params = {});
    void turnToHeading(float heading, int timeout, bool accurate = false,
                      const TurnToHeadingParams& params = {});
    void swingToPoint(float x, float y, int timeout, DriveSide lockedSide,
                     bool accurate = false, const SwingToPointParams& params = {});
    void swingToHeading(float heading, int timeout, DriveSide lockedSide,
                       bool accurate = false, const SwingToHeadingParams& params = {});
    void moveToPose(float x, float y, float heading, int timeout, bool accurate = false,
                   const MoveToPoseParams& params = {});
    void resetPose(lemlib::Pose alignmentPoint, float alignmentDistance, int imu_port);
    void resetPositionOnly(lemlib::Pose alignmentPoint, float alignmentDistance);
    void setPoseDirect(float x, float y);

    // Profile management
    void setMovementProfiles(const MovementConfig& fastProfile, 
                           const MovementConfig& accurateProfile);
    void setMovementProfiles(const ControllerSettings& fastLinear,
                           const ControllerSettings& fastAngular,
                           const ControllerSettings& accurateLinear,
                           const ControllerSettings& accurateAngular);
    void setCustomProfile(const MovementConfig& profile);
    void resetToDefaultProfile();
    MovementConfig getCurrentProfile() const;
    MovementConfig getFastProfile() const;
    MovementConfig getAccurateProfile() const;

    // Manual PID control
    void setManualPID(const ControllerSettings& linear, const ControllerSettings& angular);
    void revertToProfile();

        
    /**
     * @brief Move forward a relative distance
     * @param inches Distance to move forward in inches
     * @param timeout Maximum time in milliseconds
     * @param accurate Whether to use accurate profile
     * @param params Movement parameters
     */
    void moveForward(float inches, int timeout, bool accurate = false,
                    const MoveToPointParams& params = {});
    
    /**
     * @brief Move backward a relative distance
     * @param inches Distance to move backward in inches
     * @param timeout Maximum time in milliseconds
     * @param accurate Whether to use accurate profile
     * @param params Movement parameters
     */
    void moveBackward(float inches, int timeout, bool accurate = false,
                     const MoveToPointParams& params = {});

private:
    Chassis& chassis;
    MovementConfig fastProfile;
    MovementConfig accurateProfile;
    MovementConfig currentProfile;
    MovementConfig customProfile;
    bool usingCustomProfile = false;
    bool manualMode = false;
    
    void applyControllerSettings(const ControllerSettings& linear, 
                               const ControllerSettings& angular);
    void setProfile(bool accurate);

    



};

} // namespace lemlib