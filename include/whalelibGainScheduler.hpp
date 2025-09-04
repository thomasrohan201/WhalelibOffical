#pragma once

#include "lemlib/chassis/whalelibchassis.hpp"
#include <functional>

namespace lemlib {

/**
 * @brief PID gains structure for gain scheduling
 */
struct PIDGains {
    double kp;
    double ki;
    double kd;
    
    PIDGains(double p = 0.0, double i = 0.0, double d = 0.0) 
        : kp(p), ki(i), kd(d) {}
};
//de piedra 
/**
 * @brief Whale movement controller with gain scheduling capabilities
 */
class WhaleGainScheduler : public Whale {
public:
    /**
     * @brief Construct a new WhaleGainScheduler
     * @param chassis Reference to the chassis object
     */
    WhaleGainScheduler(Chassis& chassis);
    
    // Gain scheduling movement functions
    void moveToPointWithScheduler(float x, float y, int timeout, 
                                 bool forwards = true, float maxSpeed = 127, 
                                 float minSpeed = 0, float earlyExitRange = 0.0);
    
    void moveToPoseWithScheduler(float x, float y, float theta, int timeout, 
                                bool forwards = true, float maxSpeed = 127, 
                                float minSpeed = 0, float lead = 0.0, 
                                float horizontalDrift = 0.0);
    
    void turnToPointWithScheduler(float x, float y, int timeout, 
                                 float maxSpeed = 127, float minSpeed = 0, 
                                 float earlyExitRange = 0.0);
    
    void turnToHeadingWithScheduler(float targetHeading, int timeout, 
                                   float maxSpeed = 127, float minSpeed = 0, 
                                   AngularDirection direction = AngularDirection::CW_CLOCKWISE);
    
    // Gain configuration
    void setLinearGainScheduler(const PIDGains& farGains, const PIDGains& mediumGains, 
                               const PIDGains& closeGains, float farThreshold = 36.0f, 
                               float mediumThreshold = 24.0f);
    
    void setAngularGainScheduler(const PIDGains& farGains, const PIDGains& mediumGains, 
                                const PIDGains& closeGains, float farThreshold = 90.0f, 
                                float mediumThreshold = 45.0f);
    
    // Custom interpolation function support
    void setLinearInterpolationFunction(std::function<PIDGains(float distance, const Pose& current, const Pose& target)> func);
    void setAngularInterpolationFunction(std::function<PIDGains(float error, const Pose& current, const Pose& target)> func);
    
    // Utility functions for gain scheduling
    PIDGains getCurrentLinearGains() const;
    PIDGains getCurrentAngularGains() const;
    void resetGainScheduler();

private:
    // Gain scheduling members
    PIDGains farLinearGains;
    PIDGains mediumLinearGains;
    PIDGains closeLinearGains;
    PIDGains farAngularGains;
    PIDGains mediumAngularGains;
    PIDGains closeAngularGains;
    
    float linearFarThreshold;
    float linearMediumThreshold;
    float angularFarThreshold;
    float angularMediumThreshold;
    
    PIDGains currentLinearGains;
    PIDGains currentAngularGains;
    
    std::function<PIDGains(float distance, const Pose& current, const Pose& target)> linearInterpolationFunc;
    std::function<PIDGains(float error, const Pose& current, const Pose& target)> angularInterpolationFunc;
    
    MovementConfig originalConfig;
    bool gainsModified;
    
    // Helper functions
    PIDGains interpolatePID(const PIDGains& gains1, const PIDGains& gains2, float t) const;
    PIDGains calculateLinearGains(float distance) const;
    PIDGains calculateAngularGains(float error) const;
    void applyGainScheduler(const PIDGains& linear, const PIDGains& angular);
    ControllerSettings convertPidGainsToControllerSettings(const PIDGains& pidGains, const ControllerSettings& templateSettings) const;

};

} // namespace lemlib