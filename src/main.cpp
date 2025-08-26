#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <vector>
#include <fstream>
#include <string>
#include "lemlib/chassis/whalelibchassis.hpp"
#include "lemlib/chassis/chassis.hpp"
using pros::delay;


bool clampState = false; // Clamp state (true = open, false = closed)
bool flagState = false; // Flag state (true = open, false = closed)
bool liftState = false; // Lift state (true = raised, false = lowered)





void printToLine(bool isLeft, int lineNumber, const std::string& text) {
    // Set text color to white
    pros::screen::set_pen(0xFFFFFF); // White text

    // Calculate the Y position based on the line number
    int y = 10 + (lineNumber - 1) * 20; 

    // Calculate the X position based on left or right
    int x = isLeft ? 10 : 240; // Left starts at X=10, right at X=240

    // Print the text at the calculated position
    pros::screen::print(pros::E_TEXT_MEDIUM, x, y, text.c_str());
}

void logToFile(const std::string& filename, const std::string& text) {
    std::string fullPath = "/usd/" + filename;
    FILE* file = fopen(fullPath.c_str(), "a"); // Open in append mode
    if (file == nullptr) {
        return; // Failed to open file
    }

    fprintf(file, "%s\n", text.c_str());
    fclose(file);
}

void checkSDCard() {
    // Check if the SD card is mounted
    if (!pros::usd::is_installed()) {
        printToLine(false, 7, "SD card not detected!");
        return;
    }

    // Try to create a test file
    FILE* file = fopen("/usd/test.txt", "w");
    if (file == nullptr) {
        printToLine(false, 7, "Failed to write to SD card!");
        return;
    }

    // Write a test line to the file
    fprintf(file, "This is a test file.\n");
    fclose(file);

    // Verify the file was written
    file = fopen("/usd/test.txt", "r");
    if (file == nullptr) {
        printToLine(false, 7, "Failed to read from SD card!");
        return;
    }

    char line[100];
    if (fgets(line, sizeof(line), file) != nullptr) {
        printToLine(false, 7, "SD card working!");
    } else {
        printToLine(false, 7, "SD card read error!");
    }

    fclose(file);
}




// //PID GAIN SCHEDULER







// // Define PIDGains
// struct PIDGains {
//     double kp;
//     double ki;
//     double kd;
// };

// // Helper function to interpolate between two PIDGains
// PIDGains interpolatePID(const PIDGains& gains1, const PIDGains& gains2, double t) {
//     PIDGains result;
//     result.kp = gains1.kp * (1 - t) + gains2.kp * t;
//     result.ki = gains1.ki * (1 - t) + gains2.ki * t;
//     result.kd = gains1.kd * (1 - t) + gains2.kd * t;
//     return result;
// }


// void moveToPointWithScheduler(lemlib::Chassis& chassis, double x, double y, int timeout, 
//     bool forwards = true, float maxSpeed = 127, float minSpeed = 0, 
//     float earlyExitRange = 0.0) {
//     // Define PID gains for different phases (linear only)
//     PIDGains farLinearGains = {10.0, 0.0, 3.0}; // High kp for fast movement
//     PIDGains mediumLinearGains = {6.0, 0.5, 2.0}; // Moderate kp and ki for controlled movement
//     PIDGains closeLinearGains = {2.0, 0.1, 1.0}; // Low kp and kd for precision

//     // Thresholds for switching between gains
//     double farThreshold = 36.0; // Inches (switch to far gains beyond this distance)
//     double mediumThreshold = 24.0; // Inches (switch to medium gains beyond this distance)

//     // Get the current pose of the robot
//     lemlib::Pose currentPose = chassis.getPose();

//     // Calculate the distance to the target (linear error)
//     double distanceToTarget = std::sqrt(std::pow(x - currentPose.x, 2) + std::pow(y - currentPose.y, 2));

//     // Interpolate linear gains based on distance
//     PIDGains selectedLinearGains;
//     if (distanceToTarget > farThreshold) {
//         selectedLinearGains = farLinearGains; // Use far gains for fast movement
//     } else if (distanceToTarget > mediumThreshold) {
//     // Interpolate between far and medium gains
//         double t = (distanceToTarget - mediumThreshold) / (farThreshold - mediumThreshold);
//         selectedLinearGains = interpolatePID(mediumLinearGains, farLinearGains, t);
//     } else if (distanceToTarget > 0) {
//     // Interpolate between medium and close gains
//         double t = distanceToTarget / mediumThreshold;
//         selectedLinearGains = interpolatePID(closeLinearGains, mediumLinearGains, t);
//     } else {
//         selectedLinearGains = closeLinearGains; // Use close gains for precision
//     }

//     // Update the linear controller gains
//     chassis.setLinearGains(selectedLinearGains.kp, selectedLinearGains.ki, selectedLinearGains.kd);

//     // Call the moveToPoint function with the updated gains and additional parameters
//     chassis.moveToPoint(x, y, timeout, {
//     .forwards = forwards,
//     .maxSpeed = maxSpeed,
//     .minSpeed = minSpeed,
//     .earlyExitRange = earlyExitRange
//     });
// }

// void moveToPoseWithScheduler(lemlib::Chassis& chassis, double x, double y, double theta, int timeout, 
//     bool forwards = true, float maxSpeed = 127, float minSpeed = 0, 
//     float lead = 0.0, float horizontalDrift = 0.0) {
//     // Define PID gains for different phases (linear and angular)
//     PIDGains farLinearGains = {10.0, 0.0, 3.0}; // High kp for fast movement
//     PIDGains mediumLinearGains = {6.0, 0.5, 2.0}; // Moderate kp and ki for controlled movement
//     PIDGains closeLinearGains = {2.0, 0.1, 1.0}; // Low kp and kd for precision

//     PIDGains farAngularGains = {6.0, 0.0, 1.5}; // High kp for fast turning
//     PIDGains mediumAngularGains = {4.0, 0.2, 1.0}; // Moderate kp and ki for controlled turning
//     PIDGains closeAngularGains = {2.0, 0.1, 0.5}; // Low kp and kd for precise turning

//     // Thresholds for switching between gains
//     double farThreshold = 36.0; // Inches (switch to far gains beyond this distance)
//     double mediumThreshold = 24.0; // Inches (switch to medium gains beyond this distance)

//     double farAngularThreshold = 90.0; // Degrees (switch to far angular gains beyond this error)
//     double mediumAngularThreshold = 45.0; // Degrees (switch to medium angular gains beyond this error)

//     // Get the current pose of the robot
//     lemlib::Pose currentPose = chassis.getPose();

//     // Calculate the distance to the target (linear error)
//     double distanceToTarget = std::sqrt(std::pow(x - currentPose.x, 2) + std::pow(y - currentPose.y, 2));

//     // Calculate the heading error (angular error)
//     double headingError = std::abs(theta - currentPose.theta);
//     // Normalize heading error to [-180, 180]
//     if (headingError > 180) headingError -= 360;
//     if (headingError < -180) headingError += 360;
//     headingError = std::abs(headingError);

//     // Interpolate linear gains based on distance
//     PIDGains selectedLinearGains;
//     if (distanceToTarget > farThreshold) {
//         selectedLinearGains = farLinearGains; // Use far gains for fast movement
//     } else if (distanceToTarget > mediumThreshold) {
//     // Interpolate between far and medium gains
//         double t = (distanceToTarget - mediumThreshold) / (farThreshold - mediumThreshold);
//         selectedLinearGains = interpolatePID(mediumLinearGains, farLinearGains, t);
//     } else if (distanceToTarget > 0) {
//     // Interpolate between medium and close gains
//         double t = distanceToTarget / mediumThreshold;
//         selectedLinearGains = interpolatePID(closeLinearGains, mediumLinearGains, t);
//     } else {
//         selectedLinearGains = closeLinearGains; // Use close gains for precision
//     }

//     // Interpolate angular gains based on heading error
//     PIDGains selectedAngularGains;
//     if (headingError > farAngularThreshold) {
//         selectedAngularGains = farAngularGains; // Use far gains for fast turning
//     } else if (headingError > mediumAngularThreshold) {
//     // Interpolate between far and medium gains
//         double t = (headingError - mediumAngularThreshold) / (farAngularThreshold - mediumAngularThreshold);
//         selectedAngularGains = interpolatePID(mediumAngularGains, farAngularGains, t);
//     } else if (headingError > 0) {
//     // Interpolate between medium and close gains
//         double t = headingError / mediumAngularThreshold;
//         selectedAngularGains = interpolatePID(closeAngularGains, mediumAngularGains, t);
//     } else {
//         selectedAngularGains = closeAngularGains; // Use close gains for precise turning
//     }

//     // Update the linear and angular controller gains
//     chassis.setLinearGains(selectedLinearGains.kp, selectedLinearGains.ki, selectedLinearGains.kd);
//     chassis.setAngularGains(selectedAngularGains.kp, selectedAngularGains.ki, selectedAngularGains.kd);

//     // Call the moveToPose function with the updated gains and additional parameters
//     chassis.moveToPose(x, y, theta, timeout, {
//     .forwards = forwards,
//     .horizontalDrift = horizontalDrift,
//     .lead = lead,
//     .maxSpeed = maxSpeed,
//     .minSpeed = minSpeed,
    
//     });
// }

// // PID Gain Scheduler for turnToPoint
// void turnToPointWithScheduler(lemlib::Chassis& chassis, double x, double y, int timeout, 
//     float maxSpeed = 127, float minSpeed = 0, float earlyExitRange = 0.0) {
//     // Define PID gains for different phases (angular only)
//     PIDGains farAngularGains = {6.0, 0.0, 1.5}; // High kp for fast turning
//     PIDGains mediumAngularGains = {4.0, 0.2, 1.0}; // Moderate kp and ki for controlled turning
//     PIDGains closeAngularGains = {2.0, 0.1, 0.5}; // Low kp and kd for precise turning

//     // Thresholds for switching between gains
//     double farAngularThreshold = 90.0; // Degrees (switch to far angular gains beyond this error)
//     double mediumAngularThreshold = 45.0; // Degrees (switch to medium angular gains beyond this error)

//     // Get the current pose of the robot
//     lemlib::Pose currentPose = chassis.getPose();

//     // Calculate the heading error (angular error)
//     double targetHeading = std::atan2(y - currentPose.y, x - currentPose.x) * 180 / M_PI; // Convert to degrees
//     double headingError = std::abs(targetHeading - currentPose.theta);
//     // Normalize heading error to [-180, 180]
//     if (headingError > 180) headingError -= 360;
//     if (headingError < -180) headingError += 360;
//     headingError = std::abs(headingError);

//     // Interpolate angular gains based on heading error
//     PIDGains selectedAngularGains;
//     if (headingError > farAngularThreshold) {
//         selectedAngularGains = farAngularGains; // Use far gains for fast turning
//     } else if (headingError > mediumAngularThreshold) {
//         // Interpolate between far and medium gains
//         double t = (headingError - mediumAngularThreshold) / (farAngularThreshold - mediumAngularThreshold);
//         selectedAngularGains = interpolatePID(mediumAngularGains, farAngularGains, t);
//     } else if (headingError > 0) {
//         // Interpolate between medium and close gains
//         double t = headingError / mediumAngularThreshold;
//         selectedAngularGains = interpolatePID(closeAngularGains, mediumAngularGains, t);
//     } else {
//         selectedAngularGains = closeAngularGains; // Use close gains for precise turning
//     }

//     // Update the angular controller gains
//     chassis.setAngularGains(selectedAngularGains.kp, selectedAngularGains.ki, selectedAngularGains.kd);

//     // Call the turnToPoint function with the updated gains
//     chassis.turnToPoint(x, y, timeout, {
//         .maxSpeed = static_cast<int>(maxSpeed),
//         .minSpeed = static_cast<int>(minSpeed),
//         .earlyExitRange = earlyExitRange
//     });
// }

// // PID Gain Scheduler for turnToHeading
// void turnToHeadingWithScheduler(lemlib::Chassis& chassis, double targetHeading, int timeout, 
//     float maxSpeed = 127, float minSpeed = 0, 
//     lemlib::AngularDirection direction = lemlib::AngularDirection::CW_CLOCKWISE) {
//     // Define PID gains for different phases (angular only)
//     PIDGains farAngularGains = {100.0, 0.0, 1.5}; // High kp for fast turning
//     PIDGains mediumAngularGains = {100.0, 0.2, 1.0}; // Moderate kp and ki for controlled turning
//     PIDGains closeAngularGains = {100.0, 0.1, 0.5}; // Low kp and kd for precise turning

//     // Thresholds for switching between gains
//     double farAngularThreshold = 90.0; // Degrees (switch to far angular gains beyond this error)
//     double mediumAngularThreshold = 45.0; // Degrees (switch to medium angular gains beyond this error)

//     // Get the current pose of the robot
//     lemlib::Pose currentPose = chassis.getPose();

//     // Calculate the heading error (angular error)
//     double headingError = std::abs(targetHeading - currentPose.theta);
//     // Normalize heading error to [-180, 180]
//     if (headingError > 180) headingError -= 360;
//     if (headingError < -180) headingError += 360;
//     headingError = std::abs(headingError);

//     // Interpolate angular gains based on heading error
//     PIDGains selectedAngularGains;
//     if (headingError > farAngularThreshold) {
//         selectedAngularGains = farAngularGains; // Use far gains for fast turning
//     } else if (headingError > mediumAngularThreshold) {
//         // Interpolate between far and medium gains
//         double t = (headingError - mediumAngularThreshold) / (farAngularThreshold - mediumAngularThreshold);
//         selectedAngularGains = interpolatePID(mediumAngularGains, farAngularGains, t);
//     } else if (headingError > 0) {
//         // Interpolate between medium and close gains
//         double t = headingError / mediumAngularThreshold;
//         selectedAngularGains = interpolatePID(closeAngularGains, mediumAngularGains, t);
//     } else {
//         selectedAngularGains = closeAngularGains; // Use close gains for precise turning
//     }

//     // Update the angular controller gains
//     chassis.setAngularGains(selectedAngularGains.kp, selectedAngularGains.ki, selectedAngularGains.kd);

//     // Call the turnToHeading function with the updated gains
//     chassis.turnToHeading(targetHeading, timeout, {
//         .direction = direction,
//         .maxSpeed = static_cast<int>(maxSpeed),
//         .minSpeed = static_cast<int>(minSpeed)
//     });
// }



// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);




// FLAG CODE
pros::adi::DigitalOut flag('B');

void flagTask() {
    while (true) {
        flag.set_value(flagState);  
        pros::delay(20);  
    }
}



// CLAMP CODE
pros::adi::DigitalOut clamp('E');

void clampTask() {
    while (true) {
        clamp.set_value(clampState);  // 127 activates, 0 deactivates
        pros::delay(20);
    }
}



pros::adi::DigitalOut intakeLift('F');

void liftTask() {
    while (true) {
        intakeLift.set_value(liftState);  
        pros::delay(20);  
    }
}




//INTAKE CODE

// Intake motor setup

pros::MotorGroup intake({2,-3});

bool pneumaticEnabled = false;  // Track pneumatic state separately


//Lb Motor
pros::Motor lb(-21);
//Lb rotation sensor
pros::Rotation lbrotation(9);

// Pneumatic actuator (adjust port number)
pros::adi::DigitalOut pneumatic('C');

// Pneumatic actuator (adjust port number)
// Lift states (4 positions)
// Modify your states array and pneumatic states:
const int numstates = 5;  // Changed from 4 to 5
int states[numstates] = {0, 715, 4000, 5200, 5000};  // Added 1500 for mobile goal tip
bool pneumatic_states[numstates] = {false, false, true, false, false};  // Pneumatic on for this state

// State tracking
int currstate = 0;
int target = 0;
bool liftActive = false;

// ===== LIFT CONTROL FUNCTIONS ===== //

// Simple sequential state cycling
void nextstate() {
    currstate = (currstate + 1) % numstates;
    target = states[currstate];
    
    // Only activate pneumatic at high score position (4000)
    if (states[currstate] == 4000) {
        pneumatic.set_value(true);
        pneumaticEnabled = true;
    } else {
        // Only turn off if we're leaving the high score position
        if (pneumaticEnabled) {
            pneumatic.set_value(false);
            pneumaticEnabled = false;
        }
    }
    
    liftActive = true;
    pros::delay(20);
}

// Modified moveLiftTo() function:
void moveLiftTo(int targetPosition, bool activate_pneumatic = false) {
    target = targetPosition;
    liftActive = true;
    
    // Handle pneumatic explicitly
    if (activate_pneumatic) {
        pneumatic.set_value(true);
        pneumaticEnabled = true;
    } else if (targetPosition != 4000) {  // Don't turn off if we're at high score
        pneumatic.set_value(false);
        pneumaticEnabled = false;
    }
    
    pros::delay(10);
}

// Basic PID constants
double lb_kp = 0.07;  // Proportional gain
double lb_ki = 0.00001;  // Integral gain (set to 0 for now)
double lb_kd = 0.01;  // Derivative gain (set to 0 for now)

// Basic PID variables
double lb_integral = 0;
double lb_lastError = 0;

double normalizeError(double target, double currentPosition) {
    // Convert sensor reading to actual lift position (divide by 4)
    double actualPosition = currentPosition / 4.0;
    double error = target - actualPosition;
    
    // Normalize to 0-180° range (0-18000 centidegrees)
    while (error > 18000) error -= 36000;
    while (error < -18000) error += 36000;
    
    // If error is between -180° and 0°, convert to positive equivalent
    if (error < 0) {
        error += 36000;
    }

    // Now error is in 0-36000 range, convert to 0-18000
    if (error > 18000) {
        error = 36000 - error;  // Take the shorter way around
    }
    
    // Return signed error (positive means need to move forward)
    return (target >= actualPosition) ? error : -error;
}

void liftcontrol() {
    if (liftActive) {
        // Get current position (divided by 4 to account for 1:4 gear ratio)
        double currentPosition = lbrotation.get_position() / 4.0;

        // Normalize the error to handle wrap-around
        double error = normalizeError(target, currentPosition);

        // Debugging: Print current position, error, and target
        std::ostringstream positionStream, errorStream, targetStream;
        positionStream << "Position: " << currentPosition / 100.0;
        errorStream << "Error: " << error / 100.0;
        targetStream << "Target: " << target;

        printToLine(false, 4, positionStream.str());
        printToLine(false, 5, errorStream.str());
        printToLine(false, 6, targetStream.str());

        // Update integral term
        lb_integral += error;

        // Calculate derivative term
        double derivative = error - lb_lastError;
        lb_lastError = error;

        // Calculate PID output
        double velocity = (lb_kp * error) + (lb_ki * lb_integral) + (lb_kd * derivative);

        // Debugging: Print velocity
        std::ostringstream velocityStream;
        velocityStream << "Velocity: " << velocity;
        printToLine(true, 7, velocityStream.str());

        // Move the motor
        lb.move(velocity);
    } else {
        // Stop the motor if not active
        lb.move(0);
    }
}

void loadState() {
    target = 715;
    pneumatic.set_value(false);
    liftActive = true;
    while (fabs(lbrotation.get_position()/4.0 - 715) > 5000) pros::delay(10);
}

void scoreHigh() {
    target = 4000;
    pneumatic.set_value(true);
    liftActive = true;
    while (fabs(lbrotation.get_position()/4.0 - 4000) > 5000) pros::delay(10);
}

void scoreBig() {
    target = 5200;
    pneumatic.set_value(true);
    liftActive = true;
    while (fabs(lbrotation.get_position()/4.0 - 5200) > 5000) pros::delay(10);
}

void tipMobileGoal() {
    target = 5000;
    pneumatic.set_value(false);
    liftActive = true;
    while (fabs(lbrotation.get_position()/4.0 - 5000) > 5000) pros::delay(10);
}

void resetLift() {
    target = 0;
    pneumatic.set_value(false);
    liftActive = true;
}






// motor groups
pros::MotorGroup leftMotors({-20, -12, -14}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({18, 17, 15}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(16);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed

//hotracking
pros::Rotation horizontalEnc(-13);

// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
//fix
//pros::Rotation verticalEncleft(6);
pros::Rotation verticalEncright(-19);


// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//hotracking

lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -2);


// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel verticalleft(&verticalEncleft, lemlib::Omniwheel::NEW_325, -5.315);
lemlib::TrackingWheel verticalright(&verticalEncright, lemlib::Omniwheel::NEW_325, 5.315);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.663, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);




// Accurate linear controller - optimized for precision
lemlib::ControllerSettings accurateLinear(
    54,    // kP 
    0.08,   // kI 
    440,   // kD 
    2,     // windupRange
    1,   // smallError 
    100,   // smallErrorTimeout 
    3,     // largeError 
    500,   // largeErrorTimeout 
    20     // slew
);

// Accurate angular controller
lemlib::ControllerSettings accurateAngular(
    6.1,   // kP
    0.5,   // kI 
    70,    // kD
    4,     // windupRange
    3,     // smallError
    100,   // smallErrorTimeout
    8,     // largeError
    500,   // largeErrorTimeout
    0     // slew
);

// Fast linear controller - optimized for speed
lemlib::ControllerSettings fastLinear(
    54,    // kP 
    0.08,   // kI 
    440,   // kD 
    2,     // windupRange
    1,   // smallError 
    100,   // smallErrorTimeout 
    3,     // largeError 
    500,   // largeErrorTimeout 
    20     // slew
);

// Fast angular controller  
lemlib::ControllerSettings fastAngular(
    6.1,   // kP
    0.5,   // kI 
    70,    // kD
    4,     // windupRange
    3,     // smallError
    100,   // smallErrorTimeout
    8,     // largeError
    500,   // largeErrorTimeout
    0     // slew
);
//put fast pid values For the following linearController and angularController

// // lateral motion controller
// lemlib::ControllerSettings linearController(54, // proportional gain (kP)
//                                             0.08, // integral gain (kI)
//                                             440, // derivative gain (kD)
//                                             2, // anti windup
//                                             1, // small error range, in inches
//                                             100, // small error range timeout, in milliseconds
//                                             3, // large error range, in inches
//                                             500, // large error range timeout, in milliseconds
//                                             20 // maximum acceleration (slew)
// );


// //2.35 and 1.5
// // angular motion controller
// lemlib::ControllerSettings angularController(6.1, // proportional gain (kP)
//                                              0.5, // integral gain (kI)
//                                              70, // derivative gain (kD)
//                                              4, // anti windup
//                                              1, // small error range, in degrees
//                                              100, // small error range timeout, in milliseconds
//                                              3, // large error range, in degrees
//                                              500, // large error range timeout, in milliseconds
//                                              20 // maximum acceleration (slew)
// );

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            &verticalright, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial snsor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);


// create the chassis
//lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
lemlib::Chassis whaleacurate(drivetrain, accurateLinear, accurateAngular, sensors, &throttleCurve, &steerCurve);
lemlib::Chassis whalefast(drivetrain, fastLinear, fastAngular, sensors, &throttleCurve, &steerCurve);
lemlib::Chassis defaultChassis(drivetrain, fastLinear, fastAngular, sensors, &throttleCurve, &steerCurve);
lemlib::Whale whale(defaultChassis);
lemlib::Whale whaleFast(whalefast);
lemlib::Whale whaleAccurate(whaleacurate);




// // create the chassis
// ForcedImuChassis chassis(
//     drivetrain,
//     linearController,
//     angularController,
//     sensors,
//     &imu, // Pass IMU reference
//     &throttleCurve,
//     &steerCurve
// ):






/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

    pros::lcd::initialize(); // initialize brain screen
    whalefast.calibrate(); // calibrate sensors
    lbrotation.set_position(0);
    checkSDCard();
    pros::screen::set_pen(0x000000); // Black color
    pros::screen::fill_rect(0, 0, 480, 272); // Full screen rectangle

    // Example usage
    //printToLine(true, 1, "Left Line 1: Hello!"); // Print to left side, line 1
    // printToLine(false, 1, "Right Line 1: Hi!");  // Print to right side, line 1
    // printToLine(true, 2, "Left Line 2: Testing"); // Print to left side, line 2
    // printToLine(false, 2, "Right Line 2: OK");   // Print to right side, line 2
    //resets rotation sensor for lady brown
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // Get the chassis pose
            lemlib::Pose pose = whalefast.getPose();

            // Format the pose values as strings
            std::ostringstream xStream, yStream, thetaStream;
            xStream << "X: " << pose.x;
            yStream << "Y: " << pose.y;
            thetaStream << "Theta: " << pose.theta;

            // Print the pose values using printToLine
            printToLine(true, 1, xStream.str()); // Print X on the left side, line 1
            printToLine(true, 2, yStream.str()); // Print Y on the left side, line 2
            printToLine(true, 3, thetaStream.str()); // Print Theta on the left side, line 3

            // Log position telemetry (optional)
            lemlib::telemetrySink()->info("Chassis pose: {}", whalefast.getPose());

            // Delay to save resources
            pros::delay(50);
        }
    });

    //task for lb
    pros::Task liftcontroltask([]{
        while (true) {
            liftcontrol();  // Now this is properly defined
            pros::delay(10);
        }
    });




    // CLAMP TASK
    pros::Task clampController(clampTask);

    // FLAG TASK
    pros::Task flagControl(flagTask);

    pros::Task liftControl(liftTask);



}

/**
 * Runs while the robot is disabled
 */
void disabled() {


}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {

    // lemlib::Pose start = chassis.getPose();
    
    // // =====================================================
    // // Quadratic Curve (3 points + params)
    // // =====================================================
    // lemlib::Pose control1(start.x + 24, start.y + 12, 0);
    // lemlib::Pose control2(start.x + 36, start.y + 6, 0);
    // lemlib::Pose end_cubic(start.x + 60, start.y + 0, 90);
    
    // // Explicitly call cubic version
    // followBezier(chassis, start, control1, control2, end_cubic, 30, 80);
    // //                       4 poses            |       |
    // //                                         int    float

    //FOLLOW ARC EXAMPLE
    // // Basic usage with defaults
    // follow_arc(chassis, 30, 30, 24, true);

    // // Full parameter specification
    // follow_arc(chassis, 40, 20, 18, false, 4000, false, 90, 20, 2);


    //bezier curve 



    // EXAMPLE RT

    
    
    // // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    // chassis.moveToPose(20, 15, 90, 4000);
    // // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    // chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // // cancel the movement after it has traveled 10 inches
    // chassis.waitUntil(10);
    // chassis.cancelMotion();
    // // Turn to face the point x:45, y:-45. Timeout set to 1000
    // // dont turn faster than 60 (out of a maximum of 127)
    // chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // // Turn to face a direction of 90º. Timeout set to 1000
    // // will always be faster than 100 (out of a maximum of 127)
    // // also force it to turn clockwise, the long way around
    // chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // // following the path with the back of the robot (forwards = false)
    // // see line 116 to see how to define a path
    // chassis.follow(example_txt, 15, 4000, false);
    // // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // // the movement will run immediately
    // // Unless its another movement, in which case it will wait
    // chassis.waitUntil(10);
    // pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // // wait until the movement is done
    // chassis.waitUntilDone();
    // pros::lcd::print(4, "pure pursuit finished!");


    // //example set gains
    // // Example 1: Setting gains for mobile goal with rings
    // applyGains(chassis, LoadState::WITH_RINGS);
    // chassis.moveToPoint(x, y, timeout);

    // // Example 2: Switching to no load configuration
    // applyGains(chassis, LoadState::NO_LOAD);
    // chassis.turnToHeading(90, timeout);

    // // Example 3: Mobile goal only
    // applyGains(chassis, LoadState::MOBILE_GOAL);
    // chassis.moveToPose(x, y, theta, timeout);






    //  LB TESTING






	// // Move lift to 30°
    // moveLiftTo(3000);

    // // Intake
    // runIntake(127);
    // pros::delay(1000); 
    // stopIntake(); 

    // //  Intake
    // runIntake(127);
    // pros::delay(1000);  
    // stopIntake(); 

    // // // Move lift to 150°
    // // moveLiftTo(15000); 

    // // // Move lift back to 30°
    // // moveLiftTo(3000); 

    // // // Move lift to 150°
    // // moveLiftTo(15000); 

	// // // Move lift to 30°
    // // moveLiftTo(3000); 

    // // // Intake
    // // runIntake(127); 
    // // pros::delay(1000);  
    // // stopIntake(); 

    // // // Intake
    // // runIntake(127); 
    // // pros::delay(1000);  
    // // stopIntake(); 

    // // // Move lift to 150°
    // // moveLiftTo(15000); 

    // // // Move lift back to 30°
    // // moveLiftTo(3000);

    // // // Move lift to 150°
    // // moveLiftTo(15000);


    //Apply gains example



    // // When no mobile goal (normal operation)
    // applyDrivetrainGains(chassis, LoadState::NORMAL); 

    // // When carrying empty mobile goal
    // applyDrivetrainGains(chassis, LoadState::EMPTY);

    // // With partial/full rings
    // applyDrivetrainGains(chassis, LoadState::HALF_RINGS);
    // applyDrivetrainGains(chassis, LoadState::FULL_RINGS);


    
    // // EXAMPLE CUSTOM
    // // Move to (20, 15) with heading 90°, moving forwards, max speed 100, min speed 20, lead 0.6, horizontal drift 0.1
    // moveToPoseWithScheduler(chassis, 20, 15, 90, 4000, true, 100, 20, 0.6, 0.1);
    // // Turn to face the point (24, 24) with max speed 80, min speed 10, and early exit range of 5°
    // turnToPointWithScheduler(chassis, 24, 24, 1000, 80, 10, 5.0);
    // // Turn to face 90° with max speed 80, min speed 10, and clockwise direction
    //turnToHeadingWithScheduler(chassis, 90, 1000, 80, 10, lemlib::AngularDirection::CW_CLOCKWISE);
    // // Move to (20, 15) with max speed 100, min speed 20, and early exit range of 2 inches
    // moveToPointWithScheduler(chassis, 20, 15, 4000, true, 100, 20, 2.0);

    //Testing


    // // TUNING GAIN SCHEDUALER    
    

    // // Move forward 72 inches long distence
    // moveToPointWithScheduler(chassis, 0, 72, 10000, true);
    // //medium distence
    // moveToPointWithScheduler(chassis, 24, 0, 0, 4000);
    // //short distence
    // moveToPointWithScheduler(chassis, 12, 0, 0, 4000);

    // // Large turn
    // turnToHeadingWithScheduler(chassis, 90, 1000);
    // // medium turn
    // turnToHeadingWithScheduler(chassis, 45, 1000);
    // // small turn
    // turnToHeadingWithScheduler(chassis, 10, 1000);
   
    //Tuning with and without mobile goal
    // applyDrivetrainGains(chassis, LoadState::FULL_RINGS);
    // clampState=false;
    // pros::delay(1000);
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0,24, 50000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 1000);


    //loadState();
    //pros::delay(2000);f
    // //scoreHigh();
    //scoreBig();
    // pros::delay(2000);
    // resetLift();
    //-44.5 -13.1
    
    // chassis.setPose(0,0,0);
    // chassis.moveToPoint(0, 36, 5000);
    // pros::delay(3000);
    // chassis.turnToHeading(90, 100);

    whale.setPose(0, 0, 0);
    pros::delay(2000);
    whaleFast.moveBackward(30, 1000);

}



/**
 * Runs in driver control
 */
 void opcontrol() {

    while (true) {
        pros::Controller controller(pros::E_CONTROLLER_MASTER);
        liftState = false;


    
        // Arcade drive control
        
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        whalefast.arcade(leftY, rightX);
        
        // LADY BROWN LIFT CONTROL
        
        // R1: Loading → High Score (4000 with pneumatic) → Zero
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (target == 0) {
                // At zero - go to loading (no pneumatic)
                moveLiftTo(715);
                pneumatic.set_value(false); // Ensure pneumatic is off
            } 
            else if (target == 715) {
                // At loading - go to high score WITH PNEUMATIC
                moveLiftTo(4000);
                pneumatic.set_value(true); // Activate pneumatic
            }
            else {
                // At any position - return to zero
                moveLiftTo(0);
                pneumatic.set_value(false); // Deactivate pneumatic
            }
        }

        pros::delay(10);
        


        // L2: Loading → Big Score (5500) → Zero
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (target == 0) {
                // At zero - go to loading
                moveLiftTo(715);
                pneumatic.set_value(false);
            }
            else if (target == 715) {
                // At loading - go to big score (NO PNEUMATIC)
                moveLiftTo(5200);
                pneumatic.set_value(false);
            }
            else {
                // At any position - return to zero
                moveLiftTo(0);
                pneumatic.set_value(false);
            }
        }

        pros::delay(10);

        // In opcontrol():
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (target == 0) {
                // From zero → go straight to tip position (5500 with pneumatic)
                moveLiftTo(5000, false);  // true activates pneumatic
            }
            else {
                // At any position - return to zero
                moveLiftTo(0);
                pneumatic.set_value(false);
            }
        }


        pros::delay(10);
    

        // INTAKE
        // INTAKE CONTROL (simplified)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(127); // Run intake forward
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-127); // Run intake in reverse
        } 
        else {
            intake.move(0); // Stop intake
        }

        pros::delay(20); // Standard delay for opcontrol loop

        //CLAMP CODE

        pros::delay(10);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            clampState = !clampState;  // Toggle clamp state
        }

        pros::delay(10); // Short delay to prevent CPU hogging

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            flagState = !flagState;
        }

        pros::delay(10);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            liftState = !liftState;  // Toggle lift state
        }

        pros::delay(10);

    }
}