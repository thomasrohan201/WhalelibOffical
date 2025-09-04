#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cstddef>
#include <functional>
#include <vector>
#include <fstream>
#include <string>
#include "lemlib/chassis/whalelibchassis.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "whalelibGainScheduler.hpp"

using pros::delay;


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






// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


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
lemlib::Chassis chassis(drivetrain, fastLinear, fastAngular, sensors, &throttleCurve, &steerCurve);
lemlib::Whale whale(chassis);
lemlib::Whale whaleFast(whalefast);
lemlib::Whale whaleAccurate(whaleacurate);

// Gain scheduler instances (using the same chassis objects)
lemlib::WhaleGainScheduler whaleScheduler(chassis);
lemlib::WhaleGainScheduler whaleFastScheduler(whalefast);
lemlib::WhaleGainScheduler whaleAccurateScheduler(whaleacurate);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

    pros::lcd::initialize(); // initialize brain screen
    whalefast.calibrate(); // calibrate sensors
    checkSDCard();
    pros::screen::set_pen(0x000000); // Black color
    pros::screen::fill_rect(0, 0, 480, 272); // Full screen rectangle

    // Configure gain scheduling for each scheduler instance
    whaleFastScheduler.setLinearGainScheduler(
        {54.0, 0.08, 440.0},   
        {27.0, 0.04, 220.0},    
        {13.5, 0.02, 110.0},    
        36.0f,                  // Far threshold (inches)
        24.0f                   // Medium threshold (inches)
    );
    
    whaleFastScheduler.setAngularGainScheduler(
        {6.1, 0.5, 70.0},      
        {3.05, 0.25, 35.0},    
        {1.525, 0.125, 17.5},  
        90.0f,                  // Far threshold (degrees)
        45.0f                   // Medium threshold (degrees)
    );
    
    // More conservative gains for accurate scheduler
    whaleAccurateScheduler.setLinearGainScheduler(
        {40.0, 0.1, 300.0},    
        {25.0, 0.08, 200.0},   
        {15.0, 0.06, 100.0},   
        24.0f,                  
        12.0f
    );
    
    whaleAccurateScheduler.setAngularGainScheduler(
        {4.0, 0.6, 50.0},      
        {2.5, 0.5, 30.0},      
        {1.5, 0.4, 15.0},      
        45.0f,                  
        22.5f
    );
    
    // Default scheduler
    whaleScheduler.setLinearGainScheduler(
        {50.0, 0.09, 400.0},
        {30.0, 0.06, 250.0},
        {18.0, 0.03, 120.0},
        30.0f,
        15.0f
    );

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
 * Example with all whalelib functions
 */
void autonomous() {

    whale.setPose(0, 0, 0);
    whaleFast.moveToPoint(0, 24, 1000);
    whale.waitUntil(10);

}

/**
 * Runs in driver control
 */
 void opcontrol() {

    while (true) {
        pros::Controller controller(pros::E_CONTROLLER_MASTER);

    
        // Arcade drive control
        
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        whalefast.arcade(leftY, rightX);
        


    }
}