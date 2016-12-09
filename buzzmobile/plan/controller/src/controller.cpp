#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#include <buzzmobile/CarPose.h>
#include <buzzmobile/CarState.h>

#include <math.h>

using namespace buzzmobile;

// Buttons mapped to a PS4 controller using the Joy ROS node.
#define reverse_button buttons[15]
#define brake_button buttons[13]
#define horn_button buttons[0]
#define velocity_trigger axes[9]
#define toggle_auto_button buttons[16]

ros::Publisher motion_pub;
ros::Publisher state_pub;

int pubFreq     = 10; //hz

bool obstacleFlag = false;

// Keep track of the values to continously publish
float lastSpeed = 0;
float lastAngle = 0;
bool lastBrake = false;
bool lastHorn = 0;
uint8_t lastState = buzzmobile::CarState::START; // Start the car in manual mode
double maxSpeed = 0;

unsigned int state;
bool manualToggle = true; //start up with manual toggle = true
bool brakePushed  = false;

// Used as a latch, which is set the first time the user presses
// a button on the controller. Otherwise, the first few messages
// are a pointer to a zero memory block, and the inital button value is
// assumed to be zero, which breaks our logic.
bool speedSet = false;

void handleBrake(const sensor_msgs::Joy::ConstPtr& joy);
void handleDrive(const sensor_msgs::Joy::ConstPtr& joy);
void handleTurn(const sensor_msgs::Joy::ConstPtr& joy);
void handleHorn(const sensor_msgs::Joy::ConstPtr& joy);
void handleState(const sensor_msgs::Joy::ConstPtr& joy);

void honkHorn();
void sendMotionCommand();
void sendStateCommand();
void sendBrakeCommand();

void keepAliveCallback(const ros::TimerEvent&) {
    // To keep alive, just resend the motion command
    sendMotionCommand();
    sendStateCommand();
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
//  Always publish the message
    handleHorn(joy);
    handleBrake(joy);
    handleDrive(joy);
    handleTurn(joy);
    handleState(joy);
}

void sendMotionCommand() {
    buzzmobile::CarPose msg;
    msg.velocity = lastSpeed;
    msg.angle = lastAngle;
    msg.horn = lastHorn;
    msg.brake = lastBrake;
    msg.mode = "manual";
    motion_pub.publish(msg);
}

void sendStateCommand() {
    buzzmobile::CarState msg;
    msg.state = lastState;
    state_pub.publish(msg);
}

void handleState(const sensor_msgs::Joy::ConstPtr& joy) {
    if (joy->toggle_auto_button) { // If home button is pressed
        if (lastState == buzzmobile::CarState::AUTO) {
            lastState = buzzmobile::CarState::MANUAL;
        } else if (lastState == buzzmobile::CarState::MANUAL) {
            lastState = buzzmobile::CarState::AUTO;
        } else if (lastState == buzzmobile::CarState::START) {
            lastState = buzzmobile::CarState::MANUAL;
        } else {
            ROS_INFO("Error in switching states. Last state = %d", lastState);
            throw;
        }
    }
}

void handleDrive(const sensor_msgs::Joy::ConstPtr& joy) {
    //********************************************
    //Motor control
    float speed = 0.0;

    //ROS_INFO("Last brake %d", lastBrake);
    speedSet = (!speedSet && joy->velocity_trigger != 0.0);
    if (!lastBrake && speedSet) {
        if (joy->reverse_button) {
            float correction = -1; // Normally ranges from 1 to -1. Correct so it ranges from 0 to -2
            // Division by 2 brings to range -1 to 0. It can then be multiplied by max speed
            speed = ((correction + joy->velocity_trigger) / 2.0) * maxSpeed;
        } else {
            float correction = 1; // Once inverted, will range from -1 to 1. Correct to: 0 to 2
            // Division by 2 brings to range 0 to 1. It can then be multiplied by max speed
            speed = ((correction + -1 * joy->velocity_trigger) / 2.0) * maxSpeed;
        }
    }

    ROS_INFO("Speed: %f", speed);
    if (lastSpeed != speed) {
        lastSpeed = speed;
        sendMotionCommand();
    }
}

void handleBrake(const sensor_msgs::Joy::ConstPtr& joy) {
    int brake = 2; // 2 indicates lastBrake has not been changed;
    if (joy->brake_button) {
        // When we first press the button, toggle the flag and publish
        if (!lastBrake) {
            brake = 1;
        }
    } else {
        // When we first release the button, toggle the flag and publish
        if (lastBrake) {
            brake = 0;
        }
    }
    if (brake != 2) { // If brake was changed
        lastBrake = brake;
        sendMotionCommand();
    }
}

void handleTurn(const sensor_msgs::Joy::ConstPtr& joy) {
    float angle = 0;
    double maxAngle;
    ros::param::get("max_steering_angle", maxAngle); // Get turning radius from constants.yaml

    // first test is to see if we're close to 0
    float mag = sqrt(joy->axes[0] * joy->axes[0] + joy->axes[1] * joy->axes[1]);
    //joystick is around the center...send 0 speed
    if (mag > 1e-6) {
        angle = -(atan2(fabs(joy->axes[1]), -joy->axes[0]) - M_PI_2);
        angle = angle / (M_PI / 2); // Divide by 90 degrees to get value [0, 1]
        angle = angle * maxAngle; // Scale to be from [0, maxAngle]
        ROS_INFO("angle: [%f]", angle);
    }
    if (lastAngle != angle) {
        lastAngle = angle;
        sendMotionCommand();
    }
}

void handleHorn(const sensor_msgs::Joy::ConstPtr& joy) {
    lastHorn = joy->horn_button;
}

//TODO not sure what this is for. It isn't being used.
//void honkHorn() {
    //sound_play::SoundRequest msg;
    //msg.sound = sound_play::SoundRequest::PLAY_FILE;
    //msg.command = sound_play::SoundRequest::PLAY_ONCE;
    //msg.arg = "/home/agency/Downloads/vehicle042.wav";
    //horn_pub.publish(msg);
//}

//TODO not sure what this is for. It isn't being used
//void obstacleCallback(const std_msgs::Bool::ConstPtr& flag) {
//	if (!obstacleFlag && flag->data) {
//		honkHorn();
//	} //TODO: for some reason this will honk repeatedly (repeated obstacles)...may need to debug this
//
//	obstacleFlag = flag->data;
//}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");
    ros::param::get("max_speed", maxSpeed); // Get max speed from constants.yaml

    ros::NodeHandle n;

    motion_pub = n.advertise<buzzmobile::CarPose>("manual_car_pose", 0);
    state_pub = n.advertise<buzzmobile::CarState>("car_state", 0);

    // Initialize the latched topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("/joy", 100, joyCallback);

    //ros::Rate r(pubFreq); // Not sure what this is

    ros::Timer keepAliveTimer = n.createTimer(ros::Duration(1.0/pubFreq), keepAliveCallback);
    ros::spin();

    return 0;
}
