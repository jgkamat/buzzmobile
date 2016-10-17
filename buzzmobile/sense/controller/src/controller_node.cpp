#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

#include <buzzmobile/CarPose.h>
#include <buzzmobile/CarState.h>

#include <math.h>

using namespace buzzmobile;

#define reverse_button buttons[0]
#define brake_button buttons[14]
#define horn_button buttons[1]
#define velocity_trigger axes[4]
#define toggle_auto_button buttons[12]

ros::Publisher motion_pub;
ros::Publisher state_pub;
//NOTE: this is also published by the autonomous brake...this is ok.
ros::Publisher brake_pub;

int maxFwdSpeed = 1; //m/s
int pubFreq     = 10; //hz

bool obstacleFlag = false;

float lastSpeed = 0;
float lastAngle = 0;
bool lastHorn = 0;
uint8_t lastAuto = 0;
uint8_t lastManual = 1; // Start the car in manual mode

unsigned int state;
bool manualToggle = true; //start up with manual toggle = true
bool brakePushed  = false;

/*void velocityCallback(const velocityValue::ConstPtr& msg) {
	speed_value = msg->velocity;
}
*/
void handleBrake(const sensor_msgs::Joy::ConstPtr& joy);
void handleDrive(const sensor_msgs::Joy::ConstPtr& joy);
void handleTurn(const sensor_msgs::Joy::ConstPtr& joy);
void handleHorn(const sensor_msgs::Joy::ConstPtr& joy);
void handleState(const sensor_msgs::Joy::ConstPtr& joy);

void honkHorn(); 
void sendMotionCommand();
void sendBrakeCommand();

void keepAliveCallback(const ros::TimerEvent&) {
  // To keep alive, just resend the motion command
  sendMotionCommand();
}

//void stateCallback(const buzzmobile::CarState::ConstPtr& stateMsg) {
//    state = stateMsg->state;
//}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
//  Always publish the message
    handleHorn(joy);
    handleDrive(joy);
    handleTurn(joy);
    handleState(joy);
}

void sendMotionCommand() {
  buzzmobile::CarPose msg;
  msg.velocity = lastSpeed;
  msg.angle = lastAngle;
  msg.horn = lastHorn;
  motion_pub.publish(msg);
}

void sendStateCommand() {
    buzzmobile::CarState msg;
//    msg.AUTO = lastAuto;
//    msg.MANUAL = lastManual;
    state_pub.publish(msg);
}

void sendBrakeCommand() {
  std_msgs::Bool msg;
  msg.data = manualToggle;
  brake_pub.publish(msg);
}

void handleState(const sensor_msgs::Joy::ConstPtr& joy) {
    //if (joy->
}

void handleDrive(const sensor_msgs::Joy::ConstPtr& joy) {
  //********************************************
  //Motor control
  float speed = 0.0;
  //if the stop button isnt held, we want to go
  //otherwise, just keep sending the stop command
  
  if (joy->reverse_button) {
    float correction = -1; // Normally ranges from 1 to -1. Correct so it ranges from 0 to -2
    // Division by 2 brings to range -1 to 0. It can then be multiplied by max speed
    speed = ((correction + joy->velocity_trigger) / 2.0) * maxFwdSpeed; 
  } else {
    float correction = 1; // Once inverted, will range from -1 to 1. Correct so it is from 0 to 2
    // Division by 2 brings to range 0 to 1. It can then be multiplied by max speed
    speed = ((correction + -1 * joy->velocity_trigger) / 2.0) * maxFwdSpeed;
  }
  
  ROS_INFO("Speed: %f", speed);
  if (lastSpeed != speed) {
    lastSpeed = speed;
    sendMotionCommand();
  }
}

// TODO this is not yet implemented for the new buzzmobile
void handleBrake(const sensor_msgs::Joy::ConstPtr& joy) {
  //bool pub = false;
  if (joy->brake_button) {
    // When we first press the button, toggle the flag and publish
    if (!brakePushed) {
      brakePushed = true;
      manualToggle = true;
      sendBrakeCommand();
    }
  } else {
    // When we first release the button, toggle the flag and publish
    if (brakePushed) {
      brakePushed = false;
      manualToggle = false;
      sendBrakeCommand();
    }
  }
}

void handleTurn(const sensor_msgs::Joy::ConstPtr& joy) {
  float angle = 0;
  // first test is to see if we're close to 0
  float mag = sqrt(joy->axes[0] * joy->axes[0] + joy->axes[1] * joy->axes[1]);
  //joystick is around the center...send 0 speed
  if (mag > 1e-6) {
    angle = -(atan2(fabs(joy->axes[1]), -joy->axes[0]) - M_PI_2);
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
void honkHorn() {
    //sound_play::SoundRequest msg;
    //msg.sound = sound_play::SoundRequest::PLAY_FILE;
    //msg.command = sound_play::SoundRequest::PLAY_ONCE;
    //msg.arg = "/home/agency/Downloads/vehicle042.wav";
    //horn_pub.publish(msg);
}

//TODO not sure what this is for. It isn't being used
void obstacleCallback(const std_msgs::Bool::ConstPtr& flag) {
	if (!obstacleFlag && flag->data) {
		honkHorn();
	} //TODO: for some reason this will honk repeatedly (repeated obstacles)...may need to debug this
	
	obstacleFlag = flag->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_joy");
  
  ros::NodeHandle n;
 
  motion_pub = n.advertise<buzzmobile::CarPose>("car_pose", 0);
  state_pub = n.advertise<buzzmobile::CarState>("car_state", 0);
//  brake_pub  = n.advertise<std_msgs::Bool>("brake", 100, true);


  // Initialize the latched topic
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);
    
  //TODO implement state
  //ros::Subscriber sub2 = n.subscribe<core_msgs::State>("state", 1000, stateCallback);
  
  //ros::Rate r(pubFreq); // Not sure what this is

  ros::Timer keepAliveTimer = n.createTimer(ros::Duration(1.0/pubFreq), keepAliveCallback);
  ros::spin();

  return 0;
}
