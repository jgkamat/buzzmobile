#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <buzzmobile/CarPose.h>

class PDController {
public:
    PDController(double p, double d) : P(p), D(d) { }

    void setDesired(double value) {
        desired = value;
    }

    double operator() (double current) {
        auto error = current - desired;
        auto dError = error - lastError;
        auto ret = P * error - D * dError;
        lastError = error;
        return ret;
    }

private:
    double P, D;
    double desired = 0.0;
    double lastError = 0.0;
};

PDController left_controller{1.0, 0.05};
PDController right_controller{1.0, 0.05};

double speed_set_point = 0.0;
double speed_measured_left = 0.0;
double speed_measured_right = 0.0;

double steer_set_point = 0.0;

// TODO(sahit): change to rosparams
constexpr double chassis_length = 1.8; 
constexpr double chassis_width = 1.57;
constexpr double inv_chassis_length = 1.0 / chassis_length;
constexpr double chassis_width_2 = chassis_width / 2.0;
//TODO(sahit): this can also be calculated/measured when we have the car
constexpr double max_torque = 2.5; 

constexpr double wheel_circumference = 2.0 * M_PI * 0.3302;

void carPoseCallback(const buzzmobile::CarPose::ConstPtr &msg) {
    speed_set_point = -msg->velocity;
    steer_set_point = -msg->angle;
}

// void steeringCallback(const rr_platform::steeringConstPtr &msg) {
    // steer_set_point = -msg->angle;
// }

void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg) {

    auto iter = std::find(msg->name.begin(), msg->name.end(), std::string{"axle_to_left_wheel"});

    if(iter != msg->name.end()) {
        auto index = std::distance(msg->name.begin(),iter);

        speed_measured_left = (-msg->velocity[index]) * ( wheel_circumference / ( 2 * M_PI ) );
    }

    iter = std::find(msg->name.begin(), msg->name.end(), std::string{"axle_to_right_wheel"});

    if(iter != msg->name.end()) {
        auto index = std::distance(msg->name.begin(),iter);

        speed_measured_right = (-msg->velocity[index]) * ( wheel_circumference / ( 2 * M_PI ) );
    }
}

constexpr double get_steer_ang(double phi) {
    return (phi >= 0.0) ? (M_PI_2 - phi) : (-M_PI_2 - phi);
}

void fillSteeringPositions(const double set_angle, double &left, double &right) {
    auto center_y = chassis_length * tan( (M_PI_2) - set_angle);
    left = get_steer_ang(atan(inv_chassis_length * (center_y - chassis_width_2)));
    right = get_steer_ang(atan(inv_chassis_length * (center_y + chassis_width_2)));
}

void fillWheelSpeeds(const double steering_angle, const double speed, double &left, double &right) {
    if (steering_angle == 0.0) {
        left = speed;
        right = speed;
    } else {
        auto turning_radius = chassis_length / fabs(tan(steering_angle));
        auto radius_left = turning_radius - copysign(chassis_width_2, steering_angle);
        auto radius_right = turning_radius + copysign(chassis_width_2, steering_angle);
        left = speed * radius_left / turning_radius;
        right = speed * radius_right / turning_radius;
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sim_car_interface");

    ros::NodeHandle handle;

    ros::Publisher leftDrivePublisher = handle.advertise<std_msgs::Float64>("/left_wheel_effort_controller/command", 1);
    ros::Publisher rightDrivePublisher = handle.advertise<std_msgs::Float64>("/right_wheel_effort_controller/command", 1);
    ros::Publisher leftSteeringPublisher = handle.advertise<std_msgs::Float64>("/left_steer_position_controller/command", 1);
    ros::Publisher rightSteeringPublisher = handle.advertise<std_msgs::Float64>("/right_steer_position_controller/command", 1);

    auto carPoseSub = handle.subscribe("/buzzmobile/car_pose", 1, carPoseCallback);
    auto stateSub = handle.subscribe("/joint_states", 1, jointStateCallback);

    ros::Rate rate{30};
    while (ros::ok()) {
        ros::spinOnce();

        double left_speed, right_speed;
        fillWheelSpeeds(steer_set_point, speed_set_point, left_speed, right_speed);

        left_controller.setDesired(left_speed);
        right_controller.setDesired(right_speed);

        std_msgs::Float64 leftDriveMsg;
        auto set_torque = left_controller(speed_measured_left);
        leftDriveMsg.data = std::max(-max_torque, std::min(set_torque, max_torque));
        leftDrivePublisher.publish(leftDriveMsg);

        std_msgs::Float64 rightDriveMsg;
        set_torque = right_controller(speed_measured_right);
        rightDriveMsg.data = std::max(-max_torque, std::min(set_torque, max_torque));
        rightDrivePublisher.publish(rightDriveMsg);

        std_msgs::Float64 leftSteerMsg;
        std_msgs::Float64 rightSteerMsg;

        fillSteeringPositions(steer_set_point, leftSteerMsg.data, rightSteerMsg.data);

        leftSteeringPublisher.publish(leftSteerMsg);
        rightSteeringPublisher.publish(rightSteerMsg);

        rate.sleep();
    }

    return 0;
}
