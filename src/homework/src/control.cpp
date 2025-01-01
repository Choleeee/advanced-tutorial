#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ros_gz_interfaces/msg/param_vec.hpp"
#include <math.h>

class ThrustPublisher : public rclcpp::Node
{
public:
    ThrustPublisher() : Node("ThrustPublisher")
    {
        // create subscription
        sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/sensors/gps/gps/fix", 10,
                                                                         std::bind(&ThrustPublisher::gps_callback, this, std::placeholders::_1));
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/wamv/sensors/imu/imu/data",10,
                                                                         std::bind(&ThrustPublisher::imu_callback,this,std::placeholders::_1));
        sub_tube = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vrx/stationkeeping/goal", 10,
                                                                         std::bind(&ThrustPublisher::tube_callback, this, std::placeholders::_1));
        // create publisher (according to the topic names of the thrusters)
        left_thrust = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
        right_thrust = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
        left_pos = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/pos", 10);
        right_pos = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/pos", 10);



        timer_ = this->create_wall_timer(std::chrono::milliseconds(1),             // set timer frequency
                                         std::bind(&ThrustPublisher::func, this)); // call function
        RCLCPP_INFO(this->get_logger(), "node is activated");
    }

private:
    const double p = 0.0174532925199;
    double quaternion_vector[4] = {0, 0, 0, 0};
    double lat_wamv = 0;
    double lon_wamv = 0;
    double lat_tube = 0;
    double lon_tube = 0;
    double dist = 0;
    double angle = 0;
    double relative_angle = 0;
    double self_deg;
    int stage = 0;

#define pi 3.14159265359
    void func()
    {
        // how to publish the data into the thrusters?
        int left_thruster_thrust_value;
        auto leftmsg = std_msgs::msg::Float64();
        leftmsg.data = std::float_t(left_thruster_thrust_value);
        left_thrust->publish(left_thruster_thruster_value);

    }


    //Some user functions to calculate the angle and distance
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        lat_wamv = p * msg->latitude;
        lon_wamv = p * msg->longitude;
    }
    void tube_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        lat_tube = p * msg->pose.position.x;
        lon_tube = p * msg->pose.position.y;
        // RCLCPP_INFO(this->get_logger(), "Stage: %d", circle);
        RCLCPP_INFO(this->get_logger(), "relative_angle: %f, distance: %f, circle: %d, thing: %d", relative_angle, dist, circle, thing);
    }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        quaternion_vector[0] = msg->orientation.w;
        quaternion_vector[1] = msg->orientation.x;
        quaternion_vector[2] = msg->orientation.y;
        quaternion_vector[3] = msg->orientation.z;
        self_deg = quaternion_to_z_axis_angle(quaternion_vector);

        dist = distance(lat_wamv,lon_wamv,lat_tube,lon_tube);
        angle = latlong_to_z_axis_angle(lat_wamv,lon_wamv,lat_tube,lon_tube);
        relative_angle = object_to_wamv_frame_of_reference(self_deg,angle);
    }
    // calculation code
    double object_to_wamv_frame_of_reference(double self_deg, double target_deg)
    { // find the target object angle relative to the wamv angle, not relative to world frame
        if (target_deg < 0)
        {
            target_deg += 2 * pi;
        }
        if (self_deg > 0)
        {
            target_deg -= self_deg;
            if (target_deg > pi)
            {
                target_deg -= 2 * pi;
            }
        }
        else if (self_deg < 0)
        {
            target_deg += abs(self_deg);
            if (target_deg > pi)
            {
                target_deg -= 2 * pi;
            }
        }

        return target_deg;
    }
    double distance(double lat1, double lon1, double lat2, double lon2)
    {
        int r = 6371 * 1000; // km

        double a = 0.5 - cos((lat2 - lat1)) / 2 + cos(lat1) * cos(lat2) * (1 - cos((lon2 - lon1))) / 2;

        double d = 2 * r * asin(sqrt(a));

        return d;
    }
    double quaternion_to_z_axis_angle(double q[4])
    {
        double X = (2 * (q[0] * q[3] + q[1] * q[2]));
        double Y = (1 - 2 * (q[2] * q[2] + q[3] * q[3]));
        return atan2(-Y, X);
    }

    double latlong_to_z_axis_angle(double lat1, double lon1, double lat2, double lon2)
    {
        double diff = lon2 - lon1;
        double X = cos(lat2) * sin(diff);
        double Y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(diff);
        return -atan2(X, Y);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pos;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pos;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tube;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrustPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}