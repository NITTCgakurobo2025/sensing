#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <localization_msgs/msg/point_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

class LidarFilter : public rclcpp::Node {
public:
    LidarFilter() : Node("lidar_filter") {
        // runtime parameters
        this->declare_parameter<std::string>("lidar_topic", "scan");
        this->declare_parameter<std::string>("imu_topic", "imu");
        this->declare_parameter<std::string>("output_topic", "filtered_scan");
        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->declare_parameter<int>("buffer_size", 20);

        std::string lidar_topic, imu_topic, output_topic;
        this->get_parameter("lidar_topic", lidar_topic);
        this->get_parameter("imu_topic", imu_topic);
        this->get_parameter("output_topic", output_topic);
        this->get_parameter("buffer_size", buffer_size_);
        this->get_parameter("base_frame", base_frame_);

        // dynamic parameters
        this->declare_parameter<bool>("use_imu_orientation", false);
        this->declare_parameter<bool>("use_odom_translation", false);
        this->declare_parameter<double>("robot_radius", 0.45);

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&LidarFilter::filterLaserScan, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&LidarFilter::imuCallback, this, std::placeholders::_1));
        filtered_scan_pub_ = this->create_publisher<localization_msgs::msg::PointArray>(output_topic, 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        imu_buffer_.resize(buffer_size_);
        std::fill(imu_buffer_.begin(), imu_buffer_.end(), 0.0);
        buffer_index_ = 0;
        buffer_sum_ = 0;
        last_scan_time_ = this->now();
    }

private:
    rclcpp::Publisher<localization_msgs::msg::PointArray>::SharedPtr filtered_scan_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string base_frame_;
    double robot_radius_;
    int buffer_size_;
    int buffer_index_;
    double buffer_sum_;
    std::vector<double> imu_buffer_;
    rclcpp::Time last_scan_time_;
    geometry_msgs::msg::TransformStamped last_transform_;
    bool use_imu_orientation_;
    bool use_odom_translation_;

    double tmp_pre_z0;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        buffer_sum_ -= imu_buffer_[buffer_index_];
        buffer_sum_ += msg->angular_velocity.z;
        imu_buffer_[buffer_index_] = msg->angular_velocity.z;
        buffer_index_ = (buffer_index_ + 1) % buffer_size_;
    }

    void filterLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->get_parameter("use_imu_orientation", use_imu_orientation_);
        this->get_parameter("use_odom_translation", use_odom_translation_);
        this->get_parameter("robot_radius", robot_radius_);

        rclcpp::Time current_time = this->now();
        rclcpp::Duration dt = current_time - last_scan_time_;

        if (msg->ranges.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No Lidar data received");
            return;
        }

        geometry_msgs::msg::Point robot_center;
        try {
            geometry_msgs::msg::TransformStamped center_transform;
            center_transform = tf_buffer_->lookupTransform("map", base_frame_, rclcpp::Time(0));
            robot_center.x = center_transform.transform.translation.x;
            robot_center.y = center_transform.transform.translation.y;
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", e.what());
            return;
        }

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", msg->header.frame_id, rclcpp::Time(0));
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", e.what());
            return;
        }

        geometry_msgs::msg::TransformStamped pre_t = t;
        if (use_odom_translation_) {
            try {
                pre_t = tf_buffer_->lookupTransform("map", msg->header.frame_id, rclcpp::Time(0) - dt);
            } catch (tf2::TransformException &e) {
                RCLCPP_WARN(this->get_logger(), "Transform error: %s \n Not using previous transform", e.what());
            }
        }

        double vz_average = 0.0;
        if (!imu_buffer_.empty() && use_imu_orientation_) {
            vz_average = buffer_sum_ / (double)buffer_size_;
        }

        double dx = (t.transform.translation.x - pre_t.transform.translation.x) / (double)msg->ranges.size();
        double dy = (t.transform.translation.y - pre_t.transform.translation.y) / (double)msg->ranges.size();
        double dz = (vz_average * dt.seconds()) / (double)msg->ranges.size();

        localization_msgs::msg::PointArray filtered_scan;
        filtered_scan.header.stamp = msg->header.stamp;
        filtered_scan.header.frame_id = msg->header.frame_id;
        filtered_scan.points.reserve(msg->ranges.size());

        double x0 = pre_t.transform.translation.x;
        double y0 = pre_t.transform.translation.y;
        double z0 = getYaw(t.transform.rotation) + msg->angle_min;

        tmp_pre_z0 = z0;
        geometry_msgs::msg::Point p;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double x = x0 + dx * i;
            double y = y0 + dy * i;
            double z = z0 + (dz + msg->angle_increment) * i;
            p.x = msg->ranges[i] * std::cos(z) + x;
            p.y = msg->ranges[i] * std::sin(z) + y;

            if (((p.x - robot_center.x) * (p.x - robot_center.x) + (p.y - robot_center.y) * (p.y - robot_center.y)) <
                robot_radius_ * robot_radius_)
                continue;

            filtered_scan.points.emplace_back(p);
        }
        filtered_scan_pub_->publish(filtered_scan);

        last_scan_time_ = current_time;
    }

    inline double getYaw(const geometry_msgs::msg::Quaternion &quat) {
        tf2::Quaternion tf2_quat;
        tf2::fromMsg(quat, tf2_quat);
        tf2::Matrix3x3 m(tf2_quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilter>());
    rclcpp::shutdown();
    return 0;
}