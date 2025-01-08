#include <boost/dynamic_bitset.hpp>
#include <localization_msgs/msg/point_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class GridFilter : public rclcpp::Node {
public:
    GridFilter() : Node("grid_filter") {
        // runtime parameters
        this->declare_parameter<std::string>("input_topic", "filtered_scan");
        this->declare_parameter<std::string>("output_topic", "downsampled_scan");

        std::string input_topic, output_topic;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("output_topic", output_topic);

        // dynamic parameters
        this->declare_parameter<double>("max_field_width", 9.0);
        this->declare_parameter<double>("max_field_height", 16.0);
        this->declare_parameter<double>("grid_width", 0.5);
        this->declare_parameter<double>("grid_height", 0.5);

        input_sub_ = this->create_subscription<localization_msgs::msg::PointArray>(
            input_topic, 10, std::bind(&GridFilter::topicCallback, this, std::placeholders::_1));
        output_pub_ = this->create_publisher<localization_msgs::msg::PointArray>(output_topic, 10);
    }

private:
    rclcpp::Publisher<localization_msgs::msg::PointArray>::SharedPtr output_pub_;
    rclcpp::Subscription<localization_msgs::msg::PointArray>::SharedPtr input_sub_;

    void topicCallback(const localization_msgs::msg::PointArray::SharedPtr msg) {
        double max_field_height, max_field_width, grid_width, grid_height;
        this->get_parameter("max_field_height", max_field_height);
        this->get_parameter("max_field_width", max_field_width);
        this->get_parameter("grid_width", grid_width);
        this->get_parameter("grid_height", grid_height);

        localization_msgs::msg::PointArray downsampled_points;
        downsampled_points.points.reserve(msg->points.size());

        int grid_size_x = static_cast<int>(max_field_width / grid_width);
        int grid_size_y = static_cast<int>(max_field_height / grid_height);
        boost::dynamic_bitset<> grid(grid_size_x * grid_size_y);
        grid.reset();

        double height = max_field_height / 2.0;
        double width = max_field_width / 2.0;

        for (const auto &point : msg->points) {
            if (std::abs(point.x) > width || std::abs(point.y) > height)
                continue;
            int x_index = static_cast<int>((point.x + width) / grid_width);
            int y_index = static_cast<int>((point.y + height) / grid_height);
            if (!grid[x_index + y_index * grid_size_x]) {
                grid.set(x_index + y_index * grid_size_x);
                downsampled_points.points.push_back(point);
            }
        }

        downsampled_points.header = msg->header;
        output_pub_->publish(downsampled_points);

        RCLCPP_INFO(this->get_logger(), "Received %d points, downsampled to %d points", msg->points.size(),
                    downsampled_points.points.size());
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridFilter>());
    rclcpp::shutdown();
    return 0;
}