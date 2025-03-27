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
        this->declare_parameter<double>("field_width", 8.0);
        this->declare_parameter<double>("field_height", 15.0);
        this->declare_parameter<double>("max_diff", 0.5);
        this->declare_parameter<double>("grid_width", 0.5);
        this->declare_parameter<double>("grid_height", 0.5);
        this->declare_parameter<double>("ignore_radius", 1.5);

        input_sub_ = this->create_subscription<localization_msgs::msg::PointArray>(
            input_topic, 10, std::bind(&GridFilter::topicCallback, this, std::placeholders::_1));
        output_pub_ = this->create_publisher<localization_msgs::msg::PointArray>(output_topic, 10);
    }

private:
    rclcpp::Publisher<localization_msgs::msg::PointArray>::SharedPtr output_pub_;
    rclcpp::Subscription<localization_msgs::msg::PointArray>::SharedPtr input_sub_;

    void topicCallback(const localization_msgs::msg::PointArray::SharedPtr msg) {
        double field_height, field_width, max_diff, grid_width, grid_height, ignore_radius;
        this->get_parameter("field_height", field_height);
        this->get_parameter("field_width", field_width);
        this->get_parameter("max_diff", max_diff);
        this->get_parameter("grid_width", grid_width);
        this->get_parameter("grid_height", grid_height);
        this->get_parameter("ignore_radius", ignore_radius);

        localization_msgs::msg::PointArray downsampled_points;
        downsampled_points.points.reserve(msg->points.size());

        const int grid_size_x = static_cast<int>((field_width + 2 * max_diff) / grid_width + 1);
        const int grid_size_y = static_cast<int>((field_height + 2 * max_diff) / grid_height + 1);
        boost::dynamic_bitset<> grid(grid_size_x * grid_size_y);
        grid.reset();

        double height = field_height / 2.0;
        double width = field_width / 2.0;

        for (const auto &point : msg->points) {
            if (std::abs(point.x - width) > max_diff && std::abs(point.x + width) > max_diff &&
                std::abs(point.y - height) > max_diff && std::abs(point.y + height) > max_diff)
                continue;
            if ((point.x - msg->pose.x) * (point.x - msg->pose.x) + (point.y - msg->pose.y) * (point.y - msg->pose.y) <
                ignore_radius * ignore_radius)
                continue;
            int x_index = static_cast<int>((point.x + width) / grid_width);
            int y_index = static_cast<int>((point.y + height) / grid_height);

            if (x_index < 0 || x_index >= grid_size_x || y_index < 0 || y_index >= grid_size_y) {
                RCLCPP_WARN(this->get_logger(), "Outlier detected: x=%f, y=%f (index: %d, %d)", point.x, point.y,
                            x_index, y_index);
                continue;
            }

            if (!grid[x_index + y_index * grid_size_x]) {
                grid.set(x_index + y_index * grid_size_x);
                downsampled_points.points.push_back(point);
            }
        }

        downsampled_points.header = msg->header;
        output_pub_->publish(downsampled_points);

        RCLCPP_INFO(this->get_logger(), "Received %ld points, downsampled to %ld points", msg->points.size(),
                    downsampled_points.points.size());
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridFilter>());
    rclcpp::shutdown();
    return 0;
}