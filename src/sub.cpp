#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class CamSub : public rclcpp::Node
{
public:
    CamSub() : Node("mul_cam_to_img")
    {

        this->declare_parameter<std::string>("img_filepath", "/home/file/path/");
        this->get_parameter("img_filepath", img_filepath_);
        this->declare_parameter<std::vector<std::string>>("topic_list");
        this->get_parameter("topic_list", topic_param);
        topic_list_ = topic_param.as_string_array();

        subscription_vector.resize(topic_list_.size());
        for(size_t i = 0; i < topic_list_.size(); i++)
        {
            std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr msg)> fnc =
                std::bind(&CamSub::topic_callback, this, std::placeholders::_1, i);
            subscription_vector.at(i) = this->create_subscription<sensor_msgs::msg::Image>(
                topic_list_[i], rclcpp::QoS{1}, fnc);
        }
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, int i) 
    {
        // std::cout << msg->encoding << std::endl;
        auto img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img_cv = img->image;
        auto stamp = msg->header.stamp;
        std::string filename = img_filepath_ + std::to_string(i) + "/" + std::to_string(stamp.sec + stamp.nanosec/1000000000.0) + ".png";
        RCLCPP_INFO(this->get_logger(), "Writing png file");
        cv::imwrite(filename, img_cv);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscription_vector;

    std::string img_filepath_;
    rclcpp::Parameter topic_param;
    std::vector<std::string> topic_list_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamSub>());
  rclcpp::shutdown();
  return 0;
}