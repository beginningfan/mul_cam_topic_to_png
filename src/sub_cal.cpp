#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco/charuco.hpp"

using namespace cv;
using namespace std;

void cal(Mat& img_in, Mat& img_out)
{
    img_out = img_in.clone();
    // 标定板属性信息
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(8, 5, 0.150, 0.117, dictionary);

    vector<int> markerIds;
    vector<vector<Point2f>>  markerCorners;
    aruco::detectMarkers(img_in, board->dictionary, markerCorners, markerIds);
    if(markerIds.size() > 0)
    {
        vector<int> charucoIds;
        vector<Point2f> charucoCorners;
        aruco::interpolateCornersCharuco(markerCorners, markerIds, img_in, board, charucoCorners, charucoIds);
        if(charucoIds.size() > 0)
        {
            for(auto id : charucoIds)
            {
                circle(img_out, charucoCorners[id], 5, Scalar(0,0,255), -1);
            }
        }
    }
}


using placeholders::_1;
using placeholders::_2;

class CamSub : public rclcpp::Node
{
public:
    CamSub() : Node("mul_cam_to_img")
    {

        this->declare_parameter<string>("img_filepath", "/home/file/path/");
        this->get_parameter("img_filepath", img_filepath_);
        this->declare_parameter<vector<string>>("topic_list");
        this->get_parameter("topic_list", topic_param);
        topic_list_ = topic_param.as_string_array();

        subscription_vector.resize(topic_list_.size());
        publisher_vector.resize(topic_list_.size());
        for(size_t i = 0; i < topic_list_.size(); i++)
        {
            string cal_topic_name = topic_list_[i] + "/cal";
            publisher_vector.at(i) = this->create_publisher<sensor_msgs::msg::Image>(cal_topic_name, 10);
        }
        for(size_t i = 0; i < topic_list_.size(); i++)
        {
            function<void(const sensor_msgs::msg::Image::ConstSharedPtr msg)> fnc =
                bind(&CamSub::topic_callback, this, placeholders::_1, i);
            subscription_vector.at(i) = this->create_subscription<sensor_msgs::msg::Image>(
                topic_list_[i], rclcpp::QoS{1}, fnc);
        }

    }

private:
    void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, int i)
    {
        // cout << msg->encoding << endl;
        auto img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat img_cv = img->image;
        Mat img_out;
        cal(img_cv, img_out);


        auto header = std_msgs::msg::Header();
        header.stamp = msg->header.stamp;
        header.frame_id = msg->header.frame_id;
        auto imgPtr = std::make_shared<cv_bridge::CvImage>(header, "bgr8", img_out);
        auto pub_msg = *(imgPtr->toImageMsg());

        publisher_vector.at(i)->publish(pub_msg);

    }
    vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscription_vector;
    vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publisher_vector;

    string img_filepath_;
    rclcpp::Parameter topic_param;
    vector<string> topic_list_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamSub>());
  rclcpp::shutdown();
  return 0;
}