#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <fstream>
using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RawExporter : public rclcpp::Node
{
private:

    /* ------------------------------ variables------------------------------------------------------------------------- */

    bool init;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

    size_t count_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    double leaf_size;


    /* ------------------------------functions -------------------------------------------------------------------------*/

    void lidar_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {


        std::string filename;
        //filename.precision(16);
        double filename_d= (double(msg->header.stamp.sec)+double(msg->header.stamp.nanosec)*1e-9);

        filename=std::to_string(filename_d);

        std::cout <<"filename is "<<filename<<" "<<filename<<std::endl;

        std::string currentPath="/home/georges/export";  // TODO: make it a rosparam

        std::vector<int> indices;

        //std::cout<<"Saving "<<temp_cloud->points.size()<<" points"<<std::endl;

        std::string fullPath= currentPath + "/" + filename + ".raw" ;
        std::ofstream raw_file;
        raw_file.open(fullPath.c_str());
        raw_file.write((char*)(msg->data.data()), msg->data.size());
        raw_file.close();


       // rclcpp::sleep_for(std::chrono::nanoseconds(100ms));
    }

public:
    RawExporter()
        : Node("raw_exporter"), count_(0)
    {


        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "/camera/depth/image_rect_raw", 1, std::bind(&RawExporter::lidar_callback, this, _1));

        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RawExporter>());
    rclcpp::shutdown();
    return 0;
}
