#ifndef READING_IMAGE
#define READING_IMAGE

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter
{
public:
    ImageConverter();
    ~ImageConverter();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void imageCbDepth(const sensor_msgs::ImageConstPtr& msg);
    
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_sub_depth_;

    cv::Mat curr_image;
    cv::Mat curr_image_depth;
    bool ready;
};
#endif
