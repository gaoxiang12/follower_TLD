#include "reading_image.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace std;

ImageConverter::ImageConverter() : it_(nh_)
{
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
                               &ImageConverter::imageCb, this);
    image_sub_depth_ = it_.subscribe("/camera/depth_registered/image_raw",1,
                                    &ImageConverter::imageCbDepth, this);
    ready = false;
}

ImageConverter::~ImageConverter()
{
}
    
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception & e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    curr_image = cv_ptr->image.clone();
    //    cv_ptr->image.copyTo(curr_image);
    ready = true;
};

void ImageConverter::imageCbDepth(const sensor_msgs::ImageConstPtr& msg)
{
    //    cout<<msg->encoding<<endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "");
    }
    catch(cv_bridge::Exception & e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.convertTo(curr_image_depth, CV_32S);
    //    cv_ptr->image.copyTo(curr_image);
};
