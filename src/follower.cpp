#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
class ImageConverter
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat curr_image;
    bool ready;
    
public:
    ImageConverter() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
                                   &ImageConverter::imageCb, this);
        ready = false;
        cv::namedWindow("Image window");
    }

    ~ImageConverter()
    {}
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        ready = true;
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
        cv_ptr->image.copyTo(curr_image);
        ready = true;
        cv::imshow("Image window", cv_ptr->image);
        cv::waitKey(3);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_reading_image");
    ImageConverter ic;
    ros::Rate rate(30);
    cv::namedWindow("Image window");
    while(ros::ok())
    {
        if (ic.ready == true)
        {
            cv::imshow("Image window", ic.curr_image);
        }
        ros::spinOnce();
        cv::waitKey(3);
        rate.sleep();
    }
    return 0;
}
