#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
//  image_transport::Subscriber image_sub_;
  ros::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = nh_.subscribe("/camera/rgb/image_mono", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    std::cout << "Initialized..." << std::endl;
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cv_frame;
    try
    {
      std::cout << "Converting format..." << std::endl;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);//BAYER_GRBG8);//BAYER_BGGR8);//MONO8);
      cv_frame = cv_bridge::toCvCopy(msg, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img_mat;
//    cv_ptr->image.copyTo(img_mat);
    std::cout << "Image size " << cv_frame.size() << ", type " << cv_frame.type() << std::endl;
//    cv_frame.copyTo(img_mat);
//    std::cout << "Converting color..." << std::endl;

//    cv::Mat color_img(cv_ptr->image.size(), CV_8UC3);
    if (cv_ptr->image.type() == CV_8U)
      std::cout << "type is CV_8U" << std::endl;

//    cv_ptr->image.convertTo(img_mat, CV_32F);
//    cvtColor(cv_frame, color_img, CV_GRAY2RGB);

    // Draw an example circle on the video stream
//    if (color_img.rows > 60 && color_img.cols > 60)
//      cv::circle(color_img, cv::Point(100, 100), 5, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_frame);
//    cv::waitKey(3);
    
    // Output modified video stream
    std::cout << "Publishing new image..." << std::endl;
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
