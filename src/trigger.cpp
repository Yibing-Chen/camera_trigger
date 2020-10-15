#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

const std::string path = "/home/cyb/Pictures/camera_trigger_saved_imgs/";
const std::string jpg = ".jpg";
const std::string png = ".png";

cv::Mat raw, view, color, depth;
bool is_4cam;

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%y%m%d_%H%M%S", &tstruct);

    return buf;
}

void on_EVENT_LBUTTONDOWN(int event, int x, int y, int flags, void *userdata)
{
  int ret;
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    ret = cv::imwrite(path + currentDateTime() + "_raw" + jpg, raw);
    ret &= cv::imwrite(path + currentDateTime() + "_color" + jpg, color);
    ret &= cv::imwrite(path + currentDateTime() + "_depth" + png, depth);
    if (ret)
    {
      std::cout << "IMAGE SAVE SUCCESS!" << std::endl;
    }
    else
    {
      std::cout << "IMAGE SAVE FAILED!" << std::endl;
    }
    
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    raw = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::resize(raw, view, cv::Size(raw.cols/3, raw.rows/3)); // to half size or even smaller
    cv::imshow("camera", view);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("raw : Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    color = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::imshow("d435i", color);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("color: Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    depth = cv_bridge::toCvCopy(msg, "")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("depth: Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
  }
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trigger");
  ros::start();

  if(argc != 2)
  {
      std::cerr << std::endl << "Usage: rosrun camera_trigger trigger fisheye/4cam" << std::endl;        
      ros::shutdown();
      return 1;
  }

  if(std::string(argv[1]) == "4cam")
  {
    is_4cam = true;
  }
  else
  {
    is_4cam = false;
  }
  
  if (is_4cam)
  {
    ros::NodeHandle nh;
    cv::namedWindow("camera", CV_WINDOW_KEEPRATIO);
    cv::startWindowThread();
    cv::setMouseCallback("camera", on_EVENT_LBUTTONDOWN);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("rgb_camera/image_raw", 1, imageCallback);
    ros::spin();
  }
  else{
    ros::NodeHandle nh;
    cv::namedWindow("camera", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("d435i", CV_WINDOW_KEEPRATIO);
    cv::startWindowThread();
    cv::setMouseCallback("camera", on_EVENT_LBUTTONDOWN);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber fisheye_sub = it.subscribe("rgb_camera/image_raw", 1, imageCallback);
    image_transport::Subscriber color_sub = it.subscribe("rgbd_camera/color/image_raw", 1, rgbCallback);
    image_transport::Subscriber depth_sub = it.subscribe("rgbd_camera/aligned_depth_to_color/image_raw", 1, depthCallback);
    ros::spin();
  }

  return 0;
 }