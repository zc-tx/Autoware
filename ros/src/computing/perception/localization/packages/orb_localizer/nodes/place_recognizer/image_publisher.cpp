/*
 * image_publisher.cpp
 *
 *  Created on: May 21, 2017
 *      Author: sujiwo
 */

#include <iostream>
#include <string>
#include <ros/ros.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>




int main (int argc, char *argv[])
{
	ros::init(argc, argv, "image_testpub");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);

	cv::Mat cImage = cv::imread (argv[1], CV_LOAD_IMAGE_ANYCOLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cImage).toImageMsg();
	pub.publish (msg);

	return 0;
}
