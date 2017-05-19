/*
 * place_recognizer.cpp
 *
 *  Created on: May 18, 2017
 *      Author: sujiwo
 */

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "System.h"
#include "Map.h"

using namespace std;
namespace enc = sensor_msgs::image_encodings;

const string orbGenericVocabFile = ORB_SLAM_VOCABULARY;

ORB_SLAM2::System *SLAMSystem;


void imageCallback (const sensor_msgs::ImageConstPtr &imageMsg)
{
	cout << "Imaged\n";

	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(imageMsg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat image;
	// Check if we need debayering
	if (enc::isBayer(imageMsg->encoding)) {
		int code=-1;
		if (imageMsg->encoding == enc::BAYER_RGGB8 ||
				imageMsg->encoding == enc::BAYER_RGGB16) {
			code = cv::COLOR_BayerBG2BGR;
		}
		else if (imageMsg->encoding == enc::BAYER_BGGR8 ||
				imageMsg->encoding == enc::BAYER_BGGR16) {
			code = cv::COLOR_BayerRG2BGR;
		}
		else if (imageMsg->encoding == enc::BAYER_GBRG8 ||
				imageMsg->encoding == enc::BAYER_GBRG16) {
			code = cv::COLOR_BayerGR2BGR;
		}
		else if (imageMsg->encoding == enc::BAYER_GRBG8 ||
				imageMsg->encoding == enc::BAYER_GRBG16) {
			code = cv::COLOR_BayerGB2BGR;
		}
		cv::cvtColor(cv_ptr->image, image, code);
	}
	else
		image = cv_ptr->image;

	const double imageTime = imageMsg->header.stamp.toSec();

}


int main (int argc, char *argv[])
{
	ros::init(argc, argv, "orb_matching", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler ("~");

	string mapPath, configFile;
	nodeHandler.getParam ("map_file", mapPath);
	nodeHandler.getParam ("config_file", configFile);
	SLAMSystem = new ORB_SLAM2::System (
		orbGenericVocabFile,
		configFile,
		ORB_SLAM2::System::MONOCULAR,
		false,
		mapPath,
		ORB_SLAM2::System::LOCALIZATION);

	string imageTopic;
	nodeHandler.getParam ("camera_topic", imageTopic);
	image_transport::TransportHints th ("raw");
	image_transport::ImageTransport imageBuf (nodeHandler);
	image_transport::Subscriber imageSub = imageBuf.subscribe (imageTopic, 1, &imageCallback, th);

	ros::spin();

	ros::shutdown();
	return 0;
}
