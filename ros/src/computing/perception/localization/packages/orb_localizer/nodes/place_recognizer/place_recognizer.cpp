/*
 * place_recognizer.cpp
 *
 *  Created on: May 18, 2017
 *      Author: sujiwo
 */

#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include "System.h"
#include "Map.h"
#include "Frame.h"
#include "../common.h"

using namespace std;
namespace enc = sensor_msgs::image_encodings;
using ORB_SLAM2::Frame;
using ORB_SLAM2::KeyFrameDatabase;
using ORB_SLAM2::KeyFrame;


const string orbGenericVocabFile = ORB_SLAM_VOCABULARY;

ORB_SLAM2::System *SLAMSystem;



struct RecognizerOutput {
	cv::Mat framebuf;
	geometry_msgs::PoseWithCovariance keyframePose;
	double imageTimestamp;
};



cv::Mat RenderOutput (Frame &frame, KeyFrame *kf)
{
	cv::Mat imgOut;

	return imgOut;
}


bool relocalize (Frame &frame, RecognizerOutput &output)
{
	KeyFrameDatabase *kfDB = SLAMSystem->getKeyFrameDB();
	frame.ComputeBoW();

	vector<KeyFrame*> vpCandidateKFs
		= kfDB->DetectRelocalizationCandidatesSimple(&frame);

	if (vpCandidateKFs.empty())
		return false;

	// XXX: Find a way to select candidate
	KeyFrame *firstSel = vpCandidateKFs[0];

	tf::Transform keyPose = KeyFramePoseToTf(firstSel);
	output.imageTimestamp = frame.mTimeStamp;
	output.keyframePose.pose.position.x = keyPose.getOrigin().x();
	output.keyframePose.pose.position.y = keyPose.getOrigin().y();
	output.keyframePose.pose.position.z = keyPose.getOrigin().z();
	output.keyframePose.pose.orientation.x = keyPose.getRotation().x();
	output.keyframePose.pose.orientation.y = keyPose.getRotation().y();
	output.keyframePose.pose.orientation.z = keyPose.getRotation().z();
	output.keyframePose.pose.orientation.w = keyPose.getRotation().w();

	return true;
}


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

	cv::Mat imageGray;
	cv::cvtColor (image, imageGray, CV_BGR2GRAY);

	const double imageTime = imageMsg->header.stamp.toSec();

	Frame cframe = SLAMSystem->getTracker()->createMonocularFrame(imageGray, imageTime);

	RecognizerOutput frameRecognizerOutput;
	bool kfFound = relocalize(cframe, frameRecognizerOutput);
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
