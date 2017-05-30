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
#include "ORBmatcher.h"
#include "MapPoint.h"
#include "PnPsolver.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "../common.h"


using namespace std;
using namespace ORB_SLAM2;
namespace enc = sensor_msgs::image_encodings;



struct RecognizerOutput {
	cv::Mat framebuf;
	geometry_msgs::PoseWithCovariance keyframePose;
	double imageTimestamp;
};


const string orbGenericVocabFile = ORB_SLAM_VOCABULARY;
//ORB_SLAM2::System *SLAMSystem;
ros::Publisher recognizerPub;

cv::FileStorage sysConfig;
KeyFrameDatabase *keyframeDB;
ORBVocabulary *keyVocab;
ORBextractor *orbExtractor;
ORB_SLAM2::Map *sourceMap;

// Camera Parameters
cv::Mat CameraParam;

// Distortion Coefficients
cv::Mat DistCoef;


cv::Mat RenderOutput (Frame &frame, KeyFrame *kf)
{
	cv::Mat imgOut;

	return imgOut;
}


// XXX: No exception handling here
void SlamSystemPrepare (
	const string &mapFilename,
	const string &configPath,
	cv::FileStorage &sconf,
	Map **sMap,
	ORBVocabulary **vocab,
	KeyFrameDatabase **kfdb,
	ORBextractor **orbext
	)
{
	// open Configuration file
	sconf = cv::FileStorage (configPath.c_str(), cv::FileStorage::READ);

	// Vocabulary
	*vocab = new ORBVocabulary ();
	cout << endl << "Loading Custom ORB Vocabulary... " ;
	string mapVoc = mapFilename + ".voc";
	bool vocload = (*vocab)->loadFromTextFile (mapVoc);
	cout << "Vocabulary loaded!" << endl << endl;

	// Create KeyFrame Database
	*kfdb = new KeyFrameDatabase(**vocab);

	// Map
	*sMap = new Map ();
	(*sMap)->loadFromDisk(mapFilename, *kfdb, true);

	// ORB Extractor
	*orbext = new ORBextractor(
		2 * (int)sconf["ORBextractor.nFeatures"],
		(float)sconf["ORBextractor.scaleFactor"],
		(int)sconf["ORBextractor.nLevels"],
		(int)sconf["ORBextractor.iniThFAST"],
		(int)sconf["ORBextractor.minThFAST"]);

	// Camera Parameters
    float fx = sconf["Camera.fx"];
    float fy = sconf["Camera.fy"];
    float cx = sconf["Camera.cx"];
    float cy = sconf["Camera.cy"];
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(CameraParam);

    // Distortion Coefficients
    DistCoef = cv::Mat (4,1,CV_32F);
    DistCoef.at<float>(0) = sconf["Camera.k1"];
    DistCoef.at<float>(1) = sconf["Camera.k2"];
    DistCoef.at<float>(2) = sconf["Camera.p1"];
    DistCoef.at<float>(3) = sconf["Camera.p2"];
    const float k3 = sconf["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

}


ORB_SLAM2::Frame
monocularFrame (const cv::Mat& inputGray, const double timestamp)
{
	float mbf = 0.0, thDepth = 0.0;

	return ORB_SLAM2::Frame (inputGray, timestamp, orbExtractor, keyVocab, CameraParam, DistCoef, mbf, thDepth);
}


bool relocalize (Frame &frame, RecognizerOutput &output)
{
//	KeyFrameDatabase *kfDB = SLAMSystem->getKeyFrameDB();
	frame.ComputeBoW();

	vector<KeyFrame*> vpCandidateKFs
		= keyframeDB->DetectRelocalizationCandidatesSimple(&frame);

	if (vpCandidateKFs.empty())
		return false;

	vector<bool> vcDiscarded;
	const int nKFs = vpCandidateKFs.size();
	vcDiscarded.resize(nKFs);
	ORBmatcher matcher (0.75, true);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
        	vcDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW (pKF, frame, vvpMapPointMatches[i]);
            if(nmatches<15)
            {
            	cerr << "KF discarded: #" << pKF->mnId << endl;
            	vcDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver (frame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

//	tf::Transform keyPose = KeyFramePoseToTf(firstSel);
//	output.imageTimestamp = frame.mTimeStamp;
//	output.keyframePose.pose.position.x = keyPose.getOrigin().x();
//	output.keyframePose.pose.position.y = keyPose.getOrigin().y();
//	output.keyframePose.pose.position.z = keyPose.getOrigin().z();
//	output.keyframePose.pose.orientation.x = keyPose.getRotation().x();
//	output.keyframePose.pose.orientation.y = keyPose.getRotation().y();
//	output.keyframePose.pose.orientation.z = keyPose.getRotation().z();
//	output.keyframePose.pose.orientation.w = keyPose.getRotation().w();

	return true;
}


void imageCallback (const sensor_msgs::ImageConstPtr &imageMsg)
{
	cout << "Imaged\n";

	cv::Mat imageGray = createImageFromRosMessage(imageMsg,
		sysConfig["Camera.WorkingResolution.Width"],
		sysConfig["Camera.WorkingResolution.Height"],
		sysConfig["Camera.ROI.x0"],
		sysConfig["Camera.ROI.y0"],
		sysConfig["Camera.ROI.width"],
		sysConfig["Camera.ROI.height"],
		true
	);
	const double imageTime = imageMsg->header.stamp.toSec();

	Frame cframe = monocularFrame (imageGray, imageTime);

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

	SlamSystemPrepare(mapPath, configFile, sysConfig, &sourceMap, &keyVocab, &keyframeDB, &orbExtractor);

	string imageTopic;
	nodeHandler.getParam ("camera_topic", imageTopic);
	image_transport::TransportHints th ("raw");
	image_transport::ImageTransport imageBuf (nodeHandler);
	image_transport::Subscriber imageSub = imageBuf.subscribe (imageTopic, 1, &imageCallback, th);

	ros::spin();

	ros::shutdown();
	return 0;
}
