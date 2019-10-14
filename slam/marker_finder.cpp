/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "marker_finder.h"
#include <math.h>

using namespace std;
using namespace cv;
using namespace aruco;

void MarkerFinder::setMarkerPosesLocal()
{// This function finds ther marker pose local(camera ref frame e.g.)
	marker_poses_local_.clear();
	for(size_t i = 0; i < markers_.size(); i++)
	{
		Mat R = Mat::eye(3, 3, CV_32FC1);// Rotation Matrix
		Eigen::Affine3f P = Eigen::Affine3f::Identity(); // Aruco Pose
		
		Rodrigues(markers_[i].Rvec, R);
	
		P(0,0) = R.at<float>(0,0); P(0,1) = R.at<float>(0,1); P(0,2) = R.at<float>(0,2);
		P(1,0) = R.at<float>(1,0); P(1,1) = R.at<float>(1,1); P(1,2) = R.at<float>(1,2);
		P(2,0) = R.at<float>(2,0); P(2,1) = R.at<float>(2,1); P(2,2) = R.at<float>(2,2);
		P(0,3) = markers_[i].Tvec.at<float>(0,0); P(1,3) = markers_[i].Tvec.at<float>(1,0); P(2,3) = markers_[i].Tvec.at<float>(2,0);
		
		marker_poses_local_.push_back(P); // Pushing back the marker pose in the marker_poses_local vector
	}
}

void MarkerFinder::setMarkerPosesGlobal(Eigen::Affine3f cam_pose)
{// This funcion finds the marker pose global(map ref frame)
	marker_poses_.clear();
	for(size_t i = 0; i < markers_.size(); i++)
	{
		Mat R = Mat::eye(3, 3, CV_32FC1); // Rotation Matrix
		Eigen::Affine3f P = Eigen::Affine3f::Identity(); // Aruco Pose
		
		Rodrigues(markers_[i].Rvec, R);
	
		P(0,0) = R.at<float>(0,0); P(0,1) = R.at<float>(0,1); P(0,2) = R.at<float>(0,2);
		P(1,0) = R.at<float>(1,0); P(1,1) = R.at<float>(1,1); P(1,2) = R.at<float>(1,2);
		P(2,0) = R.at<float>(2,0); P(2,1) = R.at<float>(2,1); P(2,2) = R.at<float>(2,2);
		P(0,3) = markers_[i].Tvec.at<float>(0,0); P(1,3) = markers_[i].Tvec.at<float>(1,0); P(2,3) = markers_[i].Tvec.at<float>(2,0) - 0.5;
		// I'm adding 0.5 in z axis in order to make the robot goes to a point exatly at the front of the aruco marker
		marker_poses_.push_back(cam_pose.inverse() * P);
	}
}

void MarkerFinder::setMarkerPointPosesGlobal(Eigen::Affine3f cam_pose, int aruco_distance)
{/* This function save the marker pose where the robot need to go.
 It's the sabe aruco pose but with a value added in order to the robot always find a place inside of the map
 In Some situations the aruco marker can be detected outside of the map, since it is oftenly
 placed in a wall(Precision erros can place the aruco marker outside of the map)
 */  
	marker_point_poses_.clear();
	for(size_t i = 0; i < markers_.size(); i++)
	{
		Mat R = Mat::eye(3, 3, CV_32FC1); // Orientation 
		Eigen::Affine3f P = Eigen::Affine3f::Identity();// Marker pose
		Eigen::Vector4f F = Eigen::Vector4f(); // Distance between aruco and the 3D point we want
		Eigen::Vector4f V = Eigen::Vector4f(); // 3D point pose 
		double x=0,y=0,z=0;
		F(0,0) = 0.0;
		F(1,0) = 0.0;
		F(2,0) = 0.5;
		F(3,0) = 1.0;

		Rodrigues(markers_[i].Rvec, R);
	
		P(0,0) = R.at<float>(0,0); P(0,1) = R.at<float>(0,1); P(0,2) = R.at<float>(0,2);
		P(1,0) = R.at<float>(1,0); P(1,1) = R.at<float>(1,1); P(1,2) = R.at<float>(1,2);
		P(2,0) = R.at<float>(2,0); P(2,1) = R.at<float>(2,1); P(2,2) = R.at<float>(2,2);
		P(0,3) = markers_[i].Tvec.at<float>(0,0); P(1,3) = markers_[i].Tvec.at<float>(1,0); P(2,3) = markers_[i].Tvec.at<float>(2,0);

		x = pow(P(0,3),2);
		y = pow(P(1,3),2);
		z = pow(P(2,3),2);
		
		///getting the absolute distance between camera and marker
		///if their distance is closer then 4meters save marker pose
		if(sqrt(x + y + z) < aruco_distance){
			V = P * F; //Find the point in the Aruco ref frame
			marker_point_poses_.push_back(cam_pose.inverse() * V);  //Find the pose point 3d Global ref frame
		}
		if(sqrt(x +y +z) >= aruco_distance){
			markers_.clear();
		}
		
		
	}
}
/* Arucos dictionary
ARUCO, Original
ARUCO_MIP_25h7,
ARUCO_MIP_16h3,
ARUCO_MIP_36h12, Recommended
ARTAG,
ARTOOLKITPLUS,
ARTOOLKITPLUSBCH,
TAG16h5,TAG25h7,TAG25h9,TAG36h11,TAG36h10 
*/
MarkerFinder::MarkerFinder()
{//set dictionary
	marker_detector_.setDictionary("ARUCO_MIP_36h12", 0);
}

void MarkerFinder::markerParam(string params, float size, string aruco_dic)
{//Load params 
	try{
		marker_detector_.setDictionary(aruco_dic, 0);
  	}
  	catch(char param[]){
    	cout << "An exception occurred. Exception Nr. "  << param<<'\n';
  	}
	camera_params_.readFromXMLFile(params);
	marker_size_ = size;
}

void MarkerFinder::detectMarkers(const cv::Mat img, Eigen::Affine3f cam_pose, int aruco_distance)
{//Detect marker and calls setMarkerPointPosesGlobal
	markers_.clear();
	marker_detector_.detect(img, markers_, camera_params_, marker_size_);
	
	setMarkerPointPosesGlobal(cam_pose,aruco_distance);
}