///ROS
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
///Opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
////our include
#include <geometry.h>
#include <rgbd_loader.h>
#include <klt_tracker.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>

using namespace std;


    KLTTracker tracker;
    Intrinsics intr(0);
    MotionEstimatorRANSAC motion_estimator(intr);
    //ReconstructionVisualizer visualizer;
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
    int i=0;

void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
void rosMotionEstimator(cv::Mat rgb , cv::Mat depth);


 int main(int argc, char** argv){
    string rgb_topic,depth_topic;
    rgb_topic = "camera/rgb/image_raw";
    depth_topic = "camera/depth/image_raw";
    if(argc != 1 && argc!= 3){
        fprintf(stderr, "Usage: %s optional: <rgb_topic> <depth_topic>....bye default : camera/rgb/image_raw and camera/depth/image_raw\n", argv[0]);
        exit(0);
    }
    if(argc == 1){
        printf(" By defult using camera/rgb/image_raw and camera/depth/image_raw as ros topics\n");  
    }
    if(argc == 3){
        rgb_topic = argv[1];
        depth_topic = argv[2]; 
    }

    //ROS
    ros::init(argc, argv, "image_converter");
    ros::start();
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    //ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
 }

 void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD){

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try{
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try{
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    rosMotionEstimator(cv_ptrRGB->image, cv_ptrD->image);
}
void rosMotionEstimator(cv::Mat rgb , cv::Mat depth){

    *curr_cloud = getPointCloud(rgb, depth, intr);

    //track feature points in current frame
    tracker.track(rgb);
    cout<<i<<endl;

    //Estimate motion between the current and the previous frame/point clouds
      if(i > 0){
       	trans = motion_estimator.estimate(tracker.prev_pts_, prev_cloud, tracker.curr_pts_, curr_cloud);
        pose = pose*trans;
      }

    //View tracked points

    for(size_t k=0; k < tracker.curr_pts_.size(); k++){
        cv::Point2i pt1 = tracker.prev_pts_[k];
        cv::Point2i pt2 = tracker.curr_pts_[k];
        cv::circle(rgb, pt1, 1, CV_RGB(255,0,0), -1);
        cv::circle(rgb, pt2, 3, CV_RGB(0,0,255), -1);
        cv::line(rgb, pt1, pt2, CV_RGB(0,0,255));
    }
    
    //3D vizualization 
    /*
    if(i == 0) visualizer.addReferenceFrame(pose, "origin");
    visualizer.addQuantizedPointCloud(curr_cloud, 0.3, pose);
    visualizer.viewReferenceFrame(pose);
    visualizer.viewPointCloud(curr_cloud, pose);
    visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, pose);
    //Spin Once = waitkey for 3D window
    visualizer.spinOnce();
    */
    
    //Dividing by the scale factor in order to show depth image
    depth= depth/5;
    cv::imshow("OPENCV_WINDOW_DEPTH", rgb);
    cv::imshow("OPENCV_WINDOW", depth);
    cv::waitKey(1);
    //Let the prev. cloud in the next frame be the current cloud
    *prev_cloud = *curr_cloud;
    i++;
 }


