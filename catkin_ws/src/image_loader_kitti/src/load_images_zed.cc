/**
 * @file load_images_zed.cc
 * @author Emanuele Nencioni (you@domain.com)
 * @brief This ROS node allows loading images from saved images of the ZED camera, and publishing them on a ROS topic. 
        It also includes a "save" message on a topic to signal the completion of the sequence to a SLAM system.

        Warning: this file contains broken paths.
 * @version 0.1
 * @date 2023-09-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp> 


using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, int size);

int main(int argc, char **argv) {
    cv_bridge::CvImage img_bridge;
    ros::init(argc, argv, "load_images");
    ros::NodeHandle node_handler;
    image_transport::ImageTransport it_(node_handler);

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    string node_name = ros::this_node::getName();
    string path_to_sequence;
    node_handler.param<string>(node_name + "/path_to_sequence",path_to_sequence, "file_not_set");
    int size = 0;
    node_handler.param<int>(node_name + "/size", size, 0);
    if (size == 0) {
        ROS_ERROR("set the size");       
        ros::shutdown();
        return 1;
    } 

    if (path_to_sequence == "file_not_set") {
        ROS_ERROR("Please provide the path to sequence");       
        ros::shutdown();
        return 1;
    } 
    cout<<path_to_sequence<<endl;
    LoadImages(path_to_sequence, vstrImageLeft, vstrImageRight, size);
    image_transport::Publisher pub_left = it_.advertise(node_name + "/left", 10);
    image_transport::Publisher pub_right = it_.advertise(node_name + "/right", 10);
    auto pub_save = node_handler.advertise<std_msgs::String>(node_name + "/save", 10);
    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
   // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    float imageScale = 1.f; //TODO verify if we have to change the imageScale

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   
    double t_track = 0.f;
    double t_resize = 0.f;

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        ros::Time tframe = ros::Time::now();

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {

            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // we have to publish the images here
        sensor_msgs::Image image_left, image_right;
        std_msgs::Header header;
        
        header.stamp = tframe;
        header.frame_id = "camera";

        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, imLeft);
        img_bridge.toImageMsg(image_left); // from cv_bridge to sensor_msgs::Image
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, imRight);
        img_bridge.toImageMsg(image_right); // from cv_bridge to sensor_msgs::Image
        
        if(ni == 0)
                usleep(5e5);
        pub_left.publish(image_left); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
        pub_right.publish(image_right);
        
        //SLAM.TrackStereo(imLeft,imRight,tframe);
        cout<<"image: "<<ni<<endl;


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif


        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        if(ttrack<0.1)
            usleep((0.1-ttrack)*1e6);
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t3 = std::chrono::monotonic_clock::now();
    #endif
    double time3 = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        cout<<"frequency: "<<1.0/time3<<"time: "<<time3<<endl;
    }

    // Stop all threads

    // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
       
        std_msgs::String msg = std_msgs::String();
        msg.data = "save";
        cout<<"waiting 6 sec"<<endl;
        usleep(6e6);
        pub_save.publish(msg);
        usleep(1e6);
        pub_save.publish(msg);
        usleep(1e6);
        pub_save.publish(msg);
        cout<<"saved msg sent"<<endl;
        return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, int size)
{

    string strPrefixLeft = strPathToSequence + "/0/";
    string strPrefixRight = strPathToSequence + "/1/";

    const int nTimes = size;
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        // stringstream ss;
        // ss << setfill('0') << setw(3) << i+1;
        stringstream ss, sd;
        ss <<"image_"<< i;
        sd<<"";//setfill('0') << setw(4) << i+1;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + sd.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() +sd.str() + ".png";
    }
}
