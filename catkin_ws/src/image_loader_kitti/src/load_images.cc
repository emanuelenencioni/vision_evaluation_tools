/**
 * @file load_images.cc
 * @author Emanuele Nencioni 
 * @brief This ROS node allows loading images from the KITTI dataset and publishing them on a ROS topic. 
        It also includes a "save" message on a topic to signal the completion of the sequence to a SLAM system.
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
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

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
    if (path_to_sequence == "file_not_set") {
        ROS_ERROR("Please provide the path to sequence");       
        ros::shutdown();
        return 1;
    } 
    LoadImages(path_to_sequence, vstrImageLeft, vstrImageRight, vTimestamps);
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
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            //SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // we have to publish the images here
        sensor_msgs::Image image_left, image_right;
        std_msgs::Header header;
        header.stamp = ros::Time(tframe);
        header.frame_id = "camera";

        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, imLeft);
        img_bridge.toImageMsg(image_left); // from cv_bridge to sensor_msgs::Image
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, imRight);
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

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        //SLAM.InsertTrackTime(t_track); // TODO verify if this is needed in slam
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads

    // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        for(int ni=0; ni<nImages; ni++)
        {
                totaltime+=vTimesTrack[ni];
        }
        cout << "-------" << endl << endl;
        cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
        cout << "mean tracking time: " << totaltime/nImages << endl;
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
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_2/";
    string strPrefixRight = strPathToSequence + "/image_3/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
