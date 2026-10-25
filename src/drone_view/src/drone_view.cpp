#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

constexpr int N=6, COLS=3, ROWS=2;

struct DroneNode{
    ros::Subscriber subs[N];
    cv::Mat frames[N];
    std::mutex mtx[N];
    bool updated[N]={};

    DroneNode():nh("~"){
        for(int i=0;i<N;i++){
            std::string topic="/typhoon_h480_"+std::to_string(i)+"/realsense/depth_camera/color/image_raw";
            subs[i]=nh.subscribe<sensor_msgs::Image>(topic,1,
                boost::bind(&DroneNode::cb,this,_1,i));
            frames[i]=cv::Mat(240,320,CV_8UC3,cv::Scalar(30,30,30));
            cv::putText(frames[i],std::to_string(i),cv::Point(5,25),
                cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,255,0),2);
        }
    }

    void cb(const sensor_msgs::Image::ConstPtr& msg,int i){
        cv_bridge::CvImagePtr cv_ptr;
        try{cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);}
        catch(...){return;}
        std::lock_guard<std::mutex> lk(mtx[i]);
        cv::resize(cv_ptr->image,frames[i],cv::Size(320,240));
        cv::putText(frames[i],std::to_string(i),cv::Point(5,25),
            cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,255,0),2);
        updated[i]=true;
    }

    void run(){
        cv::namedWindow("6-Drone View",cv::WINDOW_NORMAL);
        cv::resizeWindow("6-Drone View",960,480);
        ros::Rate rate(15);
        while(ros::ok()){
            cv::Mat grid(480,960,CV_8UC3,cv::Scalar(20,20,20));
            for(int i=0;i<N;i++){
                int r=i/COLS,c=i%COLS;
                cv::Rect roi(c*320,r*240,320,240);
                std::lock_guard<std::mutex> lk(mtx[i]);
                frames[i].copyTo(grid(roi));
            }
            cv::imshow("6-Drone View",grid);
            if(cv::waitKey(1)=='q')break;
            ros::spinOnce();rate.sleep();
        }
    }
    ros::NodeHandle nh;
};

int main(int argc,char**argv){
    ros::init(argc,argv,"drone_view");
    DroneNode().run();
    return 0;
}
