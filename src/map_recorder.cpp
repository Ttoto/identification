#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sstream>

using namespace std;
using namespace cv;

string filepath;
fstream cvs_fs;
geometry_msgs::PoseStamped curr_pose;
geometry_msgs::PoseStamped last_saved_pose;
bool hasodom;
bool initpose;
int  image_save_count;

string filename;




void cvs_pose_write(int index,
                    double px, double py, double pz,
                    double qw, double qx, double qy, double qz)
{
    cvs_fs.open(filepath+"map.csv", ios::out | ios::app);
    cvs_fs << index << ","
           << px << "," << py << "," << px << ","
           << qw << "," << qx << ","<< qy << ","<< qz << "\n";
    cvs_fs.close();
}

void odomCB(const geometry_msgs::PoseStampedConstPtr msg)
{
    curr_pose = *msg;
    hasodom = true;
}

void imageCB(const sensor_msgs::ImagePtr msg)
{
    try
    {
        Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        //imshow("image",img);
        //cv::waitKey(1);
        if(hasodom)
        {
            //cout << curr_pose.pose.position.x << endl;
            if(!initpose){//if not initialized, initialize it;

                image_save_count = 1;
                //save to file
                cout << "write image: " << filepath+to_string(image_save_count)+".jpg" << endl;
                last_saved_pose = curr_pose;
                cvs_pose_write(image_save_count,
                               last_saved_pose.pose.position.x,
                               last_saved_pose.pose.position.y,
                               last_saved_pose.pose.position.z,
                               last_saved_pose.pose.orientation.w,
                               last_saved_pose.pose.orientation.x,
                               last_saved_pose.pose.orientation.y,
                               last_saved_pose.pose.orientation.z);
                imwrite(filepath+to_string(image_save_count)+".png", img);
                image_save_count++;
                initpose = true;
            }
            else {
                double dx=curr_pose.pose.position.x-last_saved_pose.pose.position.x;
                double dy=curr_pose.pose.position.y-last_saved_pose.pose.position.y;
                double dz=curr_pose.pose.position.z-last_saved_pose.pose.position.z;
                if(dx>0.3)
                {
                    last_saved_pose = curr_pose;
                    cvs_pose_write(image_save_count,
                                   last_saved_pose.pose.position.x,
                                   last_saved_pose.pose.position.y,
                                   last_saved_pose.pose.position.z,
                                   last_saved_pose.pose.orientation.w,
                                   last_saved_pose.pose.orientation.x,
                                   last_saved_pose.pose.orientation.y,
                                   last_saved_pose.pose.orientation.z);
                    imwrite(filepath+to_string(image_save_count)+".png", img );
                    image_save_count++;
                }
            }
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

    }

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "map");

    ros::NodeHandle nh;

    hasodom = false;
    initpose =  false;
    string aaa;

    nh.getParam("map_files_path", filepath);
    cout << "read file path: " << filepath << endl;


    ros::Subscriber sub1 = nh.subscribe("/odom_in", 5,  odomCB);
    ros::Subscriber sub2 = nh.subscribe("/image_in", 1, imageCB);
    cout << "strat spin()" << endl;

    ros::spin();

    return 0;
}
