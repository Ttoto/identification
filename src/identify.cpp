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
std::vector<geometry_msgs::PoseStamped> camera_poses;
bool hasodom;
double nearest_distance;
int nearest_idx;
int feature_idx;
Mat orig_img;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.4f;


void odomCB(const geometry_msgs::PoseStampedConstPtr msg)
{
    curr_pose = *msg;
    hasodom = true;
}

void imageCB(const sensor_msgs::ImagePtr msg)
{
    try
    {
        Mat curr_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        imshow("camera input",curr_img);
        moveWindow("camera input", 40,20);
        cv::waitKey(1);
        if(hasodom)
        {
            Mat orig_img;
            if(curr_pose.pose.position.z>0.3&&curr_pose.pose.position.x>0.3)//indenfication condition
            {
                //find the nearest image in the map
                nearest_distance=999;;
                nearest_idx=0;
                for(int i=0; i<camera_poses.size(); i++)
                {
                    double distance=fabs(camera_poses.at(i).pose.position.x-curr_pose.pose.position.x);
                    if(distance<nearest_distance){
                        nearest_distance = distance;
                        nearest_idx = i;
                    }
                }
                nearest_idx = camera_poses.at(nearest_idx).header.seq;
                if(nearest_distance<0.5)
                {
                    cout << "cloest image is" << nearest_idx << endl;
                    if(feature_idx!=nearest_idx)
                    {
                        feature_idx = nearest_idx;

                    }
                    orig_img = imread(filepath+to_string(nearest_idx)+".png", CV_LOAD_IMAGE_COLOR);
//                    imshow("origimg",orig_img);

                    // Convert images to gray scale;
                    Mat im1Gray, im2Gray;
                    cvtColor(orig_img, im1Gray, CV_BGR2GRAY);
                    cvtColor(curr_img, im2Gray, CV_BGR2GRAY);
                    // Variables to store keypoints and descriptors
                    std::vector<KeyPoint> keypoints1, keypoints2;
                    Mat descriptors1, descriptors2;

                    // Detect ORB features and compute descriptors.
                    Ptr<Feature2D> orb = ORB::create(2000);
                    orb->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
                    orb->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);

                    // Match features.
                    std::vector<DMatch> matches;
                    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
                    matcher->match(descriptors1, descriptors2, matches, Mat());

                    // Sort matches by score
                    std::sort(matches.begin(), matches.end());

                    // Remove not so good matches
                    const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
                    matches.erase(matches.begin()+numGoodMatches, matches.end());

                    // Draw top matches
                    Mat imMatches;
                    drawMatches(orig_img, keypoints1, curr_img, keypoints2, matches, imMatches);
                    imshow("match to map data", imMatches);
                    moveWindow("match to map data", 700,20);
                    // Extract location of good matches
                    std::vector<Point2f> points1, points2;

                    for( size_t i = 0; i < matches.size(); i++ )
                    {
                      points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
                      points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
                    }

                    // Find homography
                    Mat h = findHomography( points1, points2, RANSAC );
                    Mat orig_img_alignment;
                    // Use homography to warp image
                    warpPerspective(orig_img, orig_img_alignment, h, curr_img.size());


                    imshow("aligned input",orig_img_alignment);
                    moveWindow("aligned input", 40,540);
                    Mat diff_img = orig_img_alignment - curr_img;
                    imshow("residual of images",diff_img);
                    moveWindow("residual of images", 640,540);

                }

                //detection
            }
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

    }

}

void loadmap()
{
    cvs_fs.open(filepath+"map.csv", ios::in);
    vector<string> row;
    string line, word, temp;
    while (cvs_fs >> line) {
        row.clear();
        stringstream s(line);
        while (getline(s, word, ',')) {
            row.push_back(word);
        }
        cout << "idx:" << row[0] << " px:" << row[1] << " py: " << row[2] << " pz: " << row[3]
             << " qw:" << row[4] << " qx:" << row[5] << " qy: " << row[6] << " qz: " << row[7] << endl;
        geometry_msgs::PoseStamped pose;
        pose.header.seq=stoi(row[0]);
        pose.pose.position.x=stod(row[1]);
        pose.pose.position.y=stod(row[2]);
        pose.pose.position.z=stod(row[3]);
        pose.pose.orientation.w=stod(row[4]);
        pose.pose.orientation.x=stod(row[5]);
        pose.pose.orientation.y=stod(row[6]);
        pose.pose.orientation.z=stod(row[7]);
        camera_poses.push_back(pose);
    }
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "map");

    ros::NodeHandle nh;

    hasodom = false;
    feature_idx = 0;
    string aaa;

    nh.getParam("map_files_path", filepath);
    cout << "read file path: " << filepath << endl;
    //load map
    loadmap();

    ros::Subscriber sub1 = nh.subscribe("/odom_in", 1,  odomCB);
    ros::Subscriber sub2 = nh.subscribe("/image_in", 1, imageCB);
    cout << "strat spin()" << endl;

    ros::spin();

    return 0;
}
