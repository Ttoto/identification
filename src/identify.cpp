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
const float GOOD_MATCH_PERCENT = 0.1f;


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
        Mat processing_dispalay;
        Mat imMatches;
        Mat curr_img_identified;
        if(hasodom)
        {
            Mat orig_img;

            //find the nearest image in the map
            nearest_distance=999;;
            nearest_idx=0;
            for(int i=0; i<camera_poses.size(); i++)
            {

                double distance=sqrt(pow(camera_poses.at(i).pose.position.x-curr_pose.pose.position.x,2)+
                                     pow(camera_poses.at(i).pose.position.y-curr_pose.pose.position.y,2)+
                                     pow(camera_poses.at(i).pose.position.z-curr_pose.pose.position.z,2));
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
                // Convert images to gray scale;
                Mat im1Gray, im2Gray;
                cvtColor(orig_img, im1Gray, CV_BGR2GRAY);
                cvtColor(curr_img, im2Gray, CV_BGR2GRAY);
                // Variables to store keypoints and descriptors
                std::vector<KeyPoint> keypoints1, keypoints2;
                Mat descriptors1, descriptors2;
                // Detect ORB features and compute descriptors.
                Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
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


                drawMatches(orig_img, keypoints1, curr_img, keypoints2, matches, imMatches);
                Size size(640,240);
                cv::resize(imMatches,imMatches,size);
                // Extract location of good matches
                std::vector<Point2f> points1, points2;
                for( size_t i = 0; i < matches.size(); i++ )
                {
                    points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
                    points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
                }
                // Find homography
                Mat h = findHomography( points1, points2, cv::RANSAC , 2.0 );
                Mat orig_img_alignment;
                // Use homography to warp image
                warpPerspective(orig_img, orig_img_alignment, h, curr_img.size());

                //imshow("aligned input",orig_img_alignment);
                //moveWindow("aligned input", 40,540);
                Mat diff_img = orig_img_alignment - curr_img;
                Mat diff_grey;
                cvtColor(diff_img, diff_grey, CV_BGR2GRAY);
                //                Mat diff_grey(480, 640, CV_8U);
                //                for (int x=0;x<diff_img.cols;x++) {
                //                    for (int y=0;y<diff_img.rows;y++) {
                //                        Vec3b intensity = diff_img.at<Vec3b>(y, x);
                //                        uchar blue = intensity.val[0];
                //                        uchar green = intensity.val[1];
                //                        uchar red = intensity.val[2];
                //                        uchar itsty = (((int)blue+(int)green+(int)red)/3);
                //                        diff_grey.at<uchar>(y,x) = itsty;
                //                    }
                //                }

                Mat diff_binary;
                threshold( diff_grey, diff_binary, 150, 255, 0);
                //
                bool found_diff = false;
                double max_value=0;
                int max_x,max_y;
                for(int x = 40; x< 590; x+=10)
                {
                    for(int y = 40; y< 430; y+=10)
                    {
                        Mat patch;
                        cv::getRectSubPix(diff_binary,Size(80,80),Point2f(x,y),patch);
                        double sum = cv::sum(patch)[0];
                        if(sum>1000*255)
                        {
                            found_diff = true;
                            if(sum>max_value)
                            {
                                max_value = sum;
                                max_x = x;
                                max_y = y;
                            }
                        }
                    }

                }
                curr_img.copyTo(curr_img_identified);
                if(found_diff&&curr_pose.pose.position.z>0.6)
                {
                    cout << "found difference at " << max_x << "," << max_y << endl;
                    cv::circle(curr_img_identified,Point(max_x,max_y),40, Scalar(0, 0, 255),4);
                    cv::circle(curr_img_identified,Point(max_x,max_y),5, Scalar(0, 0, 255),2);
                    cv::putText(curr_img_identified,
                                "Warning",Point(0,475),
                                FONT_HERSHEY_SIMPLEX, 2,
                                Scalar(0,0,255),3);
                }else
                {
                    cv::putText(curr_img_identified,
                                "Normal",Point(0,475),
                                FONT_HERSHEY_SIMPLEX, 2,
                                Scalar(0,255,0),3);
                }

                cvtColor(diff_binary, diff_binary, CV_GRAY2BGR);
                cvtColor(diff_grey, diff_grey, CV_GRAY2BGR);
                hconcat(orig_img_alignment,diff_grey,processing_dispalay);
                hconcat(processing_dispalay,diff_binary,processing_dispalay);
                Size size2(960,240);
                cv::resize(processing_dispalay,processing_dispalay,size2);
            }
            imshow(    "Camera input",       curr_img);
            moveWindow("Camera input",       0,20);
            imshow(    "Identified output",  curr_img_identified);
            moveWindow("Identified output",  640,20);
            imshow(    "Matched Map Image",  orig_img);
            moveWindow("Matched Map Image",  1280,20);
            imshow(    "Input Map matching", imMatches);
            moveWindow("Input Map matching", 0,540);
            imshow(    "Processing",         processing_dispalay);
            moveWindow("Processing",         0,810);
            cv::waitKey(1);
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
