/*********************************************************/
/*******************Enjoy Robotics************************/
/*******************Violetta Robot************************/
/**************Aruco Marker Recognition*******************/
/***********************Ver 1.0***************************/
/*********************************************************/

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"

#include <tf/transform_listener.h>
#include "marker_detection/Charger.h"



/*********************************************************/
/*********************Main Parameters*********************/
/*********************************************************/

//Ros
#define DEBUG_MODE 1
#define ROS_RATE 30
#define CHARGER_TF_PREFIX "charger_"
#define ANCHOR_TF_PREFIX "anchor_"

//Marker
#define MAX_DISTANCE_FILTER 3 // [m]
#define MARKER_SIZE 0.1       // [m]
#define charger_ID 0          // For filtering other IDs

// Line Extraction Filter Tollerences
#define rho_toll 0.25
#define A_toll 0.3
#define toll 0.27

//Start Recognition Message
bool detectmsg = 1; //0
bool isDetected = false;
int detected_ID;

//Camera Calibration
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 1.2277770447306639e+03, 0., 6.7695507058928445e+02, 0.,
                        1.2204001738207367e+03, 3.8471994678823000e+02, 0., 0., 1.);
cv::Mat distCoeffs = (cv::Mat1d(1, 5) << -4.2088932849197058e-01, 3.7596913509084806e-01,
                      1.9174525282902837e-03, -2.7046264217755057e-04,
                      -5.2506166430401069e-01);
cv::Mat last_frame;
cv_bridge::CvImagePtr cv_ptr;

//Our Publishers
ros::Publisher raw_pub;
ros::Publisher overlayed_pub;
ros::Publisher laser_publisher;
ros::Publisher Charger_pub;

/*********************************************************/
/*********************Used Functions**********************/
/*********************************************************/

//Normalize to [-PI,PI):
double PIconstrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

//Check start detection msg
void detectCallback(const std_msgs::Bool::ConstPtr &msg)
{
    detectmsg = msg->data;
    //ROS_INFO("Start detcting msg recieved: %i", detectmsg);
}

float L_ang;
float M_X, M_Y, M_theta;
float L_X, L_Y;

/*********************Marker Frames**********************/

//Get Marker Pose according to /lidar frame
void getMarkerPose(tf::TransformListener &listenerGoal, tf::StampedTransform &transformGoal)
{
   //Catch marker pose
   try
      {
        ros::Time now = ros::Time::now();
        listenerGoal.lookupTransform("/lidar", "/charger_0", now-ros::Duration(0.2), transformGoal);
        M_X = transformGoal.getOrigin().x();
        M_Y = transformGoal.getOrigin().y();
        //M_theta = tf::getYaw(transformGoal.getRotation());
        M_theta = transformGoal.getRotation().z();
        //ROS_INFO("MX: %f MY: %f", M_X, M_Y);
      }

   catch (tf::TransformException &ex)
      {
        //ROS_ERROR("%s", ex.what());
        //ros::Duration(1.0).sleep();
      }
}

//Get Marker Pose according to "/map" frame
float M0_X, M0_Y, M0_theta, M0_theta0;
float M0_Z, M0_alpha, M0_beta;
bool isErr = true;
void getM0Pose(tf::TransformListener &listener, tf::StampedTransform &transform)
{
   //Catch marker pose
   try
      {
        ros::Time now = ros::Time::now();
        listener.lookupTransform("/map", "/charger_0", now-ros::Duration(0.3), transform);
        M0_X = transform.getOrigin().x();
        M0_Y = transform.getOrigin().y();
        // M0_theta = tf::getYaw(transform.getRotation());
        M0_theta = transform.getRotation().z();
        //ROS_INFO("MX: %f MY: %f", M0_X, M0_Y);

        M0_theta0 = tf::getYaw(transform.getRotation());

        M0_Z = transform.getOrigin().z();
        M0_alpha = transform.getRotation().x();
        M0_beta = transform.getRotation().y();

        isErr = true;
      }

   catch (tf::TransformException &ex)
      {
        if(isErr)
            {ROS_ERROR("%s", ex.what());
            isErr = false;
            }
        //ros::Duration(1.0).sleep();
      }
}

/*****************Laser Line Extraction******************/

//Line Extraction to correct Marker Angle
void lineSegmentsCallback(const laser_line_extraction::LineSegmentListPtr &msg)
{
    float A;
    static tf::TransformListener listenerGoal;
    tf::StampedTransform transformGoal;

    getMarkerPose(listenerGoal, transformGoal);
    for (int i = 0; i < msg->line_segments.size(); i++){
        laser_line_extraction::LineSegment line_segment = msg->line_segments[i];

        if(line_segment.angle>=0)
            A = line_segment.angle - M_PI;
        else
            A = line_segment.angle + M_PI;

        L_X = (line_segment.start[0] + line_segment.end[0])/2;
        L_Y = (line_segment.start[1] + line_segment.end[1])/2;
        //ROS_INFO("LX: %f LY: %f", L_X, L_Y);
        if ((L_X < M_X + toll && L_X > M_X - toll) && (L_Y < M_Y + toll && L_Y > M_Y - toll))
            {
                L_ang = A;
                isDetected = true;
                //ROS_INFO("MX: %f MY: %f", M_X, M_Y);
            }
    }
}

/*********************Docking Point**********************/

//Calculating Doking point position
float P0_X, P0_Y, P0_theta;
float li = 1;
void calculateDockingPoint()
{
    if(P0_theta>=0)
        P0_theta = PIconstrainAngle(M0_theta0 - M_PI);
    else
        P0_theta = PIconstrainAngle(M0_theta0 + M_PI);
    P0_X = M0_X - li*cosf(M0_theta0);
    P0_Y = M0_Y - li*sinf(M0_theta0);
    //ROS_INFO("P0_X: %f P0_Y: %f", P0_X, P0_Y);
}

//Unused fubction: If we have an already poblished Doking point
void getP0Pose(tf::TransformListener &listener, tf::StampedTransform &transform)
{
   //Catch marker pose
   try
      {
        ros::Time now = ros::Time::now();
        listener.lookupTransform("/map", "/p0", now-ros::Duration(0.2) , transform);
        P0_X = transform.getOrigin().x();
        P0_Y = transform.getOrigin().y();
        P0_theta = tf::getYaw(transform.getRotation());
        //ROS_INFO("MX: %f MY: %f", M_X, M_Y);
      }

   catch (tf::TransformException &ex)
      {
         //ROS_ERROR("%s", ex.what());
         //ros::Duration(1.0).sleep();
      }
}

//Publish docking Point as a transfer frame
void dockingPointPublish()
{
    static tf::TransformBroadcaster dr;
    tf::Transform docking_tf;

    calculateDockingPoint();

    docking_tf.setOrigin(tf::Vector3(P0_X, P0_Y, M0_Z));

    tf::Quaternion docking_q;
    docking_q.setRPY(M0_alpha, M0_beta, P0_theta);

    docking_tf.setRotation(docking_q);

    dr.sendTransform(
    tf::StampedTransform(
        docking_tf,
        ros::Time::now(),
        "map",
        "Docking_Point"));
}

/***************Publishing Charger Topic*****************/

void chargerPublish(ros::Publisher pub)
{
    static tf::TransformListener listenerM;
    tf::StampedTransform transformM;

    static tf::TransformListener listenerP;
    tf::StampedTransform transformP;

    getM0Pose(listenerM, transformM);
    //getP0Pose(listenerP, transformP);
    calculateDockingPoint();

    marker_detection::Charger charger_msg;
    charger_msg.id = detected_ID;
    charger_msg.charger_x = M0_X;
    charger_msg.charger_y = M0_Y;
    charger_msg.charger_rotation = P0_theta;
    charger_msg.docking_point_x = P0_X;
    charger_msg.docking_point_y = P0_Y;
    charger_msg.docking_point_rotation = P0_theta;
    pub.publish(charger_msg);
}

//Publish docking Point as a transfer frame
void chargerFramePublish()
{
    static tf::TransformBroadcaster ch;
    tf::Transform docking_ch;

    calculateDockingPoint();

    docking_ch.setOrigin(tf::Vector3(M0_X, M0_Y, M0_Z));

    tf::Quaternion docking_c;
    docking_c.setRPY(M0_alpha, M0_beta, P0_theta);

    docking_ch.setRotation(docking_c);

    ch.sendTransform(
    tf::StampedTransform(
        docking_ch,
        ros::Time::now(),
        "map",
        "Map_Charger"));
}

/*********************************************************/
/*******************Aruco Recognition*********************/
/*********************************************************/

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if (detectmsg == 1)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            last_frame = cv_ptr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            return;
        }

        static tf::TransformBroadcaster br;
        tf::Transform marker_tf;

        cv::Mat image_copy;
        last_frame.copyTo(image_copy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(last_frame, dictionary, corners, ids);
        if (ids.size() > 0)
        {
            //ROS_INFO("Detected %d marker(s)", (int) ids.size());
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_SIZE, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (int i = 0; i < ids.size(); i++)
            {
                cv::aruco::drawAxis(image_copy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                if (sqrt(tvecs[i][0] * tvecs[i][0] + tvecs[i][1] * tvecs[i][1] + tvecs[i][2] * tvecs[i][2]) <= MAX_DISTANCE_FILTER)
                {
                    //ROS    isDetected = false;_INFO("Marker inbound with ID: %d", ids[i]);
                    marker_tf.setOrigin(tf::Vector3(tvecs[i][2], -tvecs[i][0], tvecs[i][1]));
                    // marker_tf.setOrigin(tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
                    tf::Quaternion marker_q;

                    if(rvecs[i][0] < 0)
                    {
                        rvecs[i][2] = -rvecs[i][2];
                        rvecs[i][1] = -rvecs[i][1];
                        rvecs[i][0] = -rvecs[i][0];
                    }

                    marker_q.setRPY(rvecs[i][0]- M_PI, rvecs[i][1], rvecs[i][2]);
                    // marker_q.setRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                    if(isDetected)
                    {
                        marker_q.setRPY(rvecs[i][0]- M_PI, rvecs[i][1], L_ang);
                        //ROS_INFO("Line segment LX: %f LY: %f LA: %f", L_X, L_Y, L_ang);
                    }   
                    marker_tf.setRotation(marker_q);
                    //ROS_INFO("MX: %f MY: %f MA: %f", M_X, M_X,getYaw(marker_tf.getRotation()));

                    if (ids[i] == charger_ID)
                    {
                        detected_ID = ids[i];
                        // ROS_INFO("Matching id with predefined charger_ID");
                        br.sendTransform(
                            tf::StampedTransform(
                                marker_tf,
                                ros::Time::now(),
                                "usb_cam",
                                CHARGER_TF_PREFIX + std::to_string(ids[i])));
                        //ROS_INFO("MX: %f MY: %-f MYAW: %f", marker_tf.getOrigin().getX(), marker_tf.getOrigin().getY() ,getYaw(marker_tf.getRotation()));
                    }
                    dockingPointPublish();
                    chargerPublish(Charger_pub);
                    chargerFramePublish();
                }
            }
        }

        cv_bridge::CvImage img_raw;
        img_raw.encoding = sensor_msgs::image_encodings::BGR8; // More options here: http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html
        img_raw.image = last_frame;
        raw_pub.publish(img_raw.toImageMsg());

        cv_bridge::CvImage img_overlayed;
        img_overlayed.encoding = sensor_msgs::image_encodings::BGR8;
        img_overlayed.image = image_copy;
        overlayed_pub.publish(img_overlayed.toImageMsg());

        // chargerPublish(Charger_pub);
    }
}

/*********************************************************/
/*****************Main Programn Execution*****************/
/*********************************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_detection");
    ros::NodeHandle nh;
    ROS_INFO("Marker detection node started");

    tf::TransformListener listenerGoal;
    tf::StampedTransform transformGoal;

    //Start detction msg
    ros::Subscriber sub = nh.subscribe("detectState", 1, detectCallback);

    //Start rcognition
    ros::Subscriber image_subscriber = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

    //Lidar line segments
    ros::Subscriber line_segments_subscriber = nh.subscribe("/line_segments", 0.5, lineSegmentsCallback);

    //Publishing output image
    raw_pub = nh.advertise<sensor_msgs::Image>("/docking/camera_raw", 10);
    overlayed_pub = nh.advertise<sensor_msgs::Image>("/docking/camera_overlayed", 10);
    ROS_INFO("Matching id with predefined charger_ID: %d", charger_ID);

    Charger_pub = nh.advertise<marker_detection::Charger>("charger", 1000);

    ros::Rate ros_rate(ROS_RATE);

    while (ros::ok())
    {
        ros::spinOnce();
        ros_rate.sleep();
    }
    return 0;
}
