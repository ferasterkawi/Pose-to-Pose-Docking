/***********************************************************************/
/**************************Enjoy Robotics*******************************/
/**************************Violetta Robot*******************************/
/***********************Docking Path Planning***************************/
/******************************Ver 3.1**********************************/
/***********************************************************************/

#include <ros/ros.h>
#include <ros/rate.h>

#include <actionlib/server/simple_action_server.h>
#include <docking/DockingAction.h>

#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ContactsState.h>
#include "std_msgs/Bool.h"
#include <tf/transform_listener.h>

#include <stdlib.h>
#include <math.h>

/*********************************************************/
/*********************Main Parameters*********************/
/*********************************************************/

//Robot Parametes
#define vmax 0.10   //0.10
#define vmin 0.04   //0.04
#define wmax 0.20   //0.20
#define wmin 0.05   //0.06
#define ymax 1.00   //2.00
#define ymin 0.50   //0.50
#define amax M_PI/2 //M_PI/2
#define amin 0      //0

//Algorithm parameter
#define RHO_TLR 0.04 //minimum distance to goal
#define aTol M_PI/80
#define bTol M_PI/60

#define Kp_rho   vmax / ymax    //linear velocity coefficient
#define Kp_alpha wmax * amax    //angular velocity coefficient
#define Kp_beta 0//-wmax * amax *0.01   //parallel to goal coefficient

//Coodrdinates
#define x_start 0
#define y_start 0
#define theta_start 0

#define r 0.32 //Half of robot redius

float x_goal; //-1.62;
float y_goal; //-0.30;
float theta_goal; //Backward-of-Robot Direction Angle
#define DP 0.32  // Gaol-offset distance from marker in m
#define D0 1.00  // D0-offset distance from Goal
#define D1 0.60  // D0-offset distance from Goal

//System
#define ROS_FREQUENCY 10
int charging_time_out = 3;
int retry_num = 1;
bool isCharging = false;
bool isDetected = false;
bool isFinished = false;

/*********************************************************/
/*********************Used Functions**********************/
/*********************************************************/

//Inittialize Variables
float v = 0;
float w = 0;

float x = x_start;
float y = y_start;
float theta = theta_start;

float x_marker;
float y_marker;
float theta_marker;

float x_D0;
float y_D0;
float theta_D0;

float x_D1;
float y_D1;
float theta_D1;

float alpha, beta, x_diff, y_diff, rho;

/*********************Aux Functions**********************/

//Normalize an angle to [0,2*PI):
float constrainAngle(float x){
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
}

//Normalize to [-PI,PI):
double PIconstrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

//Limitation function
float clip(float n, float lower, float upper)
{
   return std::max(lower, std::min(n, upper));
}

//Clip velocities between max & min
void clipVelocities(float v, float w)
{
   if (v>0)
      v = clip(v, vmin, vmax);
   else
      v = clip(v, -vmax, -vmin);
   if (w > wmin / 5)
      w = clip(w, wmin, wmax);
   else if (w < -wmin / 2)
      w = clip(w, -wmax, -wmin);
   else
      w = 0; 
}

/*******************Callback Functions*******************/

//Chick charging callback
void contactCallback(const std_msgs::Bool::ConstPtr &msg)
{
   if (msg->data == true)
   {
      isCharging = true;
      // ROS_INFO("Start Charging...");
   }
}

/*******************Path Functions**********************/



//Get the current position of the robot
void getCurrentPose(tf::TransformListener &listener, tf::StampedTransform &transform)
{
   try
      {
         listener.lookupTransform("odom", "/base_link", ros::Time(0), transform);
      }
   catch (tf::TransformException &ex)
      {
         ROS_ERROR("%s", ex.what());
         ros::Duration(1.0).sleep();
      }
   x = transform.getOrigin().x();
   y = transform.getOrigin().y();
   theta = tf::getYaw(transform.getRotation()) - M_PI;
   //ROS_INFO("RX: %f RY: %f Rtheta: %f", x, y, theta);
}

//Apply Veloceties
geometry_msgs::Twist msg;
void applyVelocities(float V, float W, ros::Publisher pub)
{
   msg.linear.x = V;
   msg.angular.z = W;
   pub.publish(msg);
}

//Rotation Scanning for Marker:
std_msgs::Bool Start_Detecting_msg; //Marker detction order message
void markerScanning(tf::TransformListener &listenerGoal, tf::StampedTransform &transformGoal, ros::Publisher aruco_pub, tf::TransformListener &listener, tf::StampedTransform &transform, ros::Publisher pub, int Dir)
{
   ROS_INFO("Marker Scanning...");

   isDetected = false;
   Start_Detecting_msg.data = true; //Now the marker recognition is working all the time
   aruco_pub.publish(Start_Detecting_msg); //ROS_INFO("%i", Start_Detecting_msg.data);
   getCurrentPose(listener,transform);
   float theta_s = theta;
   float theta_dis;
   while(isDetected == false)
   {
      getCurrentPose(listener,transform);
      try
         {
            listenerGoal.lookupTransform("odom", "charger_0", ros::Time(0), transformGoal);
            isDetected = true;
            x_goal = transformGoal.getOrigin().x();
            y_goal = transformGoal.getOrigin().y();
            theta_goal = tf::getYaw(transformGoal.getRotation());
         }
      catch (tf::TransformException &ex)
         {
            ROS_INFO("No Marker yet... Rotating...");
            if (Dir == 1)
               applyVelocities(0, wmin, pub);
            else
               applyVelocities(0, -wmin, pub);
            theta_dis = constrainAngle(fmod((theta_s - theta + M_PI), (2 * M_PI)) - M_PI);
            if (theta_dis>6.1)
               {
                  applyVelocities(0, 0, pub);
                  ROS_ERROR("No Marker in Place!");
                  break;
               }
            //ROS_INFO("theta_dis: %f theta_s: %f theta: %f", theta_dis, theta_s, theta);
         }
      ros::spinOnce();
   }
}

//Sub an offset vector from goal
void calculateDockingPoint()
{
   theta_goal = theta_goal;
   x_goal = x_goal - DP*cosf(theta_goal);
   y_goal = y_goal - DP*sinf(theta_goal);
   //ROS_INFO("P0_X: %f P0_Y: %f", P0_X, P0_Y);
}

void calculateD0Point()
{
   theta_D0 = theta_goal;
   x_D0 = x_goal - D0*cosf(theta_goal);
   y_D0= y_goal - D0*sinf(theta_goal);
   //ROS_INFO("P0_X: %f P0_Y: %f", P0_X, P0_Y);
}

void calculateD1Point()
{
   theta_D1 = theta_goal;
   x_D1 = x_goal - D1*cosf(theta_goal);
   y_D1= y_goal - D1*sinf(theta_goal);
   //ROS_INFO("P0_X: %f P0_Y: %f", P0_X, P0_Y);
}

//Catch the marker
void getMarkerPose(tf::TransformListener &listenerGoal, tf::StampedTransform &transformGoal, ros::Publisher aruco_pub)
{
   //Catch marker pose
   Start_Detecting_msg.data = true; //Now the marker recognition is working all the time
   aruco_pub.publish(Start_Detecting_msg); //ROS_INFO("%i", Start_Detecting_msg.data);
   try
      {
         listenerGoal.lookupTransform("odom", "charger_0", ros::Time(0), transformGoal);
         isDetected = true;
         x_goal = transformGoal.getOrigin().x();
         y_goal = transformGoal.getOrigin().y();
         theta_goal = tf::getYaw(transformGoal.getRotation());
         ROS_INFO("x_goal: %f y_goal: %f", x_goal, y_goal);
         //calculateDockingPoint(); // Add offset vector
         // ROS_INFO("x_goal: %f y_goal: %f", x_goal, y_goal);
      }

   catch (tf::TransformException &ex)
      {
         ROS_ERROR("%s", ex.what());
         //ros::Duration(1.0).sleep();
      }
}

//Rotating to straight-line with goal
void rotateToPoint(float XG, float YG, float THG, tf::TransformListener &listener, tf::StampedTransform &transform, ros::Publisher pub)
{
   ROS_INFO("Rotating to Goal Position Direction...");
   getCurrentPose(listener,transform); // x, y, theta
   //Calculating distances:
   x_diff = XG - x;
   y_diff = YG - y;
   rho = hypotf(x_diff, y_diff);
   alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI; //to-goal rotation
   beta = fmod((THG - theta - alpha + M_PI), (2 * M_PI)) - M_PI;     //parrallel-to-goal rotation
   while (alpha > aTol || alpha < -aTol)
   {
      getCurrentPose(listener,transform); // x, y, theta 

      x_diff = XG - x;
      y_diff = YG - y;
      rho = hypot(x_diff, y_diff);
      alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI;
      beta = fmod((THG - theta - alpha + M_PI), (2 * M_PI)) - M_PI;
      //ROS_INFO("alpha: %f beta: %f", alpha, beta);
      
      applyVelocities(0, alpha/abs(alpha) * wmin, pub);

      ros::spinOnce();
   }
   applyVelocities(0, 0, pub);
   ROS_INFO("Robot is directed to Goal!");
}

//Rotate to angle
void rotateToBeta(float XG, float YG, float THG, tf::TransformListener &listener, tf::StampedTransform &transform, ros::Publisher pub)
{
   ROS_INFO("Beta Rotating...");
   getCurrentPose(listener,transform); // x, y, theta
   //Calculating distances:
   x_diff = XG - x;
   y_diff = YG - y;
   rho = hypotf(x_diff, y_diff);
   alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI; //to-goal rotation
   beta = fmod((THG - theta - alpha + M_PI), (2 * M_PI)) - M_PI;     //parrallel-to-goal rotation
   while (alpha > beta/3 + bTol || alpha < beta/3 -bTol)
   {
      getCurrentPose(listener,transform);

      x_diff = x_goal - x;
      y_diff = y_goal - y;
      rho = hypot(x_diff, y_diff);

      alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI;
      beta = fmod((theta_goal - theta - alpha + M_PI), (2 * M_PI)) - M_PI;
      //ROS_INFO("alpha: %f beta: %f", alpha, beta);

      applyVelocities(0, -beta/abs(beta) * wmin, pub);

      ros::spinOnce();
   }
   applyVelocities(0, 0, pub);
   ROS_INFO("Robot is Beta-Rotated!");
}

//Curved-lines going to specific point
void goToPoint(float XG, float YG, float THG, tf::TransformListener &listener, tf::StampedTransform &transform, ros::Publisher pub)
{
   getCurrentPose(listener,transform); // x, y, theta
   //Calculating distances:
   x_diff = XG - x;
   y_diff = YG - y;
   rho = hypotf(x_diff, y_diff);
   alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI; //to-goal rotation
   beta = fmod((THG - theta - alpha + M_PI), (2 * M_PI)) - M_PI;     //parrallel-to-goal rotation

   ROS_INFO("RHO:%f x_diff: %f y_diff: %f",rho , x_diff, y_diff);

   //if(rho>RHO_TLR && abs(x_diff)> RHO_TLR && abs(y_diff)> RHO_TLR)
   if(rho>RHO_TLR)
   {  rotateToPoint(XG,YG,THG, listener, transform, pub);
      //rotateToBeta(XG, YG, THG, listener, transform, pub);
      ROS_INFO("Going to Point...");
      ros::Duration(0.5).sleep();

      while(rho>RHO_TLR && abs(alpha)< M_PI/6)
      {  
         getCurrentPose(listener,transform); // x, y, theta
         //Calculating distances:
         x_diff = XG - x;
         y_diff = YG - y;
         rho = hypotf(x_diff, y_diff);
         
         alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI; //to-goal rotation
         beta = fmod((theta_goal - theta - alpha + M_PI), (2 * M_PI)) - M_PI;     //parrallel-to-goal rotation

         //Calculating velocities
         v = -abs(Kp_rho * rho); //Backward Direction
         w = Kp_alpha * alpha + Kp_beta * beta;

         if(v>-vmin)
            v= -vmin;

         applyVelocities(v, w, pub);
         // ROS_INFO("V: %f W: %f RHO:%f x_diff: %f y_diff: %f alpha: %f A: %f", v, w,rho , x_diff, y_diff, alpha, atan2(y_diff, x_diff));
         ros::spinOnce();
      }
   }
   applyVelocities(0, 0, pub);
   ROS_INFO("Robot reached Point!");
}

//Curved-lines going to specific point
void goBackToPoint(float XG, float YG, float THG, tf::TransformListener &listener, tf::StampedTransform &transform, ros::Publisher pub)
{
   ROS_INFO("Going back to Point...");
   getCurrentPose(listener,transform); // x, y, theta
   theta = theta + M_PI;
   //Calculating distances:
   x_diff = XG - x;
   y_diff = YG - y;
   rho = hypotf(x_diff, y_diff);
   alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI; //to-goal rotation
   beta = fmod((THG - theta - alpha + M_PI), (2 * M_PI)) - M_PI;     //parrallel-to-goal rotation

   ROS_INFO("RHO:%f x_diff: %f y_diff: %f",rho , x_diff, y_diff);

   while(rho>RHO_TLR)
      {  
         getCurrentPose(listener,transform); // x, y, theta
         theta = theta + M_PI;
         //Calculating distances:
         x_diff = XG - x;
         y_diff = YG - y;
         rho = hypotf(x_diff, y_diff);
         
         alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI; //to-goal rotation
         beta = fmod((theta_goal - theta - alpha + M_PI), (2 * M_PI)) - M_PI;     //parrallel-to-goal rotation

         //Calculating velocities
         v = abs(Kp_rho * rho); //Backward Direction
         w = Kp_alpha * alpha + Kp_beta * beta;

         if(v<vmin)
            v= vmin;

         applyVelocities(v, w, pub);
         // ROS_INFO("V: %f W: %f RHO:%f x_diff: %f y_diff: %f alpha: %f A: %f", v, w,rho , x_diff, y_diff, alpha, atan2(y_diff, x_diff));
         ros::spinOnce();
      }
   applyVelocities(0, 0, pub);
   ROS_INFO("Robot Backed to Point!");
}

void docking_init(tf::TransformListener &listener, tf::StampedTransform &transform, ros::Publisher pub, tf::TransformListener &listenerGoal, tf::StampedTransform &transformGoal, ros::Publisher aruco_pub)
{
      //Start Rotating to goal postion direction
      // rotateToPoint(x_goal, y_goal, theta_goal, listener, transform, pub);
      // ros::Duration(0.5).sleep();

      // Catch Marker Now

      calculateD0Point();
      calculateD1Point();
      goToPoint(x_D1, y_D1, theta_D1, listener, transform, pub);
      ros::Duration(1.5).sleep();

      isDetected = false;

      getMarkerPose(listenerGoal, transformGoal, aruco_pub);

      // Rotating to start curved path
      // rotateToBeta(x_goal, y_goal, theta_goal, listener, transform, pub); 
      // ros::Duration(0.5).sleep(); 

      ROS_INFO("Initialization finished!");
}

/*********************************************************************************************/
/***********************************Action Server Execution***********************************/
/*********************************************************************************************/

typedef actionlib::SimpleActionServer<docking::DockingAction> Server;

docking::DockingResult result;

void execute(const docking::DockingGoalConstPtr &goal, Server *as, ros::Publisher pub, ros::Rate rate, ros::Publisher aruco_pub)
{
   /*******************Initialization*********************/

   result.success = false;

   //Creates the tf listener for current pose and goal pose
   tf::TransformListener listener;
   tf::StampedTransform transform;

   tf::TransformListener listenerGoal;
   tf::StampedTransform transformGoal;

   ROS_INFO("Executing Action Server...");
   applyVelocities(0, 0, pub);
   ros::Duration(1).sleep();

   //If you want the robot start scan for the marker by rotation
   //markerScanning(listenerGoal, transformGoal, aruco_pub, listener, transform, pub, -1);

   //Catch Marker Now
   getMarkerPose(listenerGoal, transformGoal, aruco_pub);

   if(isDetected == true && isFinished == false)
   {
      docking_init(listener, transform, pub, listenerGoal, transformGoal, aruco_pub);
   }
   else
   {
      result.success = false;
      as->setSucceeded(result);
      ROS_ERROR("Marker is Not Detected! Docking Faild!");
      ros::Duration(1).sleep();
      isFinished = true;
   }

   /*****************************START FOLLOWING loop********************************/

   while (ros::ok())
   {
      double t1 = ros::Time::now().toSec();

      //Catch current pose
      getCurrentPose(listener,transform); // x, y, theta 

      //If you want Every-Time Catching marker pose
      //getMarkerPose(listenerGoal, transformGoal, aruco_pub); // x_goal, y_goal, theta_goal 

      //Calculating distance
      x_diff = x_goal - x;
      y_diff = y_goal - y;
      rho = hypotf(x_diff, y_diff);

      //Reaching goal condition: reacch near goal pose + start charging
      if (rho - r > RHO_TLR && isCharging == false && isDetected == true && isFinished == false)
      // if (rho > RHO_TLR && isCharging == false && isDetected == true && isFinished == false)
      {
         //Rotation angles calculating
         float alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI; //to-goal rotation
         float beta = fmod((theta_goal - theta - alpha + M_PI), (2 * M_PI)) - M_PI;     //parrallel-to-goal rotation

         //Calculating velocities
         v = -abs(Kp_rho * rho); //Backward Direction
         w = Kp_alpha * alpha + Kp_beta * beta;

         //If you want clip velocities
         //clipVelocities(v, w);
         if(v>-vmin)
            v= -vmin;

         //Out-of-range >>> inverse velocity
         if (alpha > M_PI / 2 || alpha < -M_PI / 2)
            v = -v;
         
         applyVelocities(v, w, pub);   

         //ROS_INFO("v: %f W: %f RHO: %f RHOx: %f RHOy %f", v, w, rho, rho*cosf(theta), rho*sinf(theta));
         //ROS_INFO("RX: %f RY: %f Rtheta: %f RHO: %f", x, y, theta, rho);
         //ROS_INFO("alpha: %f beta: %f", alpha, beta);
      }
      //No Marker Situation:
      else if (isDetected == false)
      {
         ROS_ERROR("Marker Not Detected! Docking Faild!");
         ROS_INFO("Robot will go Back...");
         goBackToPoint(x_D0, y_D0, theta_D0, listener, transform, pub);
         ROS_INFO("Robot Reached P0");
         if(retry_num > 0)
         {
            ROS_INFO("Lets Try Again... The number of retries remaining: %i",--retry_num);
            docking_init(listener, transform, pub, listenerGoal, transformGoal, aruco_pub);
            isFinished = false;
         }
         else
         {
            ROS_ERROR("Exhaustion of Retries! Docking Faild!");
            result.success = false;
            as->setSucceeded(result);
            isFinished = true;
         }
      }
      //Robot Successfully Reached The Charger:
      else if (isCharging == true && isFinished == false)
      {
         applyVelocities(0, 0, pub);

         result.success = true;
         as->setSucceeded(result);
         ROS_INFO("Charger is Reached Successfully!");
         ROS_INFO("Start Charging...");
         ros::Duration(1).sleep();
         isFinished = true;
      }
      //Robot Reached The Marker Position But There Is No Charging Signal:
      else if(isFinished == false)
      {
         if (charging_time_out > 0 )
         {
            v = -vmin;
            w = -alpha/abs(alpha) * wmin;
            applyVelocities(v, w, pub);
            ROS_ERROR("Goal must be reached reached! Waiting for %i seconds...", charging_time_out--);
            ros::spinOnce();
            ros::Duration(1).sleep();
         }
         else
         {
            applyVelocities(0, 0, pub);
            ROS_ERROR("Error: charging_time_out! Docking Faild!");
            ros::Duration(1).sleep();
            ROS_ERROR("Robot will go Back...");
            goBackToPoint(x_D0, y_D0, theta_D0, listener, transform, pub);
            ROS_ERROR("Robot Reached P0!");
            if(retry_num > 0)
            {
               ROS_INFO("Lets Try Again... The number of retries remaining : %i",--retry_num);
               docking_init(listener, transform, pub, listenerGoal, transformGoal, aruco_pub);
               isFinished = false;
            }
            else
            {
               ROS_ERROR("Exhaustion of Retries! Docking Faild!");
               result.success = false;
               as->setSucceeded(result);
               isFinished = true;
            }
         }
      }
      //Process callbacks
      ros::spinOnce();
      //Delays untill it is time to send another message
      rate.sleep();
   }
   ROS_INFO("Executing Docking-Server done!");
}

void execute_rotate(const docking::DockingGoalConstPtr &goal, Server *as, ros::Publisher pub, ros::Rate rate){
   ROS_INFO("Rotate to Marker Saved Position...");
   
   tf::StampedTransform stamped_transform;
   tf::TransformListener listener;

   geometry_msgs::Twist cmd_msg;

   float rotation = 0;
   float rot_diff = 0;

   do{
      try{
         ros::Time now = ros::Time::now();
         listener.waitForTransform("map", "/base_link", now, ros::Duration(5.0));
         listener.lookupTransform("map", "/base_link", now, stamped_transform);
      }
      catch (tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
         }
      rotation = tf::getYaw(stamped_transform.getRotation());

      rot_diff = goal->goal_rotation - rotation;
      if (rot_diff > M_PI) rot_diff = rot_diff - 2*M_PI;
      if (rot_diff < -M_PI) rot_diff = rot_diff + 2*M_PI;

      // ROS_INFO("goal_rotation: %f , base_rotation: %f , rot_diff: %f", goal->goal_rotation, rotation, rot_diff);
      if (rot_diff >= 0){
         cmd_msg.angular.z = 0.3;
      }
      else{
         cmd_msg.angular.z = -0.3;
      }
      pub.publish(cmd_msg);

      rate.sleep();
   }
   while(abs(rot_diff) > 0.1);

   cmd_msg.angular.z = 0.0;
   pub.publish(cmd_msg);

   result.success = true;
   as->setSucceeded(result);
}

/************************** ***MAIN*** *****************************/

int main(int argc, char **argv)
{
   //Initializes ROS, and sets up a node
   ros::init(argc, argv, "docking");
   ros::NodeHandle nh;

   ros::Rate rate(ROS_FREQUENCY);

   ros::Publisher aruco_pub = nh.advertise<std_msgs::Bool>("detectState", 1);

   ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
   //ros::Subscriber sub = nh.subscribe("/robot_bumper", 1000, contactCallback);
   ros::Subscriber sub = nh.subscribe("/docked", 1000, contactCallback);

   Server server(nh, "docking", boost::bind(&execute, _1, &server, pub, rate, aruco_pub), false);
   server.start();
   Server rotate_server(nh, "rotate_action", boost::bind(&execute_rotate, _1, &rotate_server, pub, rate), false);
   rotate_server.start();
   ROS_INFO("Docking server is started, Waiting for client...");

   while (ros::ok())
   {
      ros::spinOnce();
      rate.sleep();
   }
   return 0;
}