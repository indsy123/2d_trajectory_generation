#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <string> 
#include <fstream>
#include <sstream>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>


using namespace std;
using namespace Eigen;
using std::ofstream;
std::ofstream outdata;

class controller
{
    public: 
      controller(ros::NodeHandle* nodehandle)
      {
        ROS_INFO("in class constructor of controller");
        getParamters();
        initializeSubscribers(); 
        initializePublishers();

        curr_pose = {0,0,0,0,0,0,0,0};
        global_counter = 0;
      };
      void cmd_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
      void odometry_callback(const nav_msgs::Odometry::ConstPtr& _msg);      
    private: 
      ros::NodeHandle nh_;
      void initializeSubscribers(); 
      void initializePublishers();
      void getParamters();

      ros::Subscriber odom_sub, trajectory_sub;
      ros::Publisher cmd_pub; 

      double epsilon, k;
      string pose_topic, trajectory_topic;

      std::vector <double> curr_pose;
      std::vector <double> xdes, ydes, vxdes, vydes, thetades, wdes, traj_global_time;
      int global_counter;
      bool TrajectoryPublished = false;

};

void controller::getParamters()
{
  // get all the parameters from launch file
  nh_.param<double>("epsilon", epsilon, 0.15);    
  nh_.param<double>("k", k, 1.25);
  nh_.param<std::string>("pose_topic", pose_topic, "/odometry_topic");    
  nh_.param<std::string>("trajectory_topic", trajectory_topic, "/desired_trajectory_topic");
}

void controller::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");

  odom_sub = nh_.subscribe(pose_topic.c_str(), 1, &controller::odometry_callback, this);  
  trajectory_sub = nh_.subscribe(trajectory_topic.c_str(), 1, &controller::cmd_callback, this);  
}

void controller::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  bool latch;
  cmd_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}


void controller::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    geometry_msgs::Twist cmd_vel;
    // store odometry values in curr_pose array
    double current_time = msg->header.stamp.toSec();

    curr_pose[0] = msg->pose.pose.position.x;
    curr_pose[3] = msg->pose.pose.position.y;      
    curr_pose[1] = msg->twist.twist.linear.x;     
    curr_pose[4] = msg->twist.twist.linear.y; 
    curr_pose[6] = tf2::getYaw(msg->pose.pose.orientation);
    curr_pose[7] = msg->twist.twist.angular.z;

    if (TrajectoryPublished == true)
    {

      auto it = std::lower_bound(traj_global_time.begin(), traj_global_time.end(), current_time);
      int j = std::distance(traj_global_time.begin(), it);
      if (j == traj_global_time.size()){j = j-1;}

      double Zxdot_current, Zydot_current, Zx_current, Zy_current, Zx_ref, Zy_ref, Zxdot_ref, Zydot_ref, u1, u2, v, vx, vy, w;

      Zx_current = curr_pose[0] + epsilon * cos(curr_pose[6]);
      Zy_current = curr_pose[3] + epsilon * sin(curr_pose[6]);

      Zxdot_current = curr_pose[1] - epsilon * curr_pose[7] * sin(curr_pose[6]);
      Zxdot_current = curr_pose[4] + epsilon * curr_pose[7] * cos(curr_pose[6]);

      Zx_ref = xdes[j] + epsilon * cos(thetades[j]);
      Zy_ref = ydes[j] + epsilon * sin(thetades[j]);

      Zxdot_ref = vxdes[j] - epsilon * wdes[j] * sin(thetades[j]);
      Zydot_ref = vydes[j] + epsilon * wdes[j] * cos(thetades[j]);

      u1 = Zxdot_ref - k * (Zx_current - Zx_ref);
      u2 = Zydot_ref - k * (Zy_current - Zy_ref);

      v = u1 * cos(curr_pose[6]) + u2 * sin(curr_pose[6]);
      vx = v * cos(curr_pose[6]);
      vy = v * sin(curr_pose[6]);

      w = (-u1 * sin(curr_pose[6]) + u2 * cos(curr_pose[6]))/epsilon;

      cmd_vel.linear.x = v; 
      //cmd_vel.linear.y = vy;
      cmd_vel.angular.z = w;

      cmd_pub.publish(cmd_vel);

      outdata << curr_pose[0] << ", " << curr_pose[3] << ", " << curr_pose[1] << "," << curr_pose[4] << ", " <<   curr_pose[6] << ", " << xdes[j]
              << ", " << ydes[j] << ", " << vxdes[j] << ","<< vydes[j] << "," << thetades[j] << endl;
    }


}

void controller::cmd_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) 
{

    for (int i = 0; i < msg->points.size(); i++)
    {
      //int j = global_counter *  msg->points.size() + i;
      double tt = msg->points[i].time_from_start.toSec();
      //cout << "ttttt:" << tt<< endl;
      traj_global_time.push_back(tt);
      xdes.push_back(msg->points[i].transforms[0].translation.x);
      ydes.push_back(msg->points[i].transforms[0].translation.y);
      vxdes.push_back(msg->points[i].velocities[0].linear.x);
      vydes.push_back(msg->points[i].velocities[0].linear.y);
      double thdes = tf2::getYaw(msg->points[i].transforms[0].rotation);
      thetades.push_back(thdes);
      wdes.push_back(msg->points[i].velocities[0].angular.z);

      if (traj_global_time.size() >= 5000)
      {
        traj_global_time.erase(traj_global_time.begin());
        xdes.erase(xdes.begin());
        ydes.erase(ydes.begin());
        vxdes.erase(vxdes.begin());
        vydes.erase(vydes.begin());
        thetades.erase(thetades.begin());
        wdes.erase(wdes.begin());
      }

    }
    
    TrajectoryPublished = true;

}

int main(int argc, char **argv)
{

    // Launch controller node
    ros::init(argc, argv, "trajectory_controller_node");
    ros::NodeHandle nh("~");

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"gain - %d-%m-%Y %I:%M:%S.",now);
    outdata.open(buffer);

    // Create subscribers
    controller c(&nh);

    ROS_INFO("done...spinning to ros");
    ros::spin();

    return 0;
}