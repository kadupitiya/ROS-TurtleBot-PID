
/*
Used for teaching controller design for Turtlebot3
Lantao Liu
ISE, Indiana University
*/

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <sstream>


// global vars
tf::Point Odom_pos;	//odometry position (x, y, z)
double Odom_yaw;	//odometry orientation (yaw)
double Odom_v, Odom_w;	//odometry linear and angular speeds

// ROS Topic Publishers
ros::Publisher cmd_vel_pub;
ros::Publisher marker_pub;

// ROS Topic Subscribers
ros::Subscriber odom_sub;


/*
 * Callback function for odometry msg, used for odom subscriber
 */
void odomCallback(const nav_msgs::Odometry odom_msg)
{
 /* upon "hearing" odom msg, retrieve its position and orientation (yaw) information. 
  * tf utility functions can be found here: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
  * odom data structure definition is here: https://mirror.umd.edu/roswiki/doc/diamondback/api/nav_msgs/html/msg/Odometry.html
  */
  tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);
  Odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);

  //update observed linear and angular speeds (real speeds published from simulation)
  Odom_v = odom_msg.twist.twist.linear.x;
  Odom_w = odom_msg.twist.twist.angular.z;

  //display on terminal screen
  ROS_INFO("Position: (%f, %f); Yaw: %f", Odom_pos.x(), Odom_pos.y(), Odom_yaw);
}


/*
 * display function that draws a circular lane in Rviz, with function  (x+0.5)^2 + (y-1)^2 = 4^2 
 */
void displayLane(void)
{
    static visualization_msgs::Marker path;
    path.type = visualization_msgs::Marker::LINE_STRIP;

    path.header.frame_id = "odom";  //NOTE: this should be "paired" to the fixed frame id entry in Rviz, the default setting in Rviz for tb3-fake is "odom". Keep this line as is if you don't have an issue.
    path.header.stamp = ros::Time::now();
    path.ns = "odom";
    path.id = 0;
    path.action = visualization_msgs::Marker::ADD; // use line marker
    path.lifetime = ros::Duration();

    // path line strip is blue
    path.color.b = 1.0;
    path.color.a = 1.0;

    path.scale.x = 0.02;
    path.pose.orientation.w = 1.0;

    int num_slice2 = 50;               // divide a circle into segments
    static int slice_index2 = 0;
    while(path.points.size() <= num_slice2)  
    {
      geometry_msgs::Point p;

      float angle = slice_index2*2*M_PI/num_slice2;
      slice_index2 ++ ;
      p.x = 4 * cos(angle) - 0.5;       //some random circular trajectory, with radius 4, and offset (-0.5, 1, 0)
      p.y = 4 * sin(angle) + 1.0;
      p.z = 0;

      path.points.push_back(p);         //for drawing path, which is line strip type
    }

    marker_pub.publish(path);
}


/*
 * main function 
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "control");
  ros::NodeHandle n("~");
  tf::TransformListener m_listener;
  tf::StampedTransform transform;

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  odom_sub    = n.subscribe("odom", 10, odomCallback);

  ros::Rate loop_rate(10); // ros spins 10 frames per second

  //we use geometry_msgs::twist to specify linear and angular speeds (v, w) which also denote our control inputs to pass to turtlebot
  geometry_msgs::Twist tw_msg;

  int frame_count = 0;
  while (ros::ok())
  {
    displayLane();
    ROS_INFO("frame %d", frame_count);

/*

YOUR CONTROL STRETEGY HERE
you need to define vehicle dynamics first (dubins car model)
after you computed your control input (here angular speed) w, pass w value to "tw_msg.angular.z" below

*/

    //for linear speed, we only use the first component of 3D linear velocity "linear.x" to represent the speed "v" 
    tw_msg.linear.x = 0.3;
    //for angular speed, we only use the third component of 3D angular velocity "angular.z" to represent the speed "w" (in radian)
    tw_msg.angular.z = 0.25;

    //publish this message to the robot
    cmd_vel_pub.publish(tw_msg);

    frame_count ++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}


