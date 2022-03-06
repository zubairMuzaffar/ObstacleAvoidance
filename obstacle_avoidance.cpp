#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h" 
#include "sensor_msgs/PointCloud.h" 
#include  "geometry_msgs/PoseStamped.h"                            // informtion comes from rostopic list 

ros::Publisher cmd_pub;
ros::Publisher pcl_pub;
ros::Subscriber scan_sub; 
ros::Subscriber goal_sub; 
geometry_msgs::Twist velocity_command;

void GoalCallback ( const geometry_msgs::PoseStamped::ConstPtr& pose_msg) // check msg ?
{
    double x = pose_msg->pose.position.x;
    double y = pose_msg->pose.position.y;
    double z = pose_msg->pose.position.z;

    ROS_INFO("x: %f, y: %f, z: %f" , x, y, z);
}


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    std::vector<float> laser_ranges;


    laser_ranges = scan_in->ranges;
    ROS_INFO("Number of laser points: %d", laser_ranges.size());

    // Place your code here
    


   
    //pub.publish(velocity_command);

}


int main(int argc, char** argv)
{
    //Initializing ROS node
    ros::init(argc, argv, "obstacle_avoidance");		
    ros::NodeHandle n;
 
    scan_sub = n.subscribe("/scan",100, scanCallback);
    goal_sub = n.subscribe("/move_base_simple/goal",100, GoalCallback);

    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pcl_pub = n.advertise<sensor_msgs::PointCloud>("/", 10);

   
    ros::Duration time_between_ros_wakeups(0.1);

    while (ros::ok()) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }

    return 0;


}