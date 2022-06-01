#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h" 
#include "sensor_msgs/PointCloud.h" 
#include  "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_listener.h"                        
#include <laser_geometry/laser_geometry.h>
#include <math.h>
#include <visualization_msgs/Marker.h>


ros::Publisher cmd_pub;
ros::Publisher pcl_pub;
ros::Subscriber scan_sub; 
ros::Subscriber goal_sub; 
ros::Subscriber vel_sub;
ros::Publisher marker_pub; 
ros::Publisher velocity_pub;
laser_geometry::LaserProjection projector_;
geometry_msgs::PoseStamped received_goal;
bool goal_available = false;
double resultant_force;


void publishArrowMarker(int id, std_msgs::ColorRGBA color, double arrowX, double arrowY)
{
    uint32_t shape = visualization_msgs::Marker::ARROW;
    geometry_msgs::Point arrowBegin, arrowEnd;
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.03;
    marker.scale.y = 0.1;
    marker.scale.z = 0.3;

    // Determine start and end points of the arrow (start point is always 0)
    arrowBegin.x = 0;
    arrowBegin.y = 0;
    arrowBegin.z = 0;

    arrowEnd.x = arrowX;
    arrowEnd.y = arrowY;
    arrowEnd.z = 0;

    marker.points.push_back(arrowBegin);
    marker.points.push_back(arrowEnd);

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
}


geometry_msgs::Twist VFFCalculation(sensor_msgs::PointCloud laser_pcl, double goalX, double goalY)
{
	double sum_x_component=0, sum_y_component=0;
    double Fr = 0.05 ;
    double Fa = 2.0 ;
    double x_component_repulsiveforceVector, y_component_repulsiveforceVector;
    
    // Calculate repulsive force

    for ( int i=0; i< 360 ; i++) {
        x_component_repulsiveforceVector  = - Fr *  laser_pcl.points[i].x / (pow(laser_pcl.points[i].x,2) + pow(laser_pcl.points[i].y,2)) ;   

		    sum_x_component += x_component_repulsiveforceVector;
	 
        y_component_repulsiveforceVector  = - Fr * laser_pcl.points[i].y / (pow(laser_pcl.points[i].x,2) + pow(laser_pcl.points[i].y,2));

        sum_y_component += y_component_repulsiveforceVector;

          ROS_INFO("forces : (%d, %d)", sum_x_component , sum_y_component);
    }


    // Calculate attractive force

    double x_component_attractiveForce, y_component_attractiveForce;

    x_component_attractiveForce = Fa * goalX / sqrt(pow(goalX ,2) + pow(goalY,2)); 
    y_component_attractiveForce  = Fa * goalY / sqrt(pow(goalX ,2) + pow(goalY,2)); 


    // Calculate resultant force

    double x_component_resultantForce, y_component_resultantForce;

    x_component_resultantForce = sum_x_component + x_component_attractiveForce;
    y_component_resultantForce = sum_y_component + y_component_attractiveForce;

  

    

    // Publish visualization marker for repulsive force

    std_msgs::ColorRGBA color;
    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 0.0f;
    color.a = 1.0f;

    publishArrowMarker(0, color, sum_x_component, sum_y_component);


    // Publish visualization marker for attractive force

    color.r = 0.0f;
    color.g = 0.5f;
    color.b = 0.0f;

    publishArrowMarker(1, color, x_component_attractiveForce, y_component_attractiveForce);

    // Publish visualization marker for attractive force

    color.r = 0.0f;
    color.g = 0.0f;
    color.b = 1.0f;

    publishArrowMarker(2, color, x_component_resultantForce, y_component_resultantForce);
   

    // Calculate velocity command from the virtual force

    geometry_msgs::Twist vel_command;

    // TODO: calculation
    float resultant_force_Angle;

    resultant_force_Angle = atan2f( y_component_resultantForce, x_component_resultantForce );

    float angularGain = 1.5;    

    //   Linear: small constant velocity value
    //   Angular: proportional with the resultant force angle
    if (sqrt(pow(goalX ,2) + pow(goalY,2)) < 0.05) 
    vel_command.linear.x = 0.0;   
    else
    {
    vel_command.linear.x = 0.15;    
    }
    //}
    //velocity_command.linear.x = sqrt(pow( x_component_resultantForce,2) + pow( y_component_resultantForce,2));
    
    vel_command.angular.z = angularGain * resultant_force_Angle;   // angular velocity component
    
    vel_command.linear.y = 0;
    vel_command.linear.z = 0;
    vel_command.angular.x = 0;
    vel_command.angular.y = 0;

    return vel_command; 

}


void GoalCallback ( const geometry_msgs::PoseStamped::ConstPtr& pose_msg) 
{
    
    double x = pose_msg->pose.position.x;
    double y = pose_msg->pose.position.y;
    double z = pose_msg->pose.position.z;

    ROS_INFO("Goal received:  x: %f, y: %f, z: %f" , x, y, z );

    received_goal = *pose_msg;

    goal_available = true;

}



void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    static tf::TransformListener listener_;
    static tf::TransformListener goal_tf_listener_;

    std::vector<float> laser_ranges;
    laser_ranges = scan_in->ranges;

    //ROS_INFO("Number of laser points: %d", laser_ranges.size());


    // Transforming Laserscan to PointCloud

    if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
    }

    sensor_msgs::PointCloud cloud;
     projector_.transformLaserScanToPointCloud("base_link",*scan_in,
          cloud,listener_);

    // Do something with cloud.
  
    //ROS_INFO("Number of cloud points: %d", laser_ranges.size());
    //ROS_INFO("First point in the cloud: (%g, %g)", cloud.points[0].x, cloud.points[0].y);

    //pcl_pub.publish(cloud);
    ROS_INFO("RUNNING");

    if(!goal_available) return;

    
    // Transform the last received goal to the base_link frame
    
    if(!goal_tf_listener_.waitForTransform(
            received_goal.header.frame_id,
            "base_link",
            ros::Time(0),
            ros::Duration(1.0))){
        return;
    }

    tf::StampedTransform goal_transform;
    goal_tf_listener_.lookupTransform("base_link", received_goal.header.frame_id, ros::Time(0), goal_transform);

    tf::Vector3 original_goal(received_goal.pose.position.x, received_goal.pose.position.y, received_goal.pose.position.z);
    
    tf::Vector3 transformed_goal = goal_transform * original_goal;

    // Calculate velocity command
    geometry_msgs::Twist velocity_command;
    velocity_command = VFFCalculation(cloud, transformed_goal.x(), transformed_goal.y());

    // Publish the velocity command
     velocity_pub.publish(velocity_command);
}


int main(int argc, char** argv)
{
    //Initializing ROS node
    ros::init(argc, argv, "obstacle_avoidance");		
    ros::NodeHandle n;
    ros::Duration time_between_ros_wakeups(0.1);
    scan_sub = n.subscribe("/scan",100,scanCallback);
    goal_sub = n.subscribe("/move_base_simple/goal",100, GoalCallback);

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    while (ros::ok())
    {
      ros::spinOnce();
      time_between_ros_wakeups.sleep();
    }
    return 0;
}
