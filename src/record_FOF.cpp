
/*  FOF_vicon_data_collection_and_pub.cpp
 * @brief Gathers all data related to Flow of Flow and
 *           publishes it as on message with a shared header.
 *           ** This version is specific to Vicon
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <object_avoidance_cpp/YawRateCmdMsg.h>
#include <object_avoidance_cpp/FOFFlowMsg.h>
#include <object_avoidance_cpp/FOFDTMsg.h>
#include <object_avoidance_cpp/FOFAllDataMsg.h>
#include <math.h>

// Vicon Pose Callback
geometry_msgs::PoseStamped vicon_pose;
void vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    vicon_pose = *msg;
//    ROS_INFO_THROTTLE(2,"Pose cb");
}

// Vicon Velocity Callback
geometry_msgs::TwistStamped vicon_vel;
void vicon_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    vicon_vel = *msg;
}

// FOF Optic Flow Callback
object_avoidance_cpp::FOFFlowMsg FOF_flow_data;
void FOF_flow_cb(const object_avoidance_cpp::FOFFlowMsg::ConstPtr& msg){
    FOF_flow_data = *msg;
}

// Velocity command callback
object_avoidance_cpp::YawRateCmdMsg cmd_vel;
void cmd_vel_cb(const object_avoidance_cpp::YawRateCmdMsg::ConstPtr& msg){
    cmd_vel = *msg;
}

// FR DT Data
object_avoidance_cpp::FOFDTMsg FOF_dt;
void FOF_dt_cb(const object_avoidance_cpp::FOFDTMsg::ConstPtr& msg){
    FOF_dt = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber vicon_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/flow330/pose", 10, vicon_pose_cb);
    ros::Subscriber vicon_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/vrpn_client_node/flow330/twist", 10, vicon_vel_cb);
    ros::Subscriber FOF_flow_sub = nh.subscribe<object_avoidance_cpp::FOFFlowMsg>
            ("/FOF_flow", 10, FOF_flow_cb);
    ros::Subscriber yaw_cmd_sub = nh.subscribe<object_avoidance_cpp::YawRateCmdMsg>
            ("/yaw_cmd", 10, cmd_vel_cb);
    ros::Subscriber FOF_dt_sub = nh.subscribe<object_avoidance_cpp::FOFDTMsg>
            ("/FOF_dt", 10, FOF_dt_cb);
    
    // Publishers
    ros::Publisher FOF_flow_pub = nh.advertise<object_avoidance_cpp::FOFFlowMsg>
            ("FOF_FlowOut", 10);
    ros::Publisher FOF_data_pub = nh.advertise<object_avoidance_cpp::FOFAllDataMsg>
            ("FOF_DataOut", 10);
    
    ros::Rate rate(60.0);

    // Start timer and choose initial setpoint type
    ros::Time last_request = ros::Time::now();

    // Create full data messages
    object_avoidance_cpp::FOFFlowMsg flow_msg;
    object_avoidance_cpp::FOFAllDataMsg data_msg; 

    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Create flow message
        flow_msg.header.stamp = ros::Time::now();
        
        flow_msg.LP_OF = FOF_flow_data.LP_OF;
        flow_msg.Qdot_RFOF = FOF_flow_data.Qdot_RFOF;
        flow_msg.Qdot_tang = FOF_flow_data.Qdot_tang;

        // Create the data message
        data_msg.header.stamp = flow_msg.header.stamp;

        data_msg.yaw_cmd = cmd_vel.yaw_rate_cmd;

        data_msg.x_pos = vicon_pose.pose.position.x;
        data_msg.y_pos = vicon_pose.pose.position.y;
        data_msg.z_pos = vicon_pose.pose.position.z;
        data_msg.x_orient = vicon_pose.pose.orientation.x;
        data_msg.y_orient = vicon_pose.pose.orientation.y;
        data_msg.z_orient = vicon_pose.pose.orientation.z;
        data_msg.w_orient = vicon_pose.pose.orientation.w;

        data_msg.vel_x = vicon_vel.twist.linear.x;
        data_msg.vel_y = vicon_vel.twist.linear.y;
        data_msg.vel_z = vicon_vel.twist.linear.z;
        data_msg.angular_vel_z = vicon_vel.twist.angular.z;

        data_msg.min_threshold = FOF_dt.min_threshold;
        data_msg.r_0 = FOF_dt.r_0;
        data_msg.d_0 = FOF_dt.d_0;

        FOF_flow_pub.publish(flow_msg);
        FOF_data_pub.publish(data_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
