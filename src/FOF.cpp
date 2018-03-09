/* @https://forum.odroid.com/viewtopic.php?f=94&t=24496&view=unreadfile FOF.cpp
 * @brief Flow of Flow processing and control command calulation
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <object_avoidance_cpp/YawRateCmdMsg.h>
#include <object_avoidance_cpp/RingsFlowMsg.h>
#include <object_avoidance_cpp/FOFFlowMsg.h>
#include <object_avoidance_cpp/FOFDTMsg.h>
#include <object_avoidance_cpp/FOFAllDataMsg.h>
#include <iostream>
using namespace std;

class SubscribeAndPublish{
private:
    ros::NodeHandle nh_;
    ros::Publisher flow_out_pub_;
    ros::Publisher yaw_cmd_pub_;
    ros::Publisher data_out_pub_;
 //   ros::Publisher dt_out_pub_;
    ros::Subscriber flow_sub;
    
    std::vector<float> Qdot_u;
    std::vector<float> Qdot_v;
    
    float dt, tau, k_0, alpha, c_psi, c_d;
    float mean_val, mean_sum, std_dev, r_0, d_0;
    float average_OF_tang[];
    float OF_tang_prev[60];
    float OF_tang[60]; 
    float R_FOF[60];
    float LP_OF[60]; 
    int num_rings, num_points; 
    float yaw_rate_cmd, old_yaw_rate_cmd;
    float min_threshold;
    int sign, index;
    float gamma_arr[60], gamma_new[60];
    bool debug;
    object_avoidance_cpp::YawRateCmdMsg yaw_rate_cmd_msg; 
    object_avoidance_cpp::RingsFlowMsg current_flow;
    object_avoidance_cpp::FOFFlowMsg flow_out_msg;
//    object_avoidance_cpp::FOFDTMsg dt_out_msg;
    object_avoidance_cpp::FOFAllDataMsg all_data_out_msg;

public:
    SubscribeAndPublish(){
        // Create the yaw rate command publisher
        debug = false;
        
        if (debug == true){
            flow_out_pub_ = nh_.advertise<object_avoidance_cpp::FOFFlowMsg>
                 ("/FOF_flow", 10);
//            dt_out_pub_ = nh_.advertise<object_avoidance_cpp::FOFDTMsg>
//                 ("/FOF_dt", 10);
        }
        yaw_cmd_pub_ = nh_.advertise<object_avoidance_cpp::YawRateCmdMsg>
             ("/yaw_cmd", 10);
        data_out_pub_ = nh_.advertise<object_avoidance_cpp::FOFAllDataMsg>
             ("/FOF_data_out", 20);
 
        dt = .1;
        tau = .75;
        alpha = dt/(tau+dt);
//        alpha = 0.0;
        k_0 = 1.5;
        c_psi = .25;
        c_d = .5;
        num_rings = 3;
        num_points = 60;
        old_yaw_rate_cmd = 0.0;

        //Initialize the gamma array
        for(int i = 0; i < num_points; i++){
           gamma_arr[i] = ((2*M_PI-.017)/num_points)*i;
           if (i < num_points/2) {
               gamma_new[i] = gamma_arr[i];
           }
           else {
               gamma_new[i] = -2*M_PI + gamma_arr[i];
           }
           R_FOF[i] = 0.0;
           LP_OF[i] = 0.0;
           OF_tang[i] = 0.0;
           OF_tang_prev[i] = 0.0;
           average_OF_tang[i] = 0.0;
        }
         
    }
    void flow_cb(const object_avoidance_cpp::RingsFlowMsg::ConstPtr& msg);
}; // End of class SubscribeAndPublish

geometry_msgs::PoseStamped vicon_pose;
void vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    vicon_pose = *msg;
    ROS_INFO_THROTTLE(10, "POSE CB");
}

geometry_msgs::TwistStamped vicon_vel;
void vicon_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    vicon_vel = *msg;
}

// RCin callback
int Arr[12];
void RCin_cb(const mavros_msgs::RCIn::ConstPtr& msg){
    for(int i = 0; i < 12; i++){
        Arr[i] = msg->channels[i];
    }
}


void SubscribeAndPublish::flow_cb(const object_avoidance_cpp::RingsFlowMsg::ConstPtr& msg)
{

    current_flow = *msg;
    Qdot_u = current_flow.Qdot_u;
    Qdot_v = current_flow.Qdot_v;

    for(int i = 0; i < num_points; i++){
        average_OF_tang[i] = 0.0;
    }
 
    // Initialize all variables
    mean_val = 0.0;
    mean_sum = 0.0;
    r_0 = 0.0;
    d_0 = 0.0;
    yaw_rate_cmd = 0.0;
    min_threshold = 0.0;
    index = 0;
    // Compute average ring flow
    for(int r = 0; r < num_rings; r++){
        for(int i = 0; i < num_points; i++){
            index = r*num_points + i;
            average_OF_tang[i] = average_OF_tang[i] + (Qdot_u[index]*cos(gamma_arr[i])+Qdot_v[index]*sin(gamma_arr[i]));
        }
    }

    // Compute the spatial average
    for(int i = 0; i < num_points; i++){
        OF_tang[i] = average_OF_tang[i]/num_rings; 
        OF_tang_prev[i] = LP_OF[i];
    }
        
        
    // Compute the FOF control
    // // // // // // // // //

    // Run the LPF
    for(int i = 0; i < num_points; i++){
        LP_OF[i] = alpha*OF_tang[i] + (1 - alpha)*OF_tang_prev[i];   
    }

    // Final FOF Calc
    for(int i = 0; i < (num_points-1); i++){
        R_FOF[i] = LP_OF[i]*OF_tang[i+1] - OF_tang[i]*LP_OF[i+1];
    }

    R_FOF[num_points-1] = 0.0;
    mean_sum = 0.0;
    std_dev = 0.0;
    // Dynamic Threshold
    for(int i = 0; i < num_points; i++){
        mean_sum += R_FOF[i];
    }
    mean_val = mean_sum / num_points;
        
    for(int i = 0; i < num_points; i++){
        std_dev += pow((R_FOF[i] - mean_val), 2);
    }
    std_dev /= num_points;
    std_dev = pow(std_dev, 0.5);
    min_threshold = 3*std_dev;

    // Find the max val
    for(int i = 0; i < num_points; i++){
        if(abs(R_FOF[i]) > d_0){
            d_0 = abs(R_FOF[i]);
            r_0 = gamma_new[i];
        }
    }
        
    if(r_0 > 0){
        sign = 1;
    } else {
        sign = -1;
    }
    if(d_0 > min_threshold){
        yaw_rate_cmd = k_0*sign*exp(-c_psi*abs(r_0))*exp(-c_d/abs(d_0));
    } else {
        yaw_rate_cmd = 0.0;
    }

    yaw_rate_cmd_msg.yaw_rate_cmd = yaw_rate_cmd;
    yaw_cmd_pub_.publish(yaw_rate_cmd_msg);    

    all_data_out_msg.header.stamp = ros::Time::now();
    all_data_out_msg.yaw_cmd = yaw_rate_cmd_msg.yaw_rate_cmd;
    all_data_out_msg.d_0 = d_0;
    all_data_out_msg.r_0 = r_0;
    all_data_out_msg.sigma = std_dev;

    all_data_out_msg.x_pos = vicon_pose.pose.position.x;
    all_data_out_msg.y_pos = vicon_pose.pose.position.y;
    all_data_out_msg.z_pos = vicon_pose.pose.position.z;
    all_data_out_msg.x_orient = vicon_pose.pose.orientation.x;
    all_data_out_msg.y_orient = vicon_pose.pose.orientation.y;
    all_data_out_msg.z_orient = vicon_pose.pose.orientation.z;
    all_data_out_msg.w_orient = vicon_pose.pose.orientation.w;


    all_data_out_msg.vel_x = vicon_vel.twist.linear.x;
    all_data_out_msg.vel_y = vicon_vel.twist.linear.y;
    all_data_out_msg.vel_z = vicon_vel.twist.linear.z;
    all_data_out_msg.angular_vel_z = vicon_vel.twist.angular.z;

    for(int i = 0; i < num_points; i++){
        //all_data_out_msg.Qdot_SF.push_back(R_FOF[i]);
        all_data_out_msg.Qdot_SF.push_back(OF_tang[i]);
    }

    all_data_out_msg.Dswitch = Arr[4];

    data_out_pub_.publish(all_data_out_msg);
    all_data_out_msg.Qdot_SF.clear();


    if(debug == true){
        for(int i = 0; i < num_points; i++){
            flow_out_msg.LP_OF.push_back(LP_OF[i]);
            flow_out_msg.Qdot_RFOF.push_back(R_FOF[i]);
            flow_out_msg.Qdot_tang.push_back(OF_tang[i]);
        }
        flow_out_pub_.publish(flow_out_msg);
        flow_out_msg.LP_OF.clear();
        flow_out_msg.Qdot_RFOF.clear();
        flow_out_msg.Qdot_tang.clear();

  //      dt_out_msg.min_threshold = min_threshold;
  //      dt_out_msg.r_0 = r_0;
  //      dt_out_msg.d_0 = d_0;
  //      dt_out_pub_.publish(dt_out_msg);    
    }

    // End of flow calc  
}

int main(int argc, char **argv){
    ros::init(argc, argv, "FOF_node");
    ros::NodeHandle n;
    SubscribeAndPublish FlowSubPubObject;
    ros::Subscriber sub = n.subscribe("/flow_rings",1000, &SubscribeAndPublish::flow_cb, &FlowSubPubObject);
    ros::Subscriber vicon_pose_sub = n.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/flow330/pose", 10, vicon_pose_cb);
    ros::Subscriber vicon_vel_sub = n.subscribe<geometry_msgs::TwistStamped>
            ("/vrpn_client_node/flow330/twist", 10, vicon_vel_cb);
    ros::Subscriber rcin_sub = n.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 10, RCin_cb);

    while(ros::ok()){
        ros::spin();
    }
    return 0; // Exit main loop     
}

