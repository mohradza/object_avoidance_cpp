/* @file FR.cpp
w of Flow processing and control command calulation
 */


#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float32.h>
#include <object_avoidance_cpp/YawRateCmdMsg.h>
#include <object_avoidance_cpp/RingsFlowMsg.h>
#include <object_avoidance_cpp/FRFlowMsg.h>
#include <object_avoidance_cpp/FRHarmonicsMsg.h>
#include <object_avoidance_cpp/FRDTMsg.h>
#include <object_avoidance_cpp/FRAllDataMsg.h>
#include <iostream>
#include <numeric>
//#define PI = 3.1415

using namespace std;


class SubscribeAndPublish {
private:
    ros::NodeHandle nh_;
    ros::Publisher yaw_cmd_pub_;
    ros::Publisher flow_out_pub_;
    ros::Publisher harm_out_pub_;
    ros::Publisher dt_out_pub_;
    ros::Publisher data_out_pub_;
    ros::Subscriber flow_sub;


    std::vector<float> Qdot_u;
    std::vector<float> Qdot_v;

    float k_0, c_psi, c_d, dg;
    float a_0, a[4], b[4], a_sum[60], b_sum[60];
    float mean_val, mean_sum, std_dev, r_0, d_0;
    float average_OF_tang[];
    float OF_tang[60], Qdot_WF[60], Qdot_SF[60];
    
    int num_rings, num_points, num_harm;
    int index, index_max;
    int sign, pixel_scale;
    float yaw_rate_cmd, old_yaw_rate_cmd, min_threshold;
    float gamma_arr[60], gamma_new[60];
    bool debug;

    object_avoidance_cpp::YawRateCmdMsg yaw_rate_cmd_msg;
    object_avoidance_cpp::RingsFlowMsg current_flow;
    object_avoidance_cpp::FRFlowMsg flow_out_msg;
    object_avoidance_cpp::FRHarmonicsMsg harm_out_msg;
    object_avoidance_cpp::FRDTMsg dt_out_msg;
    object_avoidance_cpp::FRAllDataMsg all_data_out_msg;

public:
    SubscribeAndPublish() {
        // ROS_INFO("Object Created");
        
        debug = false;

        if(debug == true){
            // Create the flow message publisher
            flow_out_pub_ = nh_.advertise<object_avoidance_cpp::FRFlowMsg>
                    ("/FR_flow",10);
            // Create the Harmonics message publisher
            harm_out_pub_ = nh_.advertise<object_avoidance_cpp::FRHarmonicsMsg>
                    ("/FR_harmonics",10);
            // Create the dynamic threshold message publisher
            dt_out_pub_ = nh_.advertise<object_avoidance_cpp::FRDTMsg>
                    ("/FR_dt",10);
        } 

        // Create the yaw rate command publisher
        yaw_cmd_pub_ = nh_.advertise<object_avoidance_cpp::YawRateCmdMsg>
                ("/yaw_cmd", 10);
        data_out_pub_ = nh_.advertise<object_avoidance_cpp::FRAllDataMsg>
                ("/FR_data_out", 20);
  
        // initialize static variables
        num_points = 60;
        num_rings = 3;
        num_harm = 2;
        k_0 = 1.5;
        c_psi = .25;
        c_d = .5;
        old_yaw_rate_cmd = 0.0;
        
        //Initialize the gamma array
        for (int i = 0; i < num_points; i++) {
            gamma_arr[i] = (2*M_PI-.017)/num_points*i;
            if (i < (num_points/2)){
                gamma_new[i] = gamma_arr[i];
            } else {
                gamma_new[i] = -2*M_PI + gamma_arr[i];
            }
            OF_tang[i] = 0.0;
            average_OF_tang[i] = 0.0;
            Qdot_WF[i] = 0.0;
            Qdot_SF[i] = 0.0;
            if (i < 4){
                 a[i] = 0.0;
                 b[i] = 0.0;
            }
        }

        dg = gamma_arr[2] - gamma_arr[1];
    }
    void flow_cb(const object_avoidance_cpp::RingsFlowMsg::ConstPtr& msg);
}; // End of class SubscribeAndPublish


geometry_msgs::PoseStamped vicon_pose;
void vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    vicon_pose = *msg;
    ROS_INFO_THROTTLE(10,"POSE CB");
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
        Qdot_WF[i] = 0.0;
        a_sum[i] = 0.0;
        b_sum[i] = 0.0;
        if(i < num_harm){
            a[i] = 0.0;
            b[i] = 0.0;
        }
    }   
    // Parameters
    a_0 = 0.0;
    mean_val = 0.0;
    mean_sum = 0.0;
    std_dev = 0.0;
    r_0 = 0.0;
    d_0 = 0.0;
    index_max = 0;
    index = 0;
    yaw_rate_cmd = 0.0;
    min_threshold = 0.0;
    sign = 0;
     
    // Compute average ring flow
    for (int r = 0; r < num_rings; r++) {
        for (int i = 0; i < num_points; i++) {
            index = r * num_points + i;
            average_OF_tang[i] = average_OF_tang[i] + (Qdot_u[index]*cos(gamma_arr[i]) + Qdot_v[index]*sin(gamma_arr[i]));
        }
    }

    // Compute the spatial average
    for (int i = 0; i < num_points; i++){
        OF_tang[i] = average_OF_tang[i]/num_rings;
    }

    // Compute the FR control
    // // // // // // // // //
    
    // Compute coefficients for fourier residuals

    for (int i = 0; i < num_points; i++){
        a_0 = a_0 + cos(0 * gamma_arr[i]) * OF_tang[i];
    }

    for (int n = 0; n < num_harm; n++) {
        for (int i = 0; i < num_points; i++) {
           // a_sum[i] = cos((n + 1) * gamma_arr[i]) * OF_tang[i];
           // b_sum[i] = sin((n + 1) * gamma_arr[i]) * OF_tang[i];
           a[n] = a[n] +  cos((n+1)*gamma_arr[i])*OF_tang[i];
           b[n] = b[n] +  sin((n+1)*gamma_arr[i])*OF_tang[i];
        }
       //a[n] = std::accumulate(a_sum, a_sum + num_points, 0.0);
       //b[n] = std::accumulate(b_sum, b_sum + num_points, 0.0);
    }

    // Convert coefficients to be in terms of degrees
    a_0 = (a_0*dg)/M_PI;
    for (int n = 0; n < num_harm; n++) {
        a[n] = (a[n]*dg)/M_PI;
        b[n] = (b[n]*dg)/M_PI;
    }
    // Compute Qdot_WF
    for (int i = 0; i < num_points; i++) {
        for (int n = 0; n < 4; n++) {
            Qdot_WF[i] = Qdot_WF[i] +  a[n] * cos((n + 1) * gamma_arr[i]) + b[n] * sin((n + 1) * gamma_arr[i]);
        }
        Qdot_WF[i] = Qdot_WF[i] + a_0 / 2.0;
    }

    // Remove the WF signal from the full measured OF
    for (int i = 0; i < num_points; i++) {
        Qdot_SF[i] = OF_tang[i] - Qdot_WF[i];
    }
    
    // Calculate Dynamic Thresh
    for (int i = 0; i < num_points; i++){
        mean_sum += Qdot_SF[i];
    }
    mean_val = mean_sum / num_points;
    for(int i = 0; i < num_points; i++){
        std_dev += pow((Qdot_SF[i] - mean_val), 2);
    }
    std_dev = pow(std_dev / num_points, 0.5);
    min_threshold=3*std_dev;

    // Extract r_0 and d_0 from process OF signal based on max of Qdot_SF
    for (int i = 0; i < num_points; i++) {
        if (abs(Qdot_SF[i]) > abs(Qdot_SF[index_max])) {
            index_max = i;
        }
    }
    
    d_0 = abs(Qdot_SF[index_max]);
    r_0 = gamma_new[index_max];
    // Calculate the control signal      
    if (d_0 > min_threshold) {
        if (r_0 > 0) sign = 1;
        if (r_0 < 0) sign = -1;
        yaw_rate_cmd = k_0 * sign * exp(-c_psi * abs(r_0)) * exp(-c_d / abs(d_0));
    } else {
        yaw_rate_cmd = 0.0;
    }
/*
    if (abs(yaw_rate_cmd - old_yaw_rate_cmd) > .4) {
        yaw_rate_cmd = old_yaw_rate_cmd;
    } else {
        yaw_rate_cmd = yaw_rate_cmd;
        old_yaw_rate_cmd = yaw_rate_cmd;
    }
*/
    yaw_rate_cmd_msg.yaw_rate_cmd = yaw_rate_cmd;
    yaw_cmd_pub_.publish(yaw_rate_cmd_msg);

    // Create the Data message
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
        all_data_out_msg.Qdot_SF.push_back(Qdot_SF[i]);
    }

    all_data_out_msg.Dswitch = Arr[4]; 
    
    data_out_pub_.publish(all_data_out_msg);
    all_data_out_msg.Qdot_SF.clear();
    
    // PUBLISHING TOO MUCH WILL CAUSE THE
    // SYSTEM TO LAG, AFFECTING OF    
    if(debug == true){
    // Create messages for publishing
        for(int i= 0; i < num_points; i++){ 
//          flow_out_msg.Qdot_WF.push_back(Qdot_WF[i]);
//          flow_out_msg.Qdot_SF.push_back(Qdot_SF[i]);
//          flow_out_msg.Qdot_meas.push_back(average_OF_tang[i]);
            flow_out_msg.Qdot_tang.push_back(OF_tang[i]);
        }

        // Publish Data
        flow_out_pub_.publish(flow_out_msg);
//      flow_out_msg.Qdot_WF.clear();
//      flow_out_msg.Qdot_SF.clear();
//      flow_out_msg.Qdot_meas.clear();
        flow_out_msg.Qdot_tang.clear();

        // Publish the Harmonics
        harm_out_msg.a_0 = a_0;
        for(int i = 0; i < 4; i++){
            harm_out_msg.a.push_back(a[i]);
            harm_out_msg.b.push_back(b[i]);
        }
        harm_out_pub_.publish(harm_out_msg);
        harm_out_msg.a.clear();
        harm_out_msg.b.clear();
    
        // Publish the DT data
        dt_out_msg.min_threshold = min_threshold;
        dt_out_msg.r_0 = r_0;
        dt_out_msg.d_0 = d_0;
        dt_out_pub_.publish(dt_out_msg);
    } // End of debug publisher       
} // end of flow callback function

int main(int argc, char **argv) {
    ros::init(argc, argv, "FR");
    ros::NodeHandle n;

    SubscribeAndPublish FlowSubPubObject;

    ros::Subscriber flow_sub = n.subscribe("/flow_rings", 1000, &SubscribeAndPublish::flow_cb, &FlowSubPubObject);
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

