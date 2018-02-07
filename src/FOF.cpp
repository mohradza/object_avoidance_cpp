/* @file FOF.cpp
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
#include <iostream>
using namespace std;

class SubscribeAndPublish{
private:
    ros::NodeHandle nh_;
    ros::Publisher flow_out_pub_;
    ros::Publisher yaw_cmd_pub_;
    ros::Publisher dt_out_pub_;
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
    int num_rings; 
    float yaw_rate_cmd;
    float min_threshold;
    object_avoidance_cpp::YawRateCmdMsg yaw_rate_cmd_msg; 
    int sign, index;
    float gamma_arr[60], gamma_new[60];
    object_avoidance_cpp::RingsFlowMsg current_flow;
    object_avoidance_cpp::FOFFlowMsg flow_out_msg;
    object_avoidance_cpp::FOFDTMsg dt_out_msg;
    bool debug;
public:
    SubscribeAndPublish(){
        // Create the yaw rate command publisher
        ROS_INFO("Object Created");
        yaw_cmd_pub_ = nh_.advertise<object_avoidance_cpp::YawRateCmdMsg>
             ("/yaw_cmd", 10);
        flow_out_pub_ = nh_.advertise<object_avoidance_cpp::FOFFlowMsg>
             ("/FOF_flow", 10);
        dt_out_pub_ = nh_.advertise<object_avoidance_cpp::FOFDTMsg>
             ("/FOF_dt", 10);
    
        dt = .1;
        tau = .75;
        alpha = dt/(tau+dt);
//        alpha = 0.0;
        k_0 = .3;
        c_psi = .1;
        c_d = .25;
        num_rings = 5;
        debug = true;

        //Initialize the gamma array
        for(int i = 0; i < 60; i++){
           gamma_arr[i] = ((2*M_PI-.017)/60)*i;
           if (i < 30) {
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

void SubscribeAndPublish::flow_cb(const object_avoidance_cpp::RingsFlowMsg::ConstPtr& msg)
{

        current_flow = *msg;
        Qdot_u = current_flow.Qdot_u;
        Qdot_v = current_flow.Qdot_v;

        for(int i = 0; i < 60; i++){
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
        for(int r = 0; r < 5; r++){
            for(int i = 0; i < 60; i++){
                index = r*60 + i;
                average_OF_tang[i] = average_OF_tang[i] + (Qdot_u[index]*sin(gamma_arr[i])+Qdot_v[index]*cos(gamma_arr[i]));
            }
        }

        // Compute the spatial average
        for(int i = 0; i < 60; i++){
            OF_tang[i] = .2*average_OF_tang[i]; 
            OF_tang_prev[i] = LP_OF[i];
        }
        
        
        // Compute the FOF control
        // // // // // // // // //

//        for(int i = 0; i < 60; i++){
//            flow_out_msg.LP_OF.push_back(OF_tang_prev[i]);
//        }
    
        // Run the LPF
        for(int i = 0; i < 60; i++){
            LP_OF[i] = alpha*OF_tang[i] + (1 - alpha)*OF_tang_prev[i];   
            // THIS IS NEW
        //    OF_tang_prev[i] = LP_OF[i];
        }

        // Final FOF Calc
        for(int i = 0; i < 59; i++){
            R_FOF[i] = LP_OF[i]*OF_tang[i+1] - OF_tang[i]*LP_OF[i+1];
        }

        R_FOF[59] = 0.0;
        mean_sum = 0.0;
        std_dev = 0.0;
        // Dynamic Threshold
        for(int i = 0; i < 60; i++){
            mean_sum += R_FOF[i];
        }
        mean_val = mean_sum / 60;
        
        for(int i = 0; i < 60; i++){
            std_dev += pow((R_FOF[i] - mean_val), 2);
        }
        std_dev /= 60;
        std_dev = pow(std_dev, 0.5);
        min_threshold = 3*std_dev;

        // Find the max val
        for(int i = 0; i < 60; i++){
            if(R_FOF[i] > d_0){
                d_0 = R_FOF[i];
                r_0 = gamma_new[i];
            }
        }
        
        if(r_0 > 0){
            sign = 1;
        } else {
            sign = -1;
        }
        if(d_0 > min_threshold){
            yaw_rate_cmd_msg.yaw_rate_cmd = k_0*sign*exp(-c_psi*abs(r_0))*exp(-c_d/abs(d_0));
        } else {
            yaw_rate_cmd_msg.yaw_rate_cmd = 0.0;
        }
//        ROS_INFO_THROTTLE(.5,"d_0 = %f, min thresh = %f, r_0 = %f",d_0,min_threshold, r_0); 
        yaw_cmd_pub_.publish(yaw_rate_cmd_msg);    

    for(int i = 0; i < 60; i++){
        flow_out_msg.LP_OF.push_back(LP_OF[i]);
        flow_out_msg.Qdot_RFOF.push_back(R_FOF[i]);
        flow_out_msg.Qdot_tang.push_back(OF_tang[i]);
    }
    flow_out_pub_.publish(flow_out_msg);
    flow_out_msg.LP_OF.clear();
    flow_out_msg.Qdot_RFOF.clear();
    flow_out_msg.Qdot_tang.clear();

    if(debug = true){
        dt_out_msg.min_threshold = min_threshold;
        dt_out_msg.r_0 = r_0;
        dt_out_msg.d_0 = d_0;
        dt_out_pub_.publish(dt_out_msg);    
    }

    // End of flow calc  
}

int main(int argc, char **argv){
    ros::init(argc, argv, "FOF_node");
    ros::NodeHandle n;
    SubscribeAndPublish FlowSubPubObject;
    ros::Subscriber sub = n.subscribe("/flow_rings",1000, &SubscribeAndPublish::flow_cb, &FlowSubPubObject);
    while(ros::ok()){
        ros::spin();
    }
    return 0; // Exit main loop     
}

