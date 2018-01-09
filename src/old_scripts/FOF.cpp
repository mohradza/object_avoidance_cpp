/* @file FOF.cpp
 * @brief Flow of Flow processing and control command calulation
 */

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float32.h>
#include <object_avoidance/YawRateCmdMsg.h>
#include <object_avoidance/FlowRingOutMsg.h>

double gamma_arr[60] = {};
void initialize_gamma(){
     for(int i = 0; i < 60; i++){
         gamma_arr[i] = (2*M_PI/60)*i;
     }
}

std::vector<float> Qdot_u;
std::vector<float> Qdot_v;
std::vector<float> OF_tang;
std::vector<float> OF_tang_prev;
float yaw_rate_cmd;
object_avoidance::FlowRingOutMsg current_flow;

// Flow callback function
void flow_cb(const object_avoidance::FlowRingOutMsg::ConstPtr& msg){
    current_flow = *msg;
    Qdot_u = current_flow.Qdot_u;
    Qdot_v = current_flow.Qdot_v;

    // Parameters
    float dt = 0.5;
    float tau = .75;
    float alpha = dt/(tau + dt);
    float k_0 = .3;
    float c_psi = .1;
    float c_d = .25;
    float mean_val = 0.0;
    float mean_sum = 0.0;
    float std_dev = 0.0;
    float r_0 = 0.0;
    float d_0 = 0.0;
    std::vector<float> average_OF_tang;
    std::vector<float> R_FOF;
    std::vector<float> LP_OF;
    int num_rings = 5; 
    int index = 0; 
    float yaw_rate_cmd = 0.0;
    float min_threshold = 0.0;
    object_avoidance::YawRateCmdMsg yaw_rate_cmd_msg; 
    OF_tang_prev = OF_tang;

    // Compute average ring flow
    for(int r = 0; r< 5; r++){
        for(int i = 0; i < 60; i++){
            index = r*60 + i;
            average_OF_tang[i] = average_OF_tang[i] + (Qdot_u[index]*sin(gamma_arr[i])-1*Qdot_v[index]*cos(gamma_arr[i]));
        }
    }

    // Compute the spatial average
    for(int i = 0; i < 60; i++) average_OF_tang[i] *= .2; 

    // Reformat the vector to -pi to pi
    for(int i = 0; i < 60; i++){
        if(i < 30) OF_tang[i] = -average_OF_tang[30 - i];
        if(i >= 30) OF_tang[i] = average_OF_tang[(90 - 1) - i];
    }

    // Compute the FOF control
    // // // // // // // // //
    
    // Run the LPF
    for(int i = 0; i < 60; i++){
         LP_OF[i] = alpha*OF_tang[i] + (1 - alpha)*OF_tang_prev[i];   
    }

    // Final FOF Calc
    for(int i = 0; i < 59; i++){
         R_FOF[i] = LP_OF[i]*OF_tang[i+1] - OF_tang[i]*LP_OF[i+1];
    }

    R_FOF[59] = 0.0;

    // Dynamic Threshold
    for(int i = 0; i < 60; i++){
        mean_sum += R_FOF[i];
    }
    mean_val = mean_sum / 60;
    for(int i = 0; i < 60; i++){
        std_dev += pow((R_FOF[i] - mean_val), 2);
    }
    std_dev = pow(std_dev / 60, 0.5);
    min_threshold = 3*std_dev;

    // Extract r_0 and d_0 from process OF signal
    for(int i = 0; i < 60; i++){
       if(R_FOF[i] > r_0){
           d_0 = R_FOF[i];
           r_0 = gamma_arr[i];
       }
    }

    int sign = 0;
    if(r_0 > 0) sign = 1;
    if(r_0 < 0) sign = -1;
  
    // Calculate the control signal
    if(d_0 > min_threshold){
        yaw_rate_cmd_msg.yaw_rate_cmd = k_0*sign*exp(-c_psi*abs(r_0))*exp(-c_d/abs(d_0));
    } else {
        yaw_rate_cmd_msg.yaw_rate_cmd = 0.0;
    }
    
    // NEED TO FIX PUBLISHER IN CALLBACK
    yaw_cmd_pub.publish(yaw_rate_cmd_msg);    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "FOF_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber flow_sub = nh.subscribe<object_avoidance::FlowRingOutMsg>
             ("/flow_out", 10, flow_cb);
    
    // Publishers
    ros::Publisher yaw_cmd_pub = nh.advertise<object_avoidance::YawRateCmdMsg>
             ("/yaw_cmd_out", 10); 

    // Initialize the gamma array
    initialize_gamma();

    while(ros::ok()){
        
        

        ros::spinOnce();
        ros::rate.sleep();

    } // End of while loop 
    
    return 0; // Exit main loop     

}

