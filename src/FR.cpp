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
#include <object_avoidance_cpp/YawRateCmdMsg.h>
#include <object_avoidance_cpp/FlowRingOutMsg.h>
#include <object_avoidance_cpp/FlowOutMsg.h>
#include <iostream>
using namespace std;

class SubscribeAndPublish {
private:
    ros::NodeHandle nh_;
    ros::Publisher yaw_cmd_pub_;
    ros::Publisher flow_out_pub_;
    ros::Subscriber flow_sub;

    std::vector<float> Qdot_u;
    std::vector<float> Qdot_v;

    float k_0, c_psi, c_d, dg;
    float a_0, a[4], b[4];
    float mean_val, mean_sum, std_dev, r_0, d_0;
    float average_OF_tang[];
    float OF_tang_prev[60];
    float OF_tang[60];
    float Qdot_WF[60];
    float Qdot_SF[60];

    int num_rings, num_points;
    int index, index_max;
    int sign, pixel_scale;
    float yaw_rate_cmd;
    float min_threshold;
    float gamma_arr[60];
    object_avoidance_cpp::YawRateCmdMsg yaw_rate_cmd_msg;
    object_avoidance_cpp::FlowRingOutMsg current_flow;
    object_avoidance_cpp::FlowOutMsg flow_out_msg;
public:
    SubscribeAndPublish() {
        // ROS_INFO("Object Created");

        
        // Create the yaw rate command publisher
        yaw_cmd_pub_ = nh_.advertise<object_avoidance_cpp::YawRateCmdMsg>
                ("/yaw_cmd_out", 10);
        // Create the flow message publisher
        flow_out_pub_ = nh_.advertise<object_avoidance_cpp::FlowOutMsg>
                ("/flow_data_out",10);

        //Initialize the gamma array
        for (int i = 0; i < 60; i++) {
            gamma_arr[i] = (2 * M_PI / 60) * i;
            OF_tang[i] = 0.0;
            average_OF_tang[i] = 0.0;
            Qdot_WF[i] = 0.0;
            Qdot_SF[i] = 0.0;
            if (i < 4){
                 a[i] = 0.0;
                 b[i] = 0.0;
            }
        }

    }
    void flow_cb(const object_avoidance_cpp::FlowRingOutMsg::ConstPtr& msg);
}; // End of class SubscribeAndPublish

void SubscribeAndPublish::flow_cb(const object_avoidance_cpp::FlowRingOutMsg::ConstPtr& msg) 
{
    current_flow = *msg;
    Qdot_u = current_flow.Qdot_u;
    Qdot_v = current_flow.Qdot_v;
    for (int i = 0; i < 60; i++) {
        OF_tang_prev[i] = OF_tang[i];
    }
    // Parameters
    k_0 = .5;
    c_psi = .1;
    c_d = .1;
    dg = gamma_arr[2] - gamma_arr[1];
    a_0 = 0.0;
    mean_val = 0.0;
    mean_sum = 0.0;
    std_dev = 0.0;
    r_0 = 0.0;
    d_0 = 0.0;
    index_max = 0;
    num_rings = 5;
    num_points = 60;
    index = 0;
    yaw_rate_cmd = 0.0;
    min_threshold = 0.0;
    sign = 0;
    pixel_scale = 60;
  // Compute average ring flow
    for (int r = 0; r < num_rings; r++) {
        for (int i = 0; i < num_points; i++) {
            index = r * num_points + i;
            average_OF_tang[i] = average_OF_tang[i] + (Qdot_u[index] * sin(gamma_arr[i]) - 1 * Qdot_v[index] * cos(gamma_arr[i]));
        }
    }

    // Compute the spatial average
    for (int i = 0; i < num_points; i++) average_OF_tang[i] *= .2;

    // Reformat the vector to -pi to pi
    for (int i = 0; i < num_points; i++) {
        if (i < 30) OF_tang[i] = -average_OF_tang[30 - i]*pixel_scale;
        if (i >= 30) OF_tang[i] = average_OF_tang[(90 - 1) - i]*pixel_scale;
    }i

    // Compute the FR control
    // // // // // // // // //
    // Compute coefficients for fourier residuals
    for (int i = 0; i < num_points; i++) {
        a_0 += cos(0 * gamma_arr[i]) * OF_tang[i];
        for (int n = 0; n < num_rings; n++) {
            a[n] += cos((n + 1) * gamma_arr[i]) * OF_tang[i];
            b[n] += cos((n + 1) * gamma_arr[i]) * OF_tang[i];
        }
    }
    // Convert coefficients to be in terms of degrees
    a_0 *= dg / M_PI;
    for (int n = 0; n < num_rings; n++) {
        a[n] *= dg / M_PI;
        b[n] *= dg / M_PI;
    }
    // Compute Qdot_WF
    for (int i = 0; i < num_points; i++) {
        for (int n = 0; n < num_rings; n++) {
            Qdot_WF[i] += a[n] * cos((n + 1) * gamma_arr[i]) + b[n] * sin((n + 1) * gamma_arr[i]);
        }
        Qdot_WF[i] += a_0 / 2.0;
    }
    for (int i = 0; i < num_points; i++) {
        Qdot_SF[i] = OF_tang[i] - Qdot_WF[i];
    }
    // Dynamic TH - unused
    //    // Calculate Std dev
    //    for(int i = 0; i < 60; i++){
    //        mean_sum += Qdot_SF[i];
    //    }
    //    mean_val = mean_sum / 60;
    //    for(int i = 0; i < 60; i++){
    //        std_dev += pow((Qdot_F[i] - mean_val), 2);
    //    }
    //    std_dev = pow(std_dev / 60, 0.5);
    //    min_threshold=3*std_dev

    // Static TH - in use
    min_threshold = .3;

    // Extract r_0 and d_0 from process OF signal based on max of Qdot_SF
    for (int i = 0; i < num_points; i++) {
        if (Qdot_SF[i] > Qdot_SF[index_max]) {
            index_max = i;
        }
    }
    d_0 = Qdot_SF[index_max];
    ROS_INFO_THROTTLE(.5,"d_0 = %f", d_0);
    // Calculate the control signal      
    if (d_0 > min_threshold) {
        r_0 = gamma_arr[index_max];
        if (r_0 > 0) sign = 1;
        if (r_0 < 0) sign = -1;
        yaw_rate_cmd_msg.yaw_rate_cmd = k_0 * sign * exp(-c_psi * abs(r_0)) * exp(-c_d / abs(d_0));
    } else {
        yaw_rate_cmd_msg.yaw_rate_cmd = 0.0;
    }
/*    for(int i= 0; i < 60; i++){ 
        flow_out_msg.Qdot_WF_out.push_back(Qdot_WF[i]);
        flow_out_msg.Qdot_SF_out.push_back(Qdot_SF[i]);
    }
    flow_out_pub_.publish(flow_out_msg);
    flow_out_msg.Qdot_WF_out.clear();
    flow_out_msg.Qdot_SF_out.clear();
*/
//    ROS_INFO_THROTTLE(2,"val %f",Qdot_WF[1]);
//    copy(&Qdot_WF[0], &Qdot_WF[60], back_inserter(flow_out_msg.Qdot_WF_out));
//    copy(&Qdot_SF[0], &Qdot_SF[60], back_inserter(flow_out_msg.Qdot_SF_out));
//   flow_out_msg.Qdot_WF_out(std::begin(Qdot_WF), std::end(Qdot_WF));

//   flow_out_pub_.publish(flow_out_msg);


    yaw_cmd_pub_.publish(yaw_rate_cmd_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "FR");
    ros::NodeHandle n;

    SubscribeAndPublish FlowSubPubObject;

    ros::Subscriber flow_sub = n.subscribe("/flow_rings", 1000, &SubscribeAndPublish::flow_cb, &FlowSubPubObject);

    ros::spin();

    return 0; // Exit main loop   

}

