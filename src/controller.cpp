/* Cpp script to publish the keyboard input to the topic /key */
#include <termios.h>
#include <iostream>     
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include<nav_msgs/Odometry.h>

#define INPUTLIMIT 1000
#define MAXSPEED 0.6    //[m/s]

// Global variables 
float threshold = 0.1; //[m]
float speed = 0.25; //[m/s]
float Kd = 0.2;//speed/threshold;
float Ka = 0.16;//2.4;
float Kb = -0.6;
float speed_lim = 800;

struct rob_state_t{
    float curr_x;
    float curr_y;
    float curr_theta;
    float target_y;
    float target_x;
    float target_theta;
    float fwd_vel;
    float ang_vel;
    float l_vel;
    float r_vel;
};

rob_state_t rob_state;

using namespace std;     // belongs to blocking input mode

//functions
float checkLimits(float val, float limit){
    val = val > limit ? limit: val;
    val = val < -limit ? -limit : val;
    return val;
}

float wrapToTwoPi(float angle){
    while(angle > M_PI) {
		angle -= 2*M_PI;
	}
	while(angle <= -M_PI) {
		angle += 2*M_PI;
	}
    return angle;
}
void getSpeed(void){
    //converts target position to left and right wheel speed commands
    float l_wheel = 0;
    float r_wheel = 0;
    float delta_x = rob_state.target_x - rob_state.curr_x;
    float delta_y = rob_state.target_y - rob_state.curr_y;
    float distance = sqrtf(powf(delta_x, 2.0) + powf(delta_y, 2.0));
    float alpha = wrapToTwoPi(atan2(delta_y, delta_x) - rob_state.curr_theta);
    //float beta = wrapToTwoPi(rob_state.target_theta - rob_state.curr_theta);

    rob_state.fwd_vel = checkLimits(Kd * distance, 0.7 * MAXSPEED);
    rob_state.ang_vel = checkLimits(Ka * alpha, 0.3 * MAXSPEED); // + Kb * beta;

    cout << "Processing fwd_speed: " << rob_state.fwd_vel << " ang_speed: " << rob_state.ang_vel << endl; 
    l_wheel = 1000 * (rob_state.fwd_vel - rob_state.ang_vel) / MAXSPEED;
    r_wheel = 1000 * (rob_state.fwd_vel + rob_state.ang_vel) / MAXSPEED;
    l_wheel = floor(checkLimits(l_wheel, speed_lim));
    r_wheel = floor(checkLimits(r_wheel, speed_lim));
    // l_wheel = floor(checkLimits(l_wheel, INPUTLIMIT));
    // r_wheel = floor(checkLimits(r_wheel, INPUTLIMIT));
    cout << "Sending, Ch1: " << l_wheel << " Ch2: " << r_wheel << endl;
    rob_state.l_vel = l_wheel;
    rob_state.r_vel = r_wheel;
}

bool withinTargetRange(void){
    //check if robot is close to target pose with 0.1 threshold
    float delta_x = rob_state.target_x - rob_state.curr_x;
    float delta_y = rob_state.target_y - rob_state.curr_y;
    float distance = sqrtf(powf(delta_x, 2.0) + powf(delta_y, 2.0));
    if(distance < threshold){
        printf("Arrived: target (%.3f, %.3f), curr (%.3f, %.3f), delta (%.3f, %.3f), distance %.3f\n", rob_state.target_x, rob_state.target_y,
              rob_state.curr_x, rob_state.curr_y, delta_x, delta_y, distance);
        // cout << "Arrived:" << delta_x << "," << delta_y << "," << distance << endl;
        printf("\n");
    }
    return distance < threshold;
}

void getVals(void){
    float x_pose;
    float y_pose;
    cout << "Input target x and target y location in meters" << endl;
    if(cin >> x_pose >> y_pose){
        cout << "Processing pose [m] x:" << x_pose << ", y:" << y_pose << endl;
        rob_state.target_x = x_pose;
        rob_state.target_y = y_pose;
        rob_state.target_theta = 0.0;
    }
}

void odomCallback(const nav_msgs::Odometry&msg){
    rob_state.curr_x = (float) msg.pose.pose.position.x;
    rob_state.curr_y = (float) msg.pose.pose.position.y;
    rob_state.curr_theta = (float) msg.pose.pose.position.z;//(float)asin(2 * msg.pose.pose.orientation.z * msg.pose.pose.orientation.w);
    // ROS_INFO_STREAM("Odom callback x:" << rob_state.curr_x << " y:"<<rob_state.curr_y );
}

//main
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/key", 10);
    ros::Subscriber sub = n.subscribe("odom1", 10, odomCallback);
    ros::Rate loop_rate(100);
    int *input;
    std_msgs::Float32MultiArray msg;
    msg.data.resize(2);
    msg.data = {0,0};

    while (ros::ok())
    {
        int command;
        cout << "Duty cycle command: 1, Pose command: 2"<<endl;
        cin >> command;
        if(command == 1){
            cout << "Input -1000 to 1000 for wheel 1 and wheel 2" << endl;
            cin >> rob_state.l_vel >> rob_state.r_vel;
            msg.data = {rob_state.l_vel, rob_state.r_vel};
            pub.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
        else if(command == 2){
            getVals();
            while(ros::ok){
                getSpeed();
                // if (withinTargetRange()){
                //     msg.data = {0.0, 0.0};
                //     pub.publish(msg);
                //     break;
                // }
                msg.data = {rob_state.l_vel, rob_state.r_vel};
                pub.publish(msg);

                ros::spinOnce();
                loop_rate.sleep();
            }
        }
    }
    ros::waitForShutdown();
    return 0;
}