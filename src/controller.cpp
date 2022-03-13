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
}rob_state;

using namespace std;     // belongs to blocking input mode

//functions
float checkLimits(float val, float limit){
    val = val > limit ? limit: val;
    val = val < -limit ? -limit : val;
    return val;
}

float wrapToTwoPi(float angle){
    if(angle < 0){
        for(; angle < 0; angle+=M_2_PI);
    }
    else if(angle > M_2_PI){
        for(; angle > M_2_PI; angle -= M_2_PI);
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
    float delta_theta = atan2(delta_y, delta_x) - rob_state.curr_theta;
    delta_theta = wrapToTwoPi(delta_theta);
    float Kd = 1;
    float Ka = 1;
    float speed_lim = 500;

    rob_state.fwd_vel = Kd * distance;
    rob_state.ang_vel = Ka * delta_theta;
    cout << "Processing fwd_speed: " << rob_state.fwd_vel << " ang_speed: " << rob_state.ang_vel << endl; 
    l_wheel = 1000 * (rob_state.fwd_vel + rob_state.ang_vel) / MAXSPEED;
    r_wheel = 1000 * (rob_state.fwd_vel - rob_state.ang_vel) / MAXSPEED;
    l_wheel = floor(checkLimits(l_wheel, INPUTLIMIT));
    r_wheel = floor(checkLimits(r_wheel, INPUTLIMIT));
    l_wheel = l_wheel > speed_lim ? speed_lim : l_wheel;
    r_wheel = r_wheel > speed_lim ? speed_lim : r_wheel;
    cout << "Sending, Ch1: " << l_wheel << " Ch2: " << r_wheel << endl;
    rob_state.l_vel = l_wheel;
    rob_state.r_vel = r_wheel;
}

bool withinTargetRange(void){
    //check if robot is close to target pose with 0.1 threshold
    float threshold = 0.1; //[m]
    float delta_x = rob_state.target_x - rob_state.curr_x;
    float delta_y = rob_state.target_y - rob_state.curr_y;
    float distance = sqrtf(powf(delta_x, 2.0) + powf(delta_y, 2.0));
    if(distance < threshold){
        cout << "Arrived" << endl;
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
    rob_state.curr_theta = (float)asin(2 * msg.pose.pose.orientation.z * msg.pose.pose.orientation.w);
    // cout << rob_state.curr_x << " y:"<<rob_state.curr_y << endl;
}

//main
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/key", 1000);
    ros::Subscriber sub = n.subscribe("odom1", 50, odomCallback);
    ros::Rate loop_rate(50);
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
            while(ros::ok && !withinTargetRange()){
                getSpeed();
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