/* Cpp script to publish the keyboard input to the topic /key */
#include <termios.h>
#include <iostream>     
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#define INPUTLIMIT 1000
#define MAXSPEED 0.6    //[m/s]

using namespace std;     // belongs to blocking input mode

//functions
float checkLimits(float val, float limit){
    val = val > limit ? limit: val;
    val = val < -limit ? -limit : val;
    return val;
}

int* getVals(int* input){
    //input format: turning velocity, forward velocity
    float inCh1 = 0;
    float inCh2 = 0;
    float l_wheel = 0;
    float r_wheel = 0;

    if(std::cin >> inCh1 >> inCh2){
            inCh1 = checkLimits(inCh1, MAXSPEED);
            inCh2 = checkLimits(inCh2, MAXSPEED);
            std::cout << "Processing fwd_speed: " << inCh1 << " ang_speed: " << inCh2 << std::endl; 
            l_wheel = 1000 * (inCh1 + inCh2) / MAXSPEED;
            r_wheel = 1000 * (inCh1 - inCh2) / MAXSPEED;
            l_wheel = floor(checkLimits(l_wheel, INPUTLIMIT));
            r_wheel = floor(checkLimits(r_wheel, INPUTLIMIT));
            cout << "Sending, Ch1: " << l_wheel << " Ch2: " << r_wheel << endl;
            *input = l_wheel;
            *(input + 1) = r_wheel;
        }
    return input;
}


//main
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/key", 1000);
    ros::Rate loop_rate(10);
    int *input;
    std_msgs::Float32MultiArray msg;
    msg.data.resize(2);
    msg.data = {0,0};

    while (ros::ok())
    {
        int* ninput = getVals(input);   
        msg.data = {*ninput, *(ninput+1)};
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}