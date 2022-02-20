/* Cpp script to publish the keyboard input to the topic /key */
#include <termios.h>
#include <iostream>     
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;     // belongs to blocking input mode

//functions
int checkLimits(int val){
    val = val>1000 ? 1000 : val;
    val = val<-1000 ? -1000 : val;
    return val;
}

int* getVals(int* input){
    int inCh1 = 0;
    int inCh2 = 0;
    if(std::cin >> inCh1 >> inCh2){
            inCh1 = checkLimits(inCh1);
            inCh2 = checkLimits(inCh2);
            std::cout << "Sending Ch1: " << inCh1 << " Ch2: " << inCh2 << std::endl; 
            *input = inCh1;
            *(input + 1) = inCh2;
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