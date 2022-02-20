/* Cpp script to publish the keyboard input to the topic /key */

#include <termios.h>
#include <iostream>     
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;     // belongs to blocking input mode

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/key", 1000);
    ros::Rate loop_rate(10);
    float inCh1 = 0;
    float inCh2 = 0;
    float input[2]={};
    std_msgs::Float32MultiArray msg;
    msg.data.resize(2);
    msg.data = {inCh1, inCh2};

    while (ros::ok())
    {
        ros::spinOnce();
        if(std::cin >> inCh1 >> inCh2){
            std::cout << "Read Ch1: " << inCh1 << " Ch2: " << inCh2 << std::endl; 
            input[0] = inCh1;
            input[1] = inCh2;
        }
        msg.data = {input[0], input[1]};

        pub.publish(msg);
        loop_rate.sleep();
    }
    ros::waitForShutdown();
}