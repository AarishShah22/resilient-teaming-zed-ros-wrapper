/* Cpp script to publish the keyboard input to the topic /key */
#include <termios.h>
#include <iostream>     
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include<nav_msgs/Odometry.h>
//for keyboard operations
#include <unistd.h>
#include <stdio.h>
#include <map>

#define INPUTLIMIT 1000
#define MAXSPEED 0.6    //[m/s]

// Global variables 
float threshold = 0.05; //[m]
float speed = 0.25; //[m/s]
float Kd = 0.5;//0.2;//speed/threshold;
float Ka = 0.24; //0.16;
float Kb = -0.6; // not in use
float speed_lim = 500;//800;

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


//Map for speed keys
int keySpeed = 400;
char key(' ');
std::map<char, std::vector<int>> moveBindings
{
    {'w', {keySpeed, keySpeed}},
    {'s', {-keySpeed, -keySpeed}},
    {'a', {-keySpeed, keySpeed}},
    {'d', {keySpeed, -keySpeed}}
};

//non-binding keyboard input reader
int getch(void){
    int ch;
    struct termios oldt;
    struct termios newt;

    //store old settings, and copy to new settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

using namespace std;     // belongs to blocking input mode

/* **********functions********** */

//limit inputs to a predefined value
float checkLimits(float val, float limit){
    val = val > limit ? limit: val;
    val = val < -limit ? -limit : val;
    return val;
}

//-pi to pi coordinate system
float wrapToTwoPi(float angle){
    while(angle > M_PI) {
		angle -= 2*M_PI;
	}
	while(angle <= -M_PI) {
		angle += 2*M_PI;
	}
    return angle;
}

//calculate speed to achieve target pose
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

//check if near pose
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

//read target pose from terminal
void getVals(void){
    float x_pose;
    float y_pose;
    printf("\n");
    cout << "Input target x and target y location in meters" << endl;
    if(cin >> x_pose >> y_pose){
        cout << "Processing pose [m] x:" << x_pose << ", y:" << y_pose << endl;
        rob_state.target_x = x_pose;
        rob_state.target_y = y_pose;
        rob_state.target_theta = 0.0;
    }
}

//read current odom value
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
        int command = 0;
        printf("\nKeyboard command: 1, Pose command: 2\n\r");
        cin >> command;
        if(command == 1){
            printf("'w': Forward \n'a': Turn left \n's':Backward \n'd': Turn right\n");
            while(ros::ok()){

                //read terminal inputs
                key = getch();
                //check if key corresponds to map
                if(moveBindings.count(key) == 1){
                    rob_state.l_vel = moveBindings[key][0];
                    rob_state.r_vel = moveBindings[key][1];
                }
                else{
                    rob_state.l_vel = 0;
                    rob_state.r_vel = 0;
                    // exit if ctrl-C is pressed
                    if (key == '\x03'){
                        printf("breaking out\r");
                        break;
                    }
                }

                msg.data = {rob_state.l_vel, rob_state.r_vel};
                pub.publish(msg);

                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        else if(command == 2){
            getVals();
            while(ros::ok){
                getSpeed();
                if (withinTargetRange()){
                    msg.data = {0.0, 0.0};
                    pub.publish(msg);
                    break;
                }
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