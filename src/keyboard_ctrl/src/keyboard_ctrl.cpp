#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>
using namespace std;


#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_FORWARD 0x41
#define KEYCODE_BACKWARD 0x42
#define KEYCODE_LIFT 49    // 1
#define KEYCODE_RIGHTINCSPEED 105	//i
#define KEYCODE_RIGHTDECSPEED 107	//k
#define KEYCODE_LEFTINCSPEED 106	//j
#define KEYCODE_LEFTDECSPEED 108	//l
#define KEYCODE_Q 113
#define KEYCODE_RIGHTGO 118		//v
#define KEYCODE_RIGHTSTOP 98	//b
#define KEYCODE_LEFTGO 110		//n
#define KEYCODE_LEFTSTOP 109	//m
#define KEYCODE_RIGHTUP 119		//w
#define KEYCODE_RIGHTDOWN 115	//s
#define KEYCODE_LEFTUP 97		//a
#define KEYCODE_LEFTDOWN 100	//d

ros::Publisher key_ctrl_cmd;

int getch()
{
	static struct termios oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	int c = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	//printf("%u", (unsigned int)c);
	return c;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "keyboard_ctrl");
	ros::NodeHandle n;
	key_ctrl_cmd = n.advertise<std_msgs::String>("Keyboard_Control", 1000);

	//std_msgs::Float32MultiArray movement_msg;
	std_msgs::String movement_msg;
	puts("Reading from keyboard");
	bool processing = 1;
	puts("Ready to Go\n");
	while (ros::ok())
	{
		int c = getch();
		movement_msg.data.clear();

		if (c == KEYCODE_Q)
		{
			processing = 0;
			puts("Stop Receiving Command!\n");
		}
		switch(c)
		{
			case KEYCODE_RIGHTINCSPEED:
				puts("Right Motor Increase");
				movement_msg.data = "R_I";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_RIGHTDECSPEED:
				puts("Right Motor Decrease");
				movement_msg.data = "R_D";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_LEFTINCSPEED:
				puts("Left Motor Increase");
				movement_msg.data = "L_I";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_LEFTDECSPEED:
				puts("Left Motor Decrease");
				movement_msg.data = "L_D";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_RIGHTGO:
				puts("Right Motor Go");
				movement_msg.data = "R_START";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_RIGHTSTOP:
				puts("Right Motor Stop");
				movement_msg.data = "R_STOP";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_LEFTGO:
				puts("Left Motor Go");
				movement_msg.data = "L_START";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_LEFTSTOP:
				puts("Left Motor Stop");
				movement_msg.data = "L_STOP";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_RIGHTUP:
				puts("Right Motor Higher");
				movement_msg.data = "R_UP";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_RIGHTDOWN:
				puts("Right Motor Lower");
				movement_msg.data = "R_DOWN";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_LEFTUP:
				puts("Left Motor Higher");
				movement_msg.data = "L_UP";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_LEFTDOWN:
				puts("Left Motor Lower");
				movement_msg.data = "L_DOWN";
				key_ctrl_cmd.publish(movement_msg);
				break;
		}
		if (!processing)
			continue;
		switch(c)
		{
			case KEYCODE_LEFT:
				//ROS_DEBUG("LEFT");
				puts("LEFT");
				movement_msg.data = "LEFT";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_RIGHT:
				puts("RIGHT");
				movement_msg.data = "RIGHT";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_FORWARD:
				puts("FORWARD");
				movement_msg.data = "FORWARD";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_BACKWARD:
				puts("BACKWARD");
				movement_msg.data = "BACKWARD";
				key_ctrl_cmd.publish(movement_msg);
				break;
			case KEYCODE_LIFT:
				puts("LIFT");
				movement_msg.data = "LIFT";
				key_ctrl_cmd.publish(movement_msg);
				break;
		}
	}
	return 0;
}

