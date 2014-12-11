#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "hardware_msgs/IRDists.h"
#include <rosutil/rosutil.hpp>
#include <math.h>
#include "hardware_msgs/Odometry.h"
#include <sstream>
#include <map>
#include <vector> 
#include <string>
#include "controller_msgs/Turning.h"
#include "std_msgs/Bool.h"
#include "mapping_msgs/Node.h"
#include "mapping_msgs/NodeList.h"


#define DONOTHING			0
#define FOLLOW_LEFT_WALL	1
#define FOLLOW_RIGHT_WALL	2
#define TURN_LEFT			3
#define TURN_RIGHT			4
#define GO_FORTH			5
#define SCAN_LEFT			6
#define MOVE_FIVE			7
#define U_TURN				8
#define FORCE_TURN_LEFT		9

class Timer{
private:
	int timer;
public:
	Timer(){
		timer = 0;
	}
	
	int wait(int ms){
		timer=ms*5;
		return 1;
	}

	/*returns 1 after wait()*/
	int wait(){
		//ROS_INFO("timer = %d", timer);
		if(timer == 0){
			return 1;
		}
		else{
			timer--;
			return 0;
		}
	}
};

class fetch{
public:
	fetch(int argc, char *argv[]);

private:
	//variables
    mapping_msgs::NodeList map;		//the top map
	double absX;                    // Absolute value of X                    
    double absY;                    // Absolute value of Y
    double theta;                   // Heading, angular
	float sensor[6];		//	latest sensor data
	int started;			//	has all the nodes been started?
	int timer;				//	for wait()
	int hz;					//	operating frequency
	double marchSpeed;		//	default run speed
	int mapIsReady;
	double v;				//	velocity
	double w;				//	angular turn
	double y;				//	turn degrees
	int stop;				//	set to one if robot turn or stop
	char state;				//	current state
	char prevState;			//	previous state
	int change;				//	used to register changes in states
	void (fetch::*states[32])();	// state pointers
	void (fetch::*statep)();	//state pointer
	int move5;
	int scanState;			//scan state for finding left wall again
	int runTime;
	//int change;				//for observing multiple changes in state
	/**/
	std::vector<std::string> objects;
	
	int goToNode;
	float posX;
	float posY;
	int turn180;
	
	ros::Subscriber sub_sensor;	//sub to get distance values
	ros::Publisher pub_motor;	//for the motor
	ros::Publisher pub_turning; //for publishing when turning
	ros::Subscriber sub_isTurning; //for finished turn published by motor controller
	ros::Subscriber sub_odometry; 
	ros::Subscriber sub_map;
	
	void runNode();										//main run node
	void sensorCallback(const hardware_msgs::IRDists msg);	//for sensors
	void odometryCallback(const hardware_msgs::Odometry msg2); //for odometry
	void topologicalCallback(const mapping_msgs::NodeList msg);//for receiving map 
	void pubTurn(float degrees);						//publish turn as rostopic
	void isTurningCallback(const std_msgs::Bool msg);		//sub to finishedTurn
	
	int wait();			//call to wait
	int wait(int ms);	//set wait time in ms
	char currentState();//returns current state
	char action(char sensor_state); //what action should be taken?
	
	void printState();	//for debug
	void isThere();
	
	void donothing();	//self explainatory
	void followleftwallinit();
	void followleftwall();
	void followrightwallinit();
	void followrightwall();
	void followrightwall2();
	
	void turnleftinit();
	void turnleftstart();
	void turnleftend();
	
	void turnrightinit();
	void turnrightstart();
	void turnrightend();
	void goforth();
	
	void uturninit();
	void uturn1();
	void uturn2();
	void uturn3();
	void uturnend();
	void scanleftinit();
	void scanleft1();
	void scanleftend();
	
	void scan();
};
