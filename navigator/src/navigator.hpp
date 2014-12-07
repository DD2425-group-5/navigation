#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "controller_msgs/Turning.h"
#include "hardware_msgs/IRDists.h"
#include "hardware_msgs/Odometry.h"
#include <rosutil/rosutil.hpp>
#include <math.h>
#include <sstream>
#include <vector> 
#include <string>


class node{
private:
	int ref;
	std::vector<node> nodes;
	float x;
	float y;
	std::string label;
public:
	node(int _ref,float _x,float _y){
		ref = _ref;
		x = _x;
		y = _y;
		label = "";
	}
	
	void setLabel(std::string obj){
		label = obj;
	}
	
	void addNode(node ny){
		nodes.push_back(ny);
	}
	
	std::string getLabel(){
		return label;
	}
	
	float getX(){
		return x;
	}
	
	float getY(){
		return y;
	}
	
	node getNode(int i){
		//if(i<nodes.size()){
		return nodes.at(i);
		//}
		/*else{
			return Null;
		}*/
	}
	
	int size(){
		return nodes.size();
	}
};

class navigator{
public:
	navigator(int argc, char *argv[]);

private:
    double sensor[6];               // Latest sensor data
    double absX;                    // Absolute value of X                    
    double absY;                    // Absolute value of Y
    double theta;                   // Heading, angular

    double x;                       // Linear x
    double y;                       // Angular y
    double z;                       // Angular z

    /* From paramsnav*/
    double GP_left;
    double GI_left;
    double GD_left;
    double Gcontr_left;
    double setpoint_left;
    double GP_right;
    double GI_right;
    double GD_right;
    double Gcontr_right;
    double setpoint_right;
    float contr_time;
    float contr_freq;
    int freq;                       // Operating frequency
    double linearSpeed;

    double angvel_left;
    double angvel_right;
    
    double err_left;
    double err_left_prev;
    double err_right;
    double err_right_prev;

    double Pcontrol_left;
	double Icontrol_left;
	double Dcontrol_left;
	double Pcontrol_right;
	double Icontrol_right;
	double Dcontrol_right;
	double Pcontrol_left_prev;
	double Icontrol_left_prev;
	double Dcontrol_left_prev;
	double Pcontrol_right_prev;
	double Icontrol_right_prev;
	double Dcontrol_right_prev;
	double PIDcontrol_left;
	double PIDcontrol_left_prev;
	double PIDcontrol_right;
    double PIDcontrol_right_prev;


    void runNode();
    void avoidWall();
    void nodeTracker();
    void sensorCallback(const hardware_msgs::IRDists msg1);
    void odometryCallback(const hardware_msgs::Odometry msg2);
    //void poseCallback(const geometry_msgs::PoseStamped msg3);
    void calculatePID();
    
    
    ros::Publisher pub_motor;       // Publish to motor controller
    ros::Subscriber sub_sensor;     // Subscribe to sensors
    ros::Subscriber sub_odometry;   // Subscribe to odometry
    ros::Subscriber sub_pose;       // Subscribe to pose in odometry
};
