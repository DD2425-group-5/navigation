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
#include <queue>
#include <utility>
#include <map>
#include <algorithm>
#include "mapping_msgs/Node.h"
#include "mapping_msgs/NodeList.h"


class node{
private:
	int ref;
	std::vector<node> nodes;
	float x;
	float y;
	std::string label;
	int object;
public:
	node(){
	
	}
	node(int _ref,float _x,float _y){
		ref = _ref;
		x = _x;
		y = _y;
		label = "";
		object = 0;
	}
	
	void setLabel(std::string obj){
		label = obj;
		object = 1;
	}
	
	int hasObject(){
		return object;
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
	
	mapping_msgs::NodeList map;		//the top map
	int current;					//node robots at
	std::vector<std::string> objects;	//a list of all objects
	std::vector<int> path;

	std::vector<node> nodes;		//All nodes just in case
	//node currentNode;				//Node your at

    /* asdsadasd*/
    float contr_time;
    float contr_freq;
    int freq;                       // Operating frequency
    double linearSpeed;

    double angvel_left;
    double angvel_right;
    double Gp;
    double headingErr;
    double referenceAngle;
    double turningControl;
    double referenceHeadingY;
    double referenceHeadingX;
 

    void runNode();
    void sensorCallback(const hardware_msgs::IRDists msg1);
    void odometryCallback(const hardware_msgs::Odometry msg2);
	void topologicalCallback(const mapping_msgs::NodeList msg);
    //void poseCallback(const geometry_msgs::PoseStamped msg3);
    void calculateP();
    void calculateReferenceHeading();
    void bfsSearch(std::string obj);
    void breadthFirstSearch(int origin, int target); // Calculates the shortest distance from current node (origin) to target node (target)
    
    ros::Publisher pub_motor;       // Publish to motor controller
    ros::Subscriber sub_sensor;     // Subscribe to sensors
    ros::Subscriber sub_odometry;   // Subscribe to odometry
    ros::Subscriber sub_pose;       // Subscribe to pose in odometry
	ros::Subscriber sub_map;		// Subscribe to topological map
};
