#include "navigator.hpp"

void navigator::runNode(){
	ros::Rate loop_rate(freq);      // 50 Hz
	
	while (ros::ok()){


        geometry_msgs::Twist followHeading;   // For controlling the motor

        followHeading.linear.x = linearSpeed;
        followHeading.angular.y = 0.0;
        followHeading.angular.z = turningControl;
        
        pub_motor.publish(followHeading);
        
        
        
        navigator::calculateReferenceHeading();
        navigator::calculateP();
		ros::spinOnce();
		loop_rate.sleep();
		
		
    }
}




void navigator::sensorCallback(const hardware_msgs::IRDists msg1){
	float tmp1[] = {msg1.s0, msg1.s1, msg1.s2, msg1.s3, msg1.s4, msg1.s5};
	
    for(int i=0; i<6; i++){
		sensor[i] = tmp1[i];
	}
}



void navigator::odometryCallback(const hardware_msgs::Odometry msg2){
    absX = msg2.totalX;
    absY = msg2.totalY;
    theta = msg2.latestHeading;
}

/*void navigator::poseCallback(const geometry_msgs::PoseStamped msg3){

}*/




void navigator::calculateReferenceHeading(){
    referenceAngle = atan2(referenceHeadingY, referenceHeadingX);
    headingErr = referenceAngle - theta;

    // test
    //theta = 80 *M_PI/180.0;
    //ROS_INFO("theta = 80deg-atan2=%f",theta - atan2(5.6713, 1));

}

void navigator::bfsSearch(std::string obj){
	int pres = current;
	int size = map.list.size();
	//int res[size];
	for(int i=0;i<size;i++){
		//res[i] = i;
		path[i] = i;
	}
}

/*
 * construct the topological map and 
 */
void navigator::topologicalCallback(const mapping_msgs::NodeList msg){
	map = msg;
	current = 0;
	ROS_INFO("GOT A MAP");
	
	int size = msg.list.size();
	//node nodes;
	for(int i=0;i<size;i++){	//create nodes
		mapping_msgs::Node tmp = msg.list[i];
		node tmpN(tmp.ref,tmp.x,tmp.y);
		if(tmp.object){
			tmpN.setLabel(tmp.label);
			objects.push_back(tmp.label);
		}
		//nodes.push_back(tmpN);
	}
	/*for(int i=0;i<size;i++){
		mapping_msgs::Node tmp = msg.list[i];
		int son = tmp.links.size();
		for(int j = 0 ;j < son;j++){
			int ref = tmp.links[j];
			nodes.at(i).addNode(nodes.at(ref));
		}
	}
	currentNode = nodes.at(0);*/
}


void navigator::calculateP(){
    turningControl = Gp*headingErr; 
}
    




navigator::navigator(int argc, char *argv[]){
	ros::init(argc, argv, "navigator");	    // Name of node
	ros::NodeHandle n;					        // n = the handle

	freq = 50;
	
    ROSUtil::getParam(n, "/controllernav/Gp", Gp);
	/*ROSUtil::getParam(n, "/controllernav/GI_left", GI_left);
	ROSUtil::getParam(n, "/controllernav/GD_left", GD_left);
	ROSUtil::getParam(n, "/controllernav/Gcontr_left", Gcontr_left);
   	ROSUtil::getParam(n, "/controllernav/setpoint_left", setpoint_left);
	ROSUtil::getParam(n, "/controllernav/GP_right", GP_right);
	ROSUtil::getParam(n, "/controllernav/GI_right", GI_right);
	ROSUtil::getParam(n, "/controllernav/GD_right", GD_right);
	ROSUtil::getParam(n, "/controllernav/Gcontr_right", Gcontr_right);
  	ROSUtil::getParam(n, "/controllernav/setpoint_right", setpoint_right);*/
	ROSUtil::getParam(n, "/controllernav/control_freq", contr_freq);
	ROSUtil::getParam(n, "/controllernav/control_time", contr_time);
    //ROSUtil::getParam(n, "/controllernav/freq", freq);
    ROSUtil::getParam(n, "/controllernav/linearSpeed", linearSpeed);    


    /* Initialization variables */
    x = 0.0;
    y = 0.0;
    z = 0.0;
    
    referenceHeadingY = 0.0;
    referenceHeadingX = 0.0;


    angvel_left = 0.0;
    angvel_right = 0.0;

    headingErr = 0.0;
    referenceAngle = 0.0;
    turningControl = 0.0;
    referenceHeadingY = 0.0;
    referenceHeadingX = 0.0;

    pub_motor = n.advertise<geometry_msgs::Twist>("/motor3/twist", 1000);
    sub_sensor = n.subscribe("/ir_sensors/dists", 1000, &navigator::sensorCallback, this);

	sub_map = n.subscribe("/topological_map/map", 1000, &navigator::topologicalCallback, this);
    std::string odometry_pub_topic;
    ROSUtil::getParam(n, "/topic_list/hardware_topics/odometry/published/odometry_topic", odometry_pub_topic);
    sub_odometry = n.subscribe(odometry_pub_topic, 1000, &navigator::odometryCallback, this);
    //sub_pose = n.subscribe("/odometry/pose", 1000, &navigator::poseCallback, this);

	runNode();
}

int main(int argc, char *argv[]){
    navigator navigator(argc, argv);
}
