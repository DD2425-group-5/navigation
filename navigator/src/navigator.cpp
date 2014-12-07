#include "navigator.hpp"

void navigator::runNode(){
	ros::Rate loop_rate(freq);      // 50 Hz
	
	while (ros::ok()){


        geometry_msgs::Twist msg;   // For controlling the motor

        msg.linear.x = x;
        msg.angular.y = y;
        msg.angular.z = z;
        
        pub_motor.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
    }
}

void navigator::avoidWall(){
    x = linearSpeed;

	if(sensor[1]<0.3 && sensor[3]<0.3){
        z = PIDcontrol_right;
	}
	else{
		z = 0.0;
	}    


}

void navigator::nodeTracker(){

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

void navigator::calculatePID(){
    // Left side sensors:
    if(sensor[0] < 0){
		sensor[0] = 0.04;
	}
	if(sensor[2] < 0){
		sensor[2] = 0.04;
	}

    // Right side sensors:
	if(sensor[1] < 0){
		sensor[1] = 0.04;
	}
	if(sensor[3] < 0){
		sensor[3] = 0.04;
	}
    
	angvel_left = sensor[2] - sensor[0];
	angvel_right = sensor[3] - sensor[1];
    ROS_INFO("angvel_left = %f", angvel_left);
	
	/*ROS_INFO("sensor distance: 1: [%f] 2: [%f] 3: [%f] 4: [%f] 5: [%f] 6: [%f] \n\n",\
	sensor[0],\
	sensor[1],\
	sensor[2],\
	sensor[3],\
	sensor[4],\
	sensor[5]);*/

	// Error between target value and measured value
	err_left = setpoint_left - angvel_left;
	err_right = setpoint_right - angvel_right;

    ROS_INFO("err_left = %f", err_left);
    ROS_INFO("err_left_prev = %f", err_left_prev);
	
	// Left sensors controller
	Pcontrol_left = GP_left*err_left;
	Icontrol_left = Icontrol_left_prev + contr_time*GI_left*err_left; //+ (Gcontr_left/GP_left)*(
	Dcontrol_left = (GD_left/contr_time)*(err_left - err_left_prev);
	PIDcontrol_left = Pcontrol_left + Icontrol_left + Dcontrol_left;

    ROS_INFO("Pcontrol_left = %f", Pcontrol_left);
    ROS_INFO("Icontrol_left = %f", Icontrol_left);
    ROS_INFO("Dcontrol_left = %f", Dcontrol_left);
	
	// Right sensors controller
	Pcontrol_right = GP_right*err_right;
	Icontrol_right = Icontrol_right_prev + contr_time*GI_right*err_right;
	Dcontrol_right = (GD_right/contr_time)*(err_right - err_right_prev);
	PIDcontrol_right = Pcontrol_right + Icontrol_right + Dcontrol_right;
	
	// Define new prev values
	err_left_prev = err_left;
	Icontrol_left_prev = Icontrol_left;
	//PIDcontrol_left_prev = PIDcontrol_left;
	
	err_right_prev = err_right;
	Icontrol_right_prev = Icontrol_right;
	//PIDcontrol_right_prev = PIDcontrol_right;
}

navigator::navigator(int argc, char *argv[]){
	ros::init(argc, argv, "navigator");	    // Name of node
	ros::NodeHandle n;					        // n = the handle

    ROSUtil::getParam(n, "/controllernav/GP_left", GP_left);
	ROSUtil::getParam(n, "/controllernav/GI_left", GI_left);
	ROSUtil::getParam(n, "/controllernav/GD_left", GD_left);
	ROSUtil::getParam(n, "/controllernav/Gcontr_left", Gcontr_left);
   	ROSUtil::getParam(n, "/controllernav/setpoint_left", setpoint_left);
	ROSUtil::getParam(n, "/controllernav/GP_right", GP_right);
	ROSUtil::getParam(n, "/controllernav/GI_right", GI_right);
	ROSUtil::getParam(n, "/controllernav/GD_right", GD_right);
	ROSUtil::getParam(n, "/controllernav/Gcontr_right", Gcontr_right);
  	ROSUtil::getParam(n, "/controllernav/setpoint_right", setpoint_right);
	ROSUtil::getParam(n, "/controllernav/contr_freq", contr_freq);
	ROSUtil::getParam(n, "/controllernav/contr_time", contr_time);
    ROSUtil::getParam(n, "/controllernav/freq", freq);
    ROSUtil::getParam(n, "/controllernav/linearSpeed", linearSpeed);    


    /* Initialization variables */
    x = 0.0;
    y = 0.0;
    z = 0.0;


    angvel_left = 0.0;
    angvel_right = 0.0;

	err_left = 0.0;
	err_left_prev = 0.0;
	err_right = 0.0;
	err_right_prev = 0.0;
	
	Pcontrol_left = 0.0;
	Icontrol_left = 0.0;
	Dcontrol_left = 0.0;
	Pcontrol_right = 0.0;
	Icontrol_right = 0.0;
	Dcontrol_right = 0.0;
	//Pcontrol_left_prev = 0.0;
	Icontrol_left_prev = 0.0;
	//Dcontrol_left_prev = 0.0;
	//Pcontrol_right_prev = 0.0;
	Icontrol_right_prev = 0.0;
	//Dcontrol_right_prev = 0.0;
	PIDcontrol_left = 0.0;
	//PIDcontrol_left_prev = 0.0;
	PIDcontrol_right = 0.0;
    //PIDcontrol_right_prev = 0.0;

    pub_motor = n.advertise<geometry_msgs::Twist>("/motor3/twist", 1000);
    sub_sensor = n.subscribe("/ir_sensors/dists", 1000, &navigator::sensorCallback, this);
    sub_odometry = n.subscribe("/odometry/odometry", 1000, &navigator::odometryCallback, this);
    //sub_pose = n.subscribe("/odometry/pose", 1000, &navigator::poseCallback, this);

	runNode();
}

int main(int argc, char *argv[]){
    navigator navigator(argc, argv);
}