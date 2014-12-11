#include "fetch.hpp"

void fetch::runNode(){
	ros::Rate loop_rate(hz);	//50 Hz
	Timer tim;
	tim.wait(50);
	while (ros::ok())			//main loop of this code
	{
		//ROS_INFO("STATE = %d", state);
		if(started){
			char tmp = currentState();	
			char tmpState=action(tmp);	//what action shoul be taken given the sensor data
			//ROS_INFO("STATE = %d TMPSTATE = %d stop = %d", state,tmpState,stop);
			if(scanState==2){
				move5++;
				if(move5>=15){
					if(state == TURN_LEFT){
						ROS_INFO("ALREADY TURNED LEFT");
						scanState = 0;
					}
					else{
						ROS_INFO("FORCE TURN LEFT");
						scanState = 0;
						tmpState = TURN_LEFT;
						change = 10;
					}
				}
			}
			if(v==0 && w == 0 && y == 0){
				if(tim.wait()){
					ROS_INFO("TIME OUT");
					prevState=state;
					state = DONOTHING;
					change = 10;
				}
			}
			else{
				tim.wait(50);
			}
			if(tmpState!=state && !stop){		//has the state changed?
				change++;
				if(change >= 10){
					prevState=state;
					state=tmpState;
					printState();
					//ROS_INFO("STATE = %d", state);
					statep = states[state];
					change = 0;
				}
			}
			(this->*statep)();
			if(state == FOLLOW_RIGHT_WALL || state == GO_FORTH){
				scan();
			}
		}
		
		geometry_msgs::Twist msg;	//for controlling the motor
		
		msg.linear.x = v;
		msg.angular.y = y;
		msg.angular.z = w;
		
		/*msg.linear.x = 0.0;
		msg.angular.y = 0.0;
		msg.angular.z = 0.0;
		/**/
		
		pub_motor.publish(msg);		//pub to motor
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/*what action should be taken?*/
char fetch::action(char sensor_state){
	//return GO_FORTH;
	char tmp;
	tmp = sensor_state & 0b00110011;
	if(tmp == 51){
		return U_TURN;
	}
	tmp = sensor_state & 0b00110000;	//apply front masking
	if(tmp>0){ //are there any obstacle in the way
		//return DONOTHING;
		tmp = sensor_state & 0b00000101;
		if(tmp==0){
			return TURN_LEFT;
		}
		else{
			return TURN_RIGHT;
		}
	}
	
	/*tmp = sensor_state & 0b00000101;
	if(state == TURN_LEFT){
		return SCAN_LEFT;
	}
	if(!tmp && state==MOVE_FIVE){
		move5++;
		if(move5>20){
			return TURN_LEFT;
		}
		else{
			return MOVE_FIVE;
		}
	}
	if(!tmp && state != MOVE_FIVE){
		//ROS_INFO("END OF LEFT WALL");
		move5 = 0;
		return MOVE_FIVE;
	}*/
	
	tmp = sensor_state & 0b00000101;	//apply lw masking
	if(5==tmp){ //do we have a wall on the left
		return FOLLOW_LEFT_WALL;
	}
	tmp = sensor_state & 0b00001010;	//apply rw masking
	if(10==tmp){ //no wall on the left but the right then?
		return FOLLOW_RIGHT_WALL;
	}
	
	tmp = sensor_state & 0b00110000;
	if(!tmp){
		return GO_FORTH;
	}
	tmp = sensor_state & 0b00000101;
	if(0==tmp && prevState != TURN_LEFT){
		return TURN_LEFT;
	}
	//return DONOTHING;
	return GO_FORTH;
}

void fetch::scan(){
	char tmp = currentState();
	char tmp2 = tmp & 0b00000100;
	//ROS_INFO("SCAN %d",tmp2);
	if(tmp2 && scanState == 0){
		scanState=1;
	}
	else if(!tmp2 && scanState == 1){
		scanState = 2;
		move5 = 0;
	}
}

void fetch::followrightwallinit(){
	if(wait()){
		statep=&fetch::followrightwall;
	}
	//stop = 0;
}

void fetch::followrightwall(){
	v = marchSpeed;
	if(sensor[1]<0){
		sensor[1]=0.04;
	}
	if(sensor[3]<0){
		sensor[3]=0.04;
	}
	/*char tmp = currentState();
	char tmp2 = tmp & 0b00000100;
	if(tmp2){
		statep = &fetch::followrightwall2;
	}*/
	if(sensor[1]<0.3 && sensor[3]<0.3){
		w = -15*(sensor[1]-sensor[3]);
		//ROS_DEBUG("SENSORS DIFF %f W = %f",sensor[1]-sensor[3],w);
	}
	else{
		w = 0.0;
	}
	runTime++;
}

void fetch::followrightwall2(){
	
}

void fetch::followleftwallinit(){
	if(wait()){
		statep=&fetch::followleftwall;
	}
}

void fetch::followleftwall(){
	v = marchSpeed;
	if(sensor[0]<0){
		sensor[0]=0.04;
	}
	if(sensor[2]<0){
		sensor[2]=0.04;
	}
	if(sensor[0]<0.3 && sensor[2]<0.3){
		w = 15*(sensor[0]-sensor[2]);
		//ROS_INFO("SENSORS DIFF %f W = %f",sensor[0]-sensor[2],w);
	}
	else{
		w = 0.0;
	}
	runTime++;
	//ROS_INFO("w = %f", w);
}

void fetch::turnleftinit(){
	v = 0.0;
	w = 0.0;
	stop = 1;
	wait(5);
	pubTurn(90.0);
	statep=&fetch::turnleftstart;
}

void fetch::turnleftstart(){
	if(wait()){
		y = 90;
		wait(25);
		statep=&fetch::turnleftend;
	}
}

void fetch::turnleftend(){
	if(wait()){
		y = 0.0;
		stop = 0;
		pubTurn(0);
		scanState = 0;
		wait(5);
		runTime = 0;
	}
}

void fetch::turnrightinit(){
	v = 0.0;
	w = 0.0;
	stop = 1;
	wait(5);
	pubTurn(-90.0);
	statep=&fetch::turnrightstart;
}

void fetch::turnrightstart(){
	if(wait()){
		y = -90;
		wait(25);
		statep=&fetch::turnrightend;
	}
}

void fetch::turnrightend(){
	if(wait()){
		y = 0.0;
		stop = 0;
		scanState = 0;
		pubTurn(0);
		wait(5);
		runTime = 0;
	}
}

void fetch::scanleftinit(){
	stop = 1;
	v = marchSpeed;
	w = 0.0;
	statep = &fetch::scanleft1;
}

void fetch::scanleft1(){
	char tmp = currentState();
	char tmp2 = tmp & 0b00000101;
	if(tmp2 == 5){
		stop = 0;
	}
	tmp2 = tmp & 0b00110101;
	if(tmp2 == 53 || tmp2 == 49){
		v = 0.0;
		stop = 0;
	}
	if(sensor[2]>0 && sensor[2]<0.30){
		statep = &fetch::scanleftend;
	}
}

void fetch::scanleftend(){
	char tmp = currentState();
	tmp = tmp & 0b00000100;
	if(sensor[2]>0.3 || sensor[2]<0.0){
		stop = 0;
		v=0.0;
		w=0.0;
	}
}

void fetch::uturninit(){
	v = 0.0;
	w = 0.0;
	stop = 1;
	wait(5);
	pubTurn(180.0);
	statep=&fetch::uturn1;
}

void fetch::uturn1(){
	if(wait()){
		y = -90;
		wait(25);
		statep=&fetch::uturn2;
	}
}

void fetch::uturn2(){
	if(wait()){
		y = 0;
		wait(5);
		statep=&fetch::uturn3;
	}
}

void fetch::uturn3(){
	if(wait()){
		y = -90;
		wait(25);
		statep=&fetch::uturnend;
	}
}

void fetch::uturnend(){
	if(wait()){
		y = 0.0;
		stop = 0;
		//scanState = 0;
		pubTurn(0);
		wait(5);
		runTime = 0;
	}
}

void fetch::goforth(){
	if(wait()){
		v = marchSpeed;
		w = 0.0;
		y = 0.0;
		runTime++;
		/*if(sensor[0]<0){
			w=0.04;
		}
		if(sensor[1]<0){
			w=-0.04;
		}*/
	}
}

void fetch::donothing(){
	//guess what this does
	//ROS_INFO("STATE: DO NOTHING");
	v = 0.0;
	w = 0.0;
	y = 0.0;
}

/*calculates and returns the current state*/
char fetch::currentState(){
	//at what distance in meters the sensor readings count as walls
	
	float registrate[] = {0.25,0.25,0.25,0.25,0.21,0.21};
	if(runTime<100){
		//registrate[4]=0.17;
		//registrate[5]=0.17;
	}
	//format xx(s5)(s4)(s3)(s2)(s1)(s0)
	char tmp = 0b00000000;
	int tmp2 = 1;
	for(int i=0;i<6;i++){
		if(sensor[i] < registrate[i]){
			tmp = tmp | tmp2;
		}
		tmp2 = tmp2 * 2;
	}
	return tmp;
	//ROS_INFO("state = %d", tmp);
}

/*this function publish a message the the turn have ended if degrees is 0 otherwise 
	that the turn have just begun*/
void fetch::pubTurn(float degrees){
	controller_msgs::Turning msg;
	if(degrees == 0){
		msg.isTurning = false;
	}
	else{
		msg.isTurning = true;
	}
	msg.degrees = degrees;
	pub_turning.publish(msg);
}

/*wait for ms miliseconds*/
int fetch::wait(int ms){
	timer=ms*hz/10;
	//v=0.0;
	//w=0.0;
	//y=0.0;
	return 1;
}

/*returns 1 after wait()*/
int fetch::wait(){
	//ROS_INFO("timer = %d", timer);
	if(timer == 0){
		return 1;
	}
	else{
		timer--;
		return 0;
	}
}

void fetch::printState(){
	if(state==DONOTHING){
		ROS_INFO("state = DONOTHING");
	}
	if(state==FOLLOW_LEFT_WALL){
		ROS_INFO("state = FOLLOW_LEFT_WALL");
	}
	if(state==FOLLOW_RIGHT_WALL){
		ROS_INFO("state = FOLLOW_RIGHT_WALL");
	}
	if(state==TURN_LEFT){
		ROS_INFO("state = TURN_LEFT");
	}
	if(state==TURN_RIGHT){
		ROS_INFO("state = TURN_RIGHT");
	}
	if(state==GO_FORTH){
		ROS_INFO("state = GO_FORTH");
	}
	if(state==SCAN_LEFT){
		ROS_INFO("state = SCAN_LEFT");
	}
	if(state==MOVE_FIVE){
		ROS_INFO("state = MOVE_FIVE");
	}
	if(state==U_TURN){
		ROS_INFO("state = U_TURN");
	}
}

void fetch::isTurningCallback(const std_msgs::Bool msg){
	ROS_INFO("GOT MESSAGE %d",msg.data);
}



void fetch::topologicalCallback(const mapping_msgs::NodeList msg){
    mapIsReady=1;
	map = msg;
		
	ROS_INFO("GOT A MAP");
	
	int size = msg.list.size();
	//node nodes;
	for(int i=0;i<size;i++){	//create nodes
		mapping_msgs::Node tmp = msg.list[i];
		if(tmp.object){
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


void fetch::odometryCallback(const hardware_msgs::Odometry msg2){
    absX = msg2.totalX;
    absY = msg2.totalY;
    theta = msg2.latestHeading;
}


/*sensor callback for getting distances*/
void fetch::sensorCallback(const hardware_msgs::IRDists msg){
	float tmp[] = {msg.s0,msg.s1,msg.s2,msg.s3,msg.s4,msg.s5};
	for(int i=0;i<6;i++){
		sensor[i]=tmp[i];
	}
	if(!started /*&& sensor[0]>0*/){
		started=1;
	}
	
	/*ROS_INFO("sensor distance: 1: [%f] 2: [%f] 3: [%f] 4: [%f] 5: [%f] 6: [%f] \n\n",\
	sensor[0],\
	sensor[1],\
	sensor[2],\
	sensor[3],\
	sensor[4],\
	sensor[5]);
	/**/
}

fetch::fetch(int argc, char *argv[]){
	ros::init(argc, argv, "fetch");	//name of node
	ros::NodeHandle handle;					//the handle
	
	//init variables
	hz = 50;
	timer = 0;
	v = 0.0;
	w = 0.0;
	y = 0.0;
	marchSpeed = 0.25;
	stop = 0;
	started=0;
	change=0;
	scanState = 0;
	runTime = 0;
	
	//init state machine
	states[DONOTHING] = &fetch::donothing;
	states[FOLLOW_LEFT_WALL] = &fetch::followleftwallinit;
	states[FOLLOW_RIGHT_WALL] = &fetch::followrightwallinit;
	states[TURN_LEFT] = &fetch::turnleftinit;
	states[TURN_RIGHT] = &fetch::turnrightinit;
	states[GO_FORTH] = &fetch::goforth;
	states[SCAN_LEFT] = &fetch::scanleftinit;
	states[MOVE_FIVE] = &fetch::goforth;
	states[U_TURN] = &fetch::uturninit;
	states[FORCE_TURN_LEFT] = &fetch::turnleftinit;
	state = DONOTHING;
	prevState = DONOTHING;
	statep = &fetch::donothing;
	
	//std::string turn_pub_topic;
    //ROSUtil::getParam(handle, "/topic_list/controller_topics/wallfollower/published/turning_topic", turn_pub_topic);
	//std::string dist_sub_topic;
    //ROSUtil::getParam(handle, "/topic_list/hardware_topics/hardware_msgs/published/ir_distance_topic", dist_sub_topic);
	//std::string motor_pub_topic;
    //ROSUtil::getParam(handle, "/topic_list/controller_topics/motor3/subscribed/twist_topic", motor_pub_topic);



   //sub odometry
    std::string odometry_pub_topic;
    ROSUtil::getParam(handle, "/topic_list/hardware_topics/odometry/published/odometry_topic", odometry_pub_topic);
    sub_odometry = handle.subscribe(odometry_pub_topic, 1000, &fetch::odometryCallback, this);

    sub_map = handle.subscribe("/topological_map/map", 1000, &fetch::topologicalCallback, this);
	pub_motor = handle.advertise<geometry_msgs::Twist>("/motor3/twist", 1000);
	sub_sensor = handle.subscribe("/ir_sensors/dists", 1000, &fetch::sensorCallback, this);
	pub_turning = handle.advertise<controller_msgs::Turning>("/controller/turn", 1000);
	//sub_isTurning = handle.subscribe("/motor3/is_turning", 1, &fetch::isTurningCallback, this);
	
	runNode();
}

int main(int argc, char *argv[]) 
{
    fetch fetch(argc, argv);
}
