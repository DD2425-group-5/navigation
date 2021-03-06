#include "navigator.hpp"

void navigator::runNode(){
	ros::Rate loop_rate(freq);      // 50 Hz
	
	while (ros::ok()){


        geometry_msgs::Twist followHeading;
        followHeading.linear.x = currentLinearSpeed;
        followHeading.angular.y = turnAngle;
        followHeading.angular.z = turningControl;
        
        pub_motor.publish(followHeading);
        ROS_INFO("Is map READY? [%d]", mapIsReady);
        
        if (mapIsReady){
        
        
        ROS_INFO("YES! So, calling calcuateReferenceHeading");
        navigator::calculateReferenceHeading();
        
        ROS_INFO("YES! So, calling mapCorrectionController");
        navigator::mapCorrectionController();
        }
		ros::spinOnce();
		loop_rate.sleep();
		
		
    }
}


void navigator::sensorCallback(const hardware_msgs::IRDists msg1){
	float tmp1[] = {msg1.s0, msg1.s1, msg1.s2, msg1.s3, msg1.s4, msg1.s5};
	
    for(int i=0; i<6; i++){
		if (tmp1[i]>=0.3){
		    sensor[i] = 0.3;
		   // ROS_INFO("Sensor %d is %f, put to max 0.3m", i, sensor[i]);
		}
		if(tmp1[i]<=0.04){
		    sensor[i] = 0.04;
		    //ROS_INFO("Sensor %d is %f, put to min 0.04m", i, sensor[i]);
		}
		else{
		    sensor[i] = tmp1[i];
		    //ROS_INFO("Sensor %d is %f, OK!", i, sensor[i]);
		}
    }
		
	sensorsAreReady = 1;
}


void navigator::mapCorrectionController(){
    if (mapIsReady && sensorsAreReady){
        
            if (sensor[0]<collisionAvoidanceTreshold && sensor[2]<collisionAvoidanceTreshold){
            
                controlAngle = GpMapCorrection*(sensor[0]-sensor[2]);
                ROS_INFO("Left wall too close S0,2[%f, %f], adapting with angle %f", sensor[0],sensor[2],controlAngle);
            }
            else {
                if (sensor[1]<collisionAvoidanceTreshold && sensor[3]<collisionAvoidanceTreshold){
               
                controlAngle = GpMapCorrection*(sensor[1]-sensor[3]);
                ROS_INFO("Right wall too close S1,3[%f, %f], adapting with angle %f", sensor[1],sensor[3],controlAngle);
                }
                else{ ROS_INFO("Both walls are far away!");}
            }
            
            for(int iter = 0; iter < map.list.size(); iter++){
                mapping_msgs::Node someNode = map.list[node_num];
                //make the robot the center of the coodinate system by translation
               
                float skewedRelativeX = someNode.x - absX;
                float skewedRelativeY = someNode.y - absY;
                
                
                MathUtil::Point straightenedCoordinates = MathUtil::rotateAroundOrigin(skewedRelativeX,\
                                                                                               skewedRelativeY,\
                                                                                               controlAngle);
                someNode.x = straightenedCoordinates.x + absX;
                someNode.y = straightenedCoordinates.y + absY;
                map.list[iter] = someNode;
            }
           
    }
    else{
    ROS_INFO("Map or Sensors *NOT* ready!");
    }
}



void navigator::odometryCallback(const hardware_msgs::Odometry msg2){
    absX = msg2.totalX;
    absY = msg2.totalY;
    theta = msg2.latestHeading;
}


void navigator::calculateReferenceHeading(){
  

  /*for(int i =0; i<path.size();i++){
   ROS_INFO("element [%d] of vector path is [%d], the vector has size %d elements", int(i), int(path[i]),int(path.size()));
    }*/
   if(nodeNumber<path.size()){
        node_num = path[nodeNumber];
    }
    else{
        node_num = node_num;
    }
    
    ROS_INFO("Going to node [%d], #%d of the list", node_num, nodeNumber);
    
    mapping_msgs::Node nextTarget = map.list[node_num];
    //ROS_INFO("Going to node [%d]", nodeNumber);
    referenceHeadingX = nextTarget.x;
    referenceHeadingY = nextTarget.y;
    float euclidDistance = sqrt( pow(referenceHeadingX-absX, 2) + pow(referenceHeadingY-absY, 2));
    referenceAngle = atan2(referenceHeadingY-absY, referenceHeadingX-absX);
    headingErr = referenceAngle - theta;
    
    if (euclidDistance > 0.04){
        currentLinearSpeed = linearSpeed;
        navigator::calculateP();
        
        if (state ==2 && timer >150){
            timer = 0;
            cheat = 0;
            state = 1; //if it was turning, it is folowing now
            latch = 0;
            ROS_INFO ("latch is set to 0");
            ROS_INFO ("changing state to 1, timer = %d", timer);
            }
    }
    else{
        if(nodeNumber<path.size() && latch == 0){
            nodeNumber++;
            latch = 1;
            ROS_INFO ("latch is set to 1");
        }                       
        ROS_INFO ("Incrementing node number to %d", nodeNumber);
        currentLinearSpeed = 0;
        if (state ==1){
            state = 2; //if it was folowing, it is turning now
            ROS_INFO ("changing state to 2");
        }        
    }
    

    if (state ==1){ //if following listen to the p controller
        ROS_INFO ("State is 1, seek&destroy");
        turnAngle = 0.0;
     }
    
    else if (state ==2){ //if turning just turn to the error angle
        
        cheat++;
        timer++;
        if (state ==2 && cheat ==2){
            turnAngle = headingErr*180.0/M_PI;
            }
        ROS_INFO ("State is 2, precision turn at %f deg, timer is %d", turnAngle, timer);
        
    }
    
    // debug
    ROS_INFO("target: X[%f] Y[%f], current: X[%f] Y[%f]\nreference[%f]-theta[%f]=%f\n Euclid Disytance to target: [%f]",referenceHeadingX,referenceHeadingY,absX,absY, referenceAngle,theta,  referenceAngle-theta, euclidDistance);

}


void navigator::bfsSearch(){
    ROS_INFO("debug1");
	int pres = current;
	ROS_INFO("debug2");
	int size = map.list.size();
	ROS_INFO("debug3");
	//int res[size];
	for(int i=0;i<size;i++){
		//res[i] = i;
		
		path[i] = i;
		ROS_INFO("debug4: i=%d",i);
	}
}

void navigator::cbfs(int origin, int target){
	int size = map.list.size();	//size of map
	int visited[size];			//nodes visited
	for(int i=0;i < size;i++){
		visited[i]=-1;
	}
	std::vector<int> q;	//queue
	q.push_back(origin);//push first
	visited[origin] = origin;
	while(q.size() > 0){
		int t = q.at(0);	//get first
		q.erase(q.begin());	//erase first
		if(t == target){
			break;
		}
		for(int i=0;i<map.list[t].links.size();i++){
			int elem = map.list[t].links[i];
			if(visited[elem] == -1){
				visited[elem] = t;
				q.push_back(elem);
			}
		}
	}
	/*for(int i=0;i < size;i++){
		ROS_INFO ("path found %d",visited[i]);
	}*/
	if(visited[target] != -1){
		int t = target;
		while(t!=origin){
			ROS_INFO ("path found %d",t);
			t=visited[t];
		}
		ROS_INFO ("path found %d",t);
	}
	else{
		ROS_INFO ("no path found");
	}
}


void navigator::breadthFirstSearch(int origin, int target){
    if(map.list[target].object){
        ROS_INFO("DANGER!!! THE TARGETED NODE IS AN OBJECT! PUSHING THE ROBOT TO THE NODE NEXT TO TARGET");
        target = map.list[target].links[0];
        //return;
    }

    std::queue<std::pair<int,int> > Q;      // Create a queue Q
    std::map<int,int> V;                    // Create a vector V
    //std::vector<int> P;                   // Create a vector P
    path.clear();
    std::vector<int>& P = path;             // Created an alias for path
    Q.push(std::pair<int,int>(origin,-1));  // Start at origin node
    
    //map.list[].object;
    
    
    while(!Q.empty()){
        std::pair<int, int> nextInQ = Q.front();
        V[nextInQ.first] = nextInQ.second;                          // Add the first value in Q to vector V
        Q.pop();                                                    // Remove first value from Q
        
        if(nextInQ.first == target){
            break;
        }

        int linkedNodes = map.list[nextInQ.first].links.size(); // Amount of neighbouring nodes that current node has
        
        for(int i=0; i<linkedNodes; i++){
            //int linkedNode = map.list[nextInQ.first].links[i];      // Node [i] that links to current node
            if(!map.list[map.list[nextInQ.first].links[i]].object && 
                V.find(map.list[nextInQ.first].links[i]) == V.end()){
                Q.push(std::pair<int, int>(map.list[nextInQ.first].links[i], nextInQ.first));       // Push neighbour nodes to Q
            }
        }
    }

    int j = target;
    P.push_back(j);
  
    while(1){
        j = V[j];
        P.push_back(j);

        if(j == origin){
            break;
        }
    }

    std::reverse(P.begin(), P.end());              // Returns the shortest path from origin node to target node
}

void navigator::fakeMap(){ //not used
    mapping_msgs::Node tmp;
    tmp.x = 0.0;
    tmp.y = 0.0; 
    map.list.push_back(tmp);
   
    tmp.x = 0.0;
    tmp.y = 0.3; 
map.list.push_back(tmp);
  
    tmp.x = 0.2;
    tmp.y = 0.3; 
map.list.push_back(tmp);
    
    tmp.x = 0.2;
    tmp.y = 0.0; 
map.list.push_back(tmp);

mapIsReady = 1;
    

}

/*
 * construct the topological map and 
 */
void navigator::topologicalCallback(const mapping_msgs::NodeList msg){
    mapIsReady=1;
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
	
	

	navigator::breadthFirstSearch(0, 16); //RMOVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
//put into callback instead?
   

}


void navigator::calculateP(){
    turningControl = GpTrajectory*headingErr; 
}
    




navigator::navigator(int argc, char *argv[]){
	ros::init(argc, argv, "navigator");	    // Name of node
	ros::NodeHandle n;					        // n = the handle

	freq = 50;
	mapIsReady = 0;
	sensorsAreReady = 0;
	controlAngle = 0.0;
	state = 1;
	timer = 0;
	ROSUtil::getParam(n, "/controllernav/Gp_map_correction", GpMapCorrection);
    ROSUtil::getParam(n, "/controllernav/Gp_trajectory", GpTrajectory);
    ROSUtil::getParam(n, "/controllernav/collision_avoidance_treshold", collisionAvoidanceTreshold);
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
    cheat = 0;
    turnAngle = 0;
    nodeNumber = 1;
    node_num = 0;
    latch = 0;
    
    referenceHeadingY = 0.0;
    referenceHeadingX = 0.0;
    headingErr = 0.0;
    referenceAngle = 0.0;
    turningControl = 0.0;
    referenceHeadingY = 0.0;
    referenceHeadingX = 0.0;

    /* publish and subscribe properly*/
    std::string odometry_pub_topic;
    ROSUtil::getParam(n, "/topic_list/hardware_topics/odometry/published/odometry_topic", odometry_pub_topic);
    std::string motor3_pub_topic;
    ROSUtil::getParam(n, "/topic_list/controller_topics/motor3/subscribed/twist_topic", motor3_pub_topic);
    std::string ir_pub_topic;
    ROSUtil::getParam(n, "/topic_list/hardware_topics/ir_sensors/published/ir_distance_topic", ir_pub_topic);
     
    sub_map = n.subscribe("/topological_map/map", 1000, &navigator::topologicalCallback, this);
    
    
    pub_motor = n.advertise<geometry_msgs::Twist>( motor3_pub_topic, 1000);
    sub_sensor = n.subscribe(ir_pub_topic, 1000, &navigator::sensorCallback, this);
    sub_odometry = n.subscribe(odometry_pub_topic, 1000, &navigator::odometryCallback, this);
    
    
   	runNode();
}

int main(int argc, char *argv[]){
    navigator navigator(argc, argv);
}
