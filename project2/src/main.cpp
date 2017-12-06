//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <project2/custom_math_utils.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>
#include <time.h>
//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters you should adjust : K, margin, MaxStep
int margin = 20;
int K = 1500;
double MaxStep = 4.5;
double CAR_TARGET_MAX_SPEED = 5;
double CAR_TARGET_MIN_SPEED = 3;
int MAX_FAIL_NUMBER = 1;

//way points
std::vector<point> waypoints;

//path
//std::vector<point> path_RRT;
std::vector<traj> path_RRT;

//control
//std::vector<control> control_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;
gazebo_msgs::ModelStatesConstPtr model_states;

//FSM state
int state;

//timer
clock_t time1, time2;

//function definition
void set_waypoints();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void setcmdvel(double v, double w);
void finish();

int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",100,callback_state);
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/output",100);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

    // Load Map

    char* user = getpwuid(getuid())->pw_name;
    map = cv::imread((std::string("/home/")+std::string(user) + 
                      std::string("/catkin_ws/src/project2/src/ground_truth_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -10.0;
    world_x_max = 10.0;
    world_y_min = -10.0;
    world_y_max = 10.0;
    res = 0.05;
    printf("Load map\n");


     if(! map.data )                              // Check for invalid input
    {
        printf("Could not open or find the image\n");
        return -1;
    }

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    int look_ahead_idx;
    ros::Rate control_rate(60);

	//pid
	PID pid_ctrl;

	//
	double last_dist_squared = DOUBLE_INFINITE;

    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;
	    printf("path size : %d\n", (int)path_RRT.size());
            //visualize path
	    ros::spinOnce();
            for(int i = 0; i < path_RRT.size(); i++){
		for(int j = 0; j < model_states->name.size(); j++){
                    std::ostringstream ball_name;
                    ball_name << i;
            	    if(std::strcmp(model_states->name[j].c_str(), ball_name.str().c_str()) == 0){
                        //initialize robot position
                        geometry_msgs::Pose model_pose;
                        model_pose.position.x = path_RRT[i].x;
                        model_pose.position.y = path_RRT[i].y;
                        model_pose.position.z = 0.7;
                        model_pose.orientation.x = 0.0;
                        model_pose.orientation.y = 0.0;
                        model_pose.orientation.z = 0.0;
                        model_pose.orientation.w = 1.0;

                        geometry_msgs::Twist model_twist;
                        model_twist.linear.x = 0.0;
                        model_twist.linear.y = 0.0;
                        model_twist.linear.z = 0.0;
                        model_twist.angular.x = 0.0;
                        model_twist.angular.y = 0.0;
                        model_twist.angular.z = 0.0;

                        gazebo_msgs::ModelState modelstate;
                        modelstate.model_name = ball_name.str();
                        modelstate.reference_frame = "world";
                        modelstate.pose = model_pose;
                        modelstate.twist = model_twist;

                        gazebo_msgs::SetModelState setmodelstate;
                        setmodelstate.request.model_state = modelstate;

                        gazebo_set.call(setmodelstate);
                        continue;
                    }
    		}
	    
                gazebo_msgs::SpawnModel model;
                model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
			std::string("<static>true</static>") +
                        std::string("<link name=\"ball\">") +
                        std::string("<inertial>") +
                        std::string("<mass value=\"1.0\" />") +
                        std::string("<origin xyz=\"0 0 0\" />") +
                        std::string("<inertia  ixx=\"1.0\" ixy=\"1.0\"  ixz=\"1.0\"  iyy=\"1.0\"  iyz=\"1.0\"  izz=\"1.0\" />") +
                        std::string("</inertial>") +
                        std::string("<visual>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0.09\"/>") +
                        std::string("</geometry>") +
                        std::string("</visual>") +
                        std::string("<collision>") +
                        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                        std::string("<geometry>") +
                        std::string("<sphere radius=\"0.09\"/>") +
                        std::string("</geometry>") +
                        std::string("</collision>") +
                        std::string("</link>") +
                        std::string("<gazebo reference=\"ball\">") +
                        std::string("<mu1>10</mu1>") +
                        std::string("<mu2>10</mu2>") +
                        std::string("<material>Gazebo/Blue</material>") +
                        std::string("<turnGravityOff>true</turnGravityOff>") +
                        std::string("</gazebo>") +
                        std::string("</robot>");

                std::ostringstream ball_name;
                ball_name << i;
                model.request.model_name = ball_name.str();
                model.request.reference_frame = "world";
                model.request.initial_pose.position.x = path_RRT[i].x;
                model.request.initial_pose.position.y = path_RRT[i].y;
                model.request.initial_pose.position.z = 0.7;
                model.request.initial_pose.orientation.w = 0.0;
                model.request.initial_pose.orientation.x = 0.0;
                model.request.initial_pose.orientation.y = 0.0;
                model.request.initial_pose.orientation.z = 0.0;
                gazebo_spawn.call(model);
                ros::spinOnce();
            }
            printf("Spawn path\n");
	
            //initialize robot position
            geometry_msgs::Pose model_pose;
            model_pose.position.x = waypoints[0].x;
            model_pose.position.y = waypoints[0].y;
            model_pose.position.z = 0.3;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 0.0;
            model_pose.orientation.w = 1.0;

            geometry_msgs::Twist model_twist;
            model_twist.linear.x = 0.0;
            model_twist.linear.y = 0.0;
            model_twist.linear.z = 0.0;
            model_twist.angular.x = 0.0;
            model_twist.angular.y = 0.0;
            model_twist.angular.z = 0.0;

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = "racecar";
            modelstate.reference_frame = "world";
            modelstate.pose = model_pose;
            modelstate.twist = model_twist;

            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;

            gazebo_set.call(setmodelstate);
            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;
        } break;

        case RUNNING: {
	    // TO DO
		int toofarcount = 0;
		double steering_max = 0;

	    while(ros::ok()) {
			traj next_goal = path_RRT.at(look_ahead_idx);
			traj prev_goal; 

			if(look_ahead_idx > 0)
				prev_goal = path_RRT.at(look_ahead_idx - 1);
			else
				prev_goal = next_goal;
		
			float alpha = next_goal.alpha;
			float control = pid_ctrl.get_control(robot_pose, prev_goal, prev_goal, next_goal);
			double abs_control = myabs(control);
			float speed = getLinearlyInterpolatedValue(0, CAR_TARGET_MAX_SPEED, 0.2, CAR_TARGET_MIN_SPEED, abs_control);
			if(abs_control > steering_max)
				steering_max = abs_control;
			//printf("speed : %.3f, steering, %.3f\n", speed, control);

			setcmdvel(speed, control);

			//setcmdvel(2, pid_ctrl.get_control(robot_pose, next_goal));
			//printf("%d / %d dest : %.3f, %.3f CurPose : %.3f, %.3f, d: %.3f, alpha: %.3f \n",look_ahead_idx, (int)path_RRT.size(),  next_goal.x, next_goal.y, robot_pose.x, robot_pose.y, next_goal.d, next_goal.alpha);
	    	
		    	cmd_vel_pub.publish(cmd);
		    	
		    	ros::spinOnce();
	  	  	control_rate.sleep();
		    	
		    	double dx = robot_pose.x - next_goal.x;
		    	double dy = robot_pose.y - next_goal.y;	    	
		    	double dist_squared = dx * dx + dy * dy;

	
			//printf("%d / %d  dist_squar to dest : %.3f\n", look_ahead_idx + 1, (int)path_RRT.size(),  dist_squared);
			
		    //if(dist_squared < 0.04 || (dist_squared < 1 && dist_squared > last_dist_squared))
			if(dist_squared < 0.04 || (dist_squared < 8 && (dist_squared > last_dist_squared)))
		    	{
				if(!(dist_squared < 0.04)){
					//printf("--------------------------------------------TOO FAR\n");
					toofarcount++;
				}
					
					//printf("dest : %.3f, %.3f Reached\n", next_goal.x, next_goal.y);
		    		last_dist_squared = DOUBLE_INFINITE;
		    		look_ahead_idx++;
				pid_ctrl.clear();
				if(look_ahead_idx == path_RRT.size())
				{
					finish();
					//printf("too far count: %d\n", toofarcount);
					//printf("max steering: %.3f\n", steering_max);
					
					break;					
				}
		    	}
			else {
				last_dist_squared = dist_squared;
			}
	    }
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }
    }
    return 0;
}

void finish()
{
	time2 = clock();					
	clock_t runtime = (time2-time1) / 10000;					
	printf("Finished ROBOT\n");

	printf("Running time = %ld\n", runtime);
	state = FINISH;
}



void generate_path_RRT()
{
    /*
     * 1. for loop
     * 2.  call RRT generate function in order to make a path which connects i way point to i+1 way point.
     * 3.  store path to variable "path_RRT"
     * 4.  when you store path, you have to reverse the order of points in the generated path since BACKTRACKING makes a path in a reverse order (goal -> start).
     * 5. end
     */

	rrtTree* _rrtTree = new rrtTree(map, map_origin_x, map_origin_y, res, margin);
	while(true)
	{
		path_RRT.clear();
		int goal_number = waypoints.size();
	
		point x_init;
		x_init.x = waypoints[0].x;
		x_init.y = waypoints[0].y;
		x_init.th = waypoints[0].th;
		int i;
		
		for(i = 1; i<goal_number; i++)
		{
			
			
			point x_goal = waypoints[i];
			
			int current_fail_number = 0;
			while(i==1 || current_fail_number < MAX_FAIL_NUMBER)
			{
				_rrtTree->clearTree();
				_rrtTree->setNewPoint(x_init, x_goal);

				
				printf("%d try for %d path.\n", current_fail_number + 1, i);

				int valid = _rrtTree -> generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);

				if(valid == 1)
				{
					printf("%d path successfully created!!\n", i);
	
					std::vector<traj> backtrack_traj = _rrtTree -> backtracking_traj();
					std::reverse(backtrack_traj.begin(),backtrack_traj.end());

					int size = backtrack_traj.size();
					for(int j=0; j<size; j++)
					{
						path_RRT.push_back(backtrack_traj.at(j));
					}

					_rrtTree -> visualizeTree(path_RRT);
		
					traj last_traj = backtrack_traj.at(size-1);

					x_init.x = last_traj.x;
					x_init.y = last_traj.y;
					x_init.th = last_traj.th;
					//delete _rrtTree;
					break;
				}
				else
				{
					//delete _rrtTree;
					current_fail_number += 1;
				}
			}

			if(current_fail_number >= MAX_FAIL_NUMBER)
			{
				printf("Failed over %d times!!! Maybe previous path was wrong. Clear all found path and restart path finding!\n", MAX_FAIL_NUMBER);
				break;
			}			
			//std::vector<traj>::reverse_iterator reverse_iter(backtrack_traj.rbegin());
			//for(; reverse_iter != backtrack_traj.rend(); reverse_iter++)
			//{
			//	path_RRT.push_back(*reverse_iter);
			//}
				
		}

		if(i==goal_number)
			break;
	}

	delete _rrtTree;
}

void set_waypoints()
{
    point waypoint_candid[4];
    waypoint_candid[0].x = 5.0;
    waypoint_candid[0].y = -8.0;
    waypoint_candid[1].x = -6.0;
    waypoint_candid[1].y = -7.0;
    waypoint_candid[2].x = -7.0;
    waypoint_candid[2].y = 6.0;
    waypoint_candid[3].x = 3.0;
    waypoint_candid[3].y = 7.0;
    waypoint_candid[3].th = 0.0;

    int order[] = {3,1,2,3};
    int order_size = 3;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    model_states = msgs;
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"racecar") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}
