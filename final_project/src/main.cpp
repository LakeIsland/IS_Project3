//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
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

#include "geometry_msgs/PoseWithCovarianceStamped.h"

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

//parameters we should adjust : K, margin, MaxStep
int margin = 6;
int K = 1500;
double MaxStep = 2;

int waypoint_margin = 22;
double CAR_TARGET_MAX_SPEED = 2;
double CAR_TARGET_MIN_SPEED = 2;

int MAX_FAIL_NUMBER = 2;
int ANY_WAY_RESTART_COUNT = 200 * MAX_FAIL_NUMBER;

int CONTROL_RATE = 60;
int TRACK_NUMBER = 1;
double BREAK_SEC = 2;

double DIST_SQUARE_TO_CHECK_REACHED = 0.09;

double ALLOWED_LEFT = 	2.5;
double ALLOWED_RIGHT = 	2.5;
double ALLOWED_TOP = 	3.5;
double ALLOWED_BOTTOM = 3.5;


//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

//FSM state
int state;
//timer
clock_t time1, time2;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();
void finish();
void take_break(ros::Rate control_rate, ros::Publisher cmd_vel_pub);
void create_clock_wise_way_points(cv::Mat map, point start_point);
point find_empty_point(cv::Mat map, double corner_x, double corner_y);
void set_random_point();

PID pid_ctrl;
double last_dist_squared = DOUBLE_INFINITE;

double rrt_sec;

int outside_allowed_waypoint_index;

int main(int argc, char** argv){
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
    
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
    int look_ahead_idx;
	int track_count = 0;
    ros::Rate control_rate(CONTROL_RATE);

    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;
	    	printf("path size : %d\n", (int)path_RRT.size());

            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/final_project/src/final.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.7;
            world_x_max = 4.7;
            world_y_min = -10.2;
            world_y_max = 10.2;
            res = 0.05;
            printf("Load map\n");

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING:
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            time1 = clock();
            generate_path_RRT();
            time2 = clock();
            rrt_sec = (((double)(time2 - time1)) / CLOCKS_PER_SEC);
            printf("Generate RRT in %.3f sec.\n", rrt_sec);

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;

        case RUNNING: {
        //TODO 1
		int toofarcount = 0;
		//double steering_max = 0;
		//take_break(control_rate, cmd_vel_pub);
		look_ahead_idx = 1;
		
		time1 = clock();
	    while(ros::ok()) {

			traj current_goal = path_RRT.at(look_ahead_idx);
			traj next_goal; 
			traj prev_goal;

			if(look_ahead_idx < path_RRT.size() - 1)
				next_goal = path_RRT.at(look_ahead_idx + 1);
			else
				next_goal = current_goal;

			if(look_ahead_idx > 0)
				prev_goal = path_RRT.at(look_ahead_idx - 1);
			else
				prev_goal = current_goal;

			float control = pid_ctrl.get_control(robot_pose, prev_goal, current_goal, next_goal);

			
			double abs_control = myabs(control);
			float speed = getLinearlyInterpolatedValue(0, CAR_TARGET_MAX_SPEED, 0.18, CAR_TARGET_MIN_SPEED, abs_control);
			//float speed = getExponentiallyInterpolatecValue(CAR_TARGET_MAX_SPEED, CAR_TARGET_MIN_SPEED, abs_control);			
			//printf("speed : %.3f\n", speed);

			//if(abs_control > steering_max)
			//	steering_max = abs_control;
			setcmdvel(speed, control);
		    cmd_vel_pub.publish(cmd);
		    	
		    ros::spinOnce();
	  	  	control_rate.sleep();
		    	
		    double dx = robot_pose.x - current_goal.x;
		    double dy = robot_pose.y - current_goal.y;	    	
		    double dist_squared = dx * dx + dy * dy;

			if(dist_squared < DIST_SQUARE_TO_CHECK_REACHED || (last_dist_squared < dist_squared && dist_squared < 2))
		    	{
				if(!(dist_squared < DIST_SQUARE_TO_CHECK_REACHED)){
					printf("PASSED!!!!!!!!!!!!!!!!!!!!!!!!\n");
					toofarcount++;
				}
				else{
					printf("REACHED!!!!!!!!!!!!!!!!!!!!!!!!\n");
					
				}
		    		last_dist_squared = DOUBLE_INFINITE;
		    		look_ahead_idx++;
				pid_ctrl.clear();
				if(look_ahead_idx == path_RRT.size())
				{
					finish();
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

void take_break(ros::Rate control_rate, ros::Publisher cmd_vel_pub )
{
	double timer = 0;
	while(timer < CONTROL_RATE * BREAK_SEC)
	{
		timer += 1;
		setcmdvel(0, 0);
		cmd_vel_pub.publish(cmd);
		ros::spinOnce();
	  	control_rate.sleep();
	}
}


void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void finish()
{
	time2 = clock();					
	clock_t runtime = (time2-time1) / CLOCKS_PER_SEC;					
	printf("Finished ROBOT\n");
	printf("Running time = %ld\n", runtime);
	state = FINISH;
}

void set_waypoints()
{
}

int determin_where_point_is(point start_position)
{
	int i = (int)((start_position.x / res) + map_origin_x);
	int j = (int)((start_position.y / res) + map_origin_y);
	
	int map_width = map_x_range;
	int map_height = map_y_range;
	
	int x = i * 2 / map_width;
	int y = j * 2 / map_height;
	
	if(x == 0 && y==0)
	{
		printf("left bottom\n");
		return 0;
	}
	if(x == 0 && y==1)
	{
		printf("left top\n");
		return 1;
	}
	if(x == 1 && y==1)
	{
		printf("right top\n");
		return 2;
	}
	if(x == 1 && y==0)
	{
		printf("right bottom\n");
		return 3;
	}
}

void create_clock_wise_way_points(cv::Mat map, point start_position)
{
	waypoints.clear();

	int where = determin_where_point_is(start_position);
	int xs[] = {0,0,1,1};
	int ys[] = {0,1,1,0};

	printf("set random waypoint started\n");
	waypoints.push_back(start_position);
	for(int j=0; j<TRACK_NUMBER; j++)
	{
		for(int i=1; i<=4; i+=1)
		{
			if(i == 4 && j==TRACK_NUMBER-1){
				waypoints.push_back(start_position);
			} else {
				int idx = (i + where) % 4;
				point _point = find_empty_point(map, xs[idx], ys[idx]);
				waypoints.push_back(_point);
			}	
		}
		
	}
	
	outside_allowed_waypoint_index = (int)waypoints.size();

	printf("set clockwise waypoint finished\n");

	set_random_point();
	
}


/* SET RANDOM POINTS HERE */

void set_random_point()
{
	point waypoint_candid[2];

	waypoint_candid[0].x = 1.5;
    waypoint_candid[0].y = 1.5;
    waypoint_candid[1].x = -2;
    waypoint_candid[1].y = -9.0;
	
	int order[] = {0,1};
    int order_size = 2;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

point find_empty_point(cv::Mat map, double corner_x, double corner_y)
{
	int map_width = map_x_range;
	int map_height = map_y_range;
	
	while(true)
	{
		double x_rand = getRandomDouble();
		double y_rand = getRandomDouble();
		x_rand = lerp(x_rand, corner_x, 0.5);
		y_rand = lerp(y_rand, corner_y, 0.5);
		double world_rand_x = lerp(world_x_min, world_x_max, x_rand);
		double world_rand_y = lerp(world_y_min, world_y_max, y_rand);
		
		if(
			world_rand_x > world_x_min + ALLOWED_LEFT
			&&world_rand_x < world_x_max - ALLOWED_RIGHT
			&&world_rand_y > world_y_min + ALLOWED_BOTTOM
			&&world_rand_y < world_y_max - ALLOWED_TOP
		)
			continue;
		

		int i = (int)((world_rand_x / res) + map_origin_x);
		int j = (int)((world_rand_y / res) + map_origin_y);
		
		if(i<0)
			continue;
		if(i>=map_x_range)
			continue;
		if(j<0)
			continue;
		if(j>=map_y_range)
			continue;
			
		
		int occupied = map.at<uchar>(i,j);
    
		if(occupied > 125 )
		{
			printf("%.3f,%.3f\n",x_rand,y_rand);
		
			point _point;
			_point.x = world_rand_x;
			_point.y = world_rand_y;
		
			return _point;
		}
	}
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
     
    // set random
	std::srand(std::time(NULL));
	
	// set start point
	point start_point;
	start_point.x = -3.5;
	start_point.y = 8.5;

	// set map.
	cv::Mat map_margin = map.clone();
	int jSize = map.cols; // the number of columns
	int iSize = map.rows; // the number of rows

	
	// initialize map with margin
	for (int i = 0; i < iSize; i++) {
		for (int j = 0; j < jSize; j++) {
			if (map.at<uchar>(i, j) < 125) {
				for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
					for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
						if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
						    map_margin.at<uchar>(k, l) = 0;
						}
					}
				}
			}
		}
	}

	// create single rrt Tree. this will be reused.
	rrtTree* _rrtTree = new rrtTree(map, map_origin_x, map_origin_y, res, margin);
	std::vector<int> track_sizes;
	
	
	// find path.
	while(true)
	{
	
		// firstly create clock wise way points.
		create_clock_wise_way_points(map_margin, start_point);
		
		
		// clear path.
		path_RRT.clear();
		track_sizes.clear();
		int goal_number = waypoints.size();
		int anyway_fail_number = 0;
		
		// set first point
		point x_init;
		x_init.x = waypoints[0].x;
		x_init.y = waypoints[0].y;
		x_init.th = waypoints[0].th;
		
		traj init_traj;
		init_traj.x = x_init.x;
		init_traj.y = x_init.y;
		path_RRT.push_back( init_traj );

		// start finding path to next node.
		
		// next node index
		int i = 1;

		while(i < goal_number)
		{
			if(i >= outside_allowed_waypoint_index)
			{
				_rrtTree->set_must_go_outside(false);
			} else {
				_rrtTree->set_must_go_outside(true);
				_rrtTree->set_allowed_meter(ALLOWED_LEFT,ALLOWED_RIGHT,ALLOWED_TOP,ALLOWED_BOTTOM);	
			}

			printf("start Finding %d th path.\n", i);
			
			// next goal point
			point x_goal = waypoints[i];
			
			// now start to find a path to next goal point.
			int current_fail_number = 0;
			while(i==1 || current_fail_number < MAX_FAIL_NUMBER)
			{
				
				_rrtTree->clearTree();
				_rrtTree->setNewPoint(x_init, x_goal);
				//printf("%d try for %d path.\n", current_fail_number + 1, i);

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
					track_sizes.push_back(size);

					_rrtTree -> visualizeTree(path_RRT);
		
					traj last_traj = backtrack_traj.at(size-1);

					x_init.x = last_traj.x;
					x_init.y = last_traj.y;
					x_init.th = last_traj.th;
					i+=1;
					//delete _rrtTree;
					break;
				}
				else
				{
					//delete _rrtTree;
					current_fail_number += 1;
					anyway_fail_number += 1;
				}
			}

			if(current_fail_number >= MAX_FAIL_NUMBER)
			{
				if(i > 2 && anyway_fail_number < ANY_WAY_RESTART_COUNT)
				{
					i -= 1;
					int last_size = track_sizes.at(track_sizes.size() - 1);
					track_sizes.pop_back();
					for(int j=0; j<last_size; j++){path_RRT.pop_back();}

					traj last_traj = path_RRT.at(path_RRT.size()-1);
					x_init.x = last_traj.x;
					x_init.y = last_traj.y;
					x_init.th = last_traj.th;
					printf("BACK ONE TRACK!!!!!!!!!!! \n");
					
					continue;
				}

				printf("Failed over %d times!!! Maybe previous path was wrong. Clear all found path and restart path finding!\n", MAX_FAIL_NUMBER);
				break;
			}			
				
		}

		if(i == goal_number)
			break;
	}

	delete _rrtTree;
}
