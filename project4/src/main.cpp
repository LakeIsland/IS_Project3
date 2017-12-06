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
int margin = 7;
int K = 1500;
double MaxStep = 2;

int waypoint_margin = 22;
double CAR_TARGET_MAX_SPEED = 1;
double CAR_TARGET_MIN_SPEED = 1;

int MAX_FAIL_NUMBER = 2;
int ANY_WAY_RESTART_COUNT = 1000;

double PER_BREAK_SEC = 2;
double BREAK_SEC = 2;

int CONTROL_RATE = 60;
int TRACK_NUMBER = 2;

double DIST_SQUARE_TO_CHECK_REACHED = 0.04;

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


PID pid_ctrl;
double last_dist_squared = DOUBLE_INFINITE;


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
                              std::string("/catkin_ws/src/project4/src/slam_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.5;
            world_x_max = 4.5;
            world_y_min = -13.5;
            world_y_max = 13.5;
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
            generate_path_RRT();
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;

        case RUNNING: {
                //TODO 1
		int toofarcount = 0;
		double steering_max = 0;
		double take_break_timer = 1000000000;
		take_break(control_rate, cmd_vel_pub);
	    while(ros::ok()) {
			

			if(take_break_timer >= (CONTROL_RATE * PER_BREAK_SEC))
			{
				//take_break(control_rate, cmd_vel_pub);
				take_break_timer = 0;
			} else {
				take_break_timer += 1;
			}

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

			float control = pid_ctrl.get_control(robot_pose, prev_goal, next_goal, next_goal);

			double abs_control = myabs(control);
			float speed = getLinearlyInterpolatedValue(0, CAR_TARGET_MAX_SPEED, 0.2, CAR_TARGET_MIN_SPEED, abs_control);
			//if(abs_control > steering_max)
			//	steering_max = abs_control;
			setcmdvel(speed, control);
		    	cmd_vel_pub.publish(cmd);
		    	
		    	ros::spinOnce();
	  	  	control_rate.sleep();
		    	
		    	double dx = robot_pose.x - current_goal.x;
		    	double dy = robot_pose.y - current_goal.y;	    	
		    	double dist_squared = dx * dx + dy * dy;

			if(dist_squared < DIST_SQUARE_TO_CHECK_REACHED || (last_dist_squared < dist_squared && dist_squared < 8))
		    	{
				if(!(dist_squared < DIST_SQUARE_TO_CHECK_REACHED)){
					printf("PASSED!!!!!!!!!!!!!!!!!!!!!!!!\n");
					toofarcount++;
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
	clock_t runtime = (time2-time1) / 10000;					
	printf("Finished ROBOT\n");

	printf("Running time = %ld\n", runtime);
	state = FINISH;
}

void set_waypoints()
{
    std::srand(std::time(NULL));
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    cv::Mat map_margin = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

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

    //TODO 2
    // Make your own code to select waypoints.
    // You can randomly sample some points from the map.
    // Also, the car should follow the track in clockwise.

    /*waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;
    waypoint_candid[1].x = 2.0;
    waypoint_candid[1].y = 12.0;
    waypoint_candid[2].x = 3.5;
    waypoint_candid[2].y = -10.5;
    waypoint_candid[3].x = -2.0;
    waypoint_candid[3].y = -12.0;
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 10.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }*/

    //waypoints.push_back(waypoint_candid[0]);
    create_clock_wise_way_points(map_margin, waypoint_candid[0]);
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
	int where = determin_where_point_is(start_position);
	int xs[] = {0,0,1,1};
	int ys[] = {0,1,1,0};

	printf("set random waypoint started\n");
	waypoints.push_back(start_position);
	for(int j=0; j<TRACK_NUMBER; j++)
	{
		for(int i=2; i<=4; i+=2)
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

	//point right_top = find_empty_point(map, 1, 1);
	//point right_bottom = find_empty_point(map, 1, 0);
	//point left_bottom = find_empty_point(map, 0, 0);
	//point left_top = find_empty_point(map, 0, 1);
	
	//waypoints.push_back(right_bottom);
	//waypoints.push_back(left_bottom);
	//waypoints.push_back(left_top);
	//waypoints.push_back(right_top);
	printf("set random waypoint finished\n");
	//waypoints.push_back(right_bottom);
	//waypoints.push_back(left_bottom);
	//waypoints.push_back(left_top);
	//waypoints.push_back(right_top);
	
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

	rrtTree* _rrtTree = new rrtTree(map, map_origin_x, map_origin_y, res, margin);
	std::vector<int> track_sizes;
	
	while(true)
	{
		path_RRT.clear();
		track_sizes.clear();

		int goal_number = waypoints.size();
		int anyway_fail_number = 0;
		point x_init;
		x_init.x = waypoints[0].x;
		x_init.y = waypoints[0].y;
		x_init.th = waypoints[0].th;
		int i = 1;

		while(i<goal_number)
		{
			
			printf("start Finding %d th path.\n", i);

			point x_goal = waypoints[i];
			
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

		if(i==goal_number)
			break;
	}

	delete _rrtTree;
}
