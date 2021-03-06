#include "rrtTree.h"
#include "custom_math_utils.h"
#include <unistd.h>
#include <ros/ros.h>

double MIN_D = 1;
double MAX_D = 2;
double MAX_ALPHA = 0.18;
double L = 0.325;

int RRT_DEBUG = 1;			// 디버그 출력 여부
int ANY_WAY_K_COEFFICIENT = 5;		// 충돌 포함해서 셀 최대 횟수 (5 * K번)
double DIRECTION_SEARCH_MARGIN = 1;	// 직선경로 탐색에서 마지막에 고려할 마진 (0이면 너무 벽에 딱 붙음)
double TO_GOAL_MAX_MARGIN= 0.6;		// 최종 목표지점 허용 오차.

double MIN_CURVE_MARGIN = 0.01;		// 실제 거리.
double MAX_CURVE_MARGIN = 0.03;		// 실제 거리.

double DIST_COLLISION_MARGIN_COEFFICIENT = 0.015; // margin per meter

double FINAL_WAYPOINT_DIST = 0.4;	// 경로를 더 잘게 쪼갬.
bool CONSTANT_DIST = false;		// 일정 길이로 쪼갤지 개수 기준으로 쪼갤지.
bool DIVIDE_SMALLER = false;		// 경로를 더 잘게 쪼갤지 여부.


double FEW_METER_NEAR_GOAL = 5;		// 최종 점 근처에서 찾기 시작할 거리.
	

double dist_squared(point p1, point p2)
{
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return dx * dx + dy * dy;
}

double dist(point p1, point p2)
{
	return sqrt(dist_squared(p1, p2));
}


rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
}

rrtTree::~rrtTree(){
    for (int i = 0; i < count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

void rrtTree::clearTree()
{
	for (int i = 0; i < count; i++) {
        delete ptrTable[i];
    }
}

void rrtTree::setNewPoint(point x_init, point x_goal)
{
	this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;

	this->is_finished_to_find_near_goal = false;
	this->is_near_goal_in_few_meters = false;
}


cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
    
    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
		
	    double alpha = this->ptrTable[i]->alpha;
		if(alpha == 0)
		{
			
            double p1_x = this->ptrTable[idx_parent]->location.x;
	    	double p1_y = this->ptrTable[idx_parent]->location.y;
            double p2_x = this->ptrTable[i]->location.x;
			double p2_y = this->ptrTable[i]->location.y;

			x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
		} else {
			for(int j = 0; j < 10; j++) {
				double d = this->ptrTable[i]->d;
				double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
				double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
				double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
				double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
				double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
				double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
				x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
				x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
				cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
			}
		}
	
    }
    cv::namedWindow("Mapping");
    //cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    //cv::imshow("Mapping", imgResult(imgROI));
	cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
	
	for(int i=0; i<path.size() - 1;i++)
	{
		if( i ==0 || i == path.size() - 1)
			cv::circle(imgResult, cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
		else
			//cv::drawMarker(imgResult, cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x))), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 5, 1, 8);
		
			cv::circle(imgResult, cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x))), radius, cv::Scalar(255, 0, 0), CV_FILLED);

	}
    
    for(int i = 1; i < this->count; i++) {
		idx_parent = this->ptrTable[i]->idx_parent;
        double alpha = this->ptrTable[i]->alpha;
		double d = this->ptrTable[i]->d;

		if(alpha == 0)
		{
			
            double p1_x = this->ptrTable[idx_parent]->location.x;
	    	double p1_y = this->ptrTable[idx_parent]->location.y;
			double p2_x = this->ptrTable[i]->location.x;
			double p2_y = this->ptrTable[i]->location.y;
			
            //double p2_x = p1_x + d* cos(this->ptrTable[idx_parent]->location.th);
			//double p2_y = p1_y + d* sin(this->ptrTable[idx_parent]->location.th);

			x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
		} else {
			for(int j = 0; j < 10; j++) {
			
			double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
		    double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
		    double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
			double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
		    double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
			double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
		    x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
		    x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
		    cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
			}
		}

	
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	 	double alpha = path[i].alpha;
		double d = path[i].d;

		if(alpha == 0)
		{
            double p1_x = path[i-1].x;
	    	double p1_y = path[i-1].y;
            double p2_x = path[i].x;
			double p2_y = path[i].y;

			x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
		} else 
		{
			for(int j = 0; j < 10; j++) {
				
				double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
				double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
				double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
				double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
				double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
				double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
				x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
				x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
				cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
			}
		}

	
    }
    cv::namedWindow("Mapping");
    //cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    //cv::imshow("Mapping", imgResult(imgROI));
	cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}



void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
    //TODO

    int new_index = this->count;
    
    node* new_node = new node();
    new_node->idx = new_index;
    new_node->rand = x_rand;
    new_node->location = x_new;
    new_node->idx_parent = idx_near;

	this->ptrTable[idx_near]->is_parent = true;

    new_node->alpha = alpha;
    new_node->d = d;
	new_node->cost =  this->ptrTable[idx_near]->cost + d;
    
    this->ptrTable[new_index] = new_node;
    
    this->count++;
}

void rrtTree::check_near_goal_point()
{
	point x_new = this->ptrTable[this->count - 1]->location;
	double dist_to_goal = dist(x_new, x_goal);
	
	this->is_finished_to_find_near_goal = false;
	this->is_near_goal_in_few_meters = false;
	
	if(dist_to_goal < TO_GOAL_MAX_MARGIN)
	{
		this->is_finished_to_find_near_goal = true;
	}
		
	else if(dist_to_goal < FEW_METER_NEAR_GOAL)
	{
		this->is_near_goal_in_few_meters = true;
	}
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO
	int anyway_count_i = 0;

    double out[5];
    int ANY_WAY_COUNT_MAX = ANY_WAY_K_COEFFICIENT * K;

	double min_dist_squared_to_goal = DOUBLE_INFINITE;
	addRandomLinearPathToLastNode();
	//addMaxLinearPathToLastNode();

    while(this->count < K && anyway_count_i <ANY_WAY_COUNT_MAX && !(this->is_finished_to_find_near_goal))
    {
		
		anyway_count_i++;

    	point x_rand = randomState(x_max, x_min, y_max, y_min);
    	
    	
    	// if close enough find a random point near the goal.
		if(this->is_near_goal_in_few_meters && getRandomDouble() < 0.7)
		{
			x_rand.x = lerp(x_rand.x, x_goal.x, 0.5);
			x_rand.y = lerp(x_rand.y, x_goal.y, 0.5);
		}
		
		// find nearest neighbor
    	int x_near_idx = nearestNeighbor(x_rand, MAX_D);

		// if cannot find neighbor, continue.
		if(x_near_idx == -1)	continue;
		

    	node *x_near_node = this->ptrTable[x_near_idx];
    	point x_near = x_near_node -> location;

    	
    	int valid = newState(out, x_near, x_rand, MAX_D);    	
    	
    	if(valid == 0){
			continue;
		}

		if(this->count % 500 == 0){if(RRT_DEBUG)printf("%d !!!\n",this->count);}
		
		
		// add new point
    	point x_new;
    	x_new.x = out[0];
    	x_new.y = out[1];
    	x_new.th = out[2];
    	
		addVertex(x_new, x_rand, x_near_idx, out[3], out[4]);
		check_near_goal_point();

		// add random lines.
		addRandomLinearPathToLastNode();
		//addMaxLinearPathToLastNode();

    }
    
	if(this->is_finished_to_find_near_goal)
	{
		if(RRT_DEBUG)printf("FIND PATH in %d steps.\n", this->count);
		return 1;
	}
	if(anyway_count_i >= ANY_WAY_COUNT_MAX){
		if(RRT_DEBUG)printf("too many collisions.\n");
	}
	if(RRT_DEBUG)printf("RRT Tree not found.\n");
	return -1;
}

void rrtTree::addRandomLinearPathToLastNode()
{
	point x_new = this->ptrTable[this->count-1]->location;
	point x_rand = this->ptrTable[this->count-1]->rand;
	double random_length = lerp(MIN_D, MAX_D, getRandomDouble());
	int parent_index = this->count - 1;

	if(!isCollisionInLine(x_new,random_length))
	{
		point x_new2;
		x_new2.x = x_new.x + random_length * cos(x_new.th);
		x_new2.y = x_new.y + random_length * sin(x_new.th);
		x_new2.th = x_new.th;
		addVertex(x_new2, x_rand, parent_index, 0, random_length);
	}
	return;
}

void rrtTree::addMaxLinearPathToLastNode()
{
	point x_new = this->ptrTable[this->count-1]->location;
	point x_rand = this->ptrTable[this->count-1]->rand;
	double direction_maxd = findMaxDinDirection(x_new);
	int parent_index = this->count - 1;
	
	while(direction_maxd > MIN_D)
	{
		point x_new2;
		x_new2.x = x_new.x + direction_maxd * cos(x_new.th);
		x_new2.y = x_new.y + direction_maxd * sin(x_new.th);
		x_new2.th = x_new.th;
		addVertex(x_new2, x_rand, parent_index, 0, direction_maxd);
		direction_maxd -= lerp(MIN_D, MAX_D, getRandomDouble());
	}
	
	/*if(direction_maxd > MIN_D)
	{
		point x_new2;
		x_new2.x = x_new.x + direction_maxd * cos(x_new.th);
		x_new2.y = x_new.y + direction_maxd * sin(x_new.th);
		x_new2.th = x_new.th;
		addVertex(x_new2, x_rand, parent_index, 0, direction_maxd);
	}*/
	return;
}


point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    //TODO
    double x_rand = getRandomDouble();
    double y_rand = getRandomDouble();
    
    double nx = x_min + (x_max - x_min) * x_rand;
    double ny = y_min + (y_max - y_min) * y_rand;
    
    point _point;
    _point.x = nx;
    _point.y = ny;
    
    return _point;
}

point randomStateNearGoal(double x_max, double x_min, double y_max, double y_min){
	double x_rand = 2 * getRandomDouble() - 1;
	double y_rand = 2 * getRandomDouble() - 1;
    
	double nx = x_min + (x_max - x_min) * x_rand;
	double ny = y_min + (y_max - y_min) * y_rand;
    
	point _point;
	_point.x = nx;
	_point.y = ny;
    
	return _point;
}


int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO
	double theta_max = MaxStep * tan(MAX_ALPHA) / L ;
	
	double min_dist = DOUBLE_INFINITE;
	int min_dist_node_index = -1;

	for(int i = 0; i<this->count; i++)
	{
		node* _node = this->ptrTable[i];
		point _point = _node -> location;

		double dx = x_rand.x - _point.x;
		double dy = x_rand.y - _point.y;

		float theta_g = atan2(dy,dx);
		
		float error = theta_g - _point.th;
		error = clampToPi(error);
		error = error > 0? error : -error;

		double _dist = dist(x_rand, _point);

		double cost = _dist;
		
		if(error < theta_max && cost < min_dist)
		{
			min_dist = cost;
			min_dist_node_index = i;
		}
	}

	return min_dist_node_index;
     
}

int rrtTree::nearestNeighbor(point x_rand) {
    //TODO
    double min_dist_squared = DOUBLE_INFINITE;
    int min_dist_node_index = 0;
    
    for(int i = 0; i<this->count; i++)
    {
    	node* _node = this->ptrTable[i];
    	point _point = _node -> location;
    	
    	double dx = x_rand.x - _point.x;
    	double dy = x_rand.y - _point.y;
    	
    	double dist_squared = (dx * dx) + (dy * dy);
    	if(dist_squared < min_dist_squared)
    	{
    		min_dist_squared = dist_squared;
    		min_dist_node_index = i;
    	}
    }
    
    return min_dist_node_index;
}

point calc_new_point(point start_point, double d, double alpha)
{
	point _point;

	if(alpha == 0)
	{
		_point.x = start_point.x + d * cos(start_point.th);
		_point.y = start_point.y + d * sin(start_point.th);
		_point.th = start_point.th;
	}
	else
	{
		double R = L / tan(alpha);
		double xc = start_point.x - R * sin(start_point.th);
		double yc = start_point.y + R * cos(start_point.th);
	
		double beta = (d / R);
		double x_prime = xc + R * sin(start_point.th + beta);
		double y_prime = yc - R * cos(start_point.th + beta);
		double th_prime = start_point.th + beta;
		_point.x = x_prime;
		_point.y = y_prime;
		_point.th = th_prime;
	
	}

	return _point;
}

int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep) {
    //TODO
	double x_near_sin = sin(x_near.th);
	double x_near_cos = cos(x_near.th);

	double dx = x_near.x - x_goal.x;
	double dy = x_near.y - x_goal.y;
    
	double dist_to_goal = dx * dx + dy * dy;
	double dist_factor = clamp(0,1,dist_to_goal / 25);
	

	double min_dist_squared = DOUBLE_INFINITE;
	double min_x, min_y, min_th, min_alpha, min_d;

	int ITER_N = 20;
	for(int i=0; i<ITER_N; i++)
	{

		double alpha = (2 * getRandomDouble() - 1) * MAX_ALPHA;
		double d = lerp(MIN_D, MAX_D, getRandomDouble());
		double R = L / tan(alpha);
		
		double xc = x_near.x - R * x_near_sin;
		double yc = x_near.y + R * x_near_cos;
		
		double beta = (d / R);
		double x_prime = xc + R * sin(x_near.th + beta);
		double y_prime = yc - R * cos(x_near.th + beta);
		
		double dx = x_prime - x_rand.x;
		double dy = y_prime - x_rand.y;
		
		double dist_squared = dx * dx + dy * dy;
		
		if(dist_squared < min_dist_squared)
		{
			min_dist_squared = dist_squared;
			
			min_x = x_prime;
			min_y = y_prime;
			min_th = x_near.th + beta;
			min_alpha = alpha;
			min_d = d;
		}
	}
	
	out[0] = min_x;
	out[1] = min_y;
	out[2] = min_th;
	out[3] = min_alpha;
	out[4] = min_d;
	
	point _point;
	_point.x = min_x;
	_point.y = min_y;
	_point.th = min_th;
	
	double R = L / tan(min_alpha);
	
	return isCollision(x_near, _point, min_d, min_alpha)?0:1;
}

bool rrtTree::isCollision(point x1, point x2, double d, double alpha) {
    //TODO
	double R = L / tan(alpha);
	double dist = 0;
	double xc = x1.x - R * sin(x1.th);
	double yc = x1.y + R * cos(x1.th);
	double alpha_magnitude = myabs(alpha)/MAX_ALPHA;

	double offset = lerp(MIN_CURVE_MARGIN, MAX_CURVE_MARGIN, alpha_magnitude);//0;//(CURVE_COLLISION_MARGIN / this->res) / R;
	

	while(dist <= d)
	{
		int collision_margin = (int)((dist * DIST_COLLISION_MARGIN_COEFFICIENT + offset) / this->res);

		double beta = dist / R;
		
		double x_prime = xc + R * sin(x1.th + beta);
		double y_prime = yc - R * cos(x1.th + beta);
		
		int i = (int)((x_prime / this->res) + this->map_origin_x);
		int j = (int)((y_prime / this->res) + this->map_origin_y);
		bool collided = isCollidedWithMargin(i,j, collision_margin);
		if(collided)
			return true;
		dist += this->res;
	}
	return false;
}



bool rrtTree::isCollisionInLine(point x1, double d) {

	double dist = 0;
	double xc = x1.x ;
	double yc = x1.y ;
	
	
	while(dist <= d)
	{
		int collision_margin = (int)((dist * DIST_COLLISION_MARGIN_COEFFICIENT + MIN_CURVE_MARGIN) / this->res);

		double x_prime = xc + dist * cos(x1.th);
		double y_prime = yc + dist * sin(x1.th);
		int i = (int)((x_prime / this->res) + this->map_origin_x);
		int j = (int)((y_prime / this->res) + this->map_origin_y);
		bool collided = isCollidedWithMargin(i,j, collision_margin);
		if(collided)
			return true;

		dist += this->res;
	}
	
	return false;
}


double rrtTree::findMaxDinDirection(point x1) {

	double dist = 0;
	double xc = x1.x ;
	double yc = x1.y ;
	
	while(true)
	{
		int collision_margin = (int)((dist * DIST_COLLISION_MARGIN_COEFFICIENT + MIN_CURVE_MARGIN) / this->res);

		double x_prime = xc + dist * cos(x1.th);
		double y_prime = yc + dist * sin(x1.th);
		
		int i = (int)((x_prime / this->res) + this->map_origin_x);
		int j = (int)((y_prime / this->res) + this->map_origin_y);
		bool collided = isCollidedWithMargin(i,j, collision_margin);
		if(collided)
			return dist - DIRECTION_SEARCH_MARGIN;
		dist += this->res;
	}
	return -1;
}

bool rrtTree::isCollidedWithMargin(int x, int y, int radius)
{
	if(radius == 0)
		return isObstacleAt(x, y);

	int min_x = (x - radius);
	int max_x = (x + radius);
	int min_y = (y - radius);
	int max_y = (y + radius);

	if(min_x < 0 || max_x >= this->map.rows||min_y < 0|| max_y >= this->map.cols)
		return true;
	
	int radius_squared = (int)(radius * radius);
	for(int i = (-radius); i <= (radius); i++)
	{
		for(int j = (-radius); j<= (radius); j++)
		{
			if ((i) * (i) + (j)* (j) > radius_squared )
				continue;

			if(isObstacleAt(i+x,j+y))
				return true;

			//int occupied = this->map.at<uchar>(i + x, j + y);
			//if(occupied <= 125)
			//{
			//	return true;
			//}
		}
		
	}
	return false;
}

bool rrtTree::isObstacleAt(int x, int y)
{
	if(x < 0 || x >= this->map.rows||y < 0|| y >= this->map.cols)
		return true;

	if(this->must_go_outside)
	{
		if(x > this->allowed_left_grid 
		&& x < this->map.rows - this->allowed_right_grid 
		&& y > this->allowed_top_grid
		&& y < this->map.cols - this->allowed_bottom_grid)
		return true; 
	}
	int occupied = this->map.at<uchar>(x, y);
	if(occupied <= 125)
	{
		return true;
	}
	return false;
}

std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
    int current_index = nearestNeighbor(this->x_goal);
    node* cur_node = this->ptrTable[current_index];	
    point cur_point = cur_node -> location;

	float dx = cur_point.x - x_goal.x;
	float dy = cur_point.y - x_goal.y;
	float dist = sqrt(dx * dx + dy * dy);

	printf("Dist to goal is %.4f\n", dist);
    	
    std::vector<traj> total_traj;
    
    while(current_index > 0)
    {
    	node* cur_node = this->ptrTable[current_index];
    	
    	point cur_point = cur_node -> location;
    	
    	double alpha = cur_node->alpha;
    	double d = cur_node->d;

		if(!DIVIDE_SMALLER)
		{
			traj _traj;
			_traj.x = cur_point.x;
			_traj.y = cur_point.y;
			_traj.th = cur_point.th;
			_traj.d = d;
			_traj.alpha = alpha;
		
			total_traj.push_back(_traj);
		}

		else
		{
		if(CONSTANT_DIST)
		{
			double accum_dist = 0;
			double left_amount = d;

			while(accum_dist < d)
			{
				point _point = calc_new_point(cur_point, -accum_dist, alpha);
				
				traj _traj;
				_traj.x = _point.x;
				_traj.y = _point.y;
				_traj.th = _point.th;
				_traj.d = FINAL_WAYPOINT_DIST < left_amount ? FINAL_WAYPOINT_DIST : left_amount ;
				_traj.alpha = alpha;

				accum_dist += FINAL_WAYPOINT_DIST;
				left_amount -= FINAL_WAYPOINT_DIST;
			
				total_traj.push_back(_traj);
			}

		}
		else
		{
			int count = 1;
			while(true)
			{
				if(d / count < FINAL_WAYPOINT_DIST)
					break;
				count += 1;
			}
			double unit_dist = d / count;
			double accum_dist = 0;
			for(int i=0; i<count;i++)
			{
				point _point = calc_new_point(cur_point, -accum_dist, alpha);
				
				traj _traj;
				_traj.x = _point.x;
				_traj.y = _point.y;
				_traj.th = _point.th;
				_traj.d = unit_dist;
				_traj.alpha = alpha;
			
				total_traj.push_back(_traj);
				accum_dist += unit_dist;
			}
		}

		}
		
		//if(current_index == 0)
		//	break;
		
    	current_index = cur_node -> idx_parent;
    }

    return total_traj;
    
}

void rrtTree::set_must_go_outside(bool go_outside)
{
	if(go_outside)
	printf("Forced to go out side!\n");
	else
	printf("Not go out side!\n");
	
	this->must_go_outside = go_outside;
}

void rrtTree::set_allowed_meter(double l, double r, double t, double b)
{
	this-> allowed_left_grid = (int)(l/this->res);
	this-> allowed_right_grid = (int)(r/this->res);
	this-> allowed_top_grid = (int)(t/this->res);
	this-> allowed_bottom_grid = (int)(b/this->res);
}

