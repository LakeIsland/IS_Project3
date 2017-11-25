#include "rrtTree.h"
#include "custom_math_utils.h"
#include <unistd.h>
#include <ros/ros.h>

double MIN_D = 2;
double max_alpha = 0.2;
double L = 0.325;

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
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
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

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
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

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	for(int j = 0; j < 10; j++) {
	    double alpha = path[i].alpha;
	    double d = path[i].d;
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
    new_node->alpha = alpha;
    new_node->d = d;
    
    this->ptrTable[new_index] = new_node;
    
    this->count++;
}

double TO_GOAL_MAX_MARGIN_SQUARED = 1 * 1;

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO
    int i = 0;
	int anyway_count_i = 0;

    double out[5];
    int ANY_WAY_COUNT_MAX = 5 * K;

	double min_dist_squared_to_goal = DOUBLE_INFINITE;

    while(i < K && anyway_count_i <ANY_WAY_COUNT_MAX)
    {
    	point x_rand = randomState(x_max, x_min, y_max, y_min);

		//double randomPart = min_dist_squared_to_goal / 25;
		//randomPart = clamp(0, 1, randomPart);

		/*if(min_dist_squared_to_goal < 25 && getRandomDouble() < 0.7)
		{
			x_rand.x = lerp(x_goal.x, x_rand.x, 0.5);
			x_rand.y = lerp(x_goal.y, x_rand.y, 0.5);
		}*/

    	int x_near_idx = nearestNeighbor(x_rand, MaxStep);

		if(x_near_idx == -1)	continue;
		
		
    	node *x_near_node = this->ptrTable[x_near_idx];
    	point x_near = x_near_node -> location;
    	
    	int valid = newState(out, x_near, x_rand, MaxStep);
    	anyway_count_i++;

    	if(valid == 0){
			continue;
		}

		i++;
		if(i % 500 == 0)
			printf("%d !!!\n",i);

    	
    	point x_new;
    	
    	x_new.x = out[0];
    	x_new.y = out[1];
    	x_new.th = out[2];
    	
    	addVertex(x_new, x_rand, x_near_idx, out[3], out[4]);
    	
		double dx = x_new.x - x_goal.x;
		double dy = x_new.y - x_goal.y;

		double dist_squared = (dx * dx) + (dy * dy);
		
		if(dist_squared < min_dist_squared_to_goal)
		{
			min_dist_squared_to_goal = dist_squared;
		}

		if(min_dist_squared_to_goal < TO_GOAL_MAX_MARGIN_SQUARED) 
		{
			printf("Find path at %d steps.\n" , i);
			return 1;
			//break;
		}
			
		/*if( i > MAX_ITER)
		{
			
			 for (int j = 1; j < count; j++) {
			    delete ptrTable[j];
			 }	
			count = 1;
			i = 0; 
			min_dist_squared_to_goal = DOUBLE_INFINITE;
			continue;
		}*/			
    }
	if(anyway_count_i == ANY_WAY_COUNT_MAX){
		printf("too many collisions.\n");
	}
	printf("RRT Tree not found.\n");
	return -1;
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
	double theta_max = MaxStep * tan(max_alpha) / L ;
	
	double min_dist_squared = DOUBLE_INFINITE;
	int min_dist_node_index = -1;

	for(int i = 0; i<this->count; i++)
	{
		node* _node = this->ptrTable[i];
		point _point = _node -> location;

		double dx = x_rand.x - _point.x;
		double dy = x_rand.y - _point.y;

		float theta_g = atan2(dy,dx);
		
		float error = theta_g - _point.th;
		
		while(error > M_PI)
			error -= (2 * M_PI);
		while(error <= -M_PI)
			error += (2 * M_PI);

		double dist_squared = (dx * dx) + (dy * dy);
		
		error = error > 0? error : -error;
		if(error < theta_max && dist_squared < min_dist_squared)
		{
			min_dist_squared = dist_squared;
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

		double alpha = (2 * getRandomDouble() - 1) * max_alpha;
		double d = lerp(MIN_D, MaxStep, getRandomDouble());
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
	
	return isCollision(x_near, _point, min_d, R)?0:1;
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {
    //TODO
	//int x1i = x1.x / this->res + this.map_origin_x;
	//int x1j = x1.y / this->res + this.map_origin_y;
	//int x2i = x2.x / this->res + this.map_origin_x;
	//int x2j = x2.y / this->res + this.map_origin_y;
	
	double dist = 0;
	
	double xc = x1.x - R * sin(x1.th);
	double yc = x1.y + R * cos(x1.th);
	
	//printf("is Collision Called at x: %.3f, y: %.3f, th: %.3f, d: %.3f, R: %.3f \n", x1.x, x1.y, x1.th, d, R);
	
	while(dist <= d)
	{
		double beta = dist / R;
		
		double x_prime = xc + R * sin(x1.th + beta);
		double y_prime = yc - R * cos(x1.th + beta);
		
		int i = (int)((x_prime / this->res) + this->map_origin_x);
		int j = (int)((y_prime / this->res) + this->map_origin_y);
		
		if(i<0)
			return true;
		if(i>=800)
			return true;
		if(j<0)
			return true;
		if(j>=800)
			return true;

		int occupied = this->map.at<uchar>(i,j);
		if(occupied <= 125 )
		{
			return true;
		}
		dist += this->res;
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
    	
    	traj _traj;
    	_traj.x = cur_point.x;
    	_traj.y = cur_point.y;
    	_traj.th = cur_point.th;
    	_traj.d = d;
    	_traj.alpha = alpha;
    	
    	total_traj.push_back(_traj);
		
		//if(current_index == 0)
		//	break;
		
    	current_index = cur_node -> idx_parent;
    }



    return total_traj;
    
}
