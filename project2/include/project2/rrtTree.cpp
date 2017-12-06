#include "rrtTree.h"
#include "custom_math_utils.h"
#include <unistd.h>
#include <ros/ros.h>

double MIN_D = 1;
double MAX_D = 2;
double max_alpha = 0.16;
double L = 0.325;

int RRT_DEBUG = 1;				// 디버그 출력 여부
int ANY_WAY_K_COEFFICIENT = 5;	// 충돌 포함해서 셀 최대 횟수 (5 * K번)
double DIRECTION_SEARCH_MARGIN = 3;	// 직선경로 탐색에서 마지막에 고려할 마진 (0이면 너무 벽에 딱 붙음)
double TO_GOAL_MAX_MARGIN= 1.0;		// 최종 목표지점 허용 오차.

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

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

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
    
    this->ptrTable[new_index] = new_node;
    
    this->count++;
}

void rrtTree::check_near_goal_point()
{
	point x_new = this->ptrTable[this->count - 1]->location;
	if(dist(x_new, x_goal) < TO_GOAL_MAX_MARGIN)
	{
		this->is_finished_to_find_near_goal = true;
	}
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO
	int anyway_count_i = 0;

    double out[5];
    int ANY_WAY_COUNT_MAX = ANY_WAY_K_COEFFICIENT * K;

	double min_dist_squared_to_goal = DOUBLE_INFINITE;
	addRandomLinearPathToLastNode();
	addMaxLinearPathToLastNode();

    while(this->count < K && anyway_count_i <ANY_WAY_COUNT_MAX && !(this->is_finished_to_find_near_goal))
    {
		
		anyway_count_i++;

    	point x_rand = randomState(x_max, x_min, y_max, y_min);

    	int x_near_idx = nearestNeighbor(x_rand, MAX_D);

		if(x_near_idx == -1)	continue;
		
    	node *x_near_node = this->ptrTable[x_near_idx];
    	point x_near = x_near_node -> location;
    	
    	int valid = newState(out, x_near, x_rand, MAX_D);
    	

    	if(valid == 0){
			continue;
		}

		if(this->count % 500 == 0){if(RRT_DEBUG)printf("%d !!!\n",this->count);}

    	point x_new;
    	x_new.x = out[0];
    	x_new.y = out[1];
    	x_new.th = out[2];
    	
    	addVertex(x_new, x_rand, x_near_idx, out[3], out[4]);
		check_near_goal_point();

		addRandomLinearPathToLastNode();
		addMaxLinearPathToLastNode();

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

	if(direction_maxd > 0)
	{
		point x_new2;
		x_new2.x = x_new.x + direction_maxd * cos(x_new.th);
		x_new2.y = x_new.y + direction_maxd * sin(x_new.th);
		x_new2.th = x_new.th;
		addVertex(x_new2, x_rand, parent_index, 0, direction_maxd);
	}
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
		if(i>=this->map.rows)
			return true;
		if(j<0)
			return true;
		if(j>=this->map.cols)
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

bool rrtTree::isCollisionInLine(point x1, double d) {
    //TODO
	//int x1i = x1.x / this->res + this.map_origin_x;
	//int x1j = x1.y / this->res + this.map_origin_y;
	//int x2i = x2.x / this->res + this.map_origin_x;
	//int x2j = x2.y / this->res + this.map_origin_y;
	
	double dist = 0;
	
	double xc = x1.x ;
	double yc = x1.y ;
	
	//printf("is Collision Called at x: %.3f, y: %.3f, th: %.3f, d: %.3f, R: %.3f \n", x1.x, x1.y, x1.th, d, R);
	
	while(dist <= d)
	{
		
		double x_prime = xc + dist * cos(x1.th);
		double y_prime = yc + dist * sin(x1.th);
		
		int i = (int)((x_prime / this->res) + this->map_origin_x);
		int j = (int)((y_prime / this->res) + this->map_origin_y);
		
		if(i<0)
			return true;
		if(i>=this->map.rows)
			return true;
		if(j<0)
			return true;
		if(j>=this->map.cols)
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



double rrtTree::findMaxDinDirection(point x1) {
    //TODO
	//int x1i = x1.x / this->res + this.map_origin_x;
	//int x1j = x1.y / this->res + this.map_origin_y;
	//int x2i = x2.x / this->res + this.map_origin_x;
	//int x2j = x2.y / this->res + this.map_origin_y;
	
	double dist = 0;
	
	double xc = x1.x ;
	double yc = x1.y ;
	
	//printf("is Collision Called at x: %.3f, y: %.3f, th: %.3f, d: %.3f, R: %.3f \n", x1.x, x1.y, x1.th, d, R);

	while(true)
	{
		
		double x_prime = xc + dist * cos(x1.th);
		double y_prime = yc + dist * sin(x1.th);
		
		int i = (int)((x_prime / this->res) + this->map_origin_x);
		int j = (int)((y_prime / this->res) + this->map_origin_y);
		
		if(i<0)
			return dist - DIRECTION_SEARCH_MARGIN;
		if(i>=this->map.rows)
			return dist - DIRECTION_SEARCH_MARGIN;
		if(j<0)
			return dist - DIRECTION_SEARCH_MARGIN;
		if(j>=this->map.cols)
			return dist - DIRECTION_SEARCH_MARGIN;

		int occupied = this->map.at<uchar>(i,j);
		if(occupied <= 125 )
		{
			return dist - DIRECTION_SEARCH_MARGIN;
		}
		dist += this->res;
	}
	return -1;
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
