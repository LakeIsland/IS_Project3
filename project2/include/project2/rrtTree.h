#include <iostream>
#include <cstdlib>
#include <climits>
#include <ctime>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#ifndef POINT_H
#define POINT_H
#include <project2/point.h>
#include <project2/control.h>
#include <project2/traj.h>
#endif

class rrtTree
{
private:
	struct node
	{
		int idx;
		point rand;
		point location;
		int idx_parent;
		double alpha;
		double d;
		bool is_parent;
		double cost;

	}*root;

	int count;
	point x_init, x_goal;
	cv::Mat map;
	cv::Mat map_original;
	double map_origin_x, map_origin_y;
	double res;
	node *ptrTable[20000];
	bool is_finished_to_find_near_goal;
	
	bool must_go_outside;

	int allowed_left_grid;
	int allowed_right_grid;
	int allowed_top_grid;
	int allowed_bottom_grid;

	cv::Mat addMargin(cv::Mat map, int margin);
	void addVertex(point x_new, point x_rand, int idx_near, double alpha, double d);
	int nearestNeighbor(point x_rand, double MaxStep);
	int nearestNeighbor(point x_rand);
	bool isCollisionInLine(point x1, double d);
	
	/*-----------추가 함수 ----------------*/
	double findMaxDinDirection(point x);	// x 에서 x th 방향으로 추가적으로 충돌 없이 더 갈 수 있는 최대 거리.
	void addRandomLinearPathToLastNode();	// 랜덤 거리를 가지는 직선을 맨 마지막 노드에 추가.
	void addMaxLinearPathToLastNode();		// 위에서 구한 최대 거리를 가지는 직선을 맨 마지막 노드에 추가

	void check_near_goal_point();			// goal 에 도달했는지 여부 확인. is_finished_to_find_near_goal에 저장됨.
	bool isCollidedWithMargin(int x, int y, int radius);
	bool isObstacleAt(int x, int y);

	bool isCollision(point x1, point x2, double d, double alpha);
	point randomState(double x_max, double x_min, double y_max, double y_min);
	int newState(double *out, point x_near, point x_rand, double MaxStep);


public:
	rrtTree();
	rrtTree(cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin);
	rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin);
	rrtTree(point x_init, point x_goal);
	~rrtTree();

	void clearTree();
	void setNewPoint(point x_init, point x_goal);

	void visualizeTree();
	void visualizeTree(std::vector<traj> path);
	int generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep);
	std::vector<traj> backtracking_traj();

	void set_must_go_outside(bool go_outside);
	void set_allowed_meter(double l, double r, double t, double b);
};


double dist(point p1, point p2);
double dist_squared(point p1, point p2);

