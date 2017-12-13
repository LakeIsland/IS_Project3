#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define THRESHOLD_R 0.2
#define THRESHOLD_MAX_R 0.6

#define MAX_ROTATE 1
#define I_MAX 1.5
#define I_MIN -1.5

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     *
    */

    error = 0;
    error_sum = 0;
    error_diff = 0;
    Kp = 1;
    Ki = 0;
    Kd = 0; 
}

void PID::clear()
{
	this->error_last = 0;
	this->error_sum =0;
	this->error = 0;
	this->error_diff = 0;
}


float PID::get_control(point car_pose, traj prev_goal, traj cur_goal, traj next_goal) {
    	//TODO
	double dx = cur_goal.x - car_pose.x;
	double dy = cur_goal.y - car_pose.y;

	double dx2 = next_goal.x - car_pose.x;
	double dy2 = next_goal.y - car_pose.y;
	
	double theta_g = atan2(dy,dx);
	double theta_g2 = atan2(dy2,dx2);
	
	double theta_d = cur_goal.th;
	double theta_h = car_pose.th;

	double distance = sqrt(dx * dx + dy * dy);
	
	// max radius -> distance that start to be affected by pre defined theta.
	double max_radius = THRESHOLD_MAX_R;
	
	// linearly interpolate clamped value(function of distance)
	double clamped_value = (distance - THRESHOLD_R) / (max_radius - THRESHOLD_R);
	clamped_value = clamp(0,1,clamped_value);

	// set weight from this clamped value.
	
	double weight_g = clamped_value;
	double weight_d = 1.0 - weight_g;

	//if(weight_g < 1.0f)
	//	printf("weight_g: %.4f, weight_d: %.4f\n", weight_g, weight_d);

	double dt = 1.0 / 60;

	double error = weight_g * theta_g + weight_d * theta_d - theta_h;

	//double error = lerp(theta_g, theta_g2, clamped_value) - theta_h;
	//double error = theta_g - theta_h;
	error = clampToPi(error);


	this->error = error;
	double prop_term = (this->Kp) * (this -> error);
	
	this->error_sum = this->error_sum + dt * this -> error;
	double intg_term = (this->Ki) * (this -> error_sum);
	if(intg_term > I_MAX)
	{
		intg_term = I_MAX;
		if(this->Ki != 0)
			this->error_sum = intg_term / this->Ki;
	}
	else if(intg_term < I_MIN)
	{
		intg_term = I_MIN;
		if(this->Ki != 0)
			this->error_sum = intg_term / this->Ki;
	}
	//intg_term = clamp(-I_MAX, I_MAX, intg_term);
	
	this->error_diff = (this->error - this->error_last) / dt;
	this->error_last = this->error;
	double diff_term = (this->Kd) * (this->error_diff);

	//this->error = error;
	//this->error_sum += error;
	
	//double prop_term = (this->Kp) * (this -> error);
	//double intg_term = (this->Ki) * (this -> error_sum);
	//double diff_term = (this->Kd) * (error - (this->error)) / dt;
	
	//prop_term = clamp(-MAX_FOR_P, MAX_FOR_P, prop_term);
	//diff_term = clamp(-MAX_FOR_D, MAX_FOR_D, diff_term);
	
	double result = (prop_term + intg_term + diff_term);
	//double result = (prop_term + diff_term);
	
	//result = clampToPi(result);(??)
	
	//float alphaMixFactor = clamp(0, 1, 1/distance);
	//float final_result = lerp(result, cur_goal.alpha, alphaMixFactor);

	
	//printf("final_result, %.3f, result :%.3f, alpha: %.3f\n", final_result, result, cur_goal.alpha);
	//if(myabs(result) > MAX_ROTATE)
	printf("car : (%.3f, %.3f), dest : (%.3f, %.3f), theta: %.3f, car th: %.3f , error : %.3f, result %.3f\n",car_pose.x,car_pose.y,cur_goal.x, cur_goal.y,theta_g,theta_h,error,result);
	
	//printf("dx: %.3f, dy: %.3f, theta: %.3f, car th: %.3f , error : %.3f, result %.3f\n",dx,dy,theta_g,theta_h,error,result);
		
	//printf("pid result :%.3f, alpha: %.3f, error: %.3f\n", result, cur_goal.alpha, error);
	result = clamp(-MAX_ROTATE, MAX_ROTATE,result);
	
	return result;
	
}
