#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define THRESHOLD_R 0.2
#define THRESHOLD_MAX_R 0.6

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     *
    */

    error = 0;
    error_sum = 0;
    error_diff = 0;
    Kp = 1.5;
    Ki = 0;
    Kd = 5; 
}

void PID::clear()
{
	this->error_sum =0;
	this->error = 0;
	this->error_diff = 0;
}


float PID::get_control(point car_pose, traj prev_goal, traj cur_goal) {
    //TODO
	double dx = cur_goal.x - car_pose.x;
	double dy = cur_goal.y - car_pose.y;
	
	double dx2 = prev_goal.x - cur_goal.x;
	double dy2 = prev_goal.y - cur_goal.y;

	double theta_g = atan2(dy,dx);
	double theta_g2 = atan2(dy2,dx2);
	
	double theta_d = cur_goal.th;
	double theta_h = car_pose.th;

	double distance = sqrt(dx * dx + dy * dy);
	
	// max radius -> distance that start to be affected by pre defined theta.
	double max_radius = THRESHOLD_MAX_R;
	
	// linearly interpolate clamped value(function of distance)
	double clamped_value = (distance - max_radius) / (max_radius - THRESHOLD_R);
	
	if(clamped_value > 1)
		clamped_value = 1;
	else if(clamped_value < 0)
		clamped_value = 0;
	
	double k1 = theta_g - theta_h;
	double k2 = theta_g2 - theta_h;
	double kk = lerp(k1, k2, clamped_value);

	// set weight from this clamped value.
	
	double weight_g = clamped_value;
	//double weight_g = clamped_value;
	double weight_d = 1.0 - weight_g;

	//if(weight_g < 1.0f)
	//	printf("weight_g: %.4f, weight_d: %.4f\n", weight_g, weight_d);

	double dt = 1.0 / 10;

	double error = weight_g * theta_g + weight_d * theta_d - theta_h;
	error = clampToPi(error);
	
	double prop_term = (this->Kp) * error;
	double intg_term = (this->Ki) * (this -> error_sum) * dt;
	double diff_term = (this->Kd) * (error - (this->error)) / dt;
	//double diff_term = (this->Kd) * (this->error_diff) / dt;
	
	double result = prop_term + intg_term + diff_term;

	result = clampToPi(result);
	
	
	this->error_diff = (error - (this->error));
	this->error = error;
	this->error_sum += error;
	
	//float alphaMixFactor = clamp(0, 1, 1/distance);
	//float final_result = lerp(result, cur_goal.alpha, alphaMixFactor);

	//result = clamp(-1,1,result);

	//printf("final_result, %.3f, result :%.3f, alpha: %.3f\n", final_result, result, cur_goal.alpha);
	
	//printf("pid result :%.3f, alpha: %.3f\n", result, cur_goal.alpha);
		
	return result;
	
}