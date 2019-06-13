#include "PathPlaner.h"
#include<algorithm>

PATHPLANER::PATHPLANER(SENSOR_FUSION_LIST perception_data,double time_step){
    m_vehicle_localization = perception_data;
	m_time_step = time_step;
    cout<<"Hello Path Planer!"<<endl;

}

PATHPLANER::~PATHPLANER(){

}

vector<float>PATHPLANER::predication_within_lane(int path_size, const double &lane, double future_car_s){
	float too_close = -1;
	float targete_vel = -1;
	double too_close_min = 1000;
	for(int i=0; i<m_vehicle_localization.perception.size(); i++){
		double car_position_d = m_vehicle_localization.perception.at(i).car_position_d;
		if(car_position_d>4*lane && car_position_d<4*lane+4){

			double car_velocity_x = m_vehicle_localization.perception.at(i).car_velocity_x;
			double car_velocity_y = m_vehicle_localization.perception.at(i).car_velocity_y;
			double car_velocity = sqrt(car_velocity_x*car_velocity_x + car_velocity_y*car_velocity_y);
			double front_car_s = m_vehicle_localization.perception.at(i).car_position_s;

			double future_front_car_s = front_car_s+car_velocity*m_time_step*path_size;

			if(future_front_car_s > future_car_s){
				if((future_front_car_s - future_car_s) < too_close_min){
					too_close = future_front_car_s - future_car_s;
					too_close_min = too_close;
					targete_vel = car_velocity;
				}
			}
		}
	}

	return {too_close, targete_vel};
}

vector<float> PATHPLANER::predication(int path_size, const double &lane, double future_car_s){
	float pre_dis_front = -1;
	float pre_dis_left_front = -1;
	float pre_dis_left_back = -1;
	float pre_dis_right_front = -1;
	float pre_dis_right_back = -1;

	vector<float> pre_front;
	vector<float> pre_left_front;
	vector<float> pre_left_back;
	vector<float> pre_right_front;
	vector<float> pre_right_back;
/*
    vector<float> pre_dis_front_vec;
	vector<float> pre_dis_left_front_vec;
	vector<float> pre_dis_left_back_vec;
	vector<float> pre_dis_right_front_vec;
	vector<float> pre_dis_right_back_vec;

	pre_dis_front_vec.push_back(pre_dis_front);
	pre_dis_left_front_vec.push_back(pre_dis_left_front);
	pre_dis_left_back_vec.push_back(pre_dis_left_back);
	pre_dis_right_front_vec.push_back(pre_dis_right_front);
	pre_dis_right_back_vec.push_back(pre_dis_right_back);
*/
     string lane_num = get_main_car_lane();
	 LANE_INFO vehicle_ID_vector = get_other_car_lane(); 

	#if DEBUG_BUTTON
		cout<<"vehicle_ID_lane_0";
		for(int i=0;i<vehicle_ID_vector.vehicle_ID_lane_0.size();i++){
			cout<<","<<vehicle_ID_vector.vehicle_ID_lane_0.at(i);
		}
		cout<<endl;

		cout<<"vehicle_ID_lane_1";
		for(int i=0;i<vehicle_ID_vector.vehicle_ID_lane_1.size();i++){
		cout<<","<<vehicle_ID_vector.vehicle_ID_lane_1.at(i);
		}
		cout<<endl;

		cout<<"vehicle_ID_lane_2";
		for(int i=0;i<vehicle_ID_vector.vehicle_ID_lane_2.size();i++){
		cout<<","<<vehicle_ID_vector.vehicle_ID_lane_2.at(i);
		}
		cout<<endl;

	#endif 
	 
	if(lane_num =="lane_0"){
		
			pre_front = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_0,path_size,future_car_s,"front");
			pre_right_front = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_1,path_size,future_car_s,"front");
			pre_right_back = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_1,path_size,future_car_s,"back");
	
			
		//cout<<"main vehicle is in lane_0:--->"<<lane_num<<endl;
	 }
	else if(lane_num =="lane_1"){
		cout<<"in lane_1"<<endl;
		pre_front = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_1,path_size,future_car_s,"front");
		pre_right_front = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_2,path_size,future_car_s,"front");
		pre_right_back = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_2,path_size,future_car_s,"back");
		pre_left_front = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_0,path_size,future_car_s,"front");
		pre_left_back = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_0,path_size,future_car_s,"back");
		
		
		//cout<<"main vehicle is in lane_1:--->"<<lane_num<<endl;
	}
	else if(lane_num =="lane_2"){
		pre_front = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_2,path_size,future_car_s,"front");
		pre_left_front = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_1,path_size,future_car_s,"front");
		pre_left_back = calc_min_pre_dis(vehicle_ID_vector.vehicle_ID_lane_1,path_size,future_car_s,"back");

		//cout<<"main vehicle is in lane_2:--->"<<lane_num<<endl;
	}
	else if(lane_num =="unknown"){
		
		//cout<<"main vehicle is in unknown:--->"<<lane_num<<endl;
	}

	return {pre_front.at(0),pre_right_front.at(0),pre_right_back.at(0),pre_left_front.at(0),pre_left_back.at(0),\
			pre_front.at(1),pre_right_front.at(1),pre_right_back.at(1),pre_left_front.at(1),pre_left_back.at(1)
		};
}

vector<float> PATHPLANER::predication_neighbour_lane(int path_size, const double &lane, double future_car_s){

	vector<double> future_s_left_car, future_s_right_car;
	vector<double> future_v_left_car, future_v_right_car;
	for(int i=0; i<m_vehicle_localization.perception.size(); i++){
		if(lane > 0){

			double d_lane_l = m_vehicle_localization.perception.at(i).car_position_d;
			if(d_lane_l >4*(lane-1) && d_lane_l<4*(lane-1)+4){

				double car_velocity_x = m_vehicle_localization.perception.at(i).car_velocity_x;
				double car_velocity_y = m_vehicle_localization.perception.at(i).car_velocity_y;
				double car_velocity = sqrt(car_velocity_x*car_velocity_x + car_velocity_y*car_velocity_y);
				double front_car_s = m_vehicle_localization.perception.at(i).car_position_s;

				double future_left_car_s = front_car_s+car_velocity*m_time_step*path_size;


				future_s_left_car.push_back(future_left_car_s);
				future_v_left_car.push_back(car_velocity);
			}
		}


		if(lane <2){
			double d_lane_r = m_vehicle_localization.perception.at(i).car_position_d;
			if(d_lane_r >4*(lane+1) && d_lane_r<4*(lane+1)+4){


				double car_velocity_x = m_vehicle_localization.perception.at(i).car_velocity_x;
				double car_velocity_y = m_vehicle_localization.perception.at(i).car_velocity_y;
				double car_velocity = sqrt(car_velocity_x*car_velocity_x + car_velocity_y*car_velocity_y);
				double right_car_s = m_vehicle_localization.perception.at(i).car_position_s;

				double future_right_car_s = right_car_s+car_velocity*m_time_step*path_size;

				future_s_right_car.push_back(future_right_car_s);
				future_v_right_car.push_back(car_velocity);
			}
		}
	}


	//choice the car front and behind our car in the future time(the curent path end point).
	float left_front_close = -1;
	float left_front_vel = -1;
	float left_after_close = -1;
	float left_after_vel = -1;
	float left_front_min = 1000;
	float left_after_min = 1000;
	for( int i = 0; i<future_s_left_car.size(); i++){

		if(future_s_left_car[i] > future_car_s){
			if((future_s_left_car[i] - future_car_s)< left_front_min){

				left_front_close = future_s_left_car[i] - future_car_s;
				left_front_min = left_front_close;
				left_front_vel = future_v_left_car[i];
			}
		}
		
		if(future_s_left_car[i] < future_car_s){
		
			if((future_car_s - future_s_left_car[i])< left_after_min){
				left_after_close = future_car_s- future_s_left_car[i];
				left_after_min = left_after_close;
				left_after_vel = future_v_left_car[i];
			}
		}
		
	}

	float right_front_close = -1;
	float right_front_vel = -1;
	float right_after_close = -1;
	float right_after_vel = -1;
	float right_front_min = 1000;
	float right_after_min = 1000;
	for( int i = 0; i<future_s_right_car.size(); i++){

		if(future_s_right_car[i] > future_car_s){
			if((future_s_right_car[i] - future_car_s)< right_front_min){
				right_front_close = future_s_right_car[i] - future_car_s;
				right_front_min = right_front_close;
				right_front_vel = future_v_right_car[i];
			}
		}

		if(future_s_right_car[i] < future_car_s){
			if((future_car_s - future_s_right_car[i])< right_after_min){
				right_after_close = future_car_s- future_s_right_car[i];
				right_after_min = right_after_close;
				right_after_vel = future_v_right_car[i];
			}
		}
	}
	//cout<<left_front_close<<"  "<<right_front_close<<endl;

	return {left_front_close, right_front_close, left_after_close, right_after_close, 
       		left_front_vel, right_front_vel, left_after_vel,right_after_vel};
}


bool PATHPLANER::veicle_state_machine(vector<float> prediction_front, vector<float> prediction_left_right,double &ref_vel, double &lane, double max_v){
	double weight_crash = 20;
	double weight_save_time = 50;
	double weight_buffer = 1000;

	double lane_before = lane;
	double cost_keep_lane,cost_change_left, cost_change_right;

	if(prediction_front[0] == -1 || prediction_front[0]>30){   
		if(ref_vel<max_v){
			ref_vel +=0.23;
		}
	}
	else{
		if(ref_vel<max_v + 0.5){
			if(prediction_front[1] > ref_vel){
				ref_vel +=0.23;
			}
			else{
				ref_vel -=exp((ref_vel - prediction_front[1])/(max_v+0.5))/8.0;
			}
		}


		cost_keep_lane = cost_of_all(weight_crash, weight_buffer, weight_save_time,prediction_front[0], -1, ref_vel, prediction_front[1], -1, max_v);


		if(lane > 0){
			cost_change_left =10 + cost_of_all(weight_crash, weight_buffer, weight_save_time,prediction_left_right[0], prediction_left_right[2], ref_vel,prediction_left_right[4],prediction_left_right[6], max_v);
		}
		else{
			cost_change_left = 1000;
		}



		if(lane < 2){
			cost_change_right =12 +  cost_of_all(weight_crash,weight_buffer, weight_save_time,prediction_left_right[1],prediction_left_right[3], ref_vel,prediction_left_right[5], prediction_left_right[7], max_v);
		}
		else{
			cost_change_right = 1000;
		}
 		bool change_left_state = false;
		bool change_right_state = false;

		double min_cost;
		min_cost = cost_keep_lane;

		if(cost_change_left<min_cost){
			min_cost = cost_change_left;
			change_left_state = true;
		}
		if(cost_change_right < min_cost){
			change_right_state = true;
		}

		if(change_right_state){
			if(lane < 2){
				lane += 1;
			}
		} 

		if(change_left_state && !change_right_state){
			if(lane > 0){
				lane -= 1;
			}
		}
	}
	
	double lane_after = lane;
	bool change_lane = false;
	if(abs(lane_before - lane_after) > 0.1){
		change_lane = true;
	}

	return change_lane;
}

vector<vector<double>> PATHPLANER::trajectories_sample(
								  const vector<double> &pre_path_x, 
								  const vector<double> &pre_path_y, 
								  double ref_x,
								  double ref_y, 
								  double ref_yaw, 
								  bool change_lane, 
								  int PATH_SIZE,
								  double car_s, 
								  const vector<double> &map_waypoints_x,
								  const vector<double> &map_waypoints_y, 
								  const vector<double> &map_waypoints_s, 
								  double lane, double ref_vel){
    vector<double> spine_x,spine_y;
	vector<double> next_x_vals, next_y_vals;
	vector<vector<double>> trjectoryXY;
	int path_size = pre_path_x.size();

	if(path_size < 5){
		double pos_x = ref_x - cos(ref_yaw);
		double pos_y = ref_y - sin(ref_yaw);
		spine_x.push_back(pos_x);
		spine_y.push_back(pos_y);

		spine_x.push_back(ref_x);
		spine_y.push_back(ref_y);
	}
	else{
		ref_x = pre_path_x[path_size-1];
		ref_y = pre_path_y[path_size-1];
		double pre_ref_x = pre_path_x[path_size-5];
		double pre_ref_y = pre_path_y[path_size-5];

		spine_x.push_back(pre_ref_x);
		spine_y.push_back(pre_ref_y);

		spine_x.push_back(ref_x);
		spine_y.push_back(ref_y);

		ref_yaw = atan2(ref_y - pre_ref_y, ref_x - pre_ref_x);
	}

	vector<double> getxy1,getxy2,getxy3;
	if(change_lane == true){  
		//set further points to get more smooth trajectory, avoid large lateral jerk.
		getxy1= getXY(car_s + 50, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy2= getXY(car_s + 70, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy3= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	}
	else{
		getxy1= getXY(car_s + 30, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy2= getXY(car_s + 60, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		getxy3= getXY(car_s + 90, 4*lane +2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	}
	spine_x.push_back(getxy1[0]);
	spine_x.push_back(getxy2[0]);
	spine_x.push_back(getxy3[0]);

	spine_y.push_back(getxy1[1]);
	spine_y.push_back(getxy2[1]);
	spine_y.push_back(getxy3[1]);

	//Convert the global coordinate system to the vichle coordinate for math easy.
	for(int i =0; i<spine_x.size(); i++){
		double shifto_car_x = spine_x[i] - ref_x;
		double shifto_car_y = spine_y[i] - ref_y;

		spine_x[i] = shifto_car_x*cos(0-ref_yaw) - shifto_car_y*sin(0-ref_yaw);
		spine_y[i] = shifto_car_x*sin(0-ref_yaw) + shifto_car_y*cos(0-ref_yaw);
	}

	tk::spline s;
	s.set_points(spine_x, spine_y);

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);
	double x_add = 0;

	for(int i = 0; i<pre_path_x.size(); i++) {
		next_x_vals.push_back(pre_path_x[i]);
		next_y_vals.push_back(pre_path_y[i]);
	}

	for(int i = 1; i<= PATH_SIZE - pre_path_x.size(); i++){
		double num = target_dist/(0.02*ref_vel/2.24);
		double x_point = x_add + target_x/num;
		double y_point = s(x_point);

		x_add = x_point;
		double x_ref = x_point;
		double y_ref = y_point;

		x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
		y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}

	trjectoryXY.push_back(next_x_vals);
	trjectoryXY.push_back(next_y_vals);

	return trjectoryXY;
//cout<<"Hello TS!"<<endl;
}

string PATHPLANER::get_main_car_lane(){
            string lane_num;
            double car_position_d = m_vehicle_localization.main_car_localization.car_position_d;

            if(0 <car_position_d && car_position_d<4)
                lane_num = "lane_0";
            else if(4 <car_position_d && car_position_d<8)
                lane_num = "lane_1";
            else if(8 <car_position_d && car_position_d<12)
                lane_num = "lane_2";
            else
                lane_num = "unknown";
            return lane_num;
}

LANE_INFO PATHPLANER::get_other_car_lane(){
            string lane_num;
			LANE_INFO vehicle_ID_vector;

			int car_num = m_vehicle_localization.perception.size();
			vector<float> vehicle_ID_lane_0;
			vector<float> vehicle_ID_lane_1;
			vector<float> vehicle_ID_lane_2;
			vector<float> vehicle_ID_lane_unknow;

			vehicle_ID_lane_0.push_back(0);
			vehicle_ID_lane_1.push_back(1);
			vehicle_ID_lane_2.push_back(2);
			vehicle_ID_lane_unknow.push_back(3);

			//vector<vector<float>> vehicle_ID_vector;

			for(int i=0;i<car_num;i++){
				double car_position_d = m_vehicle_localization.perception.at(i).car_position_d;
				if(0 <car_position_d && car_position_d<4){
					lane_num = "lane_0";
					vehicle_ID_lane_0.push_back(m_vehicle_localization.perception.at(i).car_ID);
				}  
            	else if(4 <car_position_d && car_position_d<8){
					lane_num = "lane_1";
					vehicle_ID_lane_1.push_back(m_vehicle_localization.perception.at(i).car_ID);
				}   	
            	else if(8 <car_position_d && car_position_d<12){
					lane_num = "lane_2";
					vehicle_ID_lane_2.push_back(m_vehicle_localization.perception.at(i).car_ID);
				}            	
            	else
                	lane_num = "unknown";
					vehicle_ID_lane_unknow.push_back(m_vehicle_localization.perception.at(i).car_ID);
			}
			
			vehicle_ID_vector.vehicle_ID_lane_0 = vehicle_ID_lane_0;
			vehicle_ID_vector.vehicle_ID_lane_1 = vehicle_ID_lane_1;
			vehicle_ID_vector.vehicle_ID_lane_2 = vehicle_ID_lane_2;
			vehicle_ID_vector.vehicle_ID_lane_unknow = vehicle_ID_lane_unknow;

			return vehicle_ID_vector;

}

vector<float> PATHPLANER::calc_min_pre_dis(vector<float> vehicle_ID,int path_size,double future_car_s,string calc_mode){
		float pre_delta_dis = -1;
		float pre_delta_vel = -1;
		vector<float> pre_delta_dis_vec_front;
		vector<float> pre_delta_dis_vec_back;
		vector<float> pre_delta_vel_vec_front;
		vector<float> pre_delta_vel_vec_back;

		float pre_delta_dis_min = -1;
		float pre_delta_vel_min = -1;
		//double car_position_d = m_vehicle_localization.perception.at(i).car_position_d;
		if(vehicle_ID.size()==1){
			pre_delta_dis_min = -1;
			pre_delta_vel_min = -1;
		}
		else{
			for(int i=1;i<vehicle_ID.size();i++){
				for(int j=0;j<m_vehicle_localization.perception.size();j++){
					if(vehicle_ID.at(i) == m_vehicle_localization.perception.at(j).car_ID){

						double car_velocity_x = m_vehicle_localization.perception.at(j).car_velocity_x;
						double car_velocity_y = m_vehicle_localization.perception.at(j).car_velocity_y;
						double car_velocity = sqrt(car_velocity_x*car_velocity_x + car_velocity_y*car_velocity_y);
						double car_s = m_vehicle_localization.perception.at(j).car_position_s;

						double pre_car_s = car_s+car_velocity*m_time_step*path_size;
						
						pre_delta_dis = pre_car_s - future_car_s;
						if(pre_delta_dis>0){
							pre_delta_dis_vec_front.push_back(pre_delta_dis);
							pre_delta_vel_vec_front.push_back(car_velocity);
						}
						
						if(pre_delta_dis<0){
							pre_delta_dis_vec_back.push_back(pre_delta_dis);
							pre_delta_vel_vec_back.push_back(car_velocity);
						}
						#ifdef DEBUG_BUTTON
							cout<<"pre_delta_dis-->"<<","<<pre_delta_dis<<endl;
						#endif
					}
				}			
				if(calc_mode == "front"){
					cout<<"in front"<<endl;
					auto min_dis_value_address = min_element(pre_delta_dis_vec_front.begin(), pre_delta_dis_vec_front.end());
					pre_delta_dis_min = *min_dis_value_address;
					int index = distance(pre_delta_dis_vec_front.begin(),min_dis_value_address);
					
					pre_delta_vel_min = pre_delta_vel_vec_front.at(index);

				}		
				if(calc_mode == "back"){
					cout<<"in back"<<endl;
					auto min_dis_value_address = min_element(pre_delta_dis_vec_back.begin(), pre_delta_dis_vec_back.end());
					pre_delta_dis_min = *min_dis_value_address;
					int index = distance(pre_delta_dis_vec_back.begin(),min_dis_value_address);

					pre_delta_vel_min = pre_delta_vel_vec_back.at(index);
				}
			}
		} 	
		return {pre_delta_dis_min,pre_delta_vel_min};
}


