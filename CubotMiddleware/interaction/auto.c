#include "auto.h"
#include "mecanum_chassis.h"

auto_control_flags Auto_flags;
finger_gesture_flags Finger_flags;

/* 操作手控制集成函数 */
void Auto_Control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, Hiwonder_Servo* hiwo_data, custom_robot_data_t* custom){
	land_reset_control(manipulator_right, manipulator_left, auto_flags);
	servo_back_control(manipulator_right, manipulator_left, hiwo_data, auto_flags);
	point_of_view_control(hiwo_data, auto_flags);
	scope_mode_control(hiwo_data, auto_flags);
	clamp_jaw_control(manipulator_right, manipulator_left, auto_flags, custom);
	Controller_mode_start(manipulator_right, manipulator_left, auto_flags, custom);
	finger_data_test(manipulator_right, manipulator_left, custom, &Finger_flags);
	motor_start_control(manipulator_right, manipulator_left, auto_flags);
}

/* 检查手势数据 */
void finger_data_test(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, custom_robot_data_t* custom, finger_gesture_flags* finger_flags){
	finger_flags -> right_thumb_flag = custom -> image_recv.Coordinate.right_thumb_switch;
	finger_flags -> right_index_flag = custom -> image_recv.Coordinate.right_index_switch;
	finger_flags -> right_middle_flag = custom -> image_recv.Coordinate.right_middle_switch;
	finger_flags -> left_thumb_flag = custom -> image_recv.Coordinate.left_thumb_switch;
	finger_flags -> left_index_flag = custom -> image_recv.Coordinate.left_index_switch;
	finger_flags -> left_middle_flag = custom -> image_recv.Coordinate.left_middle_switch;
}

/* 机械臂登岛前复位 */
void land_reset_control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags){
	auto_flags -> lift_complish_flag = mecanum_Recv.lift_complish_flag;
	/* 标志位为0，则检测对应键位，按下则将登岛位置归零 */
	if(auto_flags -> land_flag == 0){
		if(vT13.key_V_flag == 1 || rc_Ctrl.key_V_flag == 1){
			auto_flags -> land_flag = 1;
			auto_flags -> land_count = 0;
		}
	}
	/* 标志位为1，进行复位，复位期间不再检测键位 */
	else if(auto_flags -> land_flag == 1){
		if(auto_flags -> land_count < 1500){//抬升上升
			auto_flags -> pre_lift_flag = 1;
		}
		else if(auto_flags -> land_count > 1500 && auto_flags -> land_count < 3000 && auto_flags -> lift_complish_flag == 1){//抬升已升起时，机械臂复位
			/* 右臂复位 */
			land_point_reset(manipulator_right);
			
			/* 左臂复位 */
			land_point_reset(manipulator_left);
		}
		else if(auto_flags -> land_count > 3000 && auto_flags -> land_count < 4500){//抬升下降
			auto_flags -> pre_lift_flag = 2;
		}

		auto_flags -> land_count ++;
		if(auto_flags -> land_count == 6000){
			auto_flags -> land_flag = 0;
			auto_flags -> pre_lift_flag = 0;
		}
	}
}

/* 图传转头控制 */
void servo_back_control(Manipulator_t *manipulator_right, Manipulator_t *manipulator_left, Hiwonder_Servo* hiwo_data, auto_control_flags* auto_flags){
	if(auto_flags -> servo_back_flag == 0){
		if(mecanum_Recv.back_flag == 1){
			auto_flags -> servo_back_flag = 1;
			auto_flags -> servo_back_count = 0;
			hiwo_data -> pitch_servo.target_angle = 0;
			auto_flags -> step_down_flag = 1;
			auto_flags -> scope_mode_flag = 0;
		}
		else if(mecanum_Recv.back_flag == -1){
			auto_flags -> servo_back_flag = -1;
			auto_flags -> servo_back_count = 0;
			hiwo_data -> pitch_servo.target_angle = 0;
			auto_flags -> step_down_flag = 0;
			auto_flags -> scope_mode_flag = 0;
		}
	}
	
	else if(auto_flags -> servo_back_flag == 1){
		if(auto_flags -> servo_back_count <= 1000){
			/* 舵机平视 */
			hiwo_data -> pitch_servo.position = SERVO_UP_POSITION;
			step_out_point_reset(manipulator_right);
			step_out_point_reset(manipulator_left);
		}
		else if(auto_flags -> servo_back_count > 1000 && auto_flags -> servo_back_count <= 3000){
			/* 舵机头朝后 */
			hiwo_data -> yaw_servo.position = SERVO_BACK_POSITION;
		}
		else if(auto_flags -> servo_back_count > 3000 && auto_flags -> servo_back_count <= 4000){
			/* 舵机低头 */
			hiwo_data -> pitch_servo.position = SERVO_MIDDLE_POSITION;
		}
		else if(auto_flags -> servo_back_count == 5000){
			auto_flags -> servo_back_flag = 0;
		}
		auto_flags -> servo_back_count ++;
	}
	
	else if(auto_flags -> servo_back_flag == -1){
		if(auto_flags -> servo_back_count <= 1000){
			/* 舵机平视 */
			hiwo_data -> pitch_servo.position = SERVO_UP_POSITION;
			land_point_reset(manipulator_right);
			land_point_reset(manipulator_left);
		}
		else if(auto_flags -> servo_back_count > 1000 && auto_flags -> servo_back_count <= 3000){
			/* 舵机头朝前 */
			hiwo_data -> yaw_servo.position = SERVO_FRONT_POSITION;
		}
		else if(auto_flags -> servo_back_count == 4000){
			auto_flags -> servo_back_flag = 0;
		}
		auto_flags -> servo_back_count ++;
	}
}

/* 操作手pitch视角控制 */
void point_of_view_control(Hiwonder_Servo* hiwo_data, auto_control_flags* auto_flags){
	if(auto_flags -> servo_back_flag == 0 && auto_flags -> step_down_flag == 0){
		if((vT13.key_Q_flag == 1 || rc_Ctrl.key_Q_flag == 1) && (vT13.key_ctrl_flag == 0 && rc_Ctrl.key_ctrl_flag == 0)){
			auto_flags -> scope_mode_flag = 0; 
			/* 舵机头向上运动 */
			hiwo_data -> pitch_servo.target_angle += 0.01 * servo_pitch_sensitivity;
		}
		else if((vT13.key_E_flag == 1 || rc_Ctrl.key_E_flag == 1) && (vT13.key_ctrl_flag == 0 && rc_Ctrl.key_ctrl_flag == 0)){
			auto_flags -> scope_mode_flag = 0;
			/* 舵机头向下运动 */
			hiwo_data -> pitch_servo.target_angle -= 0.01 * servo_pitch_sensitivity;
		}
		/* target_angle限幅 */
		hiwo_data -> pitch_servo.target_angle = joints_limit(hiwo_data -> pitch_servo.target_angle, SERVO_DOWN_POSITION - SERVO_UP_POSITION, 0);
		/* 将target_angle转换为舵机角度值并限幅 */
		hiwo_data -> pitch_servo.position = servo_angle_limit((int16_t)(hiwo_data -> pitch_servo.target_angle) + SERVO_UP_POSITION, SERVO_DOWN_POSITION, SERVO_UP_POSITION);
	}
}

void scope_mode_control(Hiwonder_Servo* hiwo_data, auto_control_flags* auto_flags){
	if(vT13.mouse.press_r_flag == 1 || rc_Ctrl.mouse.press_r_flag == 1){
		auto_flags -> scope_filter_count ++;
		if(auto_flags -> scope_filter_count == 20){
			auto_flags -> scope_mode_flag ++;
			auto_flags -> scope_mode_count = 0;
		}
	}
	else{
		auto_flags -> scope_filter_count = 0;
	}
	
	if(auto_flags -> scope_mode_flag % 2 == 1){
		if(auto_flags -> scope_mode_count <= 1000){
			/* 舵机平视 */
			hiwo_data -> pitch_servo.position = SERVO_UP_POSITION;
			hiwo_data -> pitch_servo.target_angle = 0;
		}
		else if(auto_flags -> scope_mode_count > 1000 && auto_flags -> scope_mode_count <= 2000){
			/* 倍镜向上旋转 */
			hiwo_data -> roll_servo.position = SERVO_ROLL_UP_POSITION;
		}
		auto_flags -> scope_mode_count ++;
	}
	else if(auto_flags -> scope_mode_flag % 2 == 0){
		if(auto_flags -> scope_mode_count <= 1000){
			/* 倍镜向下旋转 */
			hiwo_data -> roll_servo.position = SERVO_ROLL_DOWN_POSITION;
		}
		auto_flags -> scope_mode_count ++;
	}
}

void clamp_jaw_control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, custom_robot_data_t* custom){
	/* 夹爪手势检测 */
	/* 右手 */
	if(manipulator_right -> controller_mapping_flag % 2 == 1){
		if(custom -> image_recv.Coordinate.right_thumb_switch == 1 && custom -> image_recv.Coordinate.right_index_switch == 0 && custom -> image_recv.Coordinate.right_middle_switch == 0){
			manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 1;
		}
		else if(custom -> image_recv.Coordinate.right_thumb_switch == 0 && custom -> image_recv.Coordinate.right_index_switch == 0 && custom -> image_recv.Coordinate.right_middle_switch == 0){
			manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 0;
		}
	}
	/* 键位Ctrl+E控制右臂夹爪 */
	else if(manipulator_right -> controller_mapping_flag % 2 == 0){
		if((rc_Ctrl.key_ctrl_flag == 1 || vT13.key_ctrl_flag == 1) && (rc_Ctrl.key_E_flag == 1 || vT13.key_E_flag == 1)){
			auto_flags -> right_clamp_jaw_key_count ++;
			if(auto_flags -> right_clamp_jaw_key_count == Clamp_Jaw_Close_Filter_Limit_Time){
				auto_flags -> right_clamp_jaw_key_flag ++;
				manipulator_right -> clamp_jaw.clamp_jaw_close_flag = auto_flags -> right_clamp_jaw_key_flag % 2;
			}
		}
		else{
			auto_flags -> right_clamp_jaw_key_count = 0;
		}
	}

	/* 左手 */
	if(manipulator_left -> controller_mapping_flag % 2 == 1){
		if(custom -> image_recv.Coordinate.left_thumb_switch == 1 && custom -> image_recv.Coordinate.left_index_switch == 0 && custom -> image_recv.Coordinate.left_middle_switch == 0){
			manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 1;
		}
		else if(custom -> image_recv.Coordinate.left_thumb_switch == 0 && custom -> image_recv.Coordinate.left_index_switch == 0 && custom -> image_recv.Coordinate.left_middle_switch == 0){
			manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 0;
		}
	}
	/* 键位Ctrl+Q控制左臂夹爪 */
	else if(manipulator_left -> controller_mapping_flag % 2 == 0){
		if((rc_Ctrl.key_ctrl_flag == 1 || vT13.key_ctrl_flag == 1) && (rc_Ctrl.key_Q_flag == 1 || vT13.key_Q_flag == 1)){
			auto_flags -> left_clamp_jaw_key_count ++;
			if(auto_flags -> left_clamp_jaw_key_count == Clamp_Jaw_Close_Filter_Limit_Time){
				auto_flags -> left_clamp_jaw_key_flag ++;
				manipulator_left -> clamp_jaw.clamp_jaw_close_flag = auto_flags -> left_clamp_jaw_key_flag % 2;
			}
		}
		else{
			auto_flags -> left_clamp_jaw_key_count = 0;
		}
	}

	/* 右手夹爪控制 */
	if(manipulator_right -> clamp_jaw.clamp_jaw_close_flag % 2 == 1){
		/* 右臂夹爪闭合 */
		manipulator_right -> clamp_jaw.STS_3215[0].position = SERVO_RIGHT_ZERO_CLOSE_ID_ONE;
		manipulator_right -> clamp_jaw.STS_3215[1].position = SERVO_RIGHT_ONE_CLOSE_ID_THREE;
	}
	else if(manipulator_right -> clamp_jaw.clamp_jaw_close_flag % 2 == 0){
		/* 右臂夹爪张开 */
		manipulator_right -> clamp_jaw.STS_3215[0].position = SERVO_RIGHT_ZERO_OPEN_ID_ONE;
		manipulator_right -> clamp_jaw.STS_3215[1].position = SERVO_RIGHT_ONE_OPEN_ID_THREE;
	}

	/* 左手夹爪控制 */
	if(manipulator_left -> clamp_jaw.clamp_jaw_close_flag % 2 == 1){
		/* 左臂夹爪闭合 */
		manipulator_left -> clamp_jaw.STS_3215[0].position = SERVO_LEFT_ZERO_CLOSE_ID_ZERO;
		manipulator_left -> clamp_jaw.STS_3215[1].position = SERVO_LEFT_ONE_CLOSE_ID_TWO;
	}
	else if(manipulator_left -> clamp_jaw.clamp_jaw_close_flag % 2 == 0){
		/* 左臂夹爪张开 */
		manipulator_left -> clamp_jaw.STS_3215[0].position = SERVO_LEFT_ZERO_OPEN_ID_ZERO;
		manipulator_left -> clamp_jaw.STS_3215[1].position = SERVO_LEFT_ONE_OPEN_ID_TWO;

	}
}

void Controller_mode_start(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, custom_robot_data_t* custom){
	if(auto_flags -> pre_mapping_flag == 0){
		/* 检测准备映射模式按键Ctrl+B */
		if((vT13.key_ctrl_flag == 1 || rc_Ctrl.key_ctrl_flag == 1) && (vT13.key_B_flag == 1 || rc_Ctrl.key_B_flag == 1)){
			auto_flags -> pre_mapping_flag = 1;
			auto_flags -> pre_mapping_count = 0;
			auto_flags -> mapping_exit_count = 0;
		}
	}
	else if(auto_flags -> pre_mapping_flag == 1){
		/* 抬升机构抬升 */
		if(auto_flags -> pre_mapping_count <= 3000){
			auto_flags -> pre_lift_flag = 1;
		}
		/*  */
		if(auto_flags -> pre_mapping_count > 3000 && auto_flags -> pre_mapping_count <= 6000 && auto_flags -> lift_complish_flag == 1){
			/* 右臂复位至零点 */
			zero_point_reset(manipulator_right);
			
			/* 左臂复位至零点 */
			zero_point_reset(manipulator_left);
		}

		if(auto_flags -> lift_complish_flag == 1 && auto_flags -> pre_mapping_count >= 10000 && auto_flags -> mapping_exit_flag == 0){
			/* 检测映射模式手势 */
			if(custom -> image_recv.Coordinate.right_thumb_switch == 1 && custom -> image_recv.Coordinate.right_index_switch == 1 && custom -> image_recv.Coordinate.right_middle_switch == 1){
				manipulator_right -> controller_mapping_count ++;
				if(manipulator_right -> controller_mapping_count == Mapping_Filter_Limit_Time){
					if(manipulator_right -> controller_mapping_flag % 2 == 0){
						/* 此时准备进入映射，先进行一次初始化 */
						Customer_init_image_right(manipulator_right, custom);
					}
					manipulator_right -> controller_mapping_flag ++;
					if(manipulator_right -> controller_mapping_flag % 2 == 0){
						/* 此时准备退出映射模式，机械臂复位至零点 */
						zero_point_reset(manipulator_right);
					}
				}
			}
			else{
				manipulator_right -> controller_mapping_count = 0;
			}

			if(custom -> image_recv.Coordinate.left_thumb_switch == 1 && custom -> image_recv.Coordinate.left_index_switch == 1 && custom -> image_recv.Coordinate.left_middle_switch == 1){
				manipulator_left -> controller_mapping_count ++;
				if(manipulator_left -> controller_mapping_count == Mapping_Filter_Limit_Time){
					if(manipulator_left -> controller_mapping_flag % 2 == 0){
						/* 此时准备进入映射，先进行一次初始化 */
						Customer_init_image_left(manipulator_left, custom);
					}
					manipulator_left -> controller_mapping_flag ++;
					if(manipulator_left -> controller_mapping_flag % 2 == 0){
						/* 退出映射模式，机械臂复位至零点 */
						zero_point_reset(manipulator_left);
					}
				}
			}
			else{
				manipulator_left -> controller_mapping_count = 0;
			}	
		}
		Controller_mode_exit(manipulator_right, manipulator_left, auto_flags);
		auto_flags -> pre_mapping_count ++;
	}
}

void Controller_mode_exit(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags){
	if(auto_flags -> mapping_exit_flag == 0){
		if((vT13.key_ctrl_flag == 1 || rc_Ctrl.key_ctrl_flag == 1) && (vT13.key_shift_flag == 1 || rc_Ctrl.key_shift_flag == 1) && (vT13.key_B_flag == 1 || rc_Ctrl.key_B_flag == 1)){
			auto_flags -> mapping_exit_flag = 1;
			auto_flags -> mapping_exit_count = 0;
			manipulator_right -> controller_mapping_flag = 0;
			manipulator_left -> controller_mapping_flag = 0;
		}
	}
	else if(auto_flags -> mapping_exit_flag == 1){
		/* 复位至登岛姿态 */
		if(auto_flags -> mapping_exit_count == Mapping_Exit_Filter_Limit_Time){
			auto_flags -> land_flag = 1;
			auto_flags -> land_count = 3001;
		}
		if(auto_flags -> mapping_exit_count == 7000){
			auto_flags -> pre_mapping_flag = 0;
			auto_flags -> mapping_exit_flag = 0;
		}
		auto_flags -> mapping_exit_count ++;
	}
}

void zero_point_reset(Manipulator_t* manipulator){
	manipulator -> joint0_deg.angle = manipulator -> joint0_deg.rad * RtA;
	manipulator -> joint1_deg.angle = manipulator -> joint1_deg.rad * RtA;
	manipulator -> joint2_deg.angle = manipulator -> joint2_deg.rad * RtA;
	manipulator -> joint3_deg.angle = manipulator -> joint3_deg.rad * RtA;
	manipulator -> joint4_deg.angle = manipulator -> joint4_deg.rad * RtA;
	manipulator -> joint5_deg.angle = manipulator -> joint5_deg.rad * RtA;
	manipulator -> joint6_deg.angle = manipulator -> joint6_deg.rad * RtA;

	manipulator -> joint0_deg.angle_target = manipulator -> joint0_deg.zero_point * RtA - manipulator -> joint0_deg.angle_init;
	manipulator -> joint1_deg.angle_target = manipulator -> joint1_deg.zero_point * RtA - manipulator -> joint1_deg.angle_init;
	manipulator -> joint2_deg.angle_target = manipulator -> joint2_deg.zero_point * RtA - manipulator -> joint2_deg.angle_init;
	manipulator -> joint3_deg.angle_target = manipulator -> joint3_deg.zero_point * RtA - manipulator -> joint3_deg.angle_init;
	manipulator -> joint4_deg.angle_target = manipulator -> joint4_deg.zero_point * RtA - manipulator -> joint4_deg.angle_init;
	manipulator -> joint5_deg.angle_target = manipulator -> joint5_deg.zero_point * RtA - manipulator -> joint5_deg.angle_init;
	manipulator -> joint6_deg.angle_target = manipulator -> joint6_deg.zero_point * RtA - manipulator -> joint6_deg.angle_init;
}

void land_point_reset(Manipulator_t* manipulator){
	manipulator -> joint0_deg.angle_target = manipulator -> joint0_deg.land_point * RtA - manipulator -> joint0_deg.angle_init;
	manipulator -> joint1_deg.angle_target = manipulator -> joint1_deg.land_point * RtA - manipulator -> joint1_deg.angle_init;
	manipulator -> joint2_deg.angle_target = manipulator -> joint2_deg.land_point * RtA - manipulator -> joint2_deg.angle_init;
	manipulator -> joint3_deg.angle_target = manipulator -> joint3_deg.land_point * RtA - manipulator -> joint3_deg.angle_init;
	manipulator -> joint4_deg.angle_target = manipulator -> joint4_deg.land_point * RtA - manipulator -> joint4_deg.angle_init;
	manipulator -> joint5_deg.angle_target = manipulator -> joint5_deg.land_point * RtA - manipulator -> joint5_deg.angle_init;
	manipulator -> joint6_deg.angle_target = manipulator -> joint6_deg.land_point * RtA - manipulator -> joint6_deg.angle_init;
}

void step_out_point_reset(Manipulator_t *manipulator){
	manipulator -> joint0_deg.angle_target = manipulator -> joint0_deg.step_out_point * RtA - manipulator -> joint0_deg.angle_init;
	manipulator -> joint1_deg.angle_target = manipulator -> joint1_deg.step_out_point * RtA - manipulator -> joint1_deg.angle_init;
	manipulator -> joint2_deg.angle_target = manipulator -> joint2_deg.step_out_point * RtA - manipulator -> joint2_deg.angle_init;
	manipulator -> joint3_deg.angle_target = manipulator -> joint3_deg.step_out_point * RtA - manipulator -> joint3_deg.angle_init;
	manipulator -> joint4_deg.angle_target = manipulator -> joint4_deg.step_out_point * RtA - manipulator -> joint4_deg.angle_init;
	manipulator -> joint5_deg.angle_target = manipulator -> joint5_deg.step_out_point * RtA - manipulator -> joint5_deg.angle_init;
	manipulator -> joint6_deg.angle_target = manipulator -> joint6_deg.step_out_point * RtA - manipulator -> joint6_deg.angle_init;
}

void motor_start_control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags){
	if(auto_flags -> motor_start_mode_flag == 0){
		if(rc_Ctrl.rc.sw <= 524){
			auto_flags -> motor_start_mode_flag = 1;
			auto_flags -> motor_start_mode_count = 0;
		}
	}
	else if(auto_flags -> motor_start_mode_flag == 1){
		if(auto_flags -> motor_start_mode_count < 1000){
			
		}
		if(auto_flags -> motor_start_mode_count >= 1000 && auto_flags -> motor_start_mode_count < 3000){
			DM_Restart(manipulator_right, tim14.ClockTime, &can1);
			DM_Restart(manipulator_left, tim14.ClockTime, &can2);	
		}
		if(auto_flags -> motor_start_mode_count == 4000){
			auto_flags -> motor_start_mode_flag = 0;
		}
		auto_flags -> motor_start_mode_count ++;
	}
}