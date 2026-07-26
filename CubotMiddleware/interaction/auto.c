#include "auto.h"
#include "mecanum_chassis.h"

auto_control_flags Auto_flags;

/* 操作手控制集成函数 */
void Auto_Control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, Hiwonder_Servo* hiwo_data, custom_robot_data_t* custom, int32_t clock){
	land_reset_control(manipulator_right, manipulator_left, auto_flags);
	servo_back_control(manipulator_right, manipulator_left, hiwo_data, auto_flags);
	point_of_view_control(hiwo_data, auto_flags);
	// scope_mode_control(hiwo_data, auto_flags);
	gloves_data_calc(auto_flags, custom);
	clamp_jaw_control(manipulator_right, manipulator_left, auto_flags, custom);
	lifting_control(auto_flags);
	Controller_mode_start(manipulator_right, manipulator_left, auto_flags, custom);
	motor_start_control(manipulator_right, manipulator_left, auto_flags);
}

/* 机械臂登岛前复位 */
void land_reset_control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags){
	/* 标志位为0，则检测对应键位，按下则将登岛位置归零 */
	if(auto_flags -> land_flag == 0){
		// if((vT13.key_ctrl_flag == 0 && vT13.key_shift_flag == 0 && vT13.key_B_flag == 1) || (rc_Ctrl.rc.s1 == 3 && rc_Ctrl.rc.s2 == 1)){
		if((vT13.key_ctrl_flag == 0 && vT13.key_shift_flag == 0 && vT13.key_B_flag == 1)){
			if(auto_flags -> pre_mapping_flag == 0){
				auto_flags -> land_flag = 1;
				auto_flags -> land_count = 0;
			}
			/* 如果在准备映射模式则自动退出映射模式 */
			else if(auto_flags -> pre_mapping_flag == 1){
				auto_flags -> pre_mapping_flag = 0;					//退出准备映射模式
				auto_flags -> land_flag = 1;						//进入登岛模式
				auto_flags -> lifting_auto_flag = 1;
				auto_flags -> land_count = 1501;
				manipulator_right -> controller_mapping_flag = 0;
				manipulator_left -> controller_mapping_flag = 0;
			}
			
		}
	}
	/* 标志位为1，进行复位，复位期间不再检测键位 */
	else if(auto_flags -> land_flag == 1){
		if(auto_flags -> land_count < 1500){//抬升上升
			auto_flags -> pre_lift_flag = 1;
			auto_flags -> lifting_auto_flag = 1;
		}
		else if(auto_flags -> land_count > 1500 && auto_flags -> land_count < 3000){//抬升已升起时，机械臂复位
			joint_sensitivity_set(manipulator_right, 1.5);
			joint_sensitivity_set(manipulator_left, 1.5);
			/* 右臂复位 */
			land_point_reset(manipulator_right);
			
			/* 左臂复位 */
			land_point_reset(manipulator_left);
		}
		else if(auto_flags -> land_count > 3000 && auto_flags -> land_count < 4500){//抬升下降
			auto_flags -> pre_lift_flag = 2;
			joint_sensitivity_set(manipulator_right, 1.0);
			joint_sensitivity_set(manipulator_left, 1.0);
		}

		auto_flags -> land_count ++;
		if(auto_flags -> land_count == 6000){
			auto_flags -> land_flag = 0;
			auto_flags -> lifting_auto_flag = 0;
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
		if((vT13.key_Q_flag == 1 || rc_Ctrl.key_Q_flag == 1) && (vT13.key_ctrl_flag == 0 && rc_Ctrl.key_ctrl_flag == 0) && (vT13.key_shift_flag == 0 && rc_Ctrl.key_shift_flag == 0)){
			auto_flags -> scope_mode_flag = 0; 
			/* 舵机头向上运动 */
			hiwo_data -> pitch_servo.target_angle += 0.02 * servo_pitch_sensitivity;
		}
		else if((vT13.key_E_flag == 1 || rc_Ctrl.key_E_flag == 1) && (vT13.key_ctrl_flag == 0 && rc_Ctrl.key_ctrl_flag == 0) && (vT13.key_shift_flag == 0 && rc_Ctrl.key_shift_flag == 0)){
			auto_flags -> scope_mode_flag = 0;
			/* 舵机头向下运动 */
			hiwo_data -> pitch_servo.target_angle -= 0.02 * servo_pitch_sensitivity;
		}
		/* target_angle限幅 */
		hiwo_data -> pitch_servo.target_angle = joints_limit(hiwo_data -> pitch_servo.target_angle, SERVO_DOWN_POSITION - SERVO_UP_POSITION, 0);
		/* 将target_angle转换为舵机角度值并限幅 */
		hiwo_data -> pitch_servo.position = servo_angle_limit((int16_t)(hiwo_data -> pitch_servo.target_angle) + SERVO_UP_POSITION, SERVO_DOWN_POSITION, SERVO_UP_POSITION);
	}
}

// void scope_mode_control(Hiwonder_Servo* hiwo_data, auto_control_flags* auto_flags){
// 	if(vT13.mouse.press_r_flag == 1 || rc_Ctrl.mouse.press_r_flag == 1){
// 		auto_flags -> scope_filter_count ++;
// 		if(auto_flags -> scope_filter_count == 20){
// 			auto_flags -> scope_mode_flag ++;
// 			auto_flags -> scope_mode_count = 0;
// 		}
// 	}
// 	else{
// 		auto_flags -> scope_filter_count = 0;
// 	}
	
// 	if(auto_flags -> scope_mode_flag % 2 == 1){
// 		if(auto_flags -> scope_mode_count <= 1000){
// 			/* 舵机平视 */
// 			hiwo_data -> pitch_servo.position = SERVO_UP_POSITION;
// 			hiwo_data -> pitch_servo.target_angle = 0;
// 		}
// 		else if(auto_flags -> scope_mode_count > 1000 && auto_flags -> scope_mode_count <= 2000){
// 			/* 倍镜向上旋转 */
// 			hiwo_data -> roll_servo.position = SERVO_ROLL_UP_POSITION;
// 		}
// 		auto_flags -> scope_mode_count ++;
// 	}
// 	else if(auto_flags -> scope_mode_flag % 2 == 0){
// 		if(auto_flags -> scope_mode_count <= 1000){
// 			/* 倍镜向下旋转 */
// 			hiwo_data -> roll_servo.position = SERVO_ROLL_DOWN_POSITION;
// 		}
// 		auto_flags -> scope_mode_count ++;
// 	}
// }

void gloves_data_calc(auto_control_flags* auto_flags, custom_robot_data_t* custom){
	/* 手套在线检测 */
	if(custom -> image_recv.Coordinate.right_glove_online_flag == 1){
		custom -> image_recv.Coordinate.right_glove_online_count ++;
	}
	else{
		custom -> image_recv.Coordinate.right_glove_online_count = 0;
	}

	if(custom -> image_recv.Coordinate.left_glove_online_flag == 1){
		custom -> image_recv.Coordinate.left_glove_online_count ++;
	}
	else{
		custom -> image_recv.Coordinate.left_glove_online_count = 0;
	}

	/* 夹爪开关数据检测 */
	if(custom -> image_recv.Coordinate.last_switches[1] != custom -> image_recv.Coordinate.switches[1] && custom -> image_recv.Coordinate.left_glove_online_count > 20){
		custom -> image_recv.Coordinate.right_clamp_jaw_count ++;
		if(custom -> image_recv.Coordinate.right_clamp_jaw_count == 1){
			auto_flags -> right_clamp_jaw_key_flag ++;
		}
	}
	else{
		custom -> image_recv.Coordinate.right_clamp_jaw_count = 0;
	}

	if(custom -> image_recv.Coordinate.last_switches[3] != custom -> image_recv.Coordinate.switches[3] && custom -> image_recv.Coordinate.right_glove_online_count > 20){
		custom -> image_recv.Coordinate.left_clamp_jaw_count ++;
		if(custom -> image_recv.Coordinate.left_clamp_jaw_count == 1){
			auto_flags -> left_clamp_jaw_key_flag ++;
		}
	}
	else{
		custom -> image_recv.Coordinate.left_clamp_jaw_count = 0;
	}

	/* 自定义控制器模式检测 */
	if(custom -> image_recv.Coordinate.last_switches[0] != custom -> image_recv.Coordinate.switches[0] && custom -> image_recv.Coordinate.left_glove_online_count > 20){
		custom -> image_recv.Coordinate.right_controller_flag ++;
	}
	else{
		custom -> image_recv.Coordinate.right_controller_flag = 0;
	}
	
	if(custom -> image_recv.Coordinate.last_switches[2] != custom -> image_recv.Coordinate.switches[2] && custom -> image_recv.Coordinate.right_glove_online_count > 20){
		custom -> image_recv.Coordinate.left_controller_flag ++;
	}
	else{
		custom -> image_recv.Coordinate.left_controller_flag = 0;
	}
	custom -> image_recv.Coordinate.last_switches[0] = custom -> image_recv.Coordinate.switches[0];
	custom -> image_recv.Coordinate.last_switches[1] = custom -> image_recv.Coordinate.switches[1];
	custom -> image_recv.Coordinate.last_switches[2] = custom -> image_recv.Coordinate.switches[2];
	custom -> image_recv.Coordinate.last_switches[3] = custom -> image_recv.Coordinate.switches[3];
}

void clamp_jaw_control(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, custom_robot_data_t* custom){
	/* 夹爪按键检测 */
	/* 右手 */
	// if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 0 && vT13.key_G_flag == 1){
	if((vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 0 && vT13.key_G_flag == 1)){
		auto_flags -> right_clamp_jaw_key_count ++;
		if(auto_flags -> right_clamp_jaw_key_count == Clamp_Jaw_Close_Filter_Limit_Time){
			auto_flags -> right_clamp_jaw_key_flag ++;
		}
	}
	else{
		auto_flags -> right_clamp_jaw_key_count = 0;
	}
	manipulator_right -> clamp_jaw.clamp_jaw_close_flag = auto_flags -> right_clamp_jaw_key_flag % 2;

	/* 左手 */
	// if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 0 && vT13.key_F_flag == 1){
	if((vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 0 && vT13.key_F_flag == 1)){
		auto_flags -> left_clamp_jaw_key_count ++;
		if(auto_flags -> left_clamp_jaw_key_count == Clamp_Jaw_Close_Filter_Limit_Time){
			auto_flags -> left_clamp_jaw_key_flag ++;
		}
	}
	else{
		auto_flags -> left_clamp_jaw_key_count = 0;
	}
	manipulator_left -> clamp_jaw.clamp_jaw_close_flag = auto_flags -> left_clamp_jaw_key_flag % 2;

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

void lifting_control(auto_control_flags* auto_flags){
	if(auto_flags -> lifting_auto_flag == 0){
		if(vT13.rc.ch2 >= 1224){
			auto_flags -> pre_lift_flag = 1;
		}
		else if(vT13.rc.ch2 <= 824){
			auto_flags -> pre_lift_flag = 2;
		}
		else{
			auto_flags -> pre_lift_flag = 0;
		}
	}
}

void Controller_mode_start(Manipulator_t* manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags, custom_robot_data_t* custom){
	if(auto_flags -> pre_mapping_flag == 0){
		/* 检测准备映射模式按键Ctrl+B */
		if((vT13.key_ctrl_flag == 1 || rc_Ctrl.key_ctrl_flag == 1) && vT13.key_shift_flag == 0 && (vT13.key_B_flag == 1 || rc_Ctrl.key_B_flag == 1)){
			auto_flags -> pre_mapping_flag = 1;
			auto_flags -> pre_mapping_count = 0;
			auto_flags -> mapping_exit_count = 0;
		}
	}
	else if(auto_flags -> pre_mapping_flag == 1){
		/* 抬升机构抬升 */
		if(auto_flags -> pre_mapping_count <= 2000){
			auto_flags -> pre_lift_flag = 1;
			auto_flags -> lifting_auto_flag = 1;
		}
		/*  */
		else if(auto_flags -> pre_mapping_count > 2000 && auto_flags -> pre_mapping_count <= 4000){
			/* 右臂复位至零点 */
			zero_point_reset(manipulator_right);
			
			/* 左臂复位至零点 */
			zero_point_reset(manipulator_left);
			auto_flags -> pre_lift_flag = 0;
			auto_flags -> lifting_auto_flag = 0;
		}

		else if(auto_flags -> pre_mapping_count >= 4000 && auto_flags -> mapping_exit_flag == 0){
			/* 检测映射模式键位 */
			if((vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 1 && vT13.key_G_flag == 1) || custom -> image_recv.Coordinate.right_controller_flag == 1){
			// if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 1 && vT13.key_G_flag == 1){
				manipulator_right -> controller_mapping_count ++;
				if(manipulator_right -> controller_mapping_count == Mapping_Filter_Limit_Time || custom -> image_recv.Coordinate.right_controller_flag == 1){
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

			if((vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 1 && vT13.key_F_flag == 1) || (custom -> image_recv.Coordinate.left_controller_flag == 1)){
			// if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 1 && vT13.key_F_flag == 1){
				manipulator_left -> controller_mapping_count ++;
				if(manipulator_left -> controller_mapping_count == Mapping_Filter_Limit_Time || custom -> image_recv.Coordinate.left_controller_flag == 1){
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
			Auto_grab_store_control(manipulator_right, manipulator_left, auto_flags);
			Auto_take_out_control(manipulator_right, manipulator_left, auto_flags);
		}
		auto_flags -> pre_mapping_count ++;
	}
}

void angle_value_reset(Manipulator_t* manipulator){
	manipulator -> joint0_deg.angle = manipulator -> joint0_deg.rad * RtA;
	manipulator -> joint1_deg.angle = manipulator -> joint1_deg.rad * RtA;
	manipulator -> joint2_deg.angle = manipulator -> joint2_deg.rad * RtA;
	manipulator -> joint3_deg.angle = manipulator -> joint3_deg.rad * RtA;
	manipulator -> joint4_deg.angle = manipulator -> joint4_deg.rad * RtA;
	manipulator -> joint5_deg.angle = manipulator -> joint5_deg.rad * RtA;
	manipulator -> joint6_deg.angle = manipulator -> joint6_deg.rad * RtA;
}

void zero_point_reset(Manipulator_t* manipulator){
	/* 自定义控制器模式切换回target_angle控制时一定要修改angle值为当前角度，不然容易导致角度跳变 */
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

void joint_sensitivity_set(Manipulator_t *manipulator, float target_sensitivity){
	// manipulator -> joint0_deg.Accel_sensitivity = target_sensitivity;
	manipulator -> joint1_deg.Accel_sensitivity = target_sensitivity;
	manipulator -> joint2_deg.Accel_sensitivity = target_sensitivity;
	manipulator -> joint3_deg.Accel_sensitivity = target_sensitivity;
	manipulator -> joint4_deg.Accel_sensitivity = target_sensitivity;
	manipulator -> joint5_deg.Accel_sensitivity = target_sensitivity;
	manipulator -> joint6_deg.Accel_sensitivity = target_sensitivity;
}

void motor_start_control(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags* auto_flags){
	if(auto_flags -> motor_start_mode_flag == 0){
		if(rc_Ctrl.rc.sw <= 524 || vT13.rc.sw <= 524){
			auto_flags -> motor_start_mode_flag = 1;
			auto_flags -> motor_start_mode_count = 0;
		}
	}
	else if(auto_flags -> motor_start_mode_flag == 1){
		if(auto_flags -> motor_start_mode_count < 1000){
			
		}
		if(auto_flags -> motor_start_mode_count >= 1000 && auto_flags -> motor_start_mode_count < 2000){
			// DM_Error_clean(manipulator_right, tim14.ClockTime, &can1);
			// DM_Error_clean(manipulator_left, tim14.ClockTime, &can2);
		}
		if(auto_flags -> motor_start_mode_count >= 2000 && auto_flags -> motor_start_mode_count < 3500){
			DM_Restart(manipulator_right, tim14.ClockTime, &can1);
			DM_Restart(manipulator_left, tim14.ClockTime, &can2);	
		}
		if(auto_flags -> motor_start_mode_count == 4000){
			auto_flags -> motor_start_mode_flag = 0;
		}
		auto_flags -> motor_start_mode_count ++;
	}
}

/* 以下为自动存矿取矿的代码。能量单元以最上方的为0号，顺时针顺序依次为1-5号，相关函数据此命名 */
/* 自动取矿相关标志位写在Auto_flags结构体中，从存矿机构中取出能量单元的标志位写在机械臂结构体中 */
void Auto_grab_store_control(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(auto_flags -> auto_grab_store_flag == 0 && manipulator_right -> controller_mapping_flag % 2 == 0 && manipulator_left -> controller_mapping_flag % 2 == 0){
		if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 0 && vT13.key_V_flag == 1){
			auto_flags -> auto_grab_store_flag = 1;
			auto_flags -> auto_grab_store_count = 0;

			auto_flags -> auto_grab_store_L3_R2_flag = 1;
			auto_flags -> auto_grab_store_L3_R2_count = 0;
		}
		else if(vT13.key_ctrl_flag == 0 && vT13.key_shift_flag == 1 && vT13.key_V_flag == 1){
			auto_flags -> auto_grab_store_flag = 1;
			auto_flags -> auto_grab_store_count = 0;

			auto_flags -> auto_grab_store_L4_R1_flag = 1;
			auto_flags -> auto_grab_store_L4_R1_count = 0;
		}
		else if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 1 && vT13.key_V_flag == 1){
			auto_flags -> auto_grab_store_flag = 1;
			auto_flags -> auto_grab_store_count = 0;

			auto_flags -> auto_grab_store_L5_R0_flag = 1;
			auto_flags -> auto_grab_store_L5_R0_count = 0;
		}
	}
	else if(auto_flags -> auto_grab_store_flag == 1){
		if(auto_flags -> auto_grab_store_count == 1){
			auto_flags -> lifting_auto_flag = 1;
			angle_value_reset(manipulator_left);
			angle_value_reset(manipulator_right);
			zero_point_reset(manipulator_left);
			zero_point_reset(manipulator_right);
			joint_sensitivity_set(manipulator_right, 0.8);
			joint_sensitivity_set(manipulator_left, 0.8);

			/* 右手夹爪松开 */
			auto_flags -> right_clamp_jaw_key_flag = 0;
			manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 0;

			/* 左手夹爪松开 */
			auto_flags -> left_clamp_jaw_key_flag = 0;
			manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 0;
		}

		Auto_grab_store_L3_R2(manipulator_right, manipulator_left, auto_flags);
		Auto_grab_store_L4_R1(manipulator_right, manipulator_left, auto_flags);
		Auto_grab_store_L5_R0(manipulator_right, manipulator_left, auto_flags);
		auto_flags -> auto_grab_store_count ++;
	}

}

/* 自动存储能量单元函数 */
void Auto_grab_store_L3_R2(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(auto_flags -> auto_grab_store_L4_R1_flag == 0 && auto_flags -> auto_grab_store_L5_R0_flag == 0 && auto_flags -> auto_grab_store_L3_R2_flag == 1){
		if(auto_flags -> auto_grab_store_L3_R2_pause_flag == 1){
			if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 0 && vT13.key_V_flag == 1){
				auto_flags -> auto_grab_store_L3_R2_pause_flag = 0;
			}
		}
		else{
			if(auto_flags -> auto_grab_store_L3_R2_count <= 1000){
				auto_flags -> pre_lift_flag = 1;
				manipulator_left -> joint6_deg.angle_target = -1.571 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 1000) && (auto_flags -> auto_grab_store_L3_R2_count <= 2500)){
				/* 右手前伸 */
				manipulator_right -> joint0_deg.angle_target = 2.300 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.333 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.113 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.909 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = -0.431 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.840 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 3.225 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手前伸 */
				manipulator_left -> joint0_deg.angle_target = 329.548 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.339 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.817 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.412 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -1.228 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.000 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -3.114 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 2500) && (auto_flags -> auto_grab_store_L3_R2_count <= 4000)){
				auto_flags -> pre_lift_flag = 0;
				/* 右手到达取矿位置 */
				manipulator_right -> joint0_deg.angle_target = -7.175 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.585 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.996 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.443 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = -0.813 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.679 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 3.198 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手到达取矿位置 */
				manipulator_left -> joint0_deg.angle_target = 335.017 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.573 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.844 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.140 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -1.243 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.047 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -3.146 * RtA - manipulator_left -> joint6_deg.angle_init;

				if(auto_flags -> auto_grab_store_L3_R2_count == 3999){
					auto_flags -> auto_grab_store_L3_R2_pause_flag = 1;
				}
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 4000) && (auto_flags -> auto_grab_store_L3_R2_count <= 4500)){
				/* 右手夹爪闭合 */
				auto_flags -> right_clamp_jaw_key_flag = 1;
				manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 1;
	
				/* 左手夹爪闭合 */
				auto_flags -> left_clamp_jaw_key_flag = 1;
				manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 1;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 4500) && (auto_flags -> auto_grab_store_L3_R2_count <= 5500)){
				/* 右手拔矿 */
				manipulator_right -> joint0_deg.angle_target = -4.598 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.433 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.759 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.055 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = -0.973 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.231 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 3.328 * RtA - manipulator_right -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 5500) && (auto_flags -> auto_grab_store_L3_R2_count <= 7000)){
				/* 抬升下降（左手拔矿） */
				auto_flags -> pre_lift_flag = 2;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 7000) && (auto_flags -> auto_grab_store_L3_R2_count <= 8500)){
				/* 右手收回 */
				manipulator_right -> joint0_deg.angle_target = 4.598 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.505 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.418 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = 0.543 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = -1.451 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.204 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 3.333 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手收回 */
				manipulator_left -> joint0_deg.angle_target = 317.511 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.596 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 1.684 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -0.353 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -1.829 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.041 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -1.571 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 8500) && (auto_flags -> auto_grab_store_L3_R2_count <= 10000)){
				/* 机械臂复位 */
				land_point_reset(manipulator_right);
				land_point_reset(manipulator_left);
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 10000) && (auto_flags -> auto_grab_store_L3_R2_count <= 11500)){
				/* 抬升上升 */
				auto_flags -> pre_lift_flag = 1;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 11500) && (auto_flags -> auto_grab_store_L3_R2_count <= 13000)){
				/* 右臂抬起 */
				manipulator_right -> joint0_deg.angle_target = 39.981 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.618 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.387 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -1.072 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.059 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = -0.285 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 0.397 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左臂抬起 */
				manipulator_left -> joint0_deg.angle_target = 259.119 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = -0.234 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 1.522 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.915 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -0.187 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.071 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -0.138 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 13000) && (auto_flags -> auto_grab_store_L3_R2_count <= 14500)){
				/* 右手夹爪移动至后方存矿装置附近 */
				manipulator_right -> joint0_deg.angle_target = 28.885 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.848 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.544 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -1.562 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.902 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.117 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = -0.080 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手夹爪移动至后方存矿装置附近 */
				manipulator_left -> joint0_deg.angle_target = 259.137 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = -0.020 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.333 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -2.526 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -0.472 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.213 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -0.308 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 14500) && (auto_flags -> auto_grab_store_L3_R2_count <= 16000)){
				/* 右手能量单元放入存矿装置 */
				manipulator_right -> joint0_deg.angle_target = 28.883 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.864 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.963 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -1.196 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.604 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = -0.031 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = -0.045 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手能量单元放入存矿装置 */
				manipulator_left -> joint0_deg.angle_target = 259.161 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.005 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.653 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -2.128 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -0.416 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.132 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -0.076 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 16000) && (auto_flags -> auto_grab_store_L3_R2_count <= 16700)){
				/* 右手夹爪松开 */
				auto_flags -> right_clamp_jaw_key_flag = 0;
				manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 0;
	
				/* 左手夹爪松开 */
				auto_flags -> left_clamp_jaw_key_flag = 0;
				manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 0;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 16700) && (auto_flags -> auto_grab_store_L3_R2_count <= 17500)){
				/* 右手夹爪远离存矿单元 */
				manipulator_right -> joint0_deg.angle_target = 39.981 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.489 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.792 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -1.116 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.115 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = -0.543 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 0.397 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手夹爪远离存矿单元 */
				manipulator_left -> joint0_deg.angle_target = 259.120 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = -0.416 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.788 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -2.175 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -0.427 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.109 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -0.138 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 17500) && (auto_flags -> auto_grab_store_L3_R2_count <= 19000)){
				/* 右手夹爪继续远离存矿单元 */
				manipulator_right -> joint0_deg.angle_target = 39.981 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.618 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.387 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -1.072 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.059 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = -0.285 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 0.397 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手夹爪继续远离存矿单元 */
				manipulator_left -> joint0_deg.angle_target = 259.119 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = -0.234 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 1.522 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.915 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -0.187 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.071 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -0.138 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 19000) && (auto_flags -> auto_grab_store_L3_R2_count <= 20500)){
				/* 右臂复位至零点 */
				zero_point_reset(manipulator_right);
	
				/* 左臂复位至零点 */
				zero_point_reset(manipulator_left);
			}
			else if((auto_flags -> auto_grab_store_L3_R2_count > 20500) && (auto_flags -> auto_grab_store_L3_R2_count <= 21000)){
				auto_flags -> auto_grab_store_L3_R2_flag = 0;
				joint_sensitivity_set(manipulator_right, 1.0);
				joint_sensitivity_set(manipulator_left, 1.0);
				auto_flags -> auto_grab_store_flag = 0;
				auto_flags -> lifting_auto_flag = 0;
			}
			auto_flags -> auto_grab_store_L3_R2_count ++;
		}
	}
}

void Auto_grab_store_L4_R1(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(auto_flags -> auto_grab_store_L4_R1_flag == 1 && auto_flags -> auto_grab_store_L5_R0_flag == 0 && auto_flags -> auto_grab_store_L3_R2_flag == 0){
		if(auto_flags -> auto_grab_store_L4_R1_pause_flag == 1){
			if(vT13.key_ctrl_flag == 0 && vT13.key_shift_flag == 1 && vT13.key_V_flag == 1){
				auto_flags -> auto_grab_store_L4_R1_pause_flag = 0;
			}
		}
		else{
			if(auto_flags -> auto_grab_store_L4_R1_count <= 1500){
				/* 右手向前伸 */
				manipulator_right -> joint0_deg.angle_target = -18.863 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.170 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.100 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.954 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.176 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 1.022 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 1.865 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手向前伸 */
				manipulator_left -> joint0_deg.angle_target = 313.834 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.354 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.271 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.854 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = 0.184 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 1.152 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -3.149 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 1500) && (auto_flags -> auto_grab_store_L4_R1_count <= 3000)){
				/* 右手到达取矿位置 */
				manipulator_right -> joint0_deg.angle_target = -27.219 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.523 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.983 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.416 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.082 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.915 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 1.700 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手到达取矿位置 */
				manipulator_left -> joint0_deg.angle_target = 313.831 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.544 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.178 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.412 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = 0.392 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.842 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -2.988 * RtA - manipulator_left -> joint6_deg.angle_init;

				if(auto_flags -> auto_grab_store_L4_R1_count == 3000){
					auto_flags -> auto_grab_store_L4_R1_pause_flag = 1;
				}
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 3000) && (auto_flags -> auto_grab_store_L4_R1_count <= 4000)){
				/* 右手夹爪闭合 */
				auto_flags -> right_clamp_jaw_key_flag = 1;
				manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 1;
	
				/* 左手夹爪闭合 */
				auto_flags -> left_clamp_jaw_key_flag = 1;
				manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 1;
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 4000) && (auto_flags -> auto_grab_store_L4_R1_count <= 5500)){
				/* 右手拔出能量单元 */
				manipulator_right -> joint0_deg.angle_target = -27.191 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.391 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.608 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.365 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = -0.103 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.734 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 1.706 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手拔出能量单元 */
				manipulator_left -> joint0_deg.angle_target = 306.325 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.300 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.043 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.214 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = 0.239 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.471 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -2.907 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 5500) && (auto_flags -> auto_grab_store_L4_R1_count <= 7000)){
				/* 右手收回 */
				manipulator_right -> joint0_deg.angle_target = 16.493 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.259 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.065 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.823 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.819 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = -0.132 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = -0.094 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手收回 */
				manipulator_left -> joint0_deg.angle_target = 256.133 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.296 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 0.885 * RtA - manipulator_left -> joint2_deg.angle_init;
				
				manipulator_left -> joint4_deg.angle_target = -0.059 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.106 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = 0.047 * RtA - manipulator_left -> joint6_deg.angle_init;
	
				if((auto_flags -> auto_grab_store_L4_R1_count > 7200) && (auto_flags -> auto_grab_store_L4_R1_count <= 8000)){
					manipulator_left -> joint3_deg.angle_target = -2.141 * RtA - manipulator_left -> joint3_deg.angle_init;
				}
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 7000) && (auto_flags -> auto_grab_store_L4_R1_count <= 8500)){
				/* 右手夹爪移动至前方方存矿装置附近 */
				manipulator_right -> joint0_deg.angle_target = 19.485 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.947 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.428 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -1.279 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.982 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.133 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 0.060 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手夹爪移动至前方方存矿装置附近 */
				manipulator_left -> joint0_deg.angle_target = 268.397 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.091 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.310 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -2.178 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -1.003 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.004 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = 0.109 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 8500) && (auto_flags -> auto_grab_store_L4_R1_count <= 10000)){
				/* 右手能量单元放入存矿装置 */
				manipulator_right -> joint0_deg.angle_target = 19.483 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.028 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.932 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.857 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.830 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = -0.287 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = -0.098 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手能量单元放入存矿装置 */
				manipulator_left -> joint0_deg.angle_target = 268.398 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.183 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.606 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.820 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -0.690 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.064 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = 0.048 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 10000) && (auto_flags -> auto_grab_store_L4_R1_count <= 11500)){
				/* 右手夹爪松开 */
				auto_flags -> right_clamp_jaw_key_flag = 0;
				manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 0;
	
				/* 左手夹爪松开 */
				auto_flags -> left_clamp_jaw_key_flag = 0;
				manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 0;
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 11500) && (auto_flags -> auto_grab_store_L4_R1_count <= 13000)){
				/* 右手夹爪远离存矿单元 */
				manipulator_right -> joint0_deg.angle_target = 19.480 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.711 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.897 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.954 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.857 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = -0.267 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = -0.071 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手夹爪远离存矿单元 */
				manipulator_left -> joint0_deg.angle_target = 268.481 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = -0.235 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.778 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.912 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -0.501 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.156 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -0.157 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 13000) && (auto_flags -> auto_grab_store_L4_R1_count <= 14500)){
				/* 右手夹爪继续远离存矿单元 */
				manipulator_right -> joint0_deg.angle_target = 19.484 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.770 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -0.473 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.906 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.011 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = -0.102 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = -0.071 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手夹爪继续远离存矿单元 */
				manipulator_left -> joint0_deg.angle_target = 268.482 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = -0.061 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 1.514 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.707 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = 0.037 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = -0.149 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -0.157 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 14500) && (auto_flags -> auto_grab_store_L4_R1_count <= 16000)){
				/* 右臂复位至零点 */
				zero_point_reset(manipulator_right);
	
				/* 左臂复位至零点 */
				zero_point_reset(manipulator_left);
			}
			else if((auto_flags -> auto_grab_store_L4_R1_count > 16000) && (auto_flags -> auto_grab_store_L4_R1_count <= 16500)){
				auto_flags -> auto_grab_store_L4_R1_flag = 0;
				joint_sensitivity_set(manipulator_right, 1.0);
				joint_sensitivity_set(manipulator_left, 1.0);
				auto_flags -> auto_grab_store_flag = 0;
				auto_flags -> lifting_auto_flag = 0;
			}
			auto_flags -> auto_grab_store_L4_R1_count ++;
		}
	}
}

void Auto_grab_store_L5_R0(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(auto_flags -> auto_grab_store_L4_R1_flag == 0 && auto_flags -> auto_grab_store_L5_R0_flag == 1 && auto_flags -> auto_grab_store_L3_R2_flag == 0){
		if(auto_flags -> auto_grab_store_L5_R0_pause_flag == 1){
			if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 1 && vT13.key_V_flag == 1){
				auto_flags -> auto_grab_store_L5_R0_pause_flag = 0;
			}
		}
		else{
			if(auto_flags -> auto_grab_store_L5_R0_count <= 800){
				joint_sensitivity_set(manipulator_right, 1.5);
				joint_sensitivity_set(manipulator_left, 1.5);
				/* 右臂复位 */
				land_point_reset(manipulator_right);
				
				/* 左臂复位 */
				land_point_reset(manipulator_left);
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 800) && (auto_flags -> auto_grab_store_L5_R0_count <= 2000)){
				/* 抬升下降 */
				auto_flags -> pre_lift_flag = 2;
				joint_sensitivity_set(manipulator_right, 0.7);
				joint_sensitivity_set(manipulator_left, 0.7);
	
				manipulator_right -> joint0_deg.angle_target = -16.864 - manipulator_right -> joint0_deg.angle_init;
				manipulator_left -> joint0_deg.angle_target = 279.010 - manipulator_left -> joint0_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 2000) && (auto_flags -> auto_grab_store_L5_R0_count <= 3000)){
				auto_flags -> pre_lift_flag = 0;
				/* 底盘整体抬升 */
				auto_flags -> chassis_lift_flag = 1;
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 3000) && (auto_flags -> auto_grab_store_L5_R0_count <= 4500)){
				auto_flags -> chassis_lift_flag = 0;
				/* 右手向前伸 */
				manipulator_right -> joint0_deg.angle_target = -39.801 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.581 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.234 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.297 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.750 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.888 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 0.321 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手向前伸 */
				manipulator_left -> joint0_deg.angle_target = 310.851 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.414 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.010 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.433 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = 0.936 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.236 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -2.777 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 4500) && (auto_flags -> auto_grab_store_L5_R0_count <= 6000)){
				/* 右手到达取矿位置 */
				manipulator_right -> joint0_deg.angle_target = -47.177 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -2.749 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.330 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = 0.100 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.841 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.654 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 0.187 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手到达取矿位置 */
				manipulator_left -> joint0_deg.angle_target = 328.054 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.728 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.101 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -0.985 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = 1.165 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.210 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -2.805 * RtA - manipulator_left -> joint6_deg.angle_init;

				if(auto_flags -> auto_grab_store_L5_R0_count == 5999){
					auto_flags -> auto_grab_store_L5_R0_pause_flag = 1;
				}
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 6000) && (auto_flags -> auto_grab_store_L5_R0_count <= 6800)){
				/* 右手夹爪闭合 */
				auto_flags -> right_clamp_jaw_key_flag = 1;
				manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 1;
	
				/* 左手夹爪闭合 */
				auto_flags -> left_clamp_jaw_key_flag = 1;
				manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 1;
	
				joint_sensitivity_set(manipulator_right, 0.55);
				joint_sensitivity_set(manipulator_left, 0.55);
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 6800) && (auto_flags -> auto_grab_store_L5_R0_count <= 7500)){
				/* 左手拔出能量单元 */
				manipulator_left -> joint0_deg.angle_target = 328.054 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = 0.551 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 1.892 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.061 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = 1.102 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.187 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = -2.806 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 7500) && (auto_flags -> auto_grab_store_L5_R0_count <= 9000)){
				/* 抬升机构抬升（右手拔出能量单元） */
				auto_flags -> pre_lift_flag = 1;
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 9000) && (auto_flags -> auto_grab_store_L5_R0_count <= 10500)){
				/* 右手收回 */
				manipulator_right -> joint0_deg.angle_target = -44.956 - manipulator_right -> joint0_deg.angle_init;
				manipulator_right -> joint1_deg.angle_target = -1.649 * RtA - manipulator_right -> joint1_deg.angle_init;
				manipulator_right -> joint2_deg.angle_target = -1.068 * RtA - manipulator_right -> joint2_deg.angle_init;
				manipulator_right -> joint3_deg.angle_target = -0.181 * RtA - manipulator_right -> joint3_deg.angle_init;
				manipulator_right -> joint4_deg.angle_target = 0.740 * RtA - manipulator_right -> joint4_deg.angle_init;
				manipulator_right -> joint5_deg.angle_target = 0.208 * RtA - manipulator_right -> joint5_deg.angle_init;
				manipulator_right -> joint6_deg.angle_target = 0.023 * RtA - manipulator_right -> joint6_deg.angle_init;
	
				/* 左手收回 */
				manipulator_left -> joint0_deg.angle_target = 308.088 - manipulator_left -> joint0_deg.angle_init;
				manipulator_left -> joint1_deg.angle_target = -0.105 * RtA - manipulator_left -> joint1_deg.angle_init;
				manipulator_left -> joint2_deg.angle_target = 2.073 * RtA - manipulator_left -> joint2_deg.angle_init;
				manipulator_left -> joint3_deg.angle_target = -1.605 * RtA - manipulator_left -> joint3_deg.angle_init;
				manipulator_left -> joint4_deg.angle_target = -1.444 * RtA - manipulator_left -> joint4_deg.angle_init;
				manipulator_left -> joint5_deg.angle_target = 0.379 * RtA - manipulator_left -> joint5_deg.angle_init;
				manipulator_left -> joint6_deg.angle_target = 0.001 * RtA - manipulator_left -> joint6_deg.angle_init;
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 10500) && (auto_flags -> auto_grab_store_L5_R0_count <= 12000)){
				joint_sensitivity_set(manipulator_right, 0.7);
				joint_sensitivity_set(manipulator_left, 0.7);
				auto_flags -> chassis_lift_flag = 2;
				/* 右臂复位至零点 */
				land_point_reset(manipulator_right);
	
				/* 左臂复位至零点 */
				land_point_reset(manipulator_left);
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 12000) && (auto_flags -> auto_grab_store_L5_R0_count <= 13500)){
				auto_flags -> chassis_lift_flag = 0;
				auto_flags -> pre_lift_flag = 2;
			}
			else if((auto_flags -> auto_grab_store_L5_R0_count > 13500) && (auto_flags -> auto_grab_store_L5_R0_count <= 14000)){
				auto_flags -> auto_grab_store_L5_R0_flag = 0;
				joint_sensitivity_set(manipulator_right, 1.0);
				joint_sensitivity_set(manipulator_left, 1.0);
				auto_flags -> auto_grab_store_flag = 0;
				auto_flags -> lifting_auto_flag = 0;
				auto_flags -> pre_mapping_flag = 0;
				manipulator_right -> controller_mapping_flag = 0;
				manipulator_left -> controller_mapping_flag = 0;
			}
			auto_flags -> auto_grab_store_L5_R0_count ++;
		}
	}
}

/* 以下为自动取出能量单元函数 */
void Auto_take_out_control(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(manipulator_right -> controller_mapping_flag % 2 == 0 && manipulator_left -> controller_mapping_flag % 2 == 0){
		if(manipulator_right -> auto_take_out_back_flag == 0 && manipulator_right -> auto_take_out_front_flag == 0){
			if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 0 && vT13.key_E_flag == 1){
				manipulator_right -> auto_take_out_back_flag = 1;
				manipulator_right -> auto_take_out_back_count = 0;

				joint_sensitivity_set(manipulator_right, 0.7);
				angle_value_reset(manipulator_right);
				/* 右手夹爪松开 */
				auto_flags -> right_clamp_jaw_key_flag = 0;
				manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 0;
			}
			else if(vT13.key_ctrl_flag == 0 && vT13.key_shift_flag == 1 && vT13.key_E_flag == 1){
				manipulator_right -> auto_take_out_front_flag = 1;
				manipulator_right -> auto_take_out_front_count = 0;

				joint_sensitivity_set(manipulator_right, 0.7);
				angle_value_reset(manipulator_right);
				/* 右手夹爪松开 */
				auto_flags -> right_clamp_jaw_key_flag = 0;
				manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 0;
			}
		}

		if(manipulator_left -> auto_take_out_back_flag == 0 && manipulator_left -> auto_take_out_front_flag == 0){
			if(vT13.key_ctrl_flag == 1 && vT13.key_shift_flag == 0 && vT13.key_Q_flag == 1){
				manipulator_left -> auto_take_out_back_flag = 1;
				manipulator_left -> auto_take_out_back_count = 0;

				joint_sensitivity_set(manipulator_left, 0.7);
				angle_value_reset(manipulator_left);
				/* 左手夹爪松开 */
				auto_flags -> left_clamp_jaw_key_flag = 0;
				manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 0;
			}
			else if(vT13.key_ctrl_flag == 0 && vT13.key_shift_flag == 1 && vT13.key_Q_flag == 1){
				manipulator_left -> auto_take_out_front_flag = 1;
				manipulator_left -> auto_take_out_front_count = 0;

				joint_sensitivity_set(manipulator_left, 0.7);
				angle_value_reset(manipulator_left);
				/* 左手夹爪松开 */
				auto_flags -> left_clamp_jaw_key_flag = 0;
				manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 0;
			}
		}
		Auto_take_out_right_back(manipulator_right, manipulator_left, auto_flags);
		Auto_take_out_right_front(manipulator_right, manipulator_left, auto_flags);
		Auto_take_out_left_back(manipulator_right, manipulator_left, auto_flags);
		Auto_take_out_left_front(manipulator_right, manipulator_left, auto_flags);
	}
}

void Auto_take_out_right_back(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(manipulator_right -> auto_take_out_back_flag == 1 && manipulator_right -> auto_take_out_front_flag == 0){
		if(manipulator_right -> auto_take_out_back_count <= 1500){
			/* 右手夹爪抬高 */
			manipulator_right -> joint0_deg.angle_target = 36.981 - manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint1_deg.angle_target = -1.618 * RtA - manipulator_right -> joint1_deg.angle_init;
			manipulator_right -> joint2_deg.angle_target = -0.387 * RtA - manipulator_right -> joint2_deg.angle_init;
			manipulator_right -> joint3_deg.angle_target = -1.072 * RtA - manipulator_right -> joint3_deg.angle_init;
			manipulator_right -> joint4_deg.angle_target = 0.059 * RtA - manipulator_right -> joint4_deg.angle_init;
			manipulator_right -> joint5_deg.angle_target = -0.285 * RtA - manipulator_right -> joint5_deg.angle_init;
			manipulator_right -> joint6_deg.angle_target = 0.397 * RtA - manipulator_right -> joint6_deg.angle_init;
		}
		else if((manipulator_right -> auto_take_out_back_count > 1500) && (manipulator_right -> auto_take_out_back_count <= 3000)){
			/* 右手夹爪移动至存矿机构附近 */
			manipulator_right -> joint0_deg.angle_target = 37.004 - manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint1_deg.angle_target = -1.554 * RtA - manipulator_right -> joint1_deg.angle_init;
			manipulator_right -> joint2_deg.angle_target = -1.779 * RtA - manipulator_right -> joint2_deg.angle_init;
			manipulator_right -> joint3_deg.angle_target = -1.308 * RtA - manipulator_right -> joint3_deg.angle_init;
			manipulator_right -> joint4_deg.angle_target = 0.222 * RtA - manipulator_right -> joint4_deg.angle_init;
			manipulator_right -> joint5_deg.angle_target = -0.262 * RtA - manipulator_right -> joint5_deg.angle_init;
			manipulator_right -> joint6_deg.angle_target = 0.423 * RtA - manipulator_right -> joint6_deg.angle_init;
		}
		else if((manipulator_right -> auto_take_out_back_count > 3000) && (manipulator_right -> auto_take_out_back_count <= 4500)){
			/* 右手到达存矿装置位置 */
			manipulator_right -> joint0_deg.angle_target = 37.000 - manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint1_deg.angle_target = -1.921 * RtA - manipulator_right -> joint1_deg.angle_init;
			manipulator_right -> joint2_deg.angle_target = -1.747 * RtA - manipulator_right -> joint2_deg.angle_init;
			manipulator_right -> joint3_deg.angle_target = -1.223 * RtA - manipulator_right -> joint3_deg.angle_init;
			manipulator_right -> joint4_deg.angle_target = 0.046 * RtA - manipulator_right -> joint4_deg.angle_init;
			manipulator_right -> joint5_deg.angle_target = -0.158 * RtA - manipulator_right -> joint5_deg.angle_init;
			manipulator_right -> joint6_deg.angle_target = 0.420 * RtA - manipulator_right -> joint6_deg.angle_init;
		}
		else if((manipulator_right -> auto_take_out_back_count > 4500) && (manipulator_right -> auto_take_out_back_count <= 5200)){
			/* 右手夹爪闭合 */
			auto_flags -> right_clamp_jaw_key_flag = 1;
			manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 1;
		}
		else if((manipulator_right -> auto_take_out_back_count > 5200) && (manipulator_right -> auto_take_out_back_count <= 6000)){
			/* 右手夹爪取出能量单元 */
			manipulator_right -> joint0_deg.angle_target = 25.885 - manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint1_deg.angle_target = -1.848 * RtA - manipulator_right -> joint1_deg.angle_init;
			manipulator_right -> joint2_deg.angle_target = -1.544 * RtA - manipulator_right -> joint2_deg.angle_init;
			manipulator_right -> joint3_deg.angle_target = -1.562 * RtA - manipulator_right -> joint3_deg.angle_init;
			manipulator_right -> joint4_deg.angle_target = 0.902 * RtA - manipulator_right -> joint4_deg.angle_init;
			manipulator_right -> joint5_deg.angle_target = 0.117 * RtA - manipulator_right -> joint5_deg.angle_init;
			manipulator_right -> joint6_deg.angle_target = -0.080 * RtA - manipulator_right -> joint6_deg.angle_init;
		}
		else if((manipulator_right -> auto_take_out_back_count > 6000) && (manipulator_right -> auto_take_out_back_count <= 7500)){
			/* 右臂复位至零点 */
			zero_point_reset(manipulator_right);
		}
		else if((manipulator_right -> auto_take_out_back_count > 7500) && (manipulator_right -> auto_take_out_back_count <= 8000)){
			manipulator_right -> auto_take_out_back_flag = 0;
			joint_sensitivity_set(manipulator_right, 1.0);
		}
		manipulator_right -> auto_take_out_back_count ++;
	}
}

void Auto_take_out_right_front(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(manipulator_right -> auto_take_out_back_flag == 0 && manipulator_right -> auto_take_out_front_flag == 1){
		if(manipulator_right -> auto_take_out_front_count <= 1500){
			/* 右手夹爪抬起 */
			manipulator_right -> joint0_deg.angle_target = 16.484 - manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint1_deg.angle_target = -1.770 * RtA - manipulator_right -> joint1_deg.angle_init;
			manipulator_right -> joint2_deg.angle_target = -0.473 * RtA - manipulator_right -> joint2_deg.angle_init;
			manipulator_right -> joint3_deg.angle_target = -0.906 * RtA - manipulator_right -> joint3_deg.angle_init;
			manipulator_right -> joint4_deg.angle_target = 0.011 * RtA - manipulator_right -> joint4_deg.angle_init;
			manipulator_right -> joint5_deg.angle_target = -0.102 * RtA - manipulator_right -> joint5_deg.angle_init;
			manipulator_right -> joint6_deg.angle_target = -0.071 * RtA - manipulator_right -> joint6_deg.angle_init;
		}
		else if(manipulator_right -> auto_take_out_front_count > 1500 && manipulator_right -> auto_take_out_front_count <= 3000){
			/* 右手夹爪靠近存矿装置 */
			manipulator_right -> joint0_deg.angle_target = 16.484 - manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint1_deg.angle_target = -1.718 * RtA - manipulator_right -> joint1_deg.angle_init;
			manipulator_right -> joint2_deg.angle_target = -1.917 * RtA - manipulator_right -> joint2_deg.angle_init;
			manipulator_right -> joint3_deg.angle_target = -1.065 * RtA - manipulator_right -> joint3_deg.angle_init;
			manipulator_right -> joint4_deg.angle_target = 0.841 * RtA - manipulator_right -> joint4_deg.angle_init;
			manipulator_right -> joint5_deg.angle_target = -0.260 * RtA - manipulator_right -> joint5_deg.angle_init;
			manipulator_right -> joint6_deg.angle_target = -0.048 * RtA - manipulator_right -> joint6_deg.angle_init;
		}
		else if(manipulator_right -> auto_take_out_front_count > 3000 && manipulator_right -> auto_take_out_front_count <= 4500){
			/* 右手到达存矿装置位置 */
			manipulator_right -> joint0_deg.angle_target = 16.484 - manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint1_deg.angle_target = -2.032 * RtA - manipulator_right -> joint1_deg.angle_init;
			manipulator_right -> joint2_deg.angle_target = -1.871 * RtA - manipulator_right -> joint2_deg.angle_init;
			manipulator_right -> joint3_deg.angle_target = -0.908 * RtA - manipulator_right -> joint3_deg.angle_init;
			manipulator_right -> joint4_deg.angle_target = 0.706 * RtA - manipulator_right -> joint4_deg.angle_init;
			manipulator_right -> joint5_deg.angle_target = -0.157 * RtA - manipulator_right -> joint5_deg.angle_init;
			manipulator_right -> joint6_deg.angle_target = -0.048 * RtA - manipulator_right -> joint6_deg.angle_init;
		}
		else if(manipulator_right -> auto_take_out_front_count > 4500 && manipulator_right -> auto_take_out_front_count <= 5200){
			/* 右手夹爪闭合 */
			auto_flags -> right_clamp_jaw_key_flag = 1;
			manipulator_right -> clamp_jaw.clamp_jaw_close_flag = 1;
		}
		else if(manipulator_right -> auto_take_out_front_count > 5200 && manipulator_right -> auto_take_out_front_count <= 6000){
			/* 右手拔出能量单元 */
			manipulator_right -> joint0_deg.angle_target = 16.485 - manipulator_right -> joint0_deg.angle_init;
			manipulator_right -> joint1_deg.angle_target = -1.947 * RtA - manipulator_right -> joint1_deg.angle_init;
			manipulator_right -> joint2_deg.angle_target = -1.428 * RtA - manipulator_right -> joint2_deg.angle_init;
			manipulator_right -> joint3_deg.angle_target = -1.279 * RtA - manipulator_right -> joint3_deg.angle_init;
			manipulator_right -> joint4_deg.angle_target = 0.982 * RtA - manipulator_right -> joint4_deg.angle_init;
			manipulator_right -> joint5_deg.angle_target = 0.133 * RtA - manipulator_right -> joint5_deg.angle_init;
			manipulator_right -> joint6_deg.angle_target = 0.060 * RtA - manipulator_right -> joint6_deg.angle_init;
		}
		else if(manipulator_right -> auto_take_out_front_count > 6000 && manipulator_right -> auto_take_out_front_count <= 7500){
			/* 右臂复位至零点 */
			zero_point_reset(manipulator_right);
		}
		else if(manipulator_right -> auto_take_out_front_count > 7500 && manipulator_right -> auto_take_out_front_count <= 8000){
			manipulator_right -> auto_take_out_front_flag = 0;
			joint_sensitivity_set(manipulator_right, 1.0);
		}
		manipulator_right -> auto_take_out_front_count ++;
	}
}

void Auto_take_out_left_back(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(manipulator_left -> auto_take_out_back_flag == 1 && manipulator_left -> auto_take_out_front_flag == 0){
		if(manipulator_left -> auto_take_out_back_count <= 1500){
			/* 左手夹爪抬起 */
			manipulator_left -> joint0_deg.angle_target = 259.119 - manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint1_deg.angle_target = -0.234 * RtA - manipulator_left -> joint1_deg.angle_init;
			manipulator_left -> joint2_deg.angle_target = 1.522 * RtA - manipulator_left -> joint2_deg.angle_init;
			manipulator_left -> joint3_deg.angle_target = -1.915 * RtA - manipulator_left -> joint3_deg.angle_init;
			manipulator_left -> joint4_deg.angle_target = -0.187 * RtA - manipulator_left -> joint4_deg.angle_init;
			manipulator_left -> joint5_deg.angle_target = -0.071 * RtA - manipulator_left -> joint5_deg.angle_init;
			manipulator_left -> joint6_deg.angle_target = -0.138 * RtA - manipulator_left -> joint6_deg.angle_init;
		}
		else if(manipulator_left -> auto_take_out_back_count > 1500 && manipulator_left -> auto_take_out_back_count <= 3000){
			/* 左手夹爪移动至存矿机构附近 */
			manipulator_left -> joint0_deg.angle_target = 259.114 - manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint1_deg.angle_target = -0.195 * RtA - manipulator_left -> joint1_deg.angle_init;
			manipulator_left -> joint2_deg.angle_target = 2.657 * RtA - manipulator_left -> joint2_deg.angle_init;
			manipulator_left -> joint3_deg.angle_target = -2.166 * RtA - manipulator_left -> joint3_deg.angle_init;
			manipulator_left -> joint4_deg.angle_target = -0.284 * RtA - manipulator_left -> joint4_deg.angle_init;
			manipulator_left -> joint5_deg.angle_target = -0.287 * RtA - manipulator_left -> joint5_deg.angle_init;
			manipulator_left -> joint6_deg.angle_target = -0.353 * RtA - manipulator_left -> joint6_deg.angle_init;
		}
		else if(manipulator_left -> auto_take_out_back_count > 3000 && manipulator_left -> auto_take_out_back_count <= 4500){
			/* 左手到达存矿装置位置 */
			manipulator_left -> joint0_deg.angle_target = 259.114 - manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint1_deg.angle_target = 0.078 * RtA - manipulator_left -> joint1_deg.angle_init;
			manipulator_left -> joint2_deg.angle_target = 2.634 * RtA - manipulator_left -> joint2_deg.angle_init;
			manipulator_left -> joint3_deg.angle_target = -2.098 * RtA - manipulator_left -> joint3_deg.angle_init;
			manipulator_left -> joint4_deg.angle_target = -0.094 * RtA - manipulator_left -> joint4_deg.angle_init;
			manipulator_left -> joint5_deg.angle_target = -0.149 * RtA - manipulator_left -> joint5_deg.angle_init;
			manipulator_left -> joint6_deg.angle_target = -0.357 * RtA - manipulator_left -> joint6_deg.angle_init;
		}
		else if(manipulator_left -> auto_take_out_back_count > 4500 && manipulator_left -> auto_take_out_back_count <= 5200){
			/* 左手夹爪闭合 */
			auto_flags -> left_clamp_jaw_key_flag = 1;
			manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 1;
		}
		else if(manipulator_left -> auto_take_out_back_count > 5200 && manipulator_left -> auto_take_out_back_count <= 6000){
			/* 左手夹爪取出能量单元 */
			manipulator_left -> joint0_deg.angle_target = 259.155 - manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint1_deg.angle_target = 0.018 * RtA - manipulator_left -> joint1_deg.angle_init;
			manipulator_left -> joint2_deg.angle_target = 2.276 * RtA - manipulator_left -> joint2_deg.angle_init;
			manipulator_left -> joint3_deg.angle_target = -2.510 * RtA - manipulator_left -> joint3_deg.angle_init;
			manipulator_left -> joint4_deg.angle_target = -0.868 * RtA - manipulator_left -> joint4_deg.angle_init;
			manipulator_left -> joint5_deg.angle_target = 0.191 * RtA - manipulator_left -> joint5_deg.angle_init;
			manipulator_left -> joint6_deg.angle_target = 0.074 * RtA - manipulator_left -> joint6_deg.angle_init;
		}
		else if(manipulator_left -> auto_take_out_back_count > 6000 && manipulator_left -> auto_take_out_back_count <= 7500){
			/* 左臂复位至零点 */
			zero_point_reset(manipulator_left);
		}
		else if(manipulator_left -> auto_take_out_back_count > 7500 && manipulator_left -> auto_take_out_back_count <= 8000){
			manipulator_left -> auto_take_out_back_flag = 0;
			joint_sensitivity_set(manipulator_left, 1.0);
		}
		manipulator_left -> auto_take_out_back_count ++;
	}
}

void Auto_take_out_left_front(Manipulator_t *manipulator_right, Manipulator_t* manipulator_left, auto_control_flags *auto_flags){
	if(manipulator_left -> auto_take_out_back_flag == 0 && manipulator_left -> auto_take_out_front_flag == 1){
		if(manipulator_left -> auto_take_out_front_count <= 1500){
			/* 左手夹爪抬起 */
			manipulator_left -> joint0_deg.angle_target = 271.482 - manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint1_deg.angle_target = -0.061 * RtA - manipulator_left -> joint1_deg.angle_init;
			manipulator_left -> joint2_deg.angle_target = 1.514 * RtA - manipulator_left -> joint2_deg.angle_init;
			manipulator_left -> joint3_deg.angle_target = -1.707 * RtA - manipulator_left -> joint3_deg.angle_init;
			manipulator_left -> joint4_deg.angle_target = 0.037 * RtA - manipulator_left -> joint4_deg.angle_init;
			manipulator_left -> joint5_deg.angle_target = -0.149 * RtA - manipulator_left -> joint5_deg.angle_init;
			manipulator_left -> joint6_deg.angle_target = -0.157 * RtA - manipulator_left -> joint6_deg.angle_init;
		}
		else if(manipulator_left -> auto_take_out_front_count > 1500 && manipulator_left -> auto_take_out_front_count <= 3000){
			/* 左手夹爪靠近存矿装置 */
			manipulator_left -> joint0_deg.angle_target = 271.487 - manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint1_deg.angle_target = -0.109 * RtA - manipulator_left -> joint1_deg.angle_init;
			manipulator_left -> joint2_deg.angle_target = 2.683 * RtA - manipulator_left -> joint2_deg.angle_init;
			manipulator_left -> joint3_deg.angle_target = -1.953 * RtA - manipulator_left -> joint3_deg.angle_init;
			manipulator_left -> joint4_deg.angle_target = -0.693 * RtA - manipulator_left -> joint4_deg.angle_init;
			manipulator_left -> joint5_deg.angle_target = -0.243 * RtA - manipulator_left -> joint5_deg.angle_init;
			manipulator_left -> joint6_deg.angle_target = -0.081 * RtA - manipulator_left -> joint6_deg.angle_init;
		}
		else if(manipulator_left -> auto_take_out_front_count > 3000 && manipulator_left -> auto_take_out_front_count <= 4500){
			/* 左手到达存矿装置位置 */
			manipulator_left -> joint0_deg.angle_target = 271.467 - manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint1_deg.angle_target = 0.237 * RtA - manipulator_left -> joint1_deg.angle_init;
			manipulator_left -> joint2_deg.angle_target = 2.564 * RtA - manipulator_left -> joint2_deg.angle_init;
			manipulator_left -> joint3_deg.angle_target = -1.854 * RtA - manipulator_left -> joint3_deg.angle_init;
			manipulator_left -> joint4_deg.angle_target = -0.516 * RtA - manipulator_left -> joint4_deg.angle_init;
			manipulator_left -> joint5_deg.angle_target = -0.037 * RtA - manipulator_left -> joint5_deg.angle_init;
			manipulator_left -> joint6_deg.angle_target = -0.037 * RtA - manipulator_left -> joint6_deg.angle_init;
		}
		else if(manipulator_left -> auto_take_out_front_count > 4500 && manipulator_left -> auto_take_out_front_count <= 5200){
			/* 左手夹爪闭合 */
			auto_flags -> left_clamp_jaw_key_flag = 1;
			manipulator_left -> clamp_jaw.clamp_jaw_close_flag = 1;
		}
		else if(manipulator_left -> auto_take_out_front_count > 5200 && manipulator_left -> auto_take_out_front_count <= 6000){
			/* 左手拔出能量单元 */
			manipulator_left -> joint0_deg.angle_target = 271.403 - manipulator_left -> joint0_deg.angle_init;
			manipulator_left -> joint1_deg.angle_target = 0.176 * RtA - manipulator_left -> joint1_deg.angle_init;
			manipulator_left -> joint2_deg.angle_target = 2.271 * RtA - manipulator_left -> joint2_deg.angle_init;
			manipulator_left -> joint3_deg.angle_target = -2.112 * RtA - manipulator_left -> joint3_deg.angle_init;
			manipulator_left -> joint4_deg.angle_target = -0.827 * RtA - manipulator_left -> joint4_deg.angle_init;
			manipulator_left -> joint5_deg.angle_target = 0.276 * RtA - manipulator_left -> joint5_deg.angle_init;
			manipulator_left -> joint6_deg.angle_target = -0.113 * RtA - manipulator_left -> joint6_deg.angle_init;
		}
		else if(manipulator_left -> auto_take_out_front_count > 6000 && manipulator_left -> auto_take_out_front_count <= 7500){
			/* 左臂复位至零点 */
			zero_point_reset(manipulator_left);
		}
		else if(manipulator_left -> auto_take_out_front_count > 7500 && manipulator_left -> auto_take_out_front_count <= 8000){
			manipulator_left -> auto_take_out_front_flag = 0;
			joint_sensitivity_set(manipulator_left, 1.0);
		}
		manipulator_left -> auto_take_out_front_count ++;
	}
}

