#include "Can_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Motion_Task.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"
#include "arm.h"
#include "tim.h"
#include "cowork.h"
#include "usart.h"
#include "auto_mode.h"


//void Can_Task(void const * argument)
//{
//	for(;;)
//	{
//  if(RCctrl.rc_lost)
//{
// //   Chassis.Chassis_Stop(&Chassis);
//	  
//}

//  else//ң������
//{
//   if(RCctrl.CH6<1000)//����
//{

//}
//  else//ң��	
// {
//	if(RCctrl.CH10>1400)//����ģʽ��CH9����ͷ��CH9��β����˫��CH8����
//  {
//   // Chassis_mode();
//		 Chassis.Chassis_Calc(&Chassis);

//    vTaskDelay(1);
//		
//		chassis_angle1.set_mit_data(&chassis_angle1,angle_now[0],0.0f,0.6f,0.032f,0.0f);
//		chassis_angle2.set_mit_data(&chassis_angle2,angle_now[1],0.0f,0.6f,0.032f,0.0f);
//		chassis_angle3.set_mit_data(&chassis_angle3,angle_now[2],0.0f,0.6f,0.032f,0.0f);
//		chassis_angle4.set_mit_data(&chassis_angle4,angle_now[3],0.0f,0.6f,0.032f,0.0f);
//		
//		
//  }

//	else if(RCctrl.CH5<500)//����ģʽ
//  {
//		weapon_mode();
//		kfs_catch_mode();
//		
//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
//		
//		kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);
//		weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
//		
//		weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);
//		if(ABS(weapon_joint_input) <=10){weapon_joint_motor.pid_spd.Output = 0.0f;}
//	  DJIset_motor_data(&hfdcan2, 0X1FF,0 ,weapon_joint_motor.pid_spd.Output,0,0);
//		
//		vTaskDelay(2);
//		Chassis.chassis_mission(&Chassis);
//		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//	}

//	else if(RCctrl.CH5>700 && RCctrl.CH5<1200)//kfsģʽ
//  {
//	  kfs_mode();
//		vTaskDelay(1);
//		Chassis.chassis_mission(&Chassis);
//		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//	}
//	
//	else if(RCctrl.CH5>1300)//̧��ģʽ
//  {
//	  lift_mode();
//		Chassis.chassis_mission(&Chassis);//���Ż��������⣬�Ῠ������
//		
//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
//		
//		//�����Ż����ϷŻ�ǹ�˵Ĺ��ܣ���������ģʽ������(kfs_catch)
//		weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);
//		if(ABS(weapon_joint_input) <= 10){weapon_joint_motor.pid_spd.Output = 0.0f;}
//		DJIset_motor_data(&hfdcan2, 0X1FF,0 ,weapon_joint_motor.pid_spd.Output,0,0);
//		
//		vTaskDelay(2);
//		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//	}
//		
//	  chassis_motor1.PID_Calculate(&chassis_motor1, 10.0f*Chassis.param.V_out[0]);
//    chassis_motor2.PID_Calculate(&chassis_motor2, 10.0f*Chassis.param.V_out[1]);
//		chassis_motor3.PID_Calculate(&chassis_motor3, 10.0f*Chassis.param.V_out[2]);
//    chassis_motor4.PID_Calculate(&chassis_motor4, 10.0f*Chassis.param.V_out[3]);
//	
//		DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
//   

//	 // Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
//   }
//  }
//    vTaskDelay(1);
//	 // osDelay(5);
// }
//}

uint8_t step_flag = 0;

void HF_Task()          //��Ƶ������200hz
{

  if(RCctrl.rc_lost)
{
	
}
  else//ң������
{
	if(RCctrl.channel == 0) //遥控
{
	if(RCctrl.chassis == 0)//底盘
  {
		 Chassis.Chassis_Calc(&Chassis);

		//����Ҫ����ʱ
		chassis_motor1.PID_Calculate(&chassis_motor1, 10.0f*Chassis.param.V_out[0]);
    chassis_motor2.PID_Calculate(&chassis_motor2, 10.0f*Chassis.param.V_out[1]);
		chassis_motor3.PID_Calculate(&chassis_motor3, 10.0f*Chassis.param.V_out[2]);
    chassis_motor4.PID_Calculate(&chassis_motor4, 10.0f*Chassis.param.V_out[3]);
	
		chassis_angle1.set_mit_data(&chassis_angle1,angle_now[0],0.0f,0.7f,0.032f,0.0f);
		chassis_angle2.set_mit_data(&chassis_angle2,angle_now[1],0.0f,0.7f,0.032f,0.0f);
		chassis_angle3.set_mit_data(&chassis_angle3,angle_now[2],0.0f,0.7f,0.032f,0.0f);
		chassis_angle4.set_mit_data(&chassis_angle4,angle_now[3],0.0f,0.7f,0.032f,0.0f);
	
		DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);	
  }

	else if(RCctrl.chassis == 1 && RCctrl.zone == 0)//一区任务
  {
		weapon_mode();
		
		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);//抬升
	  
		weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);
		if(ABS(weapon_joint_input) <=10){weapon_joint_motor.pid_spd.Output = 0.0f;}
	    DJIset_motor_data(&hfdcan2, 0X1FF,0 ,weapon_joint_motor.pid_spd.Output,0,0); //对接

		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3); //机械臂
	}

	else if(RCctrl.chassis == 1 && RCctrl.zone == 1)//kfsģʽ
  {
		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3); //机械臂
	}
	
	else if(RCctrl.chassis == 1 && RCctrl.zone == 2)//三区
  {		
		lift_mode();
		
		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]); //抬升

		Arm_task(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);//机械臂
  }
  auto_flag=0;
  arm_s_running = 0;
}
  if(RCctrl.channel == 1) //�Զ�ģʽ
 {
     
	 Auto_Init();

     if(ABS(RCctrl.vw)<10 && ABS(RCctrl.vx)<10 && ABS(RCctrl.vy)<10)
     {
        chassis_motor1.PID_Calculate(&chassis_motor1, 0);
        chassis_motor2.PID_Calculate(&chassis_motor2, 0);
		chassis_motor3.PID_Calculate(&chassis_motor3, 0);
        chassis_motor4.PID_Calculate(&chassis_motor4, 0);
       DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
     }
     if(ABS(RCctrl.vw)>10 || ABS(RCctrl.vx)>10 || ABS(RCctrl.vy)>10) //自动模式底盘
     {
         Chassis.Chassis_Calc(&Chassis);

		//����Ҫ����ʱ
		chassis_motor1.PID_Calculate(&chassis_motor1, 10.0f*Chassis.param.V_out[0]);
        chassis_motor2.PID_Calculate(&chassis_motor2, 10.0f*Chassis.param.V_out[1]);
		chassis_motor3.PID_Calculate(&chassis_motor3, 10.0f*Chassis.param.V_out[2]);
        chassis_motor4.PID_Calculate(&chassis_motor4, 10.0f*Chassis.param.V_out[3]);
	
		chassis_angle1.set_mit_data(&chassis_angle1,angle_now[0],0.0f,0.7f,0.032f,0.0f);
		chassis_angle2.set_mit_data(&chassis_angle2,angle_now[1],0.0f,0.7f,0.032f,0.0f);
		chassis_angle3.set_mit_data(&chassis_angle3,angle_now[2],0.0f,0.7f,0.032f,0.0f);
		chassis_angle4.set_mit_data(&chassis_angle4,angle_now[3],0.0f,0.7f,0.032f,0.0f);
	
		DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
     }
	 
	 if(RCctrl.chassis == 1 && RCctrl.zone == 0 && RCctrl.takePos ==1)
    {

        auto_mission();
					//一区取杆自动流程，一区任务收杆触发

    }
      kfs_auto(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);

 }
}
}



void LF_Task()        //��Ƶ����������100hz
{
if(RCctrl.rc_lost)
{
	
}

  else//ң������
{
	if(RCctrl.channel == 0)//遥控
{
	if(RCctrl.chassis == 1 && RCctrl.zone == 0)//一区
  {
		task_chassis();
		//Chassis.chassis_mission(&Chassis);
		
		kfs_catch_mode();
		kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);//���Խ�Ƶ
	  
    weapon_hold();
		weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
		
		BUT_check();
		
	}
	
	else if(RCctrl.chassis == 1 && RCctrl.zone == 1)//kfsģʽ
  {
		//Chassis.chassis_mission(&Chassis);
		task_chassis();
		kfs_mode();
		kfs_catch.set_mit_data(&kfs_catch,0.35f,0,2.20f,0.1,0);
	}
	
	else if(RCctrl.chassis == 1 && RCctrl.zone == 2)//̧��ģʽ
  {		
		//Chassis.chassis_mission(&Chassis);
		if(RCctrl.led_3 == 0)
		{
        task_chassis();
		}
		
		//kfs_catch_mode();
		//kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);//ң��֧�̶ֳȵͣ�û��Ҫд��������ʵҲ�ǡ��˹��ܻ���������

		kfs_mode();
		
		  if(RCctrl.led_3)
			{
				weapon_hold();
        weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
				
		weapon_joint_motor.PID_Calculate(&weapon_joint_motor,weapon_joint_input);
		if(ABS(weapon_joint_input) <= 10){weapon_joint_motor.pid_spd.Output = 0.0f;}
		DJIset_motor_data(&hfdcan2, 0X1FF,0 ,weapon_joint_motor.pid_spd.Output,0,0);
	    }
	}
//  	else if(RCctrl.chassis == 0 && RCctrl.zone == 2)//����ģʽ��������
//  {
//	    lift_mode();
//		
//		R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
//		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
//  }
	  if(RCctrl.zone == 1)
    {
		  in_place_fast(0.2f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,&lift_left[2],0.0f,&lift_left[3],2.0f,0.2f,&lift_left[4],1.0f,0.0f,0.50f,1.2f);
	    in_place_fast(0.2f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,&lift_right[2],0.0f,&lift_right[3],2.0f,0.2f,&lift_right[4],1.0f,0.0f,0.50f,1.2f);
	    hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	    side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
			
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
	  	R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
		}
  	sense();
}
}
}

//void Check_Task(void const * argument)
//{
//	for(;;)
//	{
//   /* CAN TX FIFO ��� + �����Զ����� */
//    {
//        static uint32_t s_can1_stuck_cnt = 0;
//        static uint32_t s_can2_stuck_cnt = 0;
//        static uint32_t s_can3_stuck_cnt = 0;

//        uint32_t can1_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
//        uint32_t can2_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
//        uint32_t can3_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3);

//        if (can1_free == 0) {
//            if (++s_can1_stuck_cnt > 100) {
//                HAL_FDCAN_Stop(&hfdcan1);
//                HAL_FDCAN_Start(&hfdcan1);
//                s_can1_stuck_cnt = 0;
//            }
//        } else s_can1_stuck_cnt = 0;

//        if (can2_free == 0) {
//            if (++s_can2_stuck_cnt > 100) {
//                HAL_FDCAN_Stop(&hfdcan2);
//                HAL_FDCAN_Start(&hfdcan2);
//                s_can2_stuck_cnt = 0;
//            }
//        } else s_can2_stuck_cnt = 0;

//        if (can3_free == 0) {
//            if (++s_can3_stuck_cnt > 100) {
//                HAL_FDCAN_Stop(&hfdcan3);
//                HAL_FDCAN_Start(&hfdcan3);
//                s_can3_stuck_cnt = 0;
//            }
//        } else s_can3_stuck_cnt = 0;
//    }

//    /* DM ��������Զ���ʹ�ܣ�ÿ ~100ms ��飩 */
//    {
//        static uint32_t s_dm_check_tick = 0;
//        if (++s_dm_check_tick >= 20)
//        {
//            s_dm_check_tick = 0;
//            if (chassis_angle1.state == OFF)
//                chassis_angle1.send_cmd(&chassis_angle1, Motor_Enable);
//            if (chassis_angle2.state == OFF)
//                chassis_angle2.send_cmd(&chassis_angle2, Motor_Enable);
//            if (chassis_angle3.state == OFF)
//                chassis_angle3.send_cmd(&chassis_angle3, Motor_Enable);
//            if (chassis_angle4.state == OFF)
//                chassis_angle4.send_cmd(&chassis_angle4, Motor_Enable);
//            if (R2_lift_motor_left.state == OFF)
//                R2_lift_motor_left.send_cmd(&R2_lift_motor_left, Motor_Enable);
//            if (R2_lift_motor_right.state == OFF)
//                R2_lift_motor_right.send_cmd(&R2_lift_motor_right, Motor_Enable);
//        }
//    }
//		osDelay(5);
//	}
//}


void Free_Task(void const * argument)
{
  for(;;)
 {

    if(RCctrl.chassis == 1 && RCctrl.zone == 0 && RCctrl.rodPos == 0)  //˫���˰�ť��Ĭ
    {
//		  hold_weapon[0] = 0.0f;
//		  hold_weapon[1] = 2.5f;
//		  hold_weapon[2] = 0.0f;
//		  hold_weapon[3] = 0.4f;
//		  hold_weapon[4] = 0.0f;
//			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1600);//
			vTaskDelay(1);
		}
		else if(BUT_flag.button_new ==1 && BUT_flag.wep_change %2 == 0)  //��ť�������棨���ˣ�
    {
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1400);//�϶��
			
		  hold_weapon[0] = -1.5f;
		  hold_weapon[1] = 0.0f;
		  hold_weapon[2] = 1.2f;
		  hold_weapon[3] = 0.2f;
		  hold_weapon[4] = 0.0f;//���θ�ֵ
			vTaskDelay(600);
			weapon_collect_motor.set_mit_data(&weapon_collect_motor,-1.5f,0.0f,1.2f,0.2f,0.0f);//
			
			vTaskDelay(100);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,600);//�ƶ�	
			vTaskDelay(1200);
			BUT_flag.button_new =0;			
		}
		else if(BUT_flag.button_new ==1 && BUT_flag.wep_change %2 == 1)  //��ť������ȡ�����ˣ�
    {
			hold_weapon[0] = -1.5f;
		  hold_weapon[1] = 0.0f;
	  	hold_weapon[2] = 1.2f;
		  hold_weapon[3] = 0.2f;
		  hold_weapon[4] = 0.0f;//���θ�ֵ
			weapon_collect_motor.set_mit_data(&weapon_collect_motor,-1.5f,0.0f,1.2f,0.2f,0.0f);//��2325
			vTaskDelay(500);
			
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,2500);//�ƶ�	
			vTaskDelay(1600);
			
			hold_weapon[0] = 0.0f;
	  	hold_weapon[1] = 0.8f;
	  	hold_weapon[2] = 0.0f;
	  	hold_weapon[3] = 0.3f;
	  	hold_weapon[4] = 0.0f;//���θ�ֵ
			weapon_collect_motor.set_mit_data(&weapon_collect_motor,0.0f,0.8f,0.0f,0.3f,0.0f);
			vTaskDelay(1000);
			
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1600);	
			BUT_flag.button_new =0;
		}
		
	 if (sent_flag == true) 
{
	JudgeRelativeSize_Global(kfs1,(float)removed_k1); 
	 if(removed_cnt >= 2)
  {
	  vTaskDelay(4000); 
		JudgeRelativeSize_Global(kfs1,(float)removed_k1_2); 
		vTaskDelay(4000); 
	}
}
	 osDelay(5);
 }
}

void Sent_Task(void const * argument)
{
	for(;;)
{
  if(RCctrl.received == true)  //���������յ�
 {
   policy_init(kfs1, kfs2, kf);
	 vTaskDelay(1);
	 
	 bool success = policy_solve_best(
        path, &path_len,           // �����·���ͳ���
        picked_k2, &picked_cnt,    // �����ʰȡ��K2������
        &removed_k1,               // ������Ƴ���K1
        &target_k2                 // �����Ŀ��K2����
    );
	 SwapNumArray(path, path_len);
   SwapNumArray(picked_k2, picked_cnt);
	 
	 vTaskDelay(1);
	 
	 uart8_tx_init();
	 
	 RCctrl.received = false;
	 sent_flag = true;
	 vTaskDelay(1);
 }
 
 if(sent_flag == true)  //��������ȷ�Ϸ���
 {
	 if(BUT_flag.sent_count <=31)
  {
	  HAL_UART_Transmit(&huart8,uart8_tx_buf,8,100);
		vTaskDelay(1);
		HAL_UART_Transmit(&huart9,uart8_tx_buf,8,100);
		BUT_flag.sent_count ++;
		vTaskDelay(220);
	}
	if(BUT_flag.sent_count >31)
  {
	  BUT_flag.sent_count =0;
		sent_flag = false;
	}
	 
 }
 
 
 if(RCctrl.th_put ==true)   //������KFS�յ�
{
  th_order();
	
	if(BUT_flag.sent_count2 <=8)
  {
	  HAL_UART_Transmit(&huart8,four_sent,4,100);
		vTaskDelay(1);
		HAL_UART_Transmit(&huart9,four_sent,4,100);
		BUT_flag.sent_count2 ++;
		vTaskDelay(220);
	}
	if(BUT_flag.sent_count2 >8)
  {
	  BUT_flag.sent_count2 =0;
      RCctrl.area = 3;
		RCctrl.th_put = false;
	}
}

 else if(RCctrl.th_rec !=0)  //����ָ���յ�
{
  th_5mode();
	
	if(BUT_flag.sent_count3 <=8)
  {
	  HAL_UART_Transmit(&huart8,th_sent,5,100);
		vTaskDelay(1);
		HAL_UART_Transmit(&huart9,th_sent,5,100);
		BUT_flag.sent_count3 ++;
		vTaskDelay(220);
	}
	if(BUT_flag.sent_count3 >8)
  {
	  BUT_flag.sent_count3 =0;
      RCctrl.area = 3;
		RCctrl.th_rec = 0;
	}
}

else if(RCctrl.fir_rec == true)  //һ����צ�յ�
{
  fir_dismiss();
	
	if(BUT_flag.sent_count1 <=31)
  {
	  HAL_UART_Transmit(&huart8,four_sent,4,100);
		vTaskDelay(1);
		HAL_UART_Transmit(&huart9,four_sent,4,100);
		BUT_flag.sent_count1 ++;
		vTaskDelay(220);
	}
	if(BUT_flag.sent_count1 >31)
  {
	  BUT_flag.sent_count1 =0;
      RCctrl.area = 3;
		RCctrl.fir_rec = false;
      
	}
}


if(RCctrl.area == 2 && RCctrl.cmd ==3)
{
   r2_hide = true;
}


 
 
   osDelay(5);
}
}



