#include "auto_mode.h"


uint8_t auto_flag =0;
int32_t arm_s_start_time = 0;
uint8_t arm_s_running = 0;
int32_t auto_start_time = 0;
uint8_t  auto_running = 0;
uint8_t auto_init_flag=0;
uint8_t  hold_running = 0;
uint8_t  hold_start_time= 0;
uint8_t  hold_flag= 0;

void Auto_Init(void) //��Ҫ�˶�����������22cm�ĵط�
{
    if(auto_init_flag==5)	{}
	
    else if(auto_init_flag==0)
    {
             arm.mode=11;
             float kfs_2[5]={0,0,0,0,0};
             Torque=Torque_Comp_global(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
              
             if(kfs_arm_2.position<-2.55f)
             {
                kfs_arm_1.set_mit_data(&kfs_arm_1,0.6f,0.0f,55.0f,5.0f,-0.8f*Torque.Torque_1);
                in_place(-2.55f,&kfs_2[0],kfs_arm_2.position,&kfs_2[1],3.5f,&kfs_2[2],
                       50.0f,&kfs_2[3],5.0f,5.0f,&kfs_2[4],-1.65f*Torque.Torque_2,-1.0f*Torque.Torque_2,0.0f);
                kfs_arm_2.set_mit_data(&kfs_arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(kfs_arm_2.position>-2.55f)  
             {   
                in_place(-1.1f,&kfs_2[0],kfs_arm_2.position,&kfs_2[1],1.5f,&kfs_2[2],
                       60.0f,&kfs_2[3],5.0f,5.0f,&kfs_2[4],-0.75f*Torque.Torque_2,-3.0f*Torque.Torque_2,0.5f);
                kfs_arm_2.set_mit_data(&kfs_arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]); 
             }
             if(kfs_arm_2.position<-2.4f&&kfs_arm_2.position>-2.5f)
             {
                kfs_arm_3.set_mit_data(&kfs_arm_3,15.0f,0.0f,0.3f,0.5f,2.0f*Torque.Torque_3);
                kfs_arm_1.set_mit_data(&kfs_arm_1,0.1f,0.3f,55.0f,5.0f,-0.8f*Torque.Torque_1);                 
             }
             if(kfs_arm_2.position>-1.6f&&kfs_arm_2.position<-1.4)
             {
                kfs_arm_1.set_mit_data(&kfs_arm_1,0.1f,0.0f,55.0f,5.0f,-1.2f*Torque.Torque_1); 
             }
             if(kfs_arm_1.position<0.4&&kfs_arm_1.position>0.0&&kfs_arm_2.position<-0.85f&&kfs_arm_2.position>-1.5f&&kfs_arm_3.position>20.0f&&kfs_arm_3.position<36.0f)//��е�����������λ
             {
                auto_init_flag = 1;
             }
                 
    }        
    else if(auto_init_flag==1)
    {
        k_catch[0] = 1.55;
	    	k_catch[1] = 0.0f;
		    k_catch[2] = 2.2f;
		    k_catch[3] = 0.1f;
		    k_catch[4] = 0.0f;
        kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);
     
     	hold_weapon[0] = -1.8f;
		  hold_weapon[1] = 0.0f;
		  hold_weapon[2] = 1.2f;
		  hold_weapon[3] = 0.2f;
	  	hold_weapon[4] = 0.0f;
     
       weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
     
     if(kfs_catch.position >1.3f)//��צ���ſ�
     {
         auto_init_flag = 2;
     }
    }
    else if(auto_init_flag==2)
    {
      float kfs_3[5]={0,0,0,0,0};
	   Torque=Torque_Comp_global(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
       kfs_arm_1.set_mit_data(&kfs_arm_1,0.42f,0.0f,30.0f,3.0f,-0.35*Torque.Torque_1);
       kfs_arm_2.set_mit_data(&kfs_arm_2,-0.39f,0.0f,30.0f,3.0f,-0.3f*Torque.Torque_2);
       in_place(28,&kfs_3[0],kfs_arm_3.position,&kfs_3[1],0.1f,&kfs_3[2],1.0f,&kfs_3[3],0.8f,0.4,&kfs_3[4],0.0f,0.0f,0.30f*PI);
		 kfs_arm_3.set_mit_data(&kfs_arm_3,kfs_3[0],kfs_3[1],kfs_3[2],kfs_3[3],kfs_3[4]);
       if(kfs_arm_1.position>0.4&&kfs_arm_1.position<0.6&&kfs_arm_2.position>-0.32&&kfs_arm_2.position<-0.28&&kfs_arm_3.position>24.8f&&kfs_arm_3.position<30.2f)//�ո˵�λ
       {
          auto_init_flag=3; 
       }
    }
    else if(auto_init_flag==3)
    {
	 arm.mode=2;    
     arm.s=0.4615;
     arm.h=0.14; 
     angles=Arm_Inverse_Solution(&arm);             
     Torque=Torque_Comp_global(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
             
     kfs_arm_1.set_mit_data(&kfs_arm_1,angles.theta_1+0.45f,0.0f,50.0f,5.0f,-0.25*Torque.Torque_1);
     kfs_arm_2.set_mit_data(&kfs_arm_2,-angles.theta_2,0.0f,50.0f,5.0f,-Torque.Torque_2);
     kfs_arm_3.set_mit_data(&kfs_arm_3,-25*(-kfs_arm_2.position-kfs_arm_1.position+0.43+a_weapon+0.12*PI),0.0f,3.0f,2.5f,Torque.Torque_3);
        
     if(kfs_arm_1.position<0.85&&kfs_arm_1.position>0.49&&kfs_arm_2.position>-0.61&&kfs_arm_2.position<-0.25&&kfs_arm_3.position>-31.0f&&kfs_arm_3.position<-9.0f)//��е�����������λ
     {
          auto_init_flag=4;
     }
    }
    if(auto_init_flag >=1 && auto_flag < 4 && RCctrl.zone ==0)
    { 
         in_place_fast(11.5+1.0f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,&lift_left[2],4.0f,&lift_left[3],2.0f,0.5f,&lift_left[4],1.0f,0.05f,0.30f,1.2f);
         in_place_fast(11.5+1.0f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,&lift_right[2],4.0f,&lift_right[3],2.0f,0.5f,&lift_right[4],1.0f,0.05f,0.30f,1.2f);
         hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
         side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
         
         R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
         R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
			
    }
		
}

void auto_mission()  //һ���Խ��Զ�����
{ 

 if(auto_flag==0)//��е���ѵ�λ
 {
         //��צ����
	   arm.mode=2;    
     arm.s=0.4615;
     arm.h=0.14; 
     angles=Arm_Inverse_Solution(&arm);             
     Torque=Torque_Comp_global(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
             
     kfs_arm_1.set_mit_data(&kfs_arm_1,angles.theta_1+0.45f,0.0f,50.0f,5.0f,-0.25*Torque.Torque_1);
     kfs_arm_2.set_mit_data(&kfs_arm_2,-angles.theta_2,0.0f,50.0f,5.0f,-Torque.Torque_2);
     kfs_arm_3.set_mit_data(&kfs_arm_3,-25*(-kfs_arm_2.position-kfs_arm_1.position+0.43+a_weapon+0.12*PI),0.0f,3.0f,2.5f,Torque.Torque_3);
        
     if(kfs_arm_1.position<0.85&&kfs_arm_1.position>0.40&&kfs_arm_2.position>-0.61&&kfs_arm_2.position<-0.25&&kfs_arm_3.position>-31.0f&&kfs_arm_3.position<-9.0f)//��е�����������λ
     {
			  if(auto_running == 0)
              {
                  auto_running = 1;
                  auto_start_time = HAL_GetTick();
              }
              float elapsed = (float)(HAL_GetTick() - auto_start_time) / 1000.0f;
          if(elapsed >= 0.2f)
          {
          auto_flag=1; 
          auto_running = 0;
          }
     }


 }

 if(auto_flag==1)//��е���ѵ�λ
 {
         //��צ����
	  if(kfs_catch.position > 1.4f)
   {
	   k_catch[0] = 0.0f;
		 k_catch[1] = -5.0f;
		 k_catch[2] = 0.0f;
		 k_catch[3] = 0.3f;
		 k_catch[4] = 0.0f;
	 }
	 else if(kfs_catch.position < 1.4f && kfs_catch.position >= 0.40f)
   {
	   k_catch[0] = 0.0f;
		 k_catch[1] = -(((float)kfs_catch.position -0.4f)/(1.0f)*4.0f + 1.0f);
		 k_catch[2] = 0.0f;
		 k_catch[3] = 1.0f/(-k_catch[1]);
		 k_catch[4] = 0.0f;
	 }
	 else if(kfs_catch.position < 0.40 && kfs_catch.position > 0.35 )
   {
	   k_catch[0] = 0.0f;
		 k_catch[1] = 0.0f;
	   k_catch[2] = 0.0f;
	 	 k_catch[3] = 0.0f;
		 k_catch[4] = -1.0f;
	 }
    else if(kfs_catch.position < 0.38f)//��צ�Ѿ�����
    {
                  if(auto_running == 0)
              {
                  auto_running = 1;
                  auto_start_time = HAL_GetTick();
              }
              float elapsed = (float)(HAL_GetTick() - auto_start_time) / 1000.0f;
          if(elapsed >= 0.2f)
          {
          auto_flag=2; 
          auto_running = 0;
          }
    }
    kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);


 }

 else if(auto_flag==2)
 {
      //��е���ո�
	   float kfs_3[5]={0,0,0,0,0};
	   Torque=Torque_Comp_global(&kfs_arm_1,&kfs_arm_2,&kfs_arm_3);
       kfs_arm_1.set_mit_data(&kfs_arm_1,0.4f,0.0f,40.0f,4.0f,-0.35*Torque.Torque_1);
       kfs_arm_2.set_mit_data(&kfs_arm_2,-0.39f,0.0f,30.0f,3.0f,-0.3f*Torque.Torque_2);
       in_place(28,&kfs_3[0],kfs_arm_3.position,&kfs_3[1],0.1f,&kfs_3[2],1.0f,&kfs_3[3],0.8f,0.4,&kfs_3[4],0.0f,0.0f,0.30f*PI);
		 kfs_arm_3.set_mit_data(&kfs_arm_3,kfs_3[0],kfs_3[1],kfs_3[2],kfs_3[3],kfs_3[4]);
       if(kfs_arm_1.position<0.55&&kfs_arm_1.position>0.27&kfs_arm_2.position>-0.37&&kfs_arm_2.position<-0.21&&kfs_arm_3.position>24.8f&&kfs_arm_3.position<30.2f)//�ո˵�λ
       {
          if(auto_running == 0)
              {
                  auto_running = 1;
                  auto_start_time = HAL_GetTick();
              }
              float elapsed = (float)(HAL_GetTick() - auto_start_time) / 1000.0f;
          if(elapsed >= 0.6)
          {
          auto_flag=3; 
          auto_running = 0;
          }
       }

 }
 else if(auto_flag==3)//�Ѿ��ո�
 {
      //2325�н��̶�
	  if(weapon_collect_motor.position >= -11.0f && weapon_collect_motor.position <= -2.0f)
	 {
	    hold_weapon[0] = 0.0f;
		hold_weapon[1] = 0.8f;
		hold_weapon[2] = 0.0f;
		hold_weapon[3] = 0.3f;
		hold_weapon[4] = 0.0f;
   }
	 if(weapon_collect_motor.position > -2.0f&&weapon_collect_motor.position<=0.0f)
	 {
	    hold_weapon[0] = 0.0f;
		hold_weapon[1] = 0.0f;
		hold_weapon[2] = 0.0f;
		hold_weapon[3] = 0.0f;
		hold_weapon[4] = 0.5f;
	 }
     
		weapon_collect_motor.set_mit_data(&weapon_collect_motor,hold_weapon[0],hold_weapon[1],hold_weapon[2],hold_weapon[3],hold_weapon[4]);
     
     if(weapon_collect_motor.position>-0.2f)
     {
       
                   if(auto_running == 0)
              {
                  auto_running = 1;
                  auto_start_time = HAL_GetTick();
              }
              float elapsed = (float)(HAL_GetTick() - auto_start_time) / 1000.0f;
          if(elapsed >= 0.1f)
          {
          auto_flag=4; 
          auto_running = 0;
          }
     }

 }
 else if(auto_flag==4) //2325�Ѽн��̶�
 {
     //�ɻ�е�ۼ�צ
        k_catch[0] = 1.55f;
		k_catch[1] = 0.0f;
		k_catch[2] = 2.2f;
		k_catch[3] = 0.1f;
		k_catch[4] = 0.0f;
        kfs_catch.set_mit_data(&kfs_catch,k_catch[0],k_catch[1],k_catch[2],k_catch[3],k_catch[4]);
     
     if(kfs_catch.position >1.4f)//��צ���ſ�
     {
         if(auto_running == 0)
              {
                  auto_running = 1;
                  auto_start_time = HAL_GetTick();
              }
              float elapsed = (float)(HAL_GetTick() - auto_start_time) / 1000.0f;
          if(elapsed >= 0.5f)
          {
          auto_flag=5; 
          auto_running = 0;
          }
         
     }
 }
 
 else if(auto_flag==5)//��е���Ѿ���צ
 {

     in_place_fast((27.6f),&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,
	                             &lift_left[2],8.0f,&lift_left[3],2.0f,1.8f,&lift_left[4],1.0f,0.02f,0.35f,1.2f);
	 in_place_fast((27.6f),&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,
	                             &lift_right[2],8.0f,&lift_right[3],2.0f,1.8f,&lift_right[4],1.0f,0.02f,0.35f,1.2f);
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);         //初始值调好之后再解注释
     
      R2_lift_motor_left.set_mit_data(&R2_lift_motor_left,lift_left[0],lift_left[1],lift_left[2],lift_left[3],lift_left[4]);
		R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,lift_right[0],lift_right[1],lift_right[2],lift_right[3],lift_right[4]);
     
      if( R2_lift_motor_left.position<27.7&&R2_lift_motor_left.position>27.55&& R2_lift_motor_right.position<27.7&&R2_lift_motor_right.position>27.55)
         {
            auto_init_flag=6; 
         }
//	 h_compensation = remote_control_read(RCctrl.accel,ACCEL_LOW,((ACCEL_HIGH + ACCEL_LOW)/2),ACCEL_HIGH,-0.8f,0,0.8f); //可能可以考虑顶限位
//   in_place_fast((27.6f + h_compensation),&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,
//	                             &lift_left[2],8.0f,&lift_left[3],2.0f,1.8f,&lift_left[4],1.0f,0.02f,0.35f,1.2f);
//	 in_place_fast((27.6f + h_compensation),&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,
//	                             &lift_right[2],8.0f,&lift_right[3],2.0f,1.8f,&lift_right[4],1.0f,0.02f,0.35f,1.2f);
//	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
//	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);         //初始值调好之后再解注释
//     
//     
//     Chassis.Chassis_Calc(&Chassis);
//     chassis_motor1.PID_Calculate(&chassis_motor1, 10.0f*Chassis.param.V_out[0]);
//     chassis_motor2.PID_Calculate(&chassis_motor2, 10.0f*Chassis.param.V_out[1]);
//	 chassis_motor3.PID_Calculate(&chassis_motor3, 10.0f*Chassis.param.V_out[2]);
//     chassis_motor4.PID_Calculate(&chassis_motor4, 10.0f*Chassis.param.V_out[3]);
//		chassis_angle1.set_mit_data(&chassis_angle1,angle_now[0],0.0f,0.7f,0.032f,0.0f);
//		chassis_angle2.set_mit_data(&chassis_angle2,angle_now[1],0.0f,0.7f,0.032f,0.0f);
//		chassis_angle3.set_mit_data(&chassis_angle3,angle_now[2],0.0f,0.7f,0.032f,0.0f);
//		chassis_angle4.set_mit_data(&chassis_angle4,angle_now[3],0.0f,0.7f,0.032f,0.0f);
//	  DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);

//     
//        if(BUT_flag.button_new ==1 && BUT_flag.wep_change %2 == 0)  //��ť�������棨���ˣ�
//    {
//        if(hold_flag==0)
//        {
//			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1400);//�϶��
//			
//		  hold_weapon[0] = -1.5f;
//		  hold_weapon[1] = 0.0f;
//		  hold_weapon[2] = 1.2f;
//		  hold_weapon[3] = 0.2f;
//		  hold_weapon[4] = 0.0f;//���θ�ֵ
//            
//          if(hold_running == 0)
//              {
//                  hold_running = 1;
//                  hold_start_time = HAL_GetTick();
//              }
//          float elapsed = (float)(HAL_GetTick() - hold_start_time) / 1000.0f;
//          if(elapsed >= 0.8f)
//          {
//             hold_flag=1; 
//             hold_running = 0;
//          }
//        }

//         if(hold_flag==1)
//         {             
//			weapon_collect_motor.set_mit_data(&weapon_collect_motor,-1.5f,0.0f,1.2f,0.2f,0.0f);//
//          //            
//          if(hold_running == 0)
//              {
//                  hold_running = 1;
//                  hold_start_time = HAL_GetTick();
//              }
//          float elapsed = (float)(HAL_GetTick() - hold_start_time) / 1000.0f;
//          if(elapsed >= 0.8f)
//          {
//             hold_flag=1; 
//             hold_running = 0;
//          }   
//         }
//			
//			vTaskDelay(200);
//			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,600);//�ƶ�	
//			vTaskDelay(1200);
//			BUT_flag.button_new =0;			
//		}
//		else if(BUT_flag.button_new ==1 && BUT_flag.wep_change %2 == 1)  //��ť������ȡ�����ˣ�
//    {
//			hold_weapon[0] = -1.5f;
//		  hold_weapon[1] = 0.0f;
//	  	hold_weapon[2] = 1.2f;
//		  hold_weapon[3] = 0.2f;
//		  hold_weapon[4] = 0.0f;//���θ�ֵ
//			weapon_collect_motor.set_mit_data(&weapon_collect_motor,-1.5f,0.0f,1.2f,0.2f,0.0f);//��2325
//			vTaskDelay(800);
//			
//			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,2500);//�ƶ�	
//			vTaskDelay(1800);
//			
//			hold_weapon[0] = 0.0f;
//	  	hold_weapon[1] = 0.8f;
//	  	hold_weapon[2] = 0.0f;
//	  	hold_weapon[3] = 0.3f;
//	  	hold_weapon[4] = 0.0f;//���θ�ֵ
//			weapon_collect_motor.set_mit_data(&weapon_collect_motor,0.0f,0.8f,0.0f,0.3f,0.0f);
//			vTaskDelay(1100);
//			
//			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1600);	
//			BUT_flag.button_new =0;
//		}
	 
 }
 
}

//uint8_t distance_count = 0;
//float ml_dis = 1.2f;   //��������ģ�÷�ֱ߳�����
//void auto_second_inplace()  //���ٵִ�����Զ�����
//{
////  auto_inplace(1.2f,2.0f,270); //����λ
////	if(removed_k1 == 2)
//// {
//// 
//// }
//// if(removed_k1 == 11)
//// {
////   //���������ת
////	 //���Ƶ�λ
//// }
//// if(removed_k1 %3 == 1)
//// {
////   //�ҽ������ת
////	   distance_count = (removed_k1 /3) + 1;
////	   auto_inplace(1.2f + distance_count * ml_dis,2.0f,270);
////	 //���Ƶ�λ
//// }
//// if(removed_k1 %3 == 0)
//// {
////   //��������ת
////	   distance_count = removed_k1 /3;
////	   auto_inplace(1.2f + distance_count * ml_dis,2.0f,90);
////	 //���Ƶ�λ
//// }
	
//��һ��������һ������λ���ܿ�R2.�ڶ�����������Ҫȡ�ߵ�R1KFSѡ��÷�ֽ��䣬����ҡ������������Ƶ�KFSǰ
float b=0.03;
float h=0.23;

void kfs_auto(DM_MotorModule *arm_1,DM_MotorModule *arm_2,DM_MotorModule *arm_3)
{
     uint8_t collect_flag=0; 
     if(RCctrl.chassis == 1 && RCctrl.zone == 1)  //
      {                           
          if( RCctrl.key ==0 && collect_flag == 0)
          {
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET); 
              if(arm_s_running == 0)
              {
                  arm_s_running = 1;
                  arm_s_start_time = HAL_GetTick();
              }
              float elapsed = (float)(HAL_GetTick() - arm_s_start_time) / 1000.0f;

              if(elapsed <= 2.0f)
              {
                  float t = elapsed / 2.0f;
                  arm.mode=0;
                  arm.s = 0.45f + t * (0.7f - 0.45f);
                  arm.h=0.35;
                  angles=Arm_Inverse_Solution(&arm);
                  Torque=Torque_Comp_global(arm_1,arm_2,arm_3);

                  arm_1->set_mit_data(arm_1,angles.theta_1+0.43f,0.0f,30.0f,2.0f,-0.25*Torque.Torque_1);
                  arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,50.0f,4.5f,-0.6*Torque.Torque_2);
                  arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position+PI*0.01),0.0f,0.2f,0.15f,0.8*Torque.Torque_3);
              }
              else if(elapsed > 2.0f && elapsed <= 2.4f)
              {
                  float t = (elapsed - 2.0f) / 0.4f;
                  arm.mode=0;
                  arm.s = 0.45f - t * (0.7f - 0.65f);
                  arm.h=0.35;
                  angles=Arm_Inverse_Solution(&arm);
                  Torque=Torque_Comp_global(arm_1,arm_2,arm_3);

                  arm_1->set_mit_data(arm_1,angles.theta_1+0.43f,0.0f,30.0f,2.0f,-0.25*Torque.Torque_1);
                  arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,50.0f,4.5f,-0.6*Torque.Torque_2);
                  arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position+PI*0.01),0.0f,0.2f,0.15f,0.8*Torque.Torque_3);
              }
              else
              {
                    collect_flag=1;
               }
           }
          if(RCctrl.key ==1 && collect_flag == 0)  //
          { 
             HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET); 
              if(arm_s_running == 0)
              {
                  arm_s_running = 1;
                  arm_s_start_time = HAL_GetTick();
              }
              float elapsed = (float)(HAL_GetTick() - arm_s_start_time) / 1000.0f;

              if(elapsed <= 2.0f)
              {
                  float t = elapsed / 2.0f;
                  arm.mode=0;
                  arm.s = 0.45f + t * (0.7f - 0.45f);
                  arm.h=0.53;
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
   
              
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,30.0f,2.0f,-0.25*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,60.0f,5.0f,-0.5*Torque.Torque_2);
              arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position-0.00*PI),0.0f,0.15f,0.1f,0.8*Torque.Torque_3);
              }
              else if(elapsed > 2.0f && elapsed <= 2.4f)
              {
                  float t = (elapsed - 2.0f) / 0.4f;
                  arm.mode=0;
                  arm.s = 0.7f - t * (0.7f - 0.65f);
                  arm.h=0.53;
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
   
              
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,30.0f,2.0f,-0.25*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,60.0f,5.0f,-0.5*Torque.Torque_2);
              arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position-0.00*PI),0.0f,0.15f,0.1f,0.8*Torque.Torque_3);
              }
              else
              {
                     collect_flag=1;
              }
          }

         if( RCctrl.key ==2 && collect_flag == 0)  //
         {
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET); 
              if(arm_s_running == 0)
              {
                  arm_s_running = 1;
                  arm_s_start_time = HAL_GetTick();
              }
              float elapsed = (float)(HAL_GetTick() - arm_s_start_time) / 1000.0f;

              if(elapsed <= 2.0f)
              {
                  float t = elapsed / 2.0f;
                  arm.mode=0;
                  arm.s = 0.45f + t * (0.7f - 0.45f);
              arm.h=0.77;
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,45.0f,3.0f,-0.25*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,60.0f,5.0f,-0.5*Torque.Torque_2);
              arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position-0.01*PI),0.0f,0.15f,0.1f,0.8*Torque.Torque_3);
              }
              else if(elapsed > 2.0f && elapsed <= 2.4f)
              {
                  float t = (elapsed - 2.0f) / 0.4f;
                  arm.mode=0;
                  arm.s = 0.7f - t * (0.7f - 0.65f);
              arm.h=0.77;
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,45.0f,3.0f,-0.25*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,60.0f,5.0f,-0.5*Torque.Torque_2);
              arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position-0.01*PI),0.0f,0.15f,0.1f,0.8*Torque.Torque_3);
              }
              else
              {
                  collect_flag=1;            
              }
          }
        }  
      if(collect_flag==1 && RCctrl.led_3 == 0)  //led3非触发自动存矿
      {
            float kfs_1[5]={0,0,0,0,0};//位置,速度,kp,kd,t
            float kfs_2[5]={0,0,0,0,0};
            float kfs_3[5]={0,0,0,0,0};
            
                arm.mode=10;
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);              

              
             if(arm_2->position>-2.0f)  
             {                    
                in_place(-2.0f,&kfs_2[0],arm_2->position,&kfs_2[1],1.0f,&kfs_2[2],
                       30.0f,&kfs_2[3],5.0f,4.0f,&kfs_2[4],-0.5f*Torque.Torque_2,-2.0f*Torque.Torque_2,0.0f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position>-2.92f&&arm_2->position<-1.9f)   
             {   
                in_place(0.30f,&kfs_1[0],arm_1->position,&kfs_1[1],1.0f,&kfs_1[2],
                       55.0f,&kfs_1[3],5.0f,5.0f,&kfs_1[4],-0.5f*Torque.Torque_1,-0.5f*Torque.Torque_1,0.3f);
                arm_1->set_mit_data(arm_1,kfs_1[0],kfs_1[1],kfs_1[2],kfs_1[3],kfs_1[4]);                 
                in_place(-2.92f,&kfs_2[0],arm_2->position,&kfs_2[1],0.5f,&kfs_2[2],
                       30.0f,&kfs_2[3],5.0f,4.0f,&kfs_2[4],-0.2f*Torque.Torque_2,-1.8f*Torque.Torque_2,0.0f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position<-2.92f)
             {
                arm_3->set_mit_data(arm_3,target_3,0.0f,0.3f,0.6f,0.2f*Torque.Torque_3);
                arm_1->set_mit_data(arm_1,target_1,0.0f,60.0f,5.0f,-0.8f*Torque.Torque_1); 
                in_place_1(target_2,&kfs_2[0],arm_2->position,&kfs_2[1],0.05f,&kfs_2[2],
                       65.0f,&kfs_2[3],3.0f,5.0f,&kfs_2[4],-0.1f*Torque.Torque_2,-0.1f*Torque.Torque_2,0.28f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position<-1.3f&&arm_2->position>-1.4f)
             {
                arm_1->set_mit_data(arm_1,target_1,0.0f,60.0f,5.0f,-0.8f*Torque.Torque_1);  
                arm_3->set_mit_data(arm_3,target_3,0.0f,0.3f,0.6f,0.2f*Torque.Torque_3); 
             }

//             if(arm_2->position<-4.34f&&arm_1->position>0.446f&&arm_3->position>33.61f)
//             {
//                 arm_1->set_mit_data(arm_1,0.0f,0.0f,0.0f,0.0f,0.0f);
//                 arm_2->set_mit_data(arm_2,0.0f,0.0f,0.0f,0.0f,0.0f);
//                 arm_3->set_mit_data(arm_3,0.0f,0.0f,0.0f,0.0f,0.0f);
//             }

      }
      
        else if(collect_flag==1 && RCctrl.led_3 == 1)  //led3触发自动回收
        {
              float kfs_1[5]={0,0,0,0,0};//位置,速度,kp,kd,t
              float kfs_2[5]={0,0,0,0,0};
              float kfs_3[5]={0,0,0,0,0};
                          Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              // t_position, *sent_p, c_position, *sent_sp, u_speed, *s_kp, h_kp, *s_kd, h_kd, hold_kd, *s_t, h_t, hold_t ignore_diff
             if(arm_2->position>-1.6f)  
             {                    
                in_place(-1.6f,&kfs_2[0],arm_2->position,&kfs_2[1],0.6f,&kfs_2[2],
                       30.0f,&kfs_2[3],5.0f,4.0f,&kfs_2[4],-0.5f*Torque.Torque_2,-2.0f*Torque.Torque_2,0.0f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position>-2.92f&&arm_2->position<-1.6f)  
             {   
                in_place(0.2f,&kfs_1[0],arm_1->position,&kfs_1[1],2.0f,&kfs_1[2],
                       55.0f,&kfs_1[3],5.0f,5.0f,&kfs_1[4],-0.5f*Torque.Torque_1,-0.5f*Torque.Torque_1,0.3f);
                arm_1->set_mit_data(arm_1,kfs_1[0],kfs_1[1],kfs_1[2],kfs_1[3],kfs_1[4]);                 
                in_place(-2.92f,&kfs_2[0],arm_2->position,&kfs_2[1],0.1f,&kfs_2[2],
                       30.0f,&kfs_2[3],5.0f,4.0f,&kfs_2[4],-0.2f*Torque.Torque_2,-1.8f*Torque.Torque_2,0.0f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position<-2.92f)
             {
                arm_3->set_mit_data(arm_3,-52.0f,0.0f,0.13f,0.6f,1.0*Torque.Torque_3);
                arm_1->set_mit_data(arm_1,0.6f,0.0f,60.0f,5.0f,-0.8f*Torque.Torque_1);
                in_place_1(-4.3f,&kfs_2[0],arm_2->position,&kfs_2[1],-0.0f,&kfs_2[2],
                       65.0f,&kfs_2[3],3.0f,5.0f,&kfs_2[4],-0.12f*Torque.Torque_2,-1.0f*Torque.Torque_2,0.28f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position<-2.8f&&arm_2->position>-2.9f)
             {
                arm_3->set_mit_data(arm_3,-52.0f,0.0f,0.13f,0.6f,1.0*Torque.Torque_3);
                arm_1->set_mit_data(arm_1,0.4f,-0.3f,55.0f,5.0f,-0.8f*Torque.Torque_1);  
             }
        }
      
      if(!(RCctrl.chassis == 1 && RCctrl.zone == 1 && (RCctrl.key == 0 || RCctrl.key == 1 || RCctrl.key == 2)))
      {
           arm_s_running = 0;
           collect_flag = 0;
      }
}

float mid_x = 0.0f;  //mid360����
float mid_y = 0.0f;
float mid_yaw = 0.0f; //��ʼ��е�۳�ͷΪ0��

float mid_x_last = 0.0f;    //�����ϴ�����
float mid_y_last = 0.0f;
float mid_yaw_last = 0.0f; //��ʼ��е�۳�ͷΪ0��

float diff_x = 0.0f;
float diff_y = 0.0f;
float diff_w = 0.0f; //mid360���������������

float yaw_rad = 0.0f;
float auto_x = 0.0f;
float auto_y = 0.0f; //��������ϵ��...

/* by deepseek: radar-to-chassis-center mapping (radar at left-front, 392mm offset) */
ChassisCoord_t chassis_coord = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* by deepseek: debug flags — modify in Keil debugger to trigger auto move */
volatile uint8_t debug_go_flag  = 0;
volatile float   debug_target_x = 0.5f;
volatile float   debug_target_y = 0.0f;
volatile float   debug_target_w = 0.0f;

/* by deepseek: called every cycle from Sensor_Task to keep chassis coords fresh */
void chassis_coord_update(void)
{
    float yaw_r = mid_yaw * PI / 180.0f;
    chassis_coord.radar_wx = -mid_y;
    chassis_coord.radar_wy = -mid_x;
    float off_wx = 0.392f * cosf(-yaw_r) - 0.392f * sinf(-yaw_r);
    float off_wy = 0.392f * sinf(-yaw_r) + 0.392f * cosf(-yaw_r);
    chassis_coord.center_x = chassis_coord.radar_wx - off_wx;
    chassis_coord.center_y = chassis_coord.radar_wy - off_wy;
    chassis_coord.yaw_rad = yaw_r;
}

/* by deepseek */
static float _target_x_prev = -999.0f;
static float _target_y_prev = -999.0f;
static float _target_w_prev = -999.0f;

/* by deepseek: target_x/y/w = world-frame absolute coords */
void auto_inplace(float target_x,float target_y,float target_w)
{
    if(target_x != _target_x_prev || target_y != _target_y_prev || target_w != _target_w_prev)
    {
        auto_pid_reset();
        _target_x_prev = target_x;
        _target_y_prev = target_y;
        _target_w_prev = target_w;
    }

    diff_x = target_x - chassis_coord.center_x;
    diff_y = target_y - chassis_coord.center_y;
    diff_w = target_w - mid_yaw;
	
	  while(diff_w < -180.0f){diff_w += 360.0f;}
		while(diff_w >  180.0f){diff_w -= 360.0f;}
	 yaw_rad = chassis_coord.yaw_rad;

     auto_x=  diff_y * cosf(yaw_rad) + diff_x * sinf(yaw_rad);
     auto_y=  diff_y * sinf(yaw_rad) - diff_x * cosf(yaw_rad);
	
  	SP_ACCEL = 25.0f; //�Զ���10���ֶ���20~25
    SP_X = auto_set(auto_x,-0.9f,0.9f,-1.0f,1.0f,0.02f) + auto_pid(auto_x, 0);
   	SP_Y = - auto_set(auto_y,-0.9f,0.9f,-1.0f,1.0f,0.02f) + auto_pid(auto_y, 1);      
	SP_W = - (auto_set(diff_w,-30.0f,30.0f,-1.0f,1.0f,2.0f) + auto_pid(diff_w, 2)); 


    if(SP_X > 1.0f) SP_X = 1.0f; else if(SP_X < -1.0f) SP_X = -1.0f;
    if(SP_Y > 1.0f) SP_Y = 1.0f; else if(SP_Y < -1.0f) SP_Y = -1.0f;
    if(SP_W > 1.2f) SP_W = 1.2f; else if(SP_W < -1.2f) SP_W = -1.2f;

    auto_chassis();	
	
	  chassis_motor1.PID_Calculate(&chassis_motor1, 10.0f*auto_Vout[0]);
    chassis_motor2.PID_Calculate(&chassis_motor2, 10.0f*auto_Vout[1]);
		chassis_motor3.PID_Calculate(&chassis_motor3, 10.0f*auto_Vout[2]);
    chassis_motor4.PID_Calculate(&chassis_motor4, 10.0f*auto_Vout[3]);
	
		chassis_angle1.set_mit_data(&chassis_angle1,angle_now[0],0.0f,0.7f,0.032f,0.0f);
		chassis_angle2.set_mit_data(&chassis_angle2,angle_now[1],0.0f,0.7f,0.032f,0.0f);
		chassis_angle3.set_mit_data(&chassis_angle3,angle_now[2],0.0f,0.7f,0.032f,0.0f);
		chassis_angle4.set_mit_data(&chassis_angle4,angle_now[3],0.0f,0.7f,0.032f,0.0f);
	
		DJIset_motor_data(&hfdcan2, 0x200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);	
}

//�Զ����ӳ�亯��
float auto_set(float src, float src_low, float src_high, float dst_min, float dst_max, float deadzone)
{
	float persent = 0.0f;
	float res = 0.0f;
	     
	 if(src > src_high){src = src_high;}
	 if(src < src_low){src = src_low;}//��Χ��飬�û�

	if(src < 0 + deadzone  && src > 0 - deadzone)
  {
	  persent = 0;
		res = 0;
	}
  else if(src > 0 + deadzone)
  {
	  persent = (float)(src - 0)/(float)(src_high - (0 + deadzone));
		res = persent * (dst_max - 0);
	}
	else if(src < 0 - deadzone)
  {
	  persent = - (float)(0- src)/(float)((0 - deadzone) - src_low);
		res = persent * (0 - dst_min);
	} 
   return res;
}


float auto_i[3] = {0.0f, 0.0f, 0.0f};
float auto_last_err[3] = {0.0f, 0.0f, 0.0f};
float pi_max = 500.0f;  //积分限
float auto_id[2] = {0.0002,0.01};
float omega_imax = 5000;
float omega_i = 0.000015;

/* by deepseek */
void auto_pid_reset(void)
{
    auto_i[0] = 0.0f;
    auto_i[1] = 0.0f;
    auto_i[2] = 0.0f;
    auto_last_err[0] = 0.0f;
    auto_last_err[1] = 0.0f;
    auto_last_err[2] = 0.0f;
}

//PIDλ�û��ֺ���
/* by deepseek */
float auto_pid(float diff, uint8_t flag)
{
    if(flag > 2) return 0.0f;
    if(flag ==0){if(auto_x>-0.02 && auto_x<0.02) {return 0.0f;}}
    if(flag ==1){if(auto_y>-0.02 && auto_y<0.02) {return 0.0f;}}
    if(flag ==2){if(diff_w >-0.03 && diff_w <0.03) {return 0.0f;}}

    auto_i[flag] += diff;
    if(flag == 2)
    {
       if(auto_i[flag] > omega_imax)   {auto_i[flag] = omega_imax; }
       if(auto_i[flag] < -omega_imax)   {auto_i[flag] = -omega_imax; }
    }
    else
    {
    if(auto_i[flag] > pi_max)       {auto_i[flag] = pi_max; }
    else if(auto_i[flag] < -pi_max) {auto_i[flag] = -pi_max; }
    }

    float d_term = diff - auto_last_err[flag];
    auto_last_err[flag] = diff;
       if(flag == 2) 
        {
         return auto_id[0] * omega_i + auto_id[1] * d_term;
        }
       else
     {
       return auto_id[0] * auto_i[flag] + auto_id[1] * d_term;
}    }

//float auto_pid()
//{
//  distance = sqrtf(diff_x * diff_x + diff_y * diff_y);
//	
//	if(auto_i < pi_max)
// {
//   auto_i += distance;
// }
//   auto_d = distance_last - distance; //
//  
//   float res = auto_id[0] * auto_i - auto_id[1] * auto_d;
//   distance_last = distance;
// 
//   return res;
//}  //distance id��







