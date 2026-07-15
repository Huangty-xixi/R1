#include "lift.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>
#include "timer.h"
#include "motor_control.h"
#include "weapon.h"

Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;
DM_MotorModule R2_lift_motor_right;


float lift_left[5] = {0,0,0,0,0};
float lift_right[5] = {0,0,0,0,0};
float lift_change =0.0f;

void lift_mode()
{
  lift_change = remote_control_read(RCctrl.accel,ACCEL_LOW,((ACCEL_HIGH + ACCEL_LOW)/2),ACCEL_HIGH,-1.2f,0,1.2f);
	
  if(RCctrl.topPos == 0)  //r2Ì§Éý´¥·¢
 {
   in_place_fast(18.5f + lift_change,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],8.0f,3.0f,&lift_left[2],20.0f,&lift_left[3],5.0f,3.0f,&lift_left[4],6.6f,3.5f,0.34f,1.2f);
	 in_place_fast(18.5f + lift_change,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],8.0f,3.0f,&lift_right[2],20.0f,&lift_right[3],5.0f,3.0f,&lift_right[4],6.6f,3.8f,0.34f,1.2f);
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }
 else if(RCctrl.topPos == 1)  //r2Ì§Éý·Ç´¥·¢
 {
   lift_left[0] = 0.0f;
	 lift_left[1] = -1.0f;
	 lift_left[2] = 0.0f;
	 lift_left[3] = 3.5f;
	 lift_left[4] = 0.0f;
	 
	 lift_right[0] = 0.0f;
	 lift_right[1] = -1.0f;
	 lift_right[2] = 0.0f;
	 lift_right[3] = 3.46f;
	 lift_right[4] = 0.0f;     //´óDÖµ×ÔÂä
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }

 else if(RCctrl.topPos == 2)  //¹¥»÷Ì§Éý·Ç´¥·¢
 {
   in_place_fast(0.2f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,&lift_left[2],0.0f,&lift_left[3],2.0f,0.2f,&lift_left[4],1.0f,0.0f,0.50f,1.2f);
	 in_place_fast(0.2f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,&lift_right[2],0.0f,&lift_right[3],2.0f,0.2f,&lift_right[4],1.0f,0.0f,0.50f,1.2f);
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }
 else if(RCctrl.topPos == 3)  //¹¥»÷Ì§Éý´¥·¢
 {
   in_place_fast(16.0f,&lift_left[0],R2_lift_motor_left.position,&lift_left[1],18.0f,8.0f,&lift_left[2],4.0f,&lift_left[3],2.0f,0.5f,&lift_left[4],1.0f,0.05f,0.24f,1.2f);
	 in_place_fast(16.0f,&lift_right[0],R2_lift_motor_right.position,&lift_right[1],18.0f,8.0f,&lift_right[2],4.0f,&lift_right[3],2.0f,0.5f,&lift_right[4],1.0f,0.05f,0.24f,1.2f);
	 hold_step(R2_lift_motor_left.speed_w,R2_lift_motor_right.speed_w,&lift_right[4]);
	 side_by_side(R2_lift_motor_left.position,R2_lift_motor_right.position,&lift_right[4]);
 }
 
   weapon_joint_input = - 8000.0f * SPEED_X;//¹¥»÷2006³£×¤
 
}




