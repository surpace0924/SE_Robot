#include "SE_Robot.h"

#define M_PI 3.1415926535
#define K_ANGLE_CORRECT 0.93994778067

// EXTERNAL_WAV_DATA(StageClearMono);

int is_line = 1;
float mt_n_L = 0;
float mt_n_R = 0;
float robot_angle = 0;

U8 is_touch = 0;
U8 touch_flag = 0;
int start_flag = 0;

void ecrobot_device_initialize(void)
{
  nxt_motor_set_count(NXT_PORT_A, 0);
  nxt_motor_set_count(NXT_PORT_B, 0);
  nxt_motor_set_count(NXT_PORT_C, 0);

  ecrobot_set_light_sensor_active(NXT_PORT_S2);
}

void ecrobot_device_terminate(void)
{
	/* DO NOTHING*/
}

/*============================================================================
 * TOPPERS OSEK specific Function/Tasks
 *===========================================================================*/
DeclareCounter(SysTimerCnt);


void user_1ms_isr_type2(void)
{
	/* Increment System Timer Count to activate periodical Tasks */
	(void)SignalCounter(SysTimerCnt);
	/* DO NOTHING*/
}

const char target_subsystem_name[] = "OSEK SE_Robot";

TASK(DisplayTask)
{
	ecrobot_status_monitor(target_subsystem_name);

  // display_goto_xy(0, 0);
  // display_int(is_line, 15);
  // display_goto_xy(0, 1);
  // display_int((int)mt_n_L, 15);
  // display_goto_xy(0, 2);
  // display_int((int)mt_n_R, 15);
  // display_goto_xy(0, 3);
  // display_int((int)robot_angle, 15);
  display_update();

  TerminateTask();
}


TASK(TouchSensorTask)
{
	TerminateTask();
}




TASK(ActionTask)
{

  if(start_flag)
  {
    static float dt = 0.004;
    U32 duty_h = 70;
    U32 duty_l = 50;
    U16 ls_val = ecrobot_get_light_sensor(NXT_PORT_S2);
    is_line = (ls_val > 600) ? 1 : 0;

    mt_n_L = nxt_motor_get_count(NXT_PORT_B) / 360.;
    mt_n_R = nxt_motor_get_count(NXT_PORT_A) / 360.;

    robot_angle = K_ANGLE_CORRECT * 180 * 40.8 / 52 * (mt_n_R - mt_n_L);

    if(!is_touch)
    {
      is_touch = ecrobot_get_touch_sensor(NXT_PORT_S1);
      if(is_line)
      {
        nxt_motor_set_speed(NXT_PORT_A, duty_l, 0);
        nxt_motor_set_speed(NXT_PORT_B, duty_h, 0);
      }
      else
      {
        nxt_motor_set_speed(NXT_PORT_A, duty_h, 0);
        nxt_motor_set_speed(NXT_PORT_B, duty_l, 0);
      }
    }
    else
    {
      if(nxt_motor_get_count(NXT_PORT_C) < 620)
        nxt_motor_set_speed(NXT_PORT_C, 80, 0);
      else
        nxt_motor_set_speed(NXT_PORT_C, 0, 1);


      if(touch_flag ==0)
      {
        nxt_motor_set_speed(NXT_PORT_A, -70, 0);
        nxt_motor_set_speed(NXT_PORT_B, 70, 0);

        robot_angle = K_ANGLE_CORRECT * 180 * 40.8 / 52 * (mt_n_R - mt_n_L);
        if(robot_angle < -450)
          touch_flag = 1;
      }
      else
      {
        if(is_line)
        {
          nxt_motor_set_speed(NXT_PORT_A, duty_h, 0);
          nxt_motor_set_speed(NXT_PORT_B, duty_l, 0);
        }
        else
        {
          nxt_motor_set_speed(NXT_PORT_A, duty_l, 0);
          nxt_motor_set_speed(NXT_PORT_B, duty_h, 0);
        }
      }
    }
  }
  else
  {
      start_flag = ecrobot_get_touch_sensor(NXT_PORT_S3);
  }

	TerminateTask();
}
