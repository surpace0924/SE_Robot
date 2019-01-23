#include "SE_Robot.h"

// ポート定義
#define PORT_L_MT NXT_PORT_B
#define PORT_R_MT NXT_PORT_A
#define PORT_FLAG_MT NXT_PORT_C
#define PORT_LINE_SENS NXT_PORT_S2
#define PORT_START_BTN NXT_PORT_S3
#define PORT_SENSE_BIT NXT_PORT_S1

// 各種定数
#define M_PI 3.1415926535
#define K_ANGLE_CORRECT 0.93994778067
#define LINE_SENSOR_TH 600
const char target_subsystem_name[] = "OSEK SE_Robot";

// フラグ変数
U8 clip_num = 0;
U8 is_line = 1;
U8 is_discovered = 0;

// データ変数
float mt_n_L = 0;
float mt_n_R = 0;
float robot_angle = 0;

// 関数宣言
void lineTrace(U8 is_line, U8 line_side, U32 duty_h, U32 duty_l);
void appeal();

void ecrobot_device_initialize(void)
{
    nxt_motor_set_count(PORT_R_MT, 0);
    nxt_motor_set_count(PORT_L_MT, 0);
    nxt_motor_set_count(PORT_FLAG_MT, 0);
    ecrobot_set_light_sensor_active(PORT_LINE_SENS);
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
    static float dt = 0.004;
    U32 duty_h = 70;
    U32 duty_l = 50;

    // センサの値処理（２値化）
    U16 ls_val = ecrobot_get_light_sensor(PORT_LINE_SENS);
    is_line = (ls_val > LINE_SENSOR_TH) ? 1 : 0;

    // 車輪のオドメトリ計算[回転]
    mt_n_L = nxt_motor_get_count(PORT_L_MT) / 360.;
    mt_n_R = nxt_motor_get_count(PORT_R_MT) / 360.;

    // 自己角度推定
    robot_angle = K_ANGLE_CORRECT * 180 * 40.8 / 52 * (mt_n_R - mt_n_L);

    // 移動のシーケンス
    switch (clip_num)
    {
    // スタート待機
    // 次シーケンス遷移トリガ: スタートボタンタッチ
    case 0:
        if (ecrobot_get_touch_sensor(PORT_START_BTN))
            clip_num++;
        break;

    // ライントレース開始
    // 次シーケンス遷移トリガ: 障害物検出
    case 1:
        is_discovered = 1;
        lineTrace(is_line, 1, duty_h, duty_l);

        if (ecrobot_get_touch_sensor(PORT_SENSE_BIT))
            clip_num++;
        break;

    // 旋回
    // 次シーケンス遷移トリガ: 角度収束
    case 2:
        nxt_motor_set_speed(PORT_R_MT, -70, 0);
        nxt_motor_set_speed(PORT_L_MT, 70, 0);
        // robot_angle = K_ANGLE_CORRECT * 180 * 40.8 / 52 * (mt_n_R - mt_n_L);

        if (robot_angle < -450)
            clip_num++;
        break;

    // ライントレースで帰る
    // 次シーケンス遷移トリガ: なし
    case 3:
        lineTrace(is_line, 1, duty_h, duty_l);
        break;

    default:
        break;
    }

    // 発見したらアピール（移動とは非同期）
    if (is_discovered)
    {
        appeal();
    }

    TerminateTask();
}

// is_line: ライン上ならture
// line_side: ラインが車体左ならture, 右ならfalse
void lineTrace(U8 is_line, U8 line_side, U32 duty_h, U32 duty_l)
{
    if (line_side)
    {
        if (is_line)
        {
            nxt_motor_set_speed(PORT_R_MT, duty_l, 0);
            nxt_motor_set_speed(PORT_L_MT, duty_h, 0);
        }
        else
        {
            nxt_motor_set_speed(PORT_R_MT, duty_h, 0);
            nxt_motor_set_speed(PORT_L_MT, duty_l, 0);
        }
    }
    else
    {
        if (is_line)
        {
            nxt_motor_set_speed(PORT_R_MT, duty_h, 0);
            nxt_motor_set_speed(PORT_L_MT, duty_l, 0);
        }
        else
        {
            nxt_motor_set_speed(PORT_R_MT, duty_l, 0);
            nxt_motor_set_speed(PORT_L_MT, duty_h, 0);
        }
    }
}

void appeal()
{
    if (nxt_motor_get_count(PORT_FLAG_MT) < 620)
        nxt_motor_set_speed(PORT_FLAG_MT, 80, 0);
    else
        nxt_motor_set_speed(PORT_FLAG_MT, 0, 1);
}
