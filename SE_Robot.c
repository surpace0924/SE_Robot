#include "SE_Robot.h"

// ポート定義
#define PORT_L_MT NXT_PORT_B
#define PORT_R_MT NXT_PORT_A
#define PORT_FLAG_MT NXT_PORT_C
#define PORT_LINE_SENS NXT_PORT_S2
#define PORT_START_BTN NXT_PORT_S3
#define PORT_SENSE_BIT NXT_PORT_S1

// 各種定数
#define KP 0.0
#define KI 0.0
#define KD 0.0
#define M_PI 3.1415926535
#define K_ANGLE_CORRECT 1 //0.93994778067
#define LINE_SENSOR_TH 500
const char target_subsystem_name[] = "OSEK SE_Robot";

// フラグ変数
U8 clip_num = 0;
U8 is_line = 1;
U8 is_discovered = 0;
U8 is_returned = 0;
U8 sound_flag = 0;
U8 sound_clip = 0;
U8 flag_num = 0;

// データ変数
U16 ls_val;
F32 mt_n_L = 0;
F32 mt_n_R = 0;
F32 robot_angle = 0;

// 関数宣言
void lineTrace(U8 is_line, U8 line_side, U32 duty_h, U32 duty_l);
void appeal();
void controlFlag();
void controlSound();
int soundUnsync(U32 freq, U32 ms, U32 vol);

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
    // ecrobot_status_monitor(target_subsystem_name);

    display_goto_xy(0, 0);
    display_int(ls_val, 15);
    display_goto_xy(0, 1);
    display_int(is_line, 15);
    display_goto_xy(0, 2);
    display_int((int)mt_n_L, 15);
    display_goto_xy(0, 3);
    display_int((int)mt_n_R, 15);
    display_goto_xy(0, 4);
    display_int((int)robot_angle, 15);
    display_update();

    TerminateTask();
}

TASK(TouchSensorTask)
{
    TerminateTask();
}

TASK(ActionTask)
{
    static F32 dt = 0.004;
    U32 duty_h = 85;
    U32 duty_l = 25;

    // センサの値処理（２値化）
    ls_val = ecrobot_get_light_sensor(PORT_LINE_SENS);
    is_line = (ls_val > LINE_SENSOR_TH) ? 1 : 0;

    // 車輪のオドメトリ計算[回転]
    mt_n_L = nxt_motor_get_count(PORT_L_MT) / 360.;
    mt_n_R = nxt_motor_get_count(PORT_R_MT) / 360.;

    // 自己角度推定
    robot_angle = K_ANGLE_CORRECT * 180 * (55 / 2) / 52 * (mt_n_R - mt_n_L);

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
        lineTrace(is_line, 0, duty_h, duty_l);

        if (ecrobot_get_touch_sensor(PORT_SENSE_BIT))
            clip_num++;
        break;

    // 旋回
    // 次シーケンス遷移トリガ: 角度収束
    case 2:
        is_discovered = 1;
        nxt_motor_set_speed(PORT_R_MT, 70, 0);
        nxt_motor_set_speed(PORT_L_MT, -70, 0);

        if (robot_angle > -90)
            clip_num++;
        break;

    // ライントレースで帰る
    // 次シーケンス遷移トリガ: 車体角度が-5deg未満
    case 3:
        lineTrace(is_line, 1, duty_h, duty_l);

        if (robot_angle < -5)
            clip_num++;
        break;

    // 全速力で直進
    // 次シーケンス遷移トリガ: なし
    case 4:
        is_returned = 1;
        nxt_motor_set_speed(PORT_R_MT, 100, 0);
        nxt_motor_set_speed(PORT_L_MT, 100, 0);
        break;

    default:
        break;
    }

    // 発見したらアピール（移動とは非同期）
    if (is_discovered)
    {
        // 障害物発見後
        controlSound();

        if (!is_returned)
        {
            // 障害物発見後
            // 旗を揚げる
            if (nxt_motor_get_count(PORT_FLAG_MT) < 620)
                nxt_motor_set_speed(PORT_FLAG_MT, 80, 0);
            else
                nxt_motor_set_speed(PORT_FLAG_MT, 0, 1);
        }
        else
        {
            // 最後の直線侵入後
            // 旗を降ろす
            if (nxt_motor_get_count(PORT_FLAG_MT) > 0)
                nxt_motor_set_speed(PORT_FLAG_MT, -80, 0);
            else
                nxt_motor_set_speed(PORT_FLAG_MT, 0, 1);
        }
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
}

void controlFlag()
{
}

void controlSound()
{
    if (sound_flag == 0)
    {
        switch (sound_clip)
        {
        case 0:
            sound_clip += sound(196, 170, 100); // ソ
            break;
        case 1:
            sound_clip += sound(262, 170, 100); // ド
            break;
        case 2:
            sound_clip += sound(330, 170, 100); // ミ
            break;
        case 3:
            sound_clip += sound(392, 170, 100); // ソ
            break;
        case 4:
            sound_clip += sound(523, 170, 100); // ド
            break;
        case 5:
            sound_clip += sound(659, 170, 100); // ミ
            break;
        case 6:
            sound_clip += sound(784, 510, 100); // ソ
            break;
        case 7:
            sound_clip += sound(659, 510, 100); // ミ
            break;
        case 8:
            sound_clip += sound(208, 170, 100); // ラ♭
            break;
        case 9:
            sound_clip += sound(262, 170, 100); // ド
            break;
        case 10:
            sound_clip += sound(311, 170, 100); // ミ♭
            break;
        case 11:
            sound_clip += sound(415, 170, 100); // ラ♭
            break;
        case 12:
            sound_clip += sound(523, 170, 100); // ド
            break;
        case 13:
            sound_clip += sound(622, 170, 100); // ミ♭♭
            break;
        case 14:
            sound_clip += sound(831, 510, 100); // ラ♭
            break;
        case 15:
            sound_clip += sound(622, 510, 100); // ミ
            break;
        case 16:
            sound_clip += sound(233, 170, 100); // シ
            break;
        case 17:
            sound_clip += sound(294, 170, 100); // レ
            break;
        case 18:
            sound_clip += sound(349, 170, 100); // ファ
            break;
        case 19:
            sound_clip += sound(466, 170, 100); // シ♭
            break;
        case 20:
            sound_clip += sound(587, 170, 100); // レ
            break;
        case 21:
            sound_clip += sound(698, 170, 100); // ファ
            break;
        case 22:
            sound_clip += sound(932, 510 - 85, 100); // シ♭
            break;
        case 23:
            sound_clip += sound(932, 85, 0); // 休符
            break;
        case 24:
            sound_clip += sound(932, 170, 100); // シ♭
            break;
        case 25:
            sound_clip += sound(932, 85, 0); // 休符
            break;
        case 26:
            sound_clip += sound(932, 85, 100); // シ♭
            break;
        case 27:
            sound_clip += sound(932, 85, 0); // 休符
            break;
        case 28:
            sound_clip += sound(932, 85, 100); // シ♭
            break;
        case 29:
            sound_clip += sound(932, 85, 0); // 休符
            break;
        case 30:
            sound_clip += sound(1047, 2040, 100); // ド
            break;
        default:
            sound_flag = 1;
            break;
        }
    }
}

int soundUnsync(U32 freq, U32 ms, U32 vol)
{
    static int sound_num = 0; // 関数に入った回数
    sound_num++;
    int do_next = 0; // 規定時間鳴らし終わったかどうか

    ecrobot_sound_tone(freq, 4, vol);

    if (sound_num * 4 > ms)
    {
        // 規定時間鳴らし終わった
        sound_num = 0;
        do_next = 1;
    }

    return do_next;
}
