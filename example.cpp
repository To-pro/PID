#include <Arduino.h>
#include <SPI.h>              // SPI 通訊
#include <Wire.h>             // I2C 通訊
#include <binary.h>
#include <LS7366R.h> 
#include <math.h>
#include <filter.h>
#include <sbus.h>
#include "PID.h"

Command_Pre_Filter CPF_position;
Command_Pre_Filter CPF_velocity;
Filter APD_velocity;
Filter APD_acceleration;
#define ENC_LEFT 31
// #define ENC_RIGHT 10
LS7366R enc(ENC_LEFT, ENC_LEFT, MDR0_CONF, MDR1_CONF); // 它可以同時解兩個 Encoder，只需要一個腳位

#define MOTOR_PWM_CW 14     // 馬達正轉的PWM腳位
#define MOTOR_PWM_CCW 24    // 馬達負轉的PWM腳位
#define MOTOR_PWM_FREQ 2150 // 馬達PWM命令的更新頻率 (Frequency)
#define PWM_WRITE_RES 14    // 馬達PWM寫入的解析度 (Resolution)

#define MOTOR_R_CW 25
#define MOTOR_R_CCW 15


int system_state = 0;
int system_state_pre = 0;
int system_state_change = 1;
enum{
    state_error,
    state_control_position,
    state_control_velocity,
};

// Variable Declaration
long int pos_pulse = 0;
long int pos_pre_pulse = 0;
double diff_vel_pulse = 0;
double diff_vel_d_pulse = 0;

double pos = 0;
double pos_pre = 0;
double diff_vel = 0;
double diff_vel_pre = 0;
double diff_acc = 0;
double apd_vel = 0;
double apd_acc = 0;

double pos_d = 0.0;
double dpos_d = 0.0;
double ddpos_d = 0.0;

double vel_d = 0.0;
double dvel_d = 0.0;
double ddvel_d = 0.0;

double e_p1 = 0.0;
double e_p2 = 0.0;

double e_v1 = 0.0;
double e_v2 = 0.0;
int Motor_PWM_Cmd = 0;
double *CPF_temp;

PID position_control;
PID velocity_control;
double u = 0;
#define Vmax 12.0
#define Motor_PWM_Max (int)(pow(2.0, (double)(PWM_WRITE_RES)) - 1.0)

#define interval_t 1000
IntervalTimer ISR_Timer;
double t = 0;
long int t_idx = 0;
double dt = (double)(interval_t) / 1000000.0;

#define rad2deg 180.0 / PI
#define rad2pulse 30000.0 / PI
#define pulse2deg 180.0 / 30000.0
#define pulse2rad PI / 30000.0

int command_flag=0;
enum{
    rc_command,
    step_command,
    square_command, //方波
    sin_command     //sin波
};

// double alpha = 8.9442;
// double beta = 37.1264;

// double alpha = 7.0065*rad2pulse;
// double beta = 33.0030*rad2pulse;

double t_p = 10.0; //方波週期(s)
double position_at_t_p = PI;

//RC sbus setting
IEC::SBUS RC(&Serial1);
uint16_t RC_Channel[16];
#define RC_PWM_LOW 200
#define RC_PWM_HIGH 1800

#define pos_c_max PI
#define vel_c_max 6*PI
double pos_c = 0.0;
double vel_c = 0.0;

// 函數宣告
void ISR_Routine();
void Motor_Reset();
void Motor_set();
void RC_updata();
void Variable_Reset(){
    t = 0.0;
    t_idx = 0;

    pos = 0;
    pos_pre = 0;
    diff_vel = 0;
    diff_vel_pre = 0;
    diff_acc = 0;
    apd_vel = 0;
    apd_acc = 0;
    vel_d = 0;

    pos_d = 0.0;
    dpos_d = 0.0;
    ddpos_d = 0.0;

    vel_d = 0.0;
    dvel_d = 0.0;
    ddvel_d = 0.0;

    e_p1 = 0.0;
    e_p2 = 0.0;

    e_v1 = 0.0;
    e_v2 = 0.0;
    Motor_PWM_Cmd = 0;
    CPF_temp = NULL;
    position_control.Reset();
    velocity_control.Reset();
    APD_velocity.Reset();
    APD_acceleration.Reset();
    u = 0.0;
}

double sign(double u_in)
{
    if (u_in > 0.0)
        return 1.0;
    else if (u_in < 0.0)
        return -1;
    return 0.0;
}

void setup()
{
    
    // 定義腳位的功能
    pinMode(MOTOR_PWM_CW, OUTPUT);                       // 設定 MOTOR_PWM_CW 腳位為輸出
    pinMode(MOTOR_PWM_CCW, OUTPUT);                      // 設定 MOTOR_PWM_CCW 腳位為輸出
    analogWriteFrequency(MOTOR_PWM_CW, MOTOR_PWM_FREQ);  // 設定 PWM 的輸出頻率
    analogWriteFrequency(MOTOR_PWM_CCW, MOTOR_PWM_FREQ); // 設定 PWM 的輸出頻率
    analogWriteResolution(PWM_WRITE_RES);                // 設定所有 PWM 的解析度

    pinMode(MOTOR_R_CW, OUTPUT);                       // 設定 MOTOR_PWM_CW 腳位為輸出
    pinMode(MOTOR_R_CCW, OUTPUT);                      // 設定 MOTOR_PWM_CCW 腳位為輸出
    analogWrite(MOTOR_R_CW, 0);
    analogWrite(MOTOR_R_CCW, 0);

    // 將馬達正轉跟負轉的 PWM 腳位設定成0，避免暴衝。
    Motor_Reset();

    Serial.begin(115200); // 開啟序列埠傳輸，設定 baud rate 為 115200
    RC.begin();

    // 清空 QEI 的 Encoder 內存值
    enc.reset();

    // command pre-filter set
    APD_velocity.Init(APPROXIMATED_DIFFERENTIATOR,BACKWARD_METHOD,10);
    APD_acceleration.Init(APPROXIMATED_DIFFERENTIATOR,BACKWARD_METHOD,10);

    int n = 5;
    CPF_position.Init(5.0,n);
    CPF_velocity.Init(5.0,n);

    // controller set
    // position_control.Init_pole(10,5,12);
    // velocity_control.Init_gain(-15.0,0.0,12);

    // position_control.intergal_anti_windup_flag_set(no_anti_windup);
    position_control.Init_pole(9.0,4.5,1.2,12);
    velocity_control.Init_gain(15.0,12.5,0.0,12);

    delay(1000);
    ISR_Timer.begin(ISR_Routine, interval_t);
    ISR_Timer.priority(255);
}

void loop()
{
    RC_updata();
    // 序列埠傳輸
    // Serial.flush();
    // Serial.print(pos*rad2deg);
    // Serial.print(" ");
    // Serial.print(apd_vel*rad2deg);
    // Serial.print(" ");
    // Serial.print(diff_vel*rad2deg);
    // Serial.print("\n");
    // delay(10);
}

void ISR_Routine()
{
    t_idx++;
    t = t_idx * dt;

    // 讀入馬達的 pulse 數
    enc.sync();
    pos_pulse = enc.right();
    pos = (double)pos_pulse/30000.0*PI;

    // 速度估測
    APD_velocity.Update_Filter(pos_pulse);
    apd_vel = APD_velocity.Get_Filtered_Data()/30000.0*PI;;
    diff_vel_pulse = (double)(pos_pulse - pos_pre_pulse) / dt;
    pos_pre_pulse = pos_pulse;
    diff_vel = diff_vel_pulse/30000.0*PI;

    //加速度估計(非常炸裂)
    APD_acceleration.Update_Filter(apd_vel);
    apd_acc = APD_acceleration.Get_Filtered_Data();
    diff_acc = (double)(diff_vel - diff_vel_pre) / dt;
    diff_vel_pre = diff_vel;

    //命令更新與控制律計算
    switch (system_state)
    {
    case state_control_position:
        CPF_position.Update_Filter(pos_c);
        CPF_temp = CPF_position.Get_command();
        pos_d = CPF_temp[0];
        dpos_d = CPF_temp[1];
        ddpos_d = CPF_temp[2];

        e_p1 = pos - pos_d;
        e_p2 = apd_vel - dpos_d;
        position_control.Updata(e_p1,e_p2,1.0,0.0);
        u = position_control.Output();
        break;
    case state_control_velocity:
        CPF_velocity.Update_Filter(vel_c);
        CPF_temp = CPF_velocity.Get_command();
        vel_d = CPF_temp[0];
        dvel_d = CPF_temp[1];
        ddvel_d = CPF_temp[2];
        e_p2 = apd_vel - vel_d;
        velocity_control.Updata(e_p2,0.0,1.0,0.0);
        u = velocity_control.Output();
        break;
    default ://state_error
        vel_c = 0;
        pos_c = 0;
        CPF_position.Reset();
        CPF_velocity.Reset();
        Variable_Reset();
        break;
    }
    Motor_set();


    // 序列埠傳輸
    Serial.flush();
    // Serial.print(pos*rad2deg);
    // Serial.print(" ");
    // Serial.print(apd_vel*rad2deg);
    // Serial.print(" ");
    // Serial.print(diff_vel*rad2deg);
    // Serial.print(" ");
    // Serial.print(apd_acc*rad2deg);
    // Serial.print(" ");
    // Serial.print(system_state);
    // Serial.print(" ");

    Serial.print(pos*rad2deg);
    Serial.print(" ");
    Serial.print(pos_d*rad2deg);
    Serial.print(" ");
    Serial.print(pos_pulse - (int)(pos_d*rad2pulse));
    Serial.print(" ");
    Serial.print(apd_vel*rad2deg);
    Serial.print(" ");
    Serial.print(diff_vel*rad2deg);
    Serial.print(" ");
    Serial.print(vel_d*rad2deg);
    Serial.print(" ");
    Serial.print((apd_vel-vel_d)*rad2deg);
    Serial.print(" ");
    Serial.print(u);
    Serial.print(" ");
    Serial.print(Motor_PWM_Cmd);

    // Serial.print(" ");
    // Serial.print(pos_d);
    // Serial.print(" ");
    // Serial.print(dpos_d);
    // Serial.print(" ");
    // Serial.print(ddpos_d);
    // Serial.print(" ");
    // Serial.print(vel_c);
    // Serial.print(" ");
    // Serial.print(vel_d);
    // Serial.print(" ");
    // Serial.print(dvel_d);
    // Serial.print(" ");
    // Serial.print(ddvel_d);
    // Serial.print(" ");
    // Serial.print(diff_acc*rad2deg);

    // for (int i = 0; i < 16; i++)
    // { 
    //     Serial.print(RC_Channel[i]);
    //     Serial.print(" ");
    // }
    Serial.print("\n");
}


void Motor_Reset()
{
    Motor_PWM_Cmd = 0;
    analogWrite(MOTOR_PWM_CW, 0);
    analogWrite(MOTOR_PWM_CCW, 0);
}

void Motor_set()
{
    Motor_PWM_Cmd = map((double)sign(u)*u, 0.0, Vmax , 0.0, (double)Motor_PWM_Max);
    switch (int(sign(u)))
    {
    case 1:
        analogWrite(MOTOR_PWM_CW, Motor_PWM_Cmd);
        analogWrite(MOTOR_PWM_CCW, 0);
        break;
    case -1:
        analogWrite(MOTOR_PWM_CW, 0);
        analogWrite(MOTOR_PWM_CCW, Motor_PWM_Cmd);
        break;
    default:
        analogWrite(MOTOR_PWM_CW, 0);
        analogWrite(MOTOR_PWM_CCW, 0);
        break;
    }
}

void RC_updata(){
    RC.read(RC_Channel);
    if(RC_Channel[6]>1500){
        system_state = state_control_position;
    }else if((RC_Channel[6]>500)){
        system_state = state_control_velocity;
    }else{
        system_state = state_error;
    }

    if(RC_Channel[4]>1500){
        command_flag = rc_command;
    }else if((RC_Channel[4]>500)){
        command_flag = step_command;
    }else{
        command_flag = square_command;
    }

    if(RC_Channel[5]>1500){
        system_state = state_error;
    }

    switch (system_state)
    {
    case state_control_position:
        switch (command_flag)
        {
        case rc_command:
            pos_c = map((double)(RC_Channel[1]), RC_PWM_HIGH, RC_PWM_LOW, -pos_c_max, pos_c_max);
            break;
        case step_command:
            pos_c = position_at_t_p;
            break;
        case square_command:
            if(fmod(t,t_p) <= (t_p/2.0)){
                pos_c = position_at_t_p;
            }else{
                pos_c = 0;
            }
            break;
        }
        vel_c = 0.0;
        break;
    case state_control_velocity:
        switch (command_flag)
        {
        case rc_command:
            vel_c = map((double)(RC_Channel[1]), RC_PWM_HIGH, RC_PWM_LOW, -vel_c_max, vel_c_max);
            break;
        case step_command:
            vel_c = vel_c_max;
            break;
        case square_command:
            if(fmod(t,t_p) <= (t_p/2.0)){
                vel_c = vel_c_max;
            }else{
                vel_c = -vel_c_max;
            }
            break;
        }
        pos_c = 0.0;
        break;
    default ://state_error
        vel_c = 0;
        pos_c = 0;
        break;
    }
    if(system_state != system_state_pre){
        system_state_change = 1;
        vel_c = 0;
        pos_c = 0;
        CPF_position.Reset();
        CPF_velocity.Reset();
        Variable_Reset();
        enc.reset();
    }
    system_state_pre = system_state;
}