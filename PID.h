/**
 * u = alpha ( beta - kp * e - ki * int_0^t(e) - kd * de)
 * assume e = x - x_d;
 * |u| < 0.99 u_max // double 會有精度問題導致沒有飽和到 12.xxxxx < 12.xxxxxx
 * 
 */

#ifndef PID_H
#define PID_H

#include <math.h>

#ifndef interval_t
#define interval_t 1000 //間隔時間(us)
#endif

// #define intergal_dead_zone_flag 0 //(intergal_dead_zone 需要設定)1:
// #define intergal_dead_zone = 0.001 //

enum{
    no_anti_windup,
    separation,//積分分離(intergal_windup_epsilon 需要設定)
    clamping,//積分遇限削弱法
    back_calculation//反饋抑制抗飽和(Kb需要設定)
};

#ifndef intergal_anti_windup_epsilon
#define intergal_anti_windup_epsilon 0.1
#endif

#ifndef intergal_anti_windup_Kb
#define intergal_anti_windup_Kb 1
#endif


#define intergal_dead_zone_flag 1
#ifdef intergal_dead_zone_flag
    #ifndef intergal_dead_zone
        #define intergal_dead_zone PI / 30000.0 //最小為輸出解析度
    #endif
#endif

#define saturation_flag_outside_check 

class PID
{
public:
    void intergal_anti_windup_flag_set(int flag_set){
        intergal_anti_windup_flag = flag_set;//default:2(clamping)
    };
    void error_define_set(int error_define_set){
        error_define = error_define_set;//default:1(x-x_d)
    };
    void Init_pole(double lambda1,double lambda2 ,double lambda3 ,double u_max_set){
        if(error_define==1){
            ki = -(lambda1 * lambda2 * lambda3);
            kp = -(lambda1 * lambda2 + lambda2 * lambda3 + lambda3 * lambda1);
            kd = -(lambda1 + lambda2 + lambda3);
        }else{
            ki = (lambda1 * lambda2 * lambda3);
            kp = (lambda1 * lambda2 + lambda2 * lambda3 + lambda3 * lambda1);
            kd = (lambda1 + lambda2 + lambda3);
        } 
        u_max = u_max_set;
    };
    void Init_pole(double lambda1,double lambda2 ,double u_max_set){
        ki = 0.0;
        if(error_define==1){
            kp = -(lambda1 * lambda2);
            kd = -(lambda1 + lambda2);
        }else{
            kp = (lambda1 * lambda2);
            kd = (lambda1 + lambda2);
        }
        u_max = u_max_set;
    };

    void Init_gain(double kp_set,double ki_set ,double kd_set ,double u_max_set){
        if(error_define==1){
            kp = -kp_set;
            ki = -ki_set;
            kd = -kd_set;
        }else{
            kp = kp_set;
            ki = ki_set;
            kd = kd_set;
        }
        
        u_max = u_max_set;
    };

    void Init_gain(double kp_set ,double kd_set ,double u_max_set){
        ki = 0.0;
        if(error_define==1){
            kp = -kp_set;
            kd = -kd_set;
        }else{
            kp = kp_set;
            kd = kd_set;
        }
        u_max = u_max_set;
    };

    void Reset(){
        u_i = 0.0;
        u_d = 0.0;
        u = 0.0;
        e = 0.0;
        de = 0.0;
        e_pre = 0.0;
        integral_e = 0.0;
        saturation_flag = 0;
    };


    void Updata(double e_set,double alpha,double beta){
        e = e_set;
        de = (e-e_pre)/T;
        if(intergal_flag == 1){
            #ifdef intergal_dead_zone_flag
            if(fabs(e)>intergal_dead_zone){
                intergal_anti_windup();
            }
            #else
                intergal_anti_windup();
            #endif
        }
        u_d = alpha*(kp*e + u_i + kd*de+beta);
        if(fabs(u_d)>=u_max*0.99){
            u = sign(u_d)*u_max;
        }else{
            u=u_d;
        }
        e_pre=e;
    };

    void Updata(double e_set){//asume alpha = 1;beta = 0;
        e = e_set;
        de = (e-e_pre)/T;
        if(intergal_flag == 1){
            #ifdef intergal_dead_zone_flag
            if(fabs(e)>intergal_dead_zone){
                intergal_anti_windup();
            }
            #else
                intergal_anti_windup();
            #endif
        }
        u_d = kp*e + u_i + kd*de;
        if(fabs(u_d)>=u_max*0.99){
            u = sign(u_d)*u_max;
        }else{
            u=u_d;
        }
        e_pre=e;
    };

    void Updata(double e_set ,double de_set ,double alpha,double beta){// u = alpha*(kp*e + u_i + kd*de+beta)
        e = e_set;
        de = de_set;
        if(intergal_flag == 1){
            #ifdef intergal_dead_zone_flag
            if(fabs(e)>intergal_dead_zone){
                intergal_anti_windup();
            }
            #else
                intergal_anti_windup();
            #endif
        }
        u_d = alpha*(kp*e + u_i + kd*de+beta);
        if(fabs(u_d)>=u_max*0.99){
            u = sign(u_d)*u_max;
        }else{
            u=u_d;
        }
        e_pre=e;
    };

    void Updata(double e_set,double de_set){//asume alpha = 1;beta = 0;
        e = e_set;
        de = de_set;
        if(intergal_flag == 1){
            #ifdef intergal_dead_zone_flag
            if(fabs(e)>intergal_dead_zone){
                intergal_anti_windup();
            }
            #else
                intergal_anti_windup();
            #endif
        }
        u_d = kp*e + u_i + kd*de;
        if(fabs(u_d)>=u_max*0.99){
            u = sign(u_d)*u_max;
        }else{
            u=u_d;
        }
        e_pre=e;
    };

    
    double Output(){
        return u;
    };
    void Output(double *ptr_u){
        *ptr_u=u;
    };

    double Output_u_i(){
        return u_i;
    };
    double Output_u_d(){
        return u_d;
    };

    int saturation_flag = 0;
    int intergal_flag = 1;//1才會積分，不然保持積分值
protected:
    int error_define = 1;
    /**
     * 1:x-x_d
     * 0:x_d-x 
     */
    int intergal_anti_windup_flag = clamping; 
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;

    double u_i = 0.0;
    double u_d = 0.0;
    double u = 0.0;
    double e = 0.0;
    double de = 0.0;
    double e_pre = 0.0;
    double integral_e = 0.0;
    double u_max = 1.0;
    double T = interval_t / 1000000.0;
    
    void intergal_anti_windup(){
        // 3種抗積分飽和手段
    if(intergal_anti_windup_flag == separation){
        if (fabs(e) <= intergal_anti_windup_epsilon)
        {
            integral_e = integral_e + e * T;
        }
        else
        {
            integral_e = 0.0;
        }
    }else if (intergal_anti_windup_flag == clamping){
        if (((fabs(u) >= u_max * 0.99) && (u * e <= 0.0))||(saturation_flag == 1)){
            integral_e = integral_e;
        }  
        else{
            integral_e = integral_e + e * T;
        }
    }else if (intergal_anti_windup_flag == back_calculation){
        integral_e = integral_e + (e - intergal_anti_windup_Kb * ki / kp * (u_d - u)) * T;
    }else{
        //(intergal_anti_windup_flag ==0)
        integral_e = integral_e + e * T;
    }
    u_i = ki * integral_e;
    }

    double sign(double u_in)
    {
        if (u_in > 0.0)
            return 1.0;
        else if (u_in < 0.0)
            return -1;
        return 0.0;
    }
};

#endif