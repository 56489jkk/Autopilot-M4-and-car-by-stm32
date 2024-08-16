#include "uav_control.h"
float Uav_model_pid_control_roll( uint16_t rc_roll,float kp ,float ki ,float t )//roll control
{
    static float u=0;
    //roll
    static float xa_r= 0,xb_r= 0,xc_r= 0;
    static float ya_r= 0,yb_r= 0,yc_r= 0;
    /**遥控器输入计算**/
    float rc_in  = (rc_roll-1500)/20.0f;  //+-25°

    /**roll方向参数计算**/
    float ra= (10*kp)/(t*t+10*kp*ki*t+10*kp);
    float rb= (20*kp)/(t*t+10*kp*ki*t+10*kp);
    float rc= (10*kp)/(t*t+10*kp*ki*t+10*kp);
    float rd= (2*t*t-10*kp)/(t*t+10*kp*ki*t+10*kp);
    float re= (-t*t+10*kp*ki*t-10*kp)/(t*t+10*kp*ki*t+10*kp);

    /**roll方向输入输出关系**/
    xa_r = rc_in;
    ya_r=ra*xa_r+rb*xb_r+rc*xc_r+rd*yb_r+re*yc_r;
    xc_r = xb_r;
    xb_r = xa_r;
    yc_r = yb_r;
    yb_r = ya_r;

    /**系统输出**/
    u = ya_r*0.75; //roll out

    return u;
}

float Uav_model_pid_control_pitch( uint16_t rc_pitch,float kp ,float ki ,float t )//pitch control
{
    static float u=0;
    //pitch
    static float xa_r= 0,xb_r= 0,xc_r= 0;
    static float ya_r= 0,yb_r= 0,yc_r= 0;
    /**遥控器输入计算**/
    float rc_in  = (rc_pitch-1500)/20.0f;  //+-25°

    /**pitch方向参数计算**/
    float ra= (10*kp)/(t*t+10*kp*ki*t+10*kp);
    float rb= (20*kp)/(t*t+10*kp*ki*t+10*kp);
    float rc= (10*kp)/(t*t+10*kp*ki*t+10*kp);
    float rd= (2*t*t-10*kp)/(t*t+10*kp*ki*t+10*kp);
    float re= (-t*t+10*kp*ki*t-10*kp)/(t*t+10*kp*ki*t+10*kp);

    /**pitch方向输入输出关系**/
    xa_r = rc_in;
    ya_r=ra*xa_r+rb*xb_r+rc*xc_r+rd*yb_r+re*yc_r;
    xc_r = xb_r;
    xb_r = xa_r;
    yc_r = yb_r;
    yb_r = ya_r;

    /**系统输出**/
    u = ya_r*0.75; //pitch out

    return u;
}

float Uav_model_pid_control_thr( uint16_t rc_thr,float kp ,float ki ,float t ) //thr control
{
    static float u=0;
    //thr
    static float xa_r= 0,xb_r= 0,xc_r= 0;
    static float ya_r= 0,yb_r= 0,yc_r= 0;
    /**遥控器输入计算**/
    float rc_in  = (rc_thr-1500)/2.0f;  //+-250 cm/s

    /**thr方向参数计算**/
    float ra= (10*kp)/(t*t+10*kp*ki*t+10*kp);
    float rb= (20*kp)/(t*t+10*kp*ki*t+10*kp);
    float rc= (10*kp)/(t*t+10*kp*ki*t+10*kp);
    float rd= (2*t*t-10*kp)/(t*t+10*kp*ki*t+10*kp);
    float re= (-t*t+10*kp*ki*t-10*kp)/(t*t+10*kp*ki*t+10*kp);

    /**thr方向输入输出关系**/
    xa_r = rc_in;
    ya_r=ra*xa_r+rb*xb_r+rc*xc_r+rd*yb_r+re*yc_r;
    xc_r = xb_r;
    xb_r = xa_r;
    yc_r = yb_r;
    yb_r = ya_r;

    /**系统输出**/
    u = ya_r*0.75; //thr out

    return u;
}

float Uav_model_pid_control_yaw( uint16_t rc_yaw,float kp ,float ki ,float t )//yaw control
{
    static float u=0;
    //yaw
    static float xa_r= 0,xb_r= 0,xc_r= 0;
    static float ya_r= 0,yb_r= 0,yc_r= 0;
    /**遥控器输入计算**/
    float rc_in  = (rc_yaw-1500)/2.0f;  //+-250°/s

    /**yaw方向参数计算**/
    float ra= (10*kp)/(t*t+10*kp*ki*t+10*kp);
    float rb= (20*kp)/(t*t+10*kp*ki*t+10*kp);
    float rc= (10*kp)/(t*t+10*kp*ki*t+10*kp);
    float rd= (2*t*t-10*kp)/(t*t+10*kp*ki*t+10*kp);
    float re= (-t*t+10*kp*ki*t-10*kp)/(t*t+10*kp*ki*t+10*kp);

    /**yaw方向输入输出关系**/
    xa_r = rc_in;
    ya_r=ra*xa_r+rb*xb_r+rc*xc_r+rd*yb_r+re*yc_r;
    xc_r = xb_r;
    xb_r = xa_r;
    yc_r = yb_r;
    yb_r = ya_r;

    /**系统输出**/
    u = ya_r*0.75; //yaw out

    return u;
}
