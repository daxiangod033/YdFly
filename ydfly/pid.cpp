/**********************************************************
                   Yd fly
    第一代无人机飞行控制程序
维护日期 ：
          2024年1月25日

*************************************************************/
#include "pid.h"
att::att(float ap, float ai, float ad, float vp, float vi, float vd)
{
  kp = ap;
  ki = ai;
  kd = ad;
  v_kp = vp;
  v_ki = vi;
  v_kd = vd;
}

att::~att()
{
}


void pid_pro(att &ola)
{
  // 外环
  ola.angle_error = ola.angle_target - ola.angle_now;
  ola.angle_i += ola.angle_error;
  // 限幅
  if (ola.angle_i > ola.ki_limit)
    ola.angle_i = ola.ki_limit;
  if (ola.angle_i < -ola.ki_limit)
    ola.angle_i = -ola.ki_limit;

  ola.v_target = ola.kp * ola.angle_error + ola.ki * ola.angle_i + ola.kd * ola.v_now;
  // 内环
  ola.v_error = ola.v_target - ola.v_now;
  ola.v_i += ola.v_error;

  if (ola.v_i > ola.v_ki_limit)
    ola.v_i = ola.v_ki_limit;
  if (ola.v_i < -ola.v_ki_limit)
    ola.v_i = -ola.v_ki_limit;
  ola.output = ola.v_kp * ola.v_error + ola.v_ki * ola.v_i + ola.v_kd * (ola.v_error - ola.v_last_error);
  ola.v_last_error = ola.v_error;
  if (ola.output > ola.output_limint)
    ola.output = ola.output_limint;
  if (ola.output < -ola.output_limint)
    ola.output = -ola.output_limint;

}
