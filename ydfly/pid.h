/**********************************************************
                   Yd fly
    第一代无人机飞行控制程序
维护日期 ：
          2024年1月25日

*************************************************************/
class att
{
public:
  float v_kp = 0.0;
  float v_ki = 0.0;
  float v_kd = 0.0;
  float v_ki_limit = 33.3;

  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  float ki_limit = 33.3;

  float angle_now = 0.0;
  float angle_target = 0.0;
  float angle_error = 0.0;
  float angle_i =166.0;

  float v_now = 0.0;
  float v_target = 0.0;
  float v_error = 0.0;
  float v_last_error = 0.0;
  float v_i = 0.0;

  int output = 0;
  int output_limint = 1023;
  att(float ap, float ai, float ad, float vp, float vi, float vd);
  ~att();
};


// 串级pid
void pid_pro(att &ola);
