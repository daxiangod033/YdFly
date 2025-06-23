// MIT License

// Copyright (c) 2025 小羊搞飞机ydfly

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
  
  
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
