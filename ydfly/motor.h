/**********************************************************
                   Yd fly
    第一代无人机飞行控制程序
维护日期 ：
          2024年1月25日

*************************************************************/
#include <esp32-hal-ledc.h>

class motor
{
public:
    const int MAX_THROTTLE = 1023;
    const int MIN_THROTTLE = 0;
    int throttle = 0;
    int motorPin;
    int motorChannel;
    int pwmFrequency;

    motor(int pin, int channel)
    {
        setPwmFrequency(10000); // 初始化默认的PWM频率为10000Hz

        motorPin = pin;
        motorChannel = channel;
        ledcAttachPin(motorPin, motorChannel); // 将引脚连接到指定的PWM通道
    }
    ~motor()
    {
    }
    void setPwmFrequency(int frequency)
    {
        pwmFrequency = frequency;
        ledcSetup(motorChannel, pwmFrequency, 10); // 设置PWM频率和分辨率（这里设置为10）
    }

    void writeThrottle(int value)
    {
        // 对油门进行限幅
        if (value > MAX_THROTTLE)
        {
            throttle = MAX_THROTTLE;
        }
        else if (value < MIN_THROTTLE)
        {
            throttle = MIN_THROTTLE;
        }
        else
        {
            throttle = value;
        }
        // 将油门值转换为 0-1023 的范围，并输出 PWM 信号
        ledcWrite(motorChannel, throttle);
    }
};
