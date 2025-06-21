/**********************************************************
                   Yd fly
    第一代无人机飞行控制程序
          作者：杨炟 曾彦康
维护日期 ：
          2024年1月25日

*************************************************************/
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_BMP280.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <I2Cdev.h>
#include "motor.h"
#include "pid.h"
#include <Arduino.h>
struct messages
{
  int data[6] = {6};
};
messages RecData; // 用于接收数据

// 生成硬件对象
MPU6050 mpu;
Adafruit_BMP280 bmp;


//电机布局

//  d        a 
//    \    /  
//     000
//    /   \
//  c       b


motor a(27, 0), b(26, 1), c(14, 2), d(12, 3);

uint8_t fifoBuffer[64]; // dmp数据流
int16_t gyro[3];        // 原始角速度数组
Quaternion q;
VectorFloat gravity; // 向量
float euler[3];
bool interruptTriggered = false;
int i = 0;
int out_a = 0, out_b = 0, out_c = 0, out_d = 0;

att roll(1.0, 0.2, 0.0, 2.0, 0.0, -20.0);
att pitch(1.0, 0.2, 0.0, 2.0, 0.0, -20.0);
att yaw(1.5, 0.2, 0.0, 1.2, 0.0, -20);

int throttle = -1024;

// espnow回调函数
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  memcpy(&RecData, data, sizeof(RecData));
  if (RecData.data[5] == 1) // 数据校验通过
  {
    roll.angle_target = pitch.angle_target = 0.0;
    // 油门
    if (RecData.data[1] > 2595)
    {
      throttle += 10;
    }
    if (RecData.data[1] < 1500)
    {
      throttle -= 10;
    }
    // 左右旋转
    if (RecData.data[0] > 2595)
    {
      yaw.angle_target += (RecData.data[0] - 2595) / 1500.0;
    }
    if (RecData.data[0] < 1500)
    {
      yaw.angle_target += (RecData.data[0] - 1500) / 1500.0;
    }

    // 前后移动
    if (RecData.data[3] > 2595)
    {
      pitch.angle_target = (2595 - RecData.data[3]) / 75.0;
    }
    if (RecData.data[3] < 1500)
    {
      pitch.angle_target = (RecData.data[3] - 1500) / 75.0;
    }
    // 左右移动
    if (RecData.data[2] > 2595)
    {
      roll.angle_target = (RecData.data[2] - 2595) / 75.0;
    }
    if (RecData.data[2] < 1500)
    {
      roll.angle_target = (1500 - RecData.data[2]) / 75.0;
    }
    // 功能键定义
  }
}

// 用于获取开发板的mac地址
void get_mac()
{
  Serial.begin(115200);

  // 初始化WiFi模块
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();

  // 打印MAC地址
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.print("MAC address: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X", mac[i]);
    if (i < 5)
      Serial.print(':');
  }

  // 开启 ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
}

void dmpDataReady()
{
  interruptTriggered = true;
}

void init_drone()
{
  // 初始串口
  Serial.begin(115200);

  Wire.begin();

  // 初始化mpu6050
  Serial.println("正在初始化MPU6050.....");
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(23), dmpDataReady, RISING);
  pinMode(2, OUTPUT);
  WiFi.mode(WIFI_STA);

  // 初始化ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 注册回调函数
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("YDfly无人机飞控 初始化完成.....");
}

void get_mpu()
{
  if (interruptTriggered)
  {
    digitalWrite(2, HIGH);

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);

      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(euler, &q, &gravity);

      yaw.angle_now = euler[0] * RAD_TO_DEG;
      pitch.angle_now = euler[1] * RAD_TO_DEG;
      roll.angle_now = euler[2] * RAD_TO_DEG;

      mpu.dmpGetGyro(gyro, fifoBuffer);

      for (i = 0; i < 3; i++)
      {
        if (gyro[i] > -2 && gyro[i] < 2)
          gyro[i] = 0;
      }
      roll.v_now = gyro[0];
      pitch.v_now = -gyro[1];
      yaw.v_now = -gyro[2];

      Serial.print("Yaw:");
      Serial.print(yaw.angle_now);
      Serial.print("\t");

      Serial.print("Pitch:");
      Serial.print(pitch.angle_now);
      Serial.print("\t");

      Serial.print("Roll:");
      Serial.print(roll.angle_now);
      Serial.print("\t");

      // Serial.print("x:");
      // Serial.print(roll.v_now);
      // Serial.print("\t");

      // Serial.print("y:");
      // Serial.print(pitch.v_now);
      // Serial.print("\t");

      // Serial.print("z:");
      // Serial.print(yaw.v_now);
      // Serial.print("\t");

      Serial.print("t:");
      Serial.print(throttle);
      Serial.print("\t");

      Serial.print("a:");
      Serial.print(out_a);
      Serial.print("\t");

      Serial.print("b:");
      Serial.print(out_b);
      Serial.print("\t");

      Serial.print("c:");
      Serial.print(out_c);
      Serial.print("\t");

      Serial.print("d:");
      Serial.print(out_d);
      Serial.println("");
    }

    interruptTriggered = false;
    digitalWrite(2, LOW);
  }
  pid_pro(roll);
  pid_pro(pitch);
  pid_pro(yaw);
  output_motor();
}

void output_motor()
{

  out_a = throttle + roll.output + pitch.output - yaw.output;
  out_b = throttle - roll.output + pitch.output + yaw.output;
  out_c = throttle - roll.output - pitch.output - yaw.output;
  out_d = throttle + roll.output - pitch.output + yaw.output;

  a.writeThrottle(out_a);
  b.writeThrottle(out_b);
  c.writeThrottle(out_c);
  d.writeThrottle(out_d);
}

void setup()
{
  // get_mac();
  init_drone();
}

void loop()
{
  get_mpu();
}