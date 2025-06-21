//MIT License

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

/*******************************************************************/
//                 ydfly无人机遥控器                                 /
//                                                                  /
//  通信协议：DGFT、esp-now                                          /
//   update:  2023.10.23                                            /
//            2023.11.6                                             /
//                                                                  /
/********************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "UI.h"

UI ui;
// 回调函数,函数将在发送消息时执行。此函数告诉我们信息是否成功发送;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\n数据发送状态:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "成功" : "失败");
}

class Clicker
{
  /*遥控器大类*/

private:
  // 摇杆引脚定义
  const int X_AXIS_PIN = 32;  //左摇杆X轴
  const int Y_AXIS_PIN = 36;  //左摇杆y轴
  const int X_AXIS_PIN2 = 34; //右摇杆X轴
  const int Y_AXIS_PIN2 = 35; //右摇杆y轴
  // 按钮引脚定义
  const int BUTTON1_PIN = 23; //急停按钮
  const int BUTTON2_PIN = 4;  //解锁按钮

  //蜂鸣器相关变量
  const int buzzerPin = 12; // 定义蜂鸣器引脚
  const int buzzer = 0;     // PWM通道
  const int resolution = 8; // PWM分辨率

  //状态变量

  int left_x_value;  // 左摇杆x数值
  int left_y_value;  // 左摇杆y数值
  int right_x_value; // 右摇杆x数值
  int right_y_value; // 右摇杆y数值
  // 按钮状态
  int stop_button_state;
  int unlock_button_state;
  int action_key;

public:
uint8_t broadcastAddress[6] = {0xE4, 0x65, 0xB8, 0x0C, 0x03, 0xF4}; //有刷Mac地址：E4:65:B8:0C:03:F4



  // 用于发送数据的结构体
  struct message
  {
    int data[6];
  };
  message myData; // 创建一个结构体变量

  Clicker();          //构造函数
  ~Clicker();         //析构函数
  void Debug_OS();    //测试系统
  void start_music(); //开机音乐
  void oper_audio();  //操作音效
  void get_operate(); //获取操作
  void draw_oper();//绘制摇杆位置
  void boot();        //引导
  void fly();          //系统
};

Clicker::Clicker(/* args */)
{
  Serial.begin(115200);
  Serial.println("遥控器启动");
}

Clicker::~Clicker()
{
}
void Clicker::get_operate()
{
  // 读取摇杆数值
  left_x_value = analogRead(X_AXIS_PIN);
  left_y_value = analogRead(Y_AXIS_PIN);
  right_x_value = analogRead(X_AXIS_PIN2);
  right_y_value = analogRead(Y_AXIS_PIN2);

  // Serial.print(x_axis_value);
  // 读取按钮状态
  stop_button_state = digitalRead(BUTTON1_PIN);
  unlock_button_state = digitalRead(BUTTON2_PIN);
}
void Clicker::draw_oper(){
  //u8g2.drawCircle(圆心x, 圆心y, 半径rad, U8G2_DRAW_ALL);
  u8g2.drawCircle(28+left_x_value/136, 61-left_y_value/136,3);
  u8g2.drawCircle(71+right_x_value/136, 61-right_y_value/136,3);
  u8g2.sendBuffer();//
}//绘制摇杆位置



void Clicker::Debug_OS()
{

  /*打印各个值*/
  Serial.print(left_x_value);
  Serial.print("|");
  Serial.print(left_y_value);
  Serial.print("|");
  Serial.print(right_x_value);
  Serial.print("|");
  Serial.print(right_y_value);
  Serial.print("|");
  Serial.print(stop_button_state);
  Serial.print("|");
  Serial.println(unlock_button_state);
}
void Clicker::boot()
{
  /*引导程序*/
  ui.start();
  delay(100);
  WiFi.mode(WIFI_STA); // 设置WIFI模式为STA模式，即无线终端
  //  初始化ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW 协议初始化失败");
    return;
  }
  else
  {
    Serial.println("ESP-NOW 协议初始化成功！");
  }

  esp_now_register_send_cb(OnDataSent); //注册回调函数

  esp_now_peer_info_t peerInfo; // 注册通信频道

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);

  peerInfo.channel = 0; //通道

  peerInfo.encrypt = false; //是否加密为False

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("通讯频道注册失败");
    return;
  }
  else
  {
    Serial.println("通讯频道注册成功!");
  }
  ui.syslogo();
  start_music();
  delay(2000);
  ui.wait_unlock();

  while (true)
  {
    get_operate();
    if (left_x_value > 1900 && left_y_value < 1600 && right_x_value < 1600 && right_y_value < 1600)
    { //内八起桨

      //此处依照DGFT协议编写
      myData.data[0] = left_x_value;
      myData.data[1] = left_y_value;
      myData.data[2] = right_x_value;
      myData.data[3] = right_y_value;
      myData.data[4] = 2;//解锁
      myData.data[5] = 1;

      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData)); //发送
      oper_audio();
      delay(30);
      if (result == ESP_OK)
      {
        ui.unlocking();
        delay(5000);//延迟5s等待电机解锁
        break;
      };
    }
  }

}

void Clicker::fly()
{
  /*遥控器系统*/

  while (true)
  {

    get_operate();
    ui.fly_ui();
    // draw_oper();
    Debug_OS();

    action_key = 0; //操作按钮归零

    if (left_x_value > 1900)
    {
      Serial.println("右转");
    }
    if (left_x_value < 1600)
    {
      Serial.println("左转");
    }
    if (left_y_value > 1900)
    {
      Serial.println("上升");
    }
    if (left_y_value < 1600)
    {
      Serial.println("下降");
    }
    if (right_x_value > 1900)
    {
      Serial.println("右移");
    }
    if (right_x_value < 1600)
    {
      Serial.println("左移");
    }
    if (right_y_value > 1900)
    {
      Serial.println("前进");
    }
    if (right_y_value < 1600)
    {
      Serial.println("后退");
    }
    if (unlock_button_state == 0)
    {
      //急停在后赋值，以免紧张乱按导致炸鸡
      Serial.println("qd");
      action_key = 3;
      oper_audio();
    }
    if (stop_button_state == 0)
    {
      //急停在后赋值，以免紧张乱按导致炸鸡
      Serial.println("急停");
      action_key = 1;
      oper_audio();
    }
    //此处依照DGFT协议编写
    myData.data[0] = left_x_value;
    myData.data[1] = left_y_value;
    myData.data[2] = right_x_value;
    myData.data[3] = right_y_value;
    myData.data[4] = action_key;
    myData.data[5] = 1;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData)); //发送
    (result == ESP_OK) ? Serial.println("指令已发送") : Serial.println("发送失败");        //打印发送状态

    delay(30);
  }
}

void Clicker::start_music()
{
  // 初始化蜂鸣器
  ledcSetup(buzzer, 2000, resolution); // 设置PWM通道
  ledcAttachPin(buzzerPin, buzzer);    // 将引脚连接到PWM通道

  ledcWriteTone(buzzer, 700);
  delay(100);
  ledcWriteTone(buzzer, 900);
  delay(100);
  ledcWriteTone(buzzer, 1100);
  delay(100);
  ledcWriteTone(buzzer, 1300);
  delay(100);
  ledcWriteTone(buzzer, 700);
  delay(100);
  ledcWriteTone(buzzer, 900);
  delay(100);
  ledcWriteTone(buzzer, 1100);
  delay(100);
  ledcWriteTone(buzzer, 1300);
  delay(100);
  ledcWriteTone(buzzer, 0);
}

void Clicker::oper_audio()
{
  ledcWriteTone(buzzer, 1000);
  delay(50);
  ledcWriteTone(buzzer, 0);
  delay(50);
  ledcWriteTone(buzzer, 1000);
  delay(50);
  ledcWriteTone(buzzer, 0);
}

Clicker clicker;
void setup()
{
  clicker.boot(); //引导
}

void loop()
{
  clicker.fly(); 
}
