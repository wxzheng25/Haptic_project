#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// 初始化两个 IMU，指定不同的 I2C 地址
Adafruit_BNO055 imu1 = Adafruit_BNO055(55, 0x28); // IMU1 地址 0x28
Adafruit_BNO055 imu2 = Adafruit_BNO055(56, 0x29); // IMU2 地址 0x29

const int wPin = 3;
const int sPin = 4;
const int aPin = 5;
const int dPin = 6;

const int upPin = 7;
const int downPin = 8;
const int lPin = 9;
const int rPin = 10;

unsigned long previousMillis = 0; // 上次切换时间
const unsigned long interval = 500; // 振子切换间隔（毫秒）
int currentIndex = 0; // 当前振动器的索引
bool vibrationActive = false; // 是否振动标志

// 定义状态机的状态
enum State { IDLE, CLOCKWISE, COUNTERCLOCKWISE };
State currentState = IDLE; // 初始化为 IDLE 状态

void setup(void)
{
  Serial.begin(115200); // 设置串口波特率为 115200
  while (!Serial);
  Serial.println("双 IMU 姿态传感器初始化中...");

  // 初始化 IMU1
  if (!imu1.begin())
  {
    Serial.println("无法初始化 IMU1! 检查连接和 I2C 地址.");
    while (1);
  }

  // 初始化 IMU2
  if (!imu2.begin())
  {
    Serial.println("无法初始化 IMU2! 检查连接和 I2C 地址.");
    while (1);
  }

  delay(1000);

  // 使用外部晶振提高精度
  imu1.setExtCrystalUse(true);
  imu2.setExtCrystalUse(true);

  Serial.println("开始输出姿态数据...");
  pinMode(wPin, OUTPUT);
  pinMode(sPin, OUTPUT);
  pinMode(aPin, OUTPUT);
  pinMode(dPin, OUTPUT);

  pinMode(upPin, OUTPUT);
  pinMode(downPin, OUTPUT);
  pinMode(lPin, OUTPUT);
  pinMode(rPin, OUTPUT);
}

void loop(void)
{
  // 获取 IMU1 和 IMU2 的传感器数据
  sensors_event_t event1;
  imu1.getEvent(&event1);
  sensors_event_t event2;
  imu2.getEvent(&event2);

  // 提取并打印欧拉角
  float yaw1 = event1.orientation.x;
  float pitch1 = event1.orientation.y;
  float roll1 = event1.orientation.z;
  float yaw2 = event2.orientation.x;
  float pitch2 = event2.orientation.y;
  float roll2 = event2.orientation.z;

  Serial.print(yaw1, 4);
  Serial.print(",");
  Serial.print(pitch1, 4);
  Serial.print(",");
  Serial.print(roll1, 4);
  Serial.print(",");
  Serial.print(yaw2, 4);
  Serial.print(",");
  Serial.print(pitch2, 4);
  Serial.print(",");
  Serial.println(roll2, 4);

  if (Serial.available() > 0)
  {
    char command = Serial.read(); // 读取一个字符作为命令
    switch (command)
    {
      case 'w':
      case 's':
      case 'a':
      case 'd':
      case 'u':
      case 'x':
      case 'l':
      case 'r':
      case 'n':
        deactivateAllVibrators(); // 执行其他命令时停止所有振动器
        vibrationActive = false;
        currentState = IDLE; // 切换到空闲状态
        currentIndex = 0; // 重置索引
        executeSingleVibration(command); // 根据命令激活单个振动器
        break;

      case 'c': // 顺时针轮流振动
        if (currentState != CLOCKWISE) // 仅当不在顺时针状态时切换
        {
          deactivateAllVibrators(); // 重置当前振动
          vibrationActive = true;
          currentState = CLOCKWISE; // 切换到顺时针状态
          currentIndex = 0; // 重置索引
        }
        break;

      case 'e': // 逆时针轮流振动
        if (currentState != COUNTERCLOCKWISE) // 仅当不在逆时针状态时切换
        {
          deactivateAllVibrators(); // 重置当前振动
          vibrationActive = true;
          currentState = COUNTERCLOCKWISE; // 切换到逆时针状态
          currentIndex = 0; // 重置索引
        }
        break;
    }
  }

  // 根据当前状态执行相应的振动逻辑
  switch (currentState)
  {
    case CLOCKWISE:
      rotateClockwise();
      break;

    case COUNTERCLOCKWISE:
      rotateCounterclockwise();
      break;

    case IDLE:
    default:
      // 空闲状态下不执行任何操作
      break;
  }
}

// 激活单个振动器
void executeSingleVibration(char command)
{
  switch (command)
  {
    case 'w':
      activateVibrator(wPin, true);
      break;
    case 's':
      activateVibrator(sPin, true);
      break;
    case 'a':
      activateVibrator(aPin, true);
      break;
    case 'd':
      activateVibrator(dPin, true);
      break;
    case 'u':
      activateVibrator(upPin, true);
      break;
    case 'x':
      activateVibrator(downPin, true);
      break;
    case 'l':
      activateVibrator(lPin, true);
      break;
    case 'r':
      activateVibrator(rPin, true);
      break;
    case 'n':
      deactivateAllVibrators();
      vibrationActive = false;
      break;
  }
}

// 顺时针振动函数
void rotateClockwise()
{
  const int clockwisePins[] = { wPin, dPin, sPin }; // 顺时针引脚列表
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // 更新上次切换时间

    // 停止当前振动器
    analogWrite(clockwisePins[currentIndex], 0);

    // 切换到下一个振动器
    currentIndex = (currentIndex + 1) % 3; // 循环 0,1,2

    // 激活新的振动器
    analogWrite(clockwisePins[currentIndex], 100);
  }
}

// 逆时针振动函数
void rotateCounterclockwise()
{
  const int counterclockwisePins[] = { wPin, aPin, sPin }; // 逆时针引脚列表
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // 更新上次切换时间

    // 停止当前振动器
    analogWrite(counterclockwisePins[currentIndex], 0);

    // 切换到下一个振动器
    currentIndex = (currentIndex + 1) % 3; // 循环 0,1,2

    // 激活新的振动器
    analogWrite(counterclockwisePins[currentIndex], 100);
  }
}

// 激活单个振动器
void activateVibrator(int pin, bool state)
{
  analogWrite(pin, state ? 100 : 0); // PWM 信号
}

// 停止所有振动器
void deactivateAllVibrators()
{
  analogWrite(wPin, 0);
  analogWrite(sPin, 0);
  analogWrite(aPin, 0);
  analogWrite(dPin, 0);

  analogWrite(upPin, 0);
  analogWrite(downPin, 0);
  analogWrite(lPin, 0);
  analogWrite(rPin, 0);
}
