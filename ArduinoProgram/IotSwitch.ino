#include <RtcDS1302.h>  //rtc
#include <ThreeWire.h>
#include <Wire.h>  //oled
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>  //温度传感器
#include <DallasTemperature.h>
#include <WiFi.h>  //WiFi通信
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <BLEDevice.h>  // 引入 ESP32 的 BLE 设备库
#include <BLEServer.h>  // 引入 BLE 服务器库
#include <BLEUtils.h>   // 引入 BLE 工具库
#include <BLE2902.h>    // 引入 BLE 2902 描述符库

#define SDA_PIN 22                                                  // OLED SDA接GPIO 22
#define SCL_PIN 23                                                  // OLED SCL接GPIO 23
#define SCREEN_WIDTH 128                                            // OLED display width, in pixels
#define SCREEN_HEIGHT 64                                            // OLED display height, in pixels
#define OLED_RESET -1                                               // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C                                         //SSD1306驱动的I2C接口默认通信地址入口
#define ONE_WIRE_BUS 27                                             //温度传感器 DQ接GPIO 27
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"         // 定义服务的 UUID
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // 定义特征的 UUID

ThreeWire myWire(12, 14, 13);  //RTC IO, SCLK, CE  对应接GPIO号
RtcDS1302<ThreeWire> Rtc(myWire);
RtcDateTime now;
String nowTime;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  //使用I2C协议前的初始化,创建显示对象
OneWire oneWire(ONE_WIRE_BUS);                                             //初始化单总线引脚
DallasTemperature sensors(&oneWire);
WiFiMulti wifiMulti;  //创建一个复合wifi对象
WiFiClient client;
PubSubClient mqtt(client);  //创建一个MQTT客户端

const int ledPin = 2, relayPin = 19;                    // 定义引脚
bool wifiStatus = 0, mqttStatus = 0, switchStatus = 0;  //定义插座状态信息
float currentPower = 0, totalElectricity = 0, dailyElectricity = 0;
unsigned char Tx_Buffer[8] = { 0x01, 0x03, 0x00, 0x48, 0x00, 0x08, 0xC4, 0x1A };  //读取电能模块信息指令
unsigned char Rx_Buffer[40];
unsigned long Power_data = 0, Energy_data = 0;
float t = 0;
const int appUsername = 3;
const int appEquipmentname = 32;
const char *BrokerServer = "47.115.230.57";                                                        //MQTT Broker地址
const int BrokerPort = 1883;                                                                       //MQTT Broker端口号
const int BrokerPortQoS = 1;                                                                       //MQTT Broker QoS级别设置
const char *BrokerClientID = "esp32_kirtozz";                                                      //MQTT Broker Client ID
const char *BrokerName = "kirtozz";                                                                //MQTT Broker用户名
const char *BrokerPassword = "142857";                                                             //MQTT Broker密码
String willTopicString = "iotPower/will/" + String(appUsername) + "/" + String(appEquipmentname);  // 使用String连接遗嘱
const char *willTopic = willTopicString.c_str();                                                   // 转换为const char*类型
const char *willMessage = "disconnected";
const int willQoS = 1;
String topicCommandString = "iotPower/command/" + String(appUsername) + "/" + String(appEquipmentname);
const char *topicCommand = topicCommandString.c_str();
String topicAlarmString = "iotPower/alarm/" + String(appUsername) + "/" + String(appEquipmentname);
const char *topicAlarm = topicAlarmString.c_str();
String topicStatusString = "iotPower/status/" + String(appUsername) + "/" + String(appEquipmentname);
const char *topicStatus = topicStatusString.c_str();
String topicUploadString = "iotPower/upload/" + String(appUsername) + "/" + String(appEquipmentname);
const char *topicUpload = topicUploadString.c_str();
Ticker timer1, timer2, timer3;  //避免程序阻塞申明的一些计数变量
int reconnectTime1 = 0, reconnectTime2 = 0;
unsigned long previousTriggerTime = 0;        // 上次触发时间
const unsigned long triggerInterval = 60000;  // 触发间隔（毫秒）

typedef struct {
  bool isEffect;
  bool exceptStatus;  // 期待触发的开关状态
  bool isOnly;
  bool triggerDaysOfWeek[7];  // 代表周日到周六的触发状态
  int triggerHour;            // 触发的时刻（小时）
  int triggerMinute;          // 触发的时刻（分钟）
} ScheduledTime;
ScheduledTime timers[3];  // 最多存储3个定时器
int size = 3;

BLEServer *pServer;                  // 定义一个指向 BLE 服务器对象的指针
BLECharacteristic *pCharacteristic;  // 定义一个指向 BLE 特征对象的指针
bool deviceConnected = false;        // 标志变量，表示是否有设备连接到 BLE 服务器
bool oldDeviceConnected = false;     // 用于保存之前连接状态的标志变量

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    pCharacteristic->setValue("");  // 清空特征值
    pCharacteristic->notify();      // 通知已更新特征值
  }
};

void setup() {
  Serial.begin(9600);
  Serial2.begin(4800);
  Serial.println();
  pinMode(ledPin, OUTPUT);    // 设置 LED 引脚为输出
  pinMode(relayPin, OUTPUT);  // 设置 继电器 引脚为输出
  initBLE();
  sensors.begin();                                             //启动温度传感器
  Rtc.Begin();                                                 //用管脚声明来初始化RTC
  Wire.begin(SDA_PIN, SCL_PIN);                                //初始化 I2C引脚
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {  //配置oled: OLED的电压（3.3V），通信地址
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;
  }
  wifiConnection();
  if (WiFi.SSID() == "Gxsf" || WiFi.SSID() == "GXNU-YC")
    wifiLoginRequest();
  mqttConnection(BrokerName, BrokerPassword, BrokerClientID, BrokerServer, BrokerPort, willTopic, willMessage);
  timer1.attach(30, checkWiFiConnection);  //每隔30s检测一下WIFI连接
  timer2.attach(30, checkMqttConnection);  //每隔30s检测一下mqtt连接
}

void loop() {
  now = Rtc.GetDateTime();  //获取RTC时间
  nowTime = strDateTime(now);
  sensors.requestTemperatures();  //温度传感器获取温度
  getPower();
  protection();
  if (now.Hour() == 0 && now.Minute() == 0 && now.Second() == 0)
    t = totalElectricity;
  dailyElectricity = totalElectricity - t;
  uploadStatus();
  oledDisplay();
  readBLE();
  mqtt.loop();  // 处理信息以及心跳
  checkTimers();
}

String strDateTime(RtcDateTime &now) {  //字符化读取到的rtc模块时间格式(int)
  String dateTime;
  String dateTime1 = String(now.Year()) + "." + String(now.Month()) + "." + String(now.Day()) + "-" + String(now.Hour());
  String dateTime2;
  String dateTime3;
  if (now.Minute() < 10) {
    dateTime2 = ":0" + String(now.Minute());
  } else dateTime2 = ":" + String(now.Minute());
  if (now.Second() < 10) {
    dateTime3 = ":0" + String(now.Second());
  } else dateTime3 = ":" + String(now.Second());
  dateTime = dateTime1 + dateTime2 + dateTime3;
  return dateTime;
}

void oledDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("SwitchStatus: ");
  display.println(switchStatus);
  display.print("Temperature: ");
  display.print(sensors.getTempCByIndex(0));
  display.println("*C");
  display.print("CurrentPower: ");
  display.print(currentPower);
  display.println("W");
  display.print("WiFiStatus: ");
  display.println(wifiStatus);
  display.print("MQTTStatus: ");
  display.println(mqttStatus);
  display.println();
  display.println(nowTime);
  display.display();
}

void wifiConnection() {
  wifiMulti.addAP("Gxsf", "");
  wifiMulti.addAP("GXNU-YC", "");
  wifiMulti.addAP("Xiaomi12", "okkkkkkk");
  Serial.println("Connecting ...");
  while (wifiMulti.run() != WL_CONNECTED && reconnectTime1 <= 100) {  // 尝试进行wifi连接,至多尝试25s
    delay(250);
    reconnectTime1++;
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiStatus = 1;
    reconnectTime1 = 0;
    Serial.print("Connected to ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void wifiLoginRequest() {
  WiFiClient client;
  HTTPClient http;
  String url1 = "http://yc.gxnu.edu.cn/drcom/login?callback=dr1003&DDDDD=202012702169&upass=142857&0MKKey=123456&R1=0&R2=&R3=0&R6=0&para=00&v6ip=&terminal_type=1&lang=zh-cn&jsVersion=4.1.3&v=6431&lang=zh";
  String url0 = "http://yc.gxnu.edu.cn/drcom/logout?callback=dr1002&jsVersion=4.1.3&v=6768&lang=zh";
  http.begin(client, url1);  //开始HTTP连接
  http.addHeader("Connection", "close");
  int httpResponseCode = http.GET();  // 发送GET请求
  if (httpResponseCode > 0) {
    Serial.print("WifiLoginRequest Response code: ");
    Serial.println(httpResponseCode);
    // String payload = http.getString();  // 获取响应内容
    // Serial.println("WifiLoginReques Response payload: ");
    // Serial.println(payload);
  } else {
    Serial.print("WifiLoginRequest Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void mqttConnection(const char *userName, const char *userPassword, const char *clientID, const char *brokerAddress, const int brokerPort, const char *willTopic, const char *willMessage) {
  Serial.println("Connecting to  MQTT Broker...");
  mqtt.setServer(brokerAddress, brokerPort);
  mqtt.setCallback(mqttcallback);
  while (!mqtt.connected() && WiFi.status() == WL_CONNECTED && reconnectTime2 <= 100) {  //在WiFi连接的情况下连接mqtt服务器，最多尝试25s
    if (mqtt.connect(clientID, userName, userPassword, willTopic, willQoS, false, willMessage)) {
      Serial.println("MQTT Broker connected!");
      mqttStatus = 1;
      reconnectTime2 = 0;
    } else {
      Serial.print("Connection failed with state ");
      Serial.println(mqtt.state());
      delay(250);
      reconnectTime2++;
    }
    mqtt.subscribe(topicCommand, BrokerPortQoS);  //订阅mqtt服务器的控制命令主题
    mqtt.subscribe(topicAlarm, BrokerPortQoS);    //订阅mqtt服务器的控制命令主题
  }
}

void mqttcallback(char *topic, byte *payload, unsigned int length) {
  String message = "";
  Serial.print("Message Received [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Payload: ");
  Serial.println(message);
  if (strcmp(topic, topicCommand) == 0) {
    if (message == "powerON") {
      digitalWrite(ledPin, HIGH);    // 点亮 LED 灯
      digitalWrite(relayPin, HIGH);  // 闭合继电器
      switchStatus = 1;
      mqtt.publish(topicStatus, "powerOntrue", false);
    } else if (message == "powerOFF") {
      digitalWrite(ledPin, LOW);
      digitalWrite(relayPin, LOW);
      switchStatus = 0;
      mqtt.publish(topicStatus, "powerOFFtrue", false);
    } else {
      Serial.println("Unknown message received");
    }
  } else if (strcmp(topic, topicAlarm) == 0) {
    int num = message[0] - '1';
    bool isEffect = (message[1] == '1');
    if (isEffect) {
      bool exceptStatus = (message[2] == '1');
      bool isOnly = (message[3] == '1');
      bool triggerDaysOfWeek[7];
      int triggerHour, triggerMinute;
      if (isOnly) {
        for (int i = 0; i < 7; i++)
          triggerDaysOfWeek[i] = true;
        triggerHour = message.substring(11, 13).toInt();
        triggerMinute = message.substring(13, 15).toInt();
      } else {
        for (int i = 0; i < 7; i++) {
          triggerDaysOfWeek[i] = (message.charAt(i + 4) == '1');
        }
        String subString1 = message.substring(11, 13);
        String subString2 = message.substring(13, 15);
        triggerHour = subString1.toInt();
        triggerMinute = subString2.toInt();
      }
      addTimer(num, isEffect, exceptStatus, isOnly, triggerDaysOfWeek, triggerHour, triggerMinute);
    } else
      Serial.println("The timer is not effective!");
  }
}

void checkWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiStatus = 1;
    Serial.println("WiFi connected!");
  } else {
    wifiStatus = 0;
    Serial.println("Connection losing");
    Serial.println("Try to connect Wifi again...");
    wifiConnection();
  }
}

void checkMqttConnection() {
  if (mqtt.connected()) {
    mqttStatus = 1;
    Serial.println("MQTT Broker connected!");
  } else {
    mqttStatus = 0;
    Serial.println("Connection losing");
    Serial.println("Try to connect mqtt broker again...");
    mqttConnection(BrokerName, BrokerPassword, BrokerClientID, BrokerServer, BrokerPort, willTopic, willMessage);
  }
}

void uploadStatus() {
  float temperature = sensors.getTempCByIndex(0);
  delay(100);
  StaticJsonDocument<200> doc;
  doc["powerUid"] = appEquipmentname;
  doc["switch"] = switchStatus;
  JsonObject data = doc.createNestedObject("data");
  data["dailyElectricity"] = dailyElectricity;
  data["currentPower"] = currentPower;
  data["currentTemperature"] = temperature;
  char buffer[512];
  serializeJson(doc, buffer);
  mqtt.publish(topicUpload, buffer, false);
}

void addTimer(int num, bool isEffect, bool exceptStatus, bool isOnly, bool* triggerDaysOfWeek, int triggerHour, int triggerMinute) {
  timers[num].isEffect = isEffect;
  timers[num].exceptStatus = exceptStatus;
  timers[num].isOnly = isOnly;
  for (int i = 0; i < 7; i++) {
    timers[num].triggerDaysOfWeek[i] = triggerDaysOfWeek[i];
  }
  timers[num].triggerHour = triggerHour;
  timers[num].triggerMinute = triggerMinute;
}


void getPower() {
  Serial2.write(Tx_Buffer, 8);
  Serial2.readBytes(Rx_Buffer, 40);
  if (Rx_Buffer[0] == 0x01)  // 确认 ID 正确
  {
    Power_data = (((unsigned long)(Rx_Buffer[11])) << 24) | (((unsigned long)(Rx_Buffer[12])) << 16) | (((unsigned long)(Rx_Buffer[13])) << 8) | Rx_Buffer[14];
    Energy_data = (((unsigned long)(Rx_Buffer[15])) << 24) | (((unsigned long)(Rx_Buffer[16])) << 16) | (((unsigned long)(Rx_Buffer[17])) << 8) | Rx_Buffer[18];
  }
  currentPower = Power_data * 0.0001;
  totalElectricity = Energy_data * 0.0001;
}

void protection() {
  if (currentPower >= 2500 || sensors.getTempCByIndex(0) >= 70) {
    digitalWrite(ledPin, LOW);    // 熄灭 LED 灯
    digitalWrite(relayPin, LOW);  // 继电器不触发
    switchStatus = 0;
  }
}

void initBLE() {
  BLEDevice::init("SmartSwitch BLE Server");                    // 初始化 BLE 设备，并设置设备名称
  pServer = BLEDevice::createServer();                          // 创建一个 BLE 服务器对象
  pServer->setCallbacks(new MyServerCallbacks());               // 设置服务器回调函数
  BLEService *pService = pServer->createService(SERVICE_UUID);  // 创建一个指定 UUID 的 BLE 服务对象
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,                    // 特征的 UUID
    BLECharacteristic::PROPERTY_READ |      // 特征的属性：可读
      BLECharacteristic::PROPERTY_WRITE |   // 特征的属性：可写
      BLECharacteristic::PROPERTY_NOTIFY |  // 特征的属性：通知
      BLECharacteristic::PROPERTY_INDICATE  // 特征的属性：指示
  );
  pCharacteristic->addDescriptor(new BLE2902());             // 添加 BLE2902 描述符
  pService->start();                                         // 启动 BLE 服务
  BLEAdvertising *pAdvertising = pServer->getAdvertising();  // 获取广播对象
  pAdvertising->start();                                     // 开始广播
  Serial.println("BLE advertising started");
}

void readBLE() {
  if (deviceConnected) {
    pCharacteristic->notify();
    delay(1000);
  } else {
    pCharacteristic->setValue("");  // 清空特征值
    pCharacteristic->notify();      // 通知已更新特征值
  }
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Lost connection,start advertising again");
    oldDeviceConnected = deviceConnected;
  }
  oldDeviceConnected = deviceConnected;
  if (pCharacteristic->getValue().length() > 0) {
    String value = pCharacteristic->getValue().c_str();
    if (value == "1") {
      digitalWrite(ledPin, HIGH);    // 点亮 LED 灯
      digitalWrite(relayPin, HIGH);  // 继电器触发
      switchStatus = 1;
    } else if (value == "0") {
      digitalWrite(ledPin, LOW);    // 熄灭 LED 灯
      digitalWrite(relayPin, LOW);  // 继电器不触发
      switchStatus = 0;
    }
  }
}

void checkTimers() {
  for (int i = 0; i < size; i++) {
    if (timers[i].isEffect && timers[i].triggerDaysOfWeek[now.DayOfWeek()] && timers[i].triggerHour == now.Hour() && timers[i].triggerMinute == now.Minute() && millis() - previousTriggerTime >= triggerInterval) {
      switchStatus = timers[i].exceptStatus;
      if (switchStatus) {
        digitalWrite(ledPin, HIGH);    // 点亮 LED 灯
        digitalWrite(relayPin, HIGH);  // 继电器触发
      } else {
        digitalWrite(ledPin, LOW);    // 关闭 LED 灯
        digitalWrite(relayPin, LOW);  // 继电器关闭
      }
      if (timers[i].isOnly) {
        timers[i].isEffect = false;
      }
      previousTriggerTime = millis();
    }
  }
}