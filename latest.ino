#include <Wire.h>
#include <BH1750.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>

// ========== WiFi配置 ==========
const char* ssid = "nyx";
const char* password = "12345678";

// ========== MQTT配置 ==========
const char* mqttServer = "bemfa.com";
const int mqttPort = 9501;
const char* clientId = " ";

// ========== MQTT主题 ==========
const char* topicMic = "mic004";      // 麦克风传感器
const char* topicLight = "light004";  // 光照传感器
const char* topicTouch = "touch004";  // 触摸传感器
const char* topicPIR = "pir004";      // 人体检测传感器
const char* topicServo = "feeder004"; // 喂食机舵机

// ========== 引脚定义 ==========
const int touchPin = 14;
const int micPinDigital = 12;
const int micPinAnalog = A0;
const int pirPin = 0;
const int servoPin = 13;
const int ledPin = LED_BUILTIN;

// ========== BH1750光照传感器 ==========
BH1750 lightMeter(0x23);
#define LIGHT_THRESHOLD_HIGH 100
#define LIGHT_THRESHOLD_LOW 20

// ========== 触摸模块变量 ==========
bool lastTouchState = HIGH;
const int debounceDelay = 100;
bool isLedOnByTouch = false;
bool touchLedState = false;

// ========== 声音模块变量 ==========
unsigned long soundTurnOffAt = 0;
const unsigned long soundLedDuration = 5000;
bool isLedOnBySound = false;
bool currentSoundDetected = false;
int currentSoundLevel = 0;

// ========== 光照模块变量 ==========
bool isLedOnByLight = false;
bool isInActiveTimeByLight = false;

// ========== 人体检测模块变量 ==========
bool pirState = false;
bool lastPirState = false;
unsigned long pirTurnOffAt = 0;
const unsigned long pirLedDuration = 5000;
bool isLedOnByPIR = false;
unsigned long lastPirTriggerTime = 0;
const unsigned long pirCooldown = 30000;
bool pirInCooldown = false;

// ========== 舵机喂食机变量 ==========
Servo feederServo;                    // 喂食机舵机
bool feederIsOpen = false;            // 喂食口是否打开
unsigned long feederOpenTime = 0;     // 开闸时间
const unsigned long feederDuration = 3000;  // 开闸3秒
unsigned long lastFeedTime = 0;       // 上次喂食时间
const unsigned long feedInterval = 30000;   // 喂食间隔30秒（演示用，实际可改成14400000=4小时）
int feedCount = 0;                    // 今日喂食次数
const int closedAngle = 0;            // 关闭角度
const int openAngle = 90;             // 打开角度

// ========== LED状态 ==========
bool ledState = HIGH;

// ========== WiFi和MQTT客户端 ==========
WiFiClient espClient;
PubSubClient client(espClient);

// ========== 定时上传 ==========
unsigned long previousStatusMillis = 0;
unsigned long previousCloudMillis = 0;
const long statusInterval = 500;
const long cloudInterval = 200;

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n========================================");
  Serial.println("🚀 ESP8266 四传感器智能照明系统启动");
  Serial.println("========================================");
  
  pinMode(touchPin, INPUT_PULLUP);
  pinMode(micPinDigital, INPUT);
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  Wire.begin(D2, D1);
  delay(100);
  
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("✅ BH1750光照传感器初始化成功！");
  } else {
    Serial.println("❌ BH1750光照传感器初始化失败！");
  }
  
  // 初始化喂食机舵机
  feederServo.attach(servoPin);
  feederServo.write(closedAngle);  // 初始关闭
  delay(500);
  Serial.println("✅ 喂食机舵机初始化成功（闸门关闭）");
  feederIsOpen = false;
  lastFeedTime = 0;  // 初始化为0，表示可以立即喂食一次
  
  setupWiFi();
  client.setServer(mqttServer, mqttPort);
  
  Serial.println("\n📋 功能说明：");
  Serial.println("1. 触摸传感器：开关模式（触摸一次开，再触摸一次关）");
  Serial.println("2. 声音传感器：检测到声音LED亮5秒");
  Serial.println("3. 光照传感器：智能作息补光");
  Serial.println("4. 宠物活动检测：24小时工作");
  Serial.println("5. 智能喂食机：检测到宠物活动 + 距上次喂食>30秒");
  Serial.println("   - 舵机打开闸门3秒投食");
  Serial.println("   - 记录喂食次数和时间");
  Serial.println("   - 防止过度喂食");
  Serial.println("   ⚠️ 演示模式：30秒间隔（实际使用建议4小时）");
  Serial.println("6. 串口数据每0.5秒更新");
  Serial.println("7. 云端数据每0.2秒上传");
  Serial.println("========================================\n");
}

void loop() {
  unsigned long now = millis();
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  handleTouch(now);
  handleSound(now);
  handleLight();
  handlePIR(now);
  handleFeeder(now);  // 处理喂食机
  updateLED(now);
  
  if (now - previousStatusMillis >= statusInterval) {
    printAllSensorStatus();
    previousStatusMillis = now;
  }
  
  if (now - previousCloudMillis >= cloudInterval) {
    sendToCloud();
    previousCloudMillis = now;
  }
  
  delay(50);
}

void setupWiFi() {
  Serial.print("正在连接WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi连接成功!");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi连接失败!");
  }
}

void reconnect() {
  int retries = 0;
  while (!client.connected() && retries < 3) {
    Serial.println("\n========================================");
    Serial.print("🔄 尝试连接MQTT服务器 (");
    Serial.print(retries + 1);
    Serial.println("/3)...");
    
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println("✅ MQTT连接成功!");
      Serial.println("========================================\n");
      retries = 0;
    } else {
      Serial.print("❌ 连接失败, 错误代码: ");
      Serial.println(client.state());
      Serial.println("⏰ 5秒后重试...");
      delay(5000);
      retries++;
    }
  }
}

void handleTouch(unsigned long now) {
  bool currentTouchState = digitalRead(touchPin);
  delay(debounceDelay);
  currentTouchState = digitalRead(touchPin);
  
  if (currentTouchState == LOW && lastTouchState == HIGH) {
    touchLedState = !touchLedState;
    isLedOnByTouch = touchLedState;
    
    if (touchLedState) {
      Serial.println("👉 [触摸] 触摸触发 - 灯已开启（保持常亮）");
    } else {
      Serial.println("👉 [触摸] 触摸触发 - 灯已关闭");
    }
  }
  
  lastTouchState = currentTouchState;
}

void handleSound(unsigned long now) {
  currentSoundLevel = analogRead(micPinAnalog);
  currentSoundDetected = (digitalRead(micPinDigital) == HIGH);
  
  if (currentSoundDetected) {
    soundTurnOffAt = now + soundLedDuration;
    
    if (!isLedOnBySound) {
      isLedOnBySound = true;
      Serial.print("🔊 [声音] 检测到声音！强度: ");
      Serial.print(currentSoundLevel);
      Serial.println(" - LED已点亮（持续5秒）");
    }
  }
  
  if (isLedOnBySound && (now >= soundTurnOffAt)) {
    isLedOnBySound = false;
    Serial.println("⏰ [声音] 5秒已到，声音触发结束");
  }
}

void handleLight() {
  float lux = lightMeter.readLightLevel();
  
  if (lux >= LIGHT_THRESHOLD_LOW && lux < LIGHT_THRESHOLD_HIGH) {
    if (!isInActiveTimeByLight) {
      isInActiveTimeByLight = true;
      Serial.print("🌆 [光照] 进入补光时段 (");
      Serial.print(lux);
      Serial.println(" lx) - 自动补光开启");
    }
    
    if (!isLedOnByLight) {
      isLedOnByLight = true;
      Serial.print("💡 [光照] 环境光偏暗(");
      Serial.print(lux);
      Serial.println(" lx)，补光中");
    }
  } else {
    if (isInActiveTimeByLight) {
      isInActiveTimeByLight = false;
      if (lux >= LIGHT_THRESHOLD_HIGH) {
        Serial.print("☀️ [光照] 白天模式 (");
        Serial.print(lux);
        Serial.println(" lx) - 光线充足，停止补光");
      } else {
        Serial.print("🌙 [光照] 深夜模式 (");
        Serial.print(lux);
        Serial.println(" lx) - 睡眠时间，停止补光");
      }
    }
    
    if (isLedOnByLight) {
      isLedOnByLight = false;
      Serial.println("💡 [光照] 退出补光时段");
    }
  }
}

void handlePIR(unsigned long now) {
  pirState = digitalRead(pirPin);
  
  float lux = lightMeter.readLightLevel();
  bool shouldTriggerLight = (lux < LIGHT_THRESHOLD_HIGH);
  
  if (pirInCooldown) {
    if (now - lastPirTriggerTime >= pirCooldown) {
      pirInCooldown = false;
      Serial.println("⏰ [宠物活动] 冷却期结束，可再次触发灯光");
    }
  }
  
  if (pirState != lastPirState) {
    if (pirState == HIGH) {
      Serial.print("🐾 [宠物活动] 检测到活动！");
      
      if (!pirInCooldown && shouldTriggerLight) {
        pirTurnOffAt = now + pirLedDuration;
        isLedOnByPIR = true;
        lastPirTriggerTime = now;
        pirInCooldown = true;
        Serial.print(" LED已点亮 (光照:");
        Serial.print(lux);
        Serial.println(" lx)");
        Serial.println("   ⏳ 进入30秒冷却期");
      } else if (pirInCooldown) {
        Serial.println(" [冷却期中，不触发灯]");
      } else if (!shouldTriggerLight) {
        Serial.print(" [白天，不触发灯，光照:");
        Serial.print(lux);
        Serial.println(" lx]");
      }
      
      // 检测到活动时，检查是否可以喂食
      unsigned long timeSinceLastFeed = now - lastFeedTime;
      
      if (timeSinceLastFeed >= feedInterval && !feederIsOpen) {
        // 可以喂食：距离上次喂食足够久 且 闸门未打开
        feederServo.write(openAngle);
        feederIsOpen = true;
        feederOpenTime = now;
        feedCount++;
        lastFeedTime = now;
        
        Serial.println("🍖 [喂食机] ===== 开始喂食 =====");
        Serial.print("   原因：检测到宠物活动，距上次喂食已过 ");
        Serial.print(timeSinceLastFeed / 1000);
        Serial.println(" 秒");
        Serial.print("   今日喂食次数：");
        Serial.println(feedCount);
      } else if (timeSinceLastFeed < feedInterval) {
        // 距离上次喂食太近，不能喂
        unsigned long remainingTime = (feedInterval - timeSinceLastFeed) / 1000;
        Serial.print("🍖 [喂食机] 检测到宠物，但距上次喂食仅 ");
        Serial.print(timeSinceLastFeed / 1000);
        Serial.print(" 秒，需等待 ");
        Serial.print(remainingTime);
        Serial.println(" 秒后才可再次喂食");
      }
    } else {
      Serial.println("👋 [宠物活动] 活动结束");
    }
    lastPirState = pirState;
  }
  
  if (isLedOnByPIR && (now >= pirTurnOffAt)) {
    isLedOnByPIR = false;
    Serial.println("⏰ [宠物活动] 灯光5秒已到，LED关闭");
  }
}

void handleFeeder(unsigned long now) {
  // 检查是否需要关闭闸门
  if (feederIsOpen) {
    if (now - feederOpenTime >= feederDuration) {
      // 3秒到了，关闭闸门
      feederServo.write(closedAngle);
      feederIsOpen = false;
      Serial.println("🍖 [喂食机] 闸门关闭，喂食完成");
      Serial.println("========================================");
    }
  }
}

void printAllSensorStatus() {
  Serial.println("\n========== 传感器状态更新 ==========");
  
  float lux = lightMeter.readLightLevel();
  Serial.print("💡 [光照] 当前强度: ");
  Serial.print(lux);
  Serial.print(" lx - ");
  
  if (lux >= LIGHT_THRESHOLD_HIGH) {
    Serial.println("白天 (光线充足，不补光)");
  } else if (lux >= LIGHT_THRESHOLD_LOW) {
    Serial.println("傍晚/阴天 (补光中)");
  } else {
    Serial.println("深夜 (睡眠时间，不补光)");
  }
  
  Serial.print("🔊 [声音] 强度值: ");
  Serial.print(currentSoundLevel);
  Serial.print(" (0-1023)");
  if (currentSoundDetected) {
    Serial.println(" - 超过阈值 ✓");
  } else {
    Serial.println();
  }
  
  bool touchDetected = digitalRead(touchPin) == LOW;
  Serial.print("👉 [触摸] 当前状态: ");
  if (touchDetected) {
    Serial.print("正在触摸 ✓ - ");
  } else {
    Serial.print("无触摸 - ");
  }
  
  if (isLedOnByTouch) {
    Serial.println("触摸灯：开启");
  } else {
    Serial.println("触摸灯：关闭");
  }
  
  Serial.print("🐾 [宠物活动] 当前状态: ");
  
  if (pirState == HIGH) {
    Serial.print("检测到活动 ✓");
    
    if (pirInCooldown) {
      unsigned long remainingCooldown = (pirCooldown - (millis() - lastPirTriggerTime)) / 1000;
      Serial.print(" [冷却中,剩余");
      Serial.print(remainingCooldown);
      Serial.print("秒]");
    }
    
    if (lux >= LIGHT_THRESHOLD_HIGH) {
      Serial.println(" [白天不触发灯]");
    } else if (isLedOnByPIR) {
      Serial.println(" [已触发灯]");
    } else {
      Serial.println();
    }
  } else {
    if (isLedOnByPIR) {
      Serial.println("无活动 [灯光5秒倒计时中]");
    } else if (pirInCooldown) {
      unsigned long remainingCooldown = (pirCooldown - (millis() - lastPirTriggerTime)) / 1000;
      Serial.print("无活动 [冷却中,剩余");
      Serial.print(remainingCooldown);
      Serial.println("秒]");
    } else {
      Serial.println("无活动 (待机中)");
    }
  }
  
  // 显示喂食机状态
  Serial.print("🍖 [喂食机] 当前状态: ");
  if (feederIsOpen) {
    unsigned long elapsed = (millis() - feederOpenTime) / 1000;
    unsigned long remaining = (feederDuration / 1000) - elapsed;
    Serial.print("喂食中 (剩余 ");
    Serial.print(remaining);
    Serial.println(" 秒)");
  } else {
    unsigned long timeSinceLastFeed = millis() - lastFeedTime;
    if (lastFeedTime == 0) {
      Serial.println("待机中 (随时可喂食)");
    } else if (timeSinceLastFeed >= feedInterval) {
      Serial.print("待机中 (可喂食) 今日喂食:");
      Serial.print(feedCount);
      Serial.println("次");
    } else {
      unsigned long remainingTime = (feedInterval - timeSinceLastFeed) / 1000;
      Serial.print("冷却中 (");
      Serial.print(remainingTime);
      Serial.print("秒后可喂食) 今日:");
      Serial.print(feedCount);
      Serial.println("次");
    }
  }
  
  Serial.print("💡 [LED] 当前状态: ");
  if (ledState == LOW) {
    Serial.print("开启");
    Serial.print(" (触发原因: ");
    if (isLedOnByTouch) Serial.print("触摸 ");
    if (isLedOnBySound) Serial.print("声音 ");
    if (isLedOnByLight) Serial.print("光照补光 ");
    if (isLedOnByPIR) Serial.print("宠物活动 ");
    Serial.println(")");
  } else {
    Serial.println("关闭");
  }
  
  Serial.println("====================================\n");
}

void updateLED(unsigned long now) {
  bool shouldLedBeOn = isLedOnByTouch || isLedOnBySound || isLedOnByLight || isLedOnByPIR;
  bool newLedState = shouldLedBeOn ? LOW : HIGH;
  
  if (newLedState != ledState) {
    ledState = newLedState;
    digitalWrite(ledPin, ledState);
    
    if (ledState == LOW) {
      Serial.println("🔆 ========== LED 开启 ==========");
    } else {
      Serial.println("🌑 ========== LED 关闭 ==========");
    }
  }
}

void sendToCloud() {
  if (!client.connected()) {
    return;
  }
  
  // 1. 上传光照强度
  float lux = lightMeter.readLightLevel();
  char lightMsg[10];
  dtostrf(lux, 4, 1, lightMsg);
  client.publish(topicLight, lightMsg);
  
  // 2. 上传声音强度
  char micMsg[10];
  sprintf(micMsg, "%d", currentSoundLevel);
  client.publish(topicMic, micMsg);
  
  // 3. 上传触摸状态
  const char* touchMsg = isLedOnByTouch ? "1" : "0";
  client.publish(topicTouch, touchMsg);
  
  // 4. 上传人体检测状态
  const char* pirMsg = (pirState == HIGH) ? "1" : "0";
  client.publish(topicPIR, pirMsg);
  
  // 5. 上传喂食机状态（0=闲置, 1=喂食中）
  const char* feederMsg = feederIsOpen ? "1" : "0";
  client.publish(topicServo, feederMsg);
}
