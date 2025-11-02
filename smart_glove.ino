
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <ArduinoHttpClient.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const char* WIFI_SSID = "Tisha";
const char* WIFI_PASS = "1234567800";
const char* MQTT_BROKER = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* DEVICE_ID = "glove_01";

const char* WEBHOOK_HOST = "hooks.zapier.com";
const int   WEBHOOK_PORT = 443;
const char* WEBHOOK_PATH = "https://hooks.zapier.com/hooks/catch/25126370/ui3dued/"; 
const unsigned long MIN_ZAP_INTERVAL_MS = 10000; 

const int FLEX_PIN = A0;
const int FSR_PIN  = A1;
const int MOTOR_PIN = 2;

const int FLEX_FULL_THRESH = 500;
const int FLEX_START = 350;
int FSR_THRESHOLD = 400;
const int FSR_START = 50;

const unsigned long SAMPLE_MS = 50;
const unsigned long PUB_INTERVAL = 500;
const unsigned long LCD_UPDATE_MS = 1000; 
const unsigned long VIBE_MS = 300;
const unsigned long MIN_VIBE_GAP_MS = 1500;
const unsigned long START_DEBOUNCE_MS = 300;

const unsigned long WINDOW_MS = 3000;
const float TARGET_DEGREES = 340.0;


unsigned long lastSample = 0, lastPublish = 0, lastLCDUpdate = 0;
unsigned long lastVibeTime = 0, vibeStart = 0;
bool vibState = false;
float yawAccumulator = 0.0;
unsigned long lastGyroTime = 0, circleWindowStart = 0;
bool sessionActive = false;
unsigned long startCandidateTime = 0;
unsigned long lastZapSent = 0;
const unsigned long EMAIL_DISPLAY_MS = 3000;
unsigned long emailSentDisplayUntil = 0;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WiFiSSLClient sslClient;
HttpClient httpClient(sslClient, WEBHOOK_HOST, WEBHOOK_PORT);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void triggerVibe(unsigned long now, const char* reason);
bool sendZapierWebhook(const char* eventType, int flex, int fsr,
                       float accelMag, float yawRateDPS, bool vib);
void mqttReconnect();
void connectWiFi();
void safePublishSensors(int flexRaw, int fsrRaw, float accelMag, float gz_dps);

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(FLEX_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);

  Serial.println("Booting Smart Glove...");

  if (!IMU.begin()) Serial.println("IMU init failed");
  else Serial.println("IMU ready");

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Smart Glove Boot");
  delay(800);
  lcd.clear();

  connectWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  lastSample = millis();
  circleWindowStart = millis();
  lastGyroTime = millis();

  lcd.setCursor(0,0); lcd.print("WiFi:-- MQTT:--");
  lcd.setCursor(0,1); lcd.print("Sensors Loading");
}

// ------------------ LOOP ------------------
void loop() {
  unsigned long now = millis();

  if (!mqtt.connected()) mqttReconnect();
  if (mqtt.connected()) mqtt.loop();

  if (vibState && (now - vibeStart >= VIBE_MS)) {
    digitalWrite(MOTOR_PIN, LOW);
    vibState = false;
  }

  if (now - lastSample < SAMPLE_MS) return;
  lastSample = now;

  int flexRaw = analogRead(FLEX_PIN);
  int fsrRaw  = analogRead(FSR_PIN);

  float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gx, gy, gz);
  float accelMag = sqrt(ax*ax + ay*ay + az*az);
  float gz_dps = gz * 180.0 / 3.14159265;
  if (fabs(gz_dps) < 2.0) gz_dps = 0.0;
  

  // ---- Session start detection ----
  bool startCandidate = (flexRaw > FLEX_START) || (fsrRaw > FSR_START) || (fabs(gz_dps) > 20.0);
  if (startCandidate) {
    if (startCandidateTime == 0) startCandidateTime = now;
    else if ((now - startCandidateTime) >= START_DEBOUNCE_MS && !sessionActive) {
      sessionActive = true;
      Serial.println("Session START detected");
      if (now - lastZapSent >= MIN_ZAP_INTERVAL_MS) {
        bool ok = sendZapierWebhook("start", flexRaw, fsrRaw, accelMag, gz_dps, false);
        if (ok) {
          lastZapSent = now;
          emailSentDisplayUntil = millis() + EMAIL_DISPLAY_MS;
        } else {
          Serial.println(" Zapier send failed!");
        }
      }
    }
  } else startCandidateTime = 0;

  // ---- IMU rotation detection ----
  unsigned long dt = now - lastGyroTime;
  if (dt > 0 && IMU.gyroscopeAvailable()) {
    yawAccumulator += gz_dps * (dt / 1000.0);
    lastGyroTime = now;
  }
  if (now - circleWindowStart >= WINDOW_MS) {
    bool circleDetected = (fabs(yawAccumulator) >= TARGET_DEGREES);
    circleWindowStart = now;
    yawAccumulator = 0.0;
    if (circleDetected && (now - lastVibeTime) >= MIN_VIBE_GAP_MS && !vibState)
      triggerVibe(now, "circle");
  }

  if (flexRaw <= FLEX_FULL_THRESH && (now - lastVibeTime) >= MIN_VIBE_GAP_MS && !vibState)
    triggerVibe(now, "flex");
  if (fsrRaw >= FSR_THRESHOLD && (now - lastVibeTime) >= MIN_VIBE_GAP_MS && !vibState)
    triggerVibe(now, "fsr");

  if (now - lastPublish >= PUB_INTERVAL) {
    lastPublish = now;
    safePublishSensors(flexRaw, fsrRaw, accelMag, gz_dps);
  }


  if (now - lastLCDUpdate >= LCD_UPDATE_MS) {
    lastLCDUpdate = now;

    char line1[17], line2[17];
    snprintf(line1, sizeof(line1), "WiFi:%2s MQTT:%2s",
             (WiFi.status() == WL_CONNECTED ? "OK" : "--"),
             (mqtt.connected() ? "OK" : "--"));
    lcd.setCursor(0,0);
    lcd.print(line1);
    lcd.print("    ");  

    if (millis() < emailSentDisplayUntil) {
      snprintf(line2, sizeof(line2), "EMAIL: SENT      ");
    } else {
      snprintf(line2, sizeof(line2), "P:%3d F:%3d A:%.1f", fsrRaw, flexRaw, accelMag);
    }

    lcd.setCursor(0,1);
    lcd.print(line2);
  }
}

// ------------------ FUNCTIONS ------------------
void triggerVibe(unsigned long now, const char* reason) {
  Serial.print("Vibration: "); Serial.println(reason);
  digitalWrite(MOTOR_PIN, HIGH);
  vibState = true;
  vibeStart = now;
  lastVibeTime = now;
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("Connecting WiFi ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(200);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
  else
    Serial.println("\nWiFi connect failed");
}

void mqttReconnect() {
  if (mqtt.connected()) return;
  Serial.print("Connecting MQTT ");
  Serial.print(MQTT_BROKER); Serial.print(":"); Serial.println(MQTT_PORT);
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return;

  String clientId = String(DEVICE_ID) + "-" + String(random(0xffff), HEX);
  if (mqtt.connect(clientId.c_str())) {
    Serial.println("MQTT connected");
    mqtt.subscribe((String("glove/") + DEVICE_ID + "/commands").c_str());
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqtt.state());
  }
}

void safePublishSensors(int flexRaw, int fsrRaw, float accelMag, float gz_dps) {
  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_ID;
  doc["ts"] = millis();
  doc["flex_raw"] = flexRaw;
  doc["fsr_raw"] = fsrRaw;
  doc["accel_mag"] = accelMag;
  doc["yaw_rate_dps"] = gz_dps;
  doc["vib"] = vibState ? 1 : 0;

  char buf[256];
  size_t n = serializeJson(doc, buf);
  String topic = String("glove/") + DEVICE_ID + "/sensors";

  if (!mqtt.connected()) {
    Serial.println("MQTT NOT connected — skipped publish");
    return;
  }
  bool ok = mqtt.publish(topic.c_str(), buf, n);
  Serial.print("MQTT publish: "); Serial.println(ok ? "OK" : "FAIL");
}

// ✅ Merged Zapier logic from your final calibrated code
bool sendZapierWebhook(const char* eventType, int flex, int fsr,
                       float accelMag, float yawRateDPS, bool vib) {
  unsigned long now = millis();
  if (now - lastZapSent < MIN_ZAP_INTERVAL_MS) {
    Serial.println("Zapier skipped (rate-limit)");
    return false;
  }

  StaticJsonDocument<256> payload;
  payload["device_id"] = DEVICE_ID;
  payload["event"] = eventType;
  payload["ts"] = now;
  payload["flex"] = flex;
  payload["fsr"] = fsr;
  payload["accel"] = accelMag;
  payload["yaw_rate_dps"] = yawRateDPS;
  payload["vib"] = vib ? 1 : 0;
  payload["message"] = String("Smart Glove: session start detected");

  char body[256];
  size_t len = serializeJson(payload, body, sizeof(body));

  Serial.print("Sending Zapier webhook: "); Serial.println(WEBHOOK_PATH);
  Serial.print("Body: "); Serial.println(body);

  httpClient.beginRequest();
  httpClient.post(WEBHOOK_PATH, "application/json", body);
  httpClient.sendHeader("Content-Type", "application/json");
  httpClient.sendHeader("Content-Length", len);
  httpClient.beginBody();
  httpClient.print(body);
  httpClient.endRequest();

  int status = httpClient.responseStatusCode();
  String resp = httpClient.responseBody();
  Serial.print("Zapier HTTP status: "); Serial.println(status);
  if (resp.length()) Serial.println(resp);

  lastZapSent = now;
  return (status >= 200 && status < 300);
}
