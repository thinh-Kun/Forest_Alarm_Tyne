#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <esp_task_wdt.h>


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */
#define DeviceName  "GPS_Device1"
#define MqttId      "vikings"
#define MqttPass    "19013005"
#define Mqtt_Port   1883
#define broker      "hungviet.hopto.org"   //513booyoungct4.ddns.net
#define topicPub     "GsmClientTest/led"
#define TINY_GSM_MODEM_SIM7600
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]      = "Vina";
const char gprsUser[] = "mms";
const char gprsPass[] = "mms";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
// MQTT details

#define RXPin 19
#define TXPin 18
#define DHTPIN 27
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define GPSBaud 9600
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
TinyGsm modem(Serial2);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

float humidity = 0;
float temperature = 0;
char temperature_str[10];
String battery_vol;
String receivedString = "";
String longtitue;
String lattitue;

uint32_t lastReconnectAttempt = 0;
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();

  // Only proceed if incoming message's topic matches
  //  if (String(topic) == topicLed) {
  //    ledStatus = !ledStatus;
  //
  //    mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  //  }

}
long time_wifi = 0;
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect("GPS_Device1", "vikings", "19013005")) {
      Serial.println("connected");
      // Subscribe
      mqtt.publish("GsmClientTest/init", "Connect Success");
      delay(2000);
      mqtt.publish("esp32/status", "Connect Failed");
    }
    else {
      long now = millis();
      if (now - time_wifi > 15000)
      {
        setup_GPRS();
        time_wifi = now;
        delay(5000);
      }
      Serial.print("failed, err_code:");
      Serial.println(mqtt.state());
      delay(100);
      Serial.println(" try again in 5 seconds");
      mqtt.disconnect();
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void setup() {
  // Set console baud rate
  Serial.begin(115200);
  ss.begin(GPSBaud);
  longtitue = "";
  lattitue = "";
  delay(1000);



  // !!!!!!!!!!!
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!

  Serial.println("Wait...");
  Serial2.begin(115200);
  dht.begin();
  delay(6000);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.restart();
  // modem.init();
  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);
  delay(1000);
  for (int i = 0; i < 5; i++)
  {
    Serial2.println("AT+CBC");
    while (!Serial2.available());
    while (Serial2.available()) {
      char receivedChar = Serial2.read(); // Đọc kí tự từ cổng UART
      if (receivedChar == 'V') break;

      if (receivedChar == '\n') { // Nếu gặp kí tự newline
        receivedString.trim();    // Loại bỏ các khoảng trắng ở đầu và cuối chuỗi
        //      Serial.println("Dữ liệu đã nhận: " + receivedString);
        //      receivedString = "";      // Reset biến receivedString
      } else {
        receivedString += receivedChar; // Thêm kí tự vào biến receivedString
      }
    }

    receivedString = receivedString.substring(6);
    if (receivedString[0] != 'D' || receivedString[1] != 'D') break;

    if (receivedString.length() > 4 && receivedString.length() < 7) break;
  }

  Serial.print("Battery Voltage: ");  Serial.println(receivedString);
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  esp_task_wdt_init(30, true); // Set WDT to reset after 10 seconds
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  //if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }
  setup_GPRS();
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

}
void setup_GPRS() {
  delay(100);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  modem.gprsConnect(apn, gprsUser, gprsPass);
  delay(1000);
  uint8_t count = 0;
  while (!modem.isNetworkConnected()) {
    Serial.print(".");
    delay(500);
    count ++;
    if (count > 30) ESP.restart();
  }
  Serial.println(" success");
}
void loop() {
  if (!mqtt.connected()) {
    reconnect();
  }
  //  esp_task_wdt_reset();
  while (ss.available() > 0)
  {
    mqtt.loop();
    if (gps.encode(ss.read()))
    {
      Serial.print(F("Location: "));
      if (gps.location.isValid())
      {
        longtitue = gps.location.lng();
        lattitue = gps.location.lat();
        Serial.print("lattitue: "); Serial.println(lattitue);
        Serial.print("longtitue: "); Serial.println(longtitue);
      }
      if (longtitue.length() > 0 && lattitue.length() > 0)
      {
        StaticJsonDocument<256> doc1;
        doc1["latitute"] = lattitue;
        doc1["longtitute"] = longtitue;
        //  humidity = dht.readHumidity();
        //  float temp = dht.readTemperature();
        //  temperature = round(temp * 100) / 100.0;
        //  sprintf(temperature_str, "%.2f", temperature);
        //    doc1["humidity"] = humidity;
        //    doc1["temperature"] = temperature_str;
        doc1["humidity"] = 60;
        doc1["temperature"] = 30;
        doc1["battery_vol"] = receivedString;
        Serial.println("###############################################");

        String Jdata;
        serializeJson(doc1, Jdata);
        //  //gửi json đến topic esp32/json
        mqtt.publish("v1/devices/me/telemetry", Jdata.c_str());
        esp_task_wdt_reset();
        esp_deep_sleep_start();
        Serial.println("MQTT Send Data");
        delay(2000);
      }
    }
  }

}