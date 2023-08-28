#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <esp_task_wdt.h>


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */
#define DeviceName  "GPS_Device1"
const char *MqttId =  "tyne";
const char *MqttPass =  "123456";
#define Mqtt_Port   1893
const char *broker =  "513booyoungct4.ddns.net";
//#define broker      "513booyoungct4.ddns.net"   //513booyoungct4.ddns.net
#define topicPub     "v1/devices/me/telemetry"
#define TINY_GSM_MODEM_SIM7600
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200
#define PinPower 21

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]      = "Viettel";
const char gprsUser[] = "mms";
const char gprsPass[] = "mms";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
// MQTT details

#define RXPin 18
#define TXPin 19
#define DHTPIN 32
#define DHTTYPE DHT11
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
double Longitude;
double Latitude;
String stateDevice = "";
bool StateInforSim = false;
bool StateInforGps = false;
bool StateInforDht = false;
StaticJsonDocument<256> data_send;
StaticJsonDocument<256> state_device;


uint32_t lastReconnectAttempt = 0;
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();

}
long time_wifi = 0;
void reconnect() {
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect(DeviceName, MqttId, MqttPass)) {
      Serial.println("connected");
      // Subscribe

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
  pinMode(PinPower, OUTPUT);
  digitalWrite(PinPower, 1);
  Serial.begin(115200);
  ss.begin(GPSBaud);
  Longitude = 0;
  Latitude = 0;
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
  long time_pin = millis();
  while (1) {
    Serial2.println("AT+CBC");
    delay(10);
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
    if (receivedString[0] == '+') {
      receivedString = receivedString.substring(6);
      
      if (receivedString.length() > 4 && receivedString.length() < 7)
      {
        Serial.print("receivedString: "); Serial.println(receivedString);
        break;
      }
    }

    //    receivedString = receivedString.substring(6);
    //    if (receivedString[0] != 'D' || receivedString[1] != 'D') {
    //    if (receivedString.length() > 4 && receivedString.length() < 7)
    //    {
    //      break;
    //    }
    //    }

//    Serial.print("receivedString: "); Serial.println(receivedString);
    //    if (receivedString[0] != 'D' || receivedString[1] != 'D') {
    //      //StateInforSim = false;
    //      continue;
    //    }


    if (millis() - time_pin > 10000) break;
  }


  Serial.print("Battery Voltage: ");  Serial.println(receivedString);
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  esp_task_wdt_init(50, true); // Set WDT to reset after 30 seconds
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  //if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }
  setup_GPRS();
  mqtt.setServer(broker, Mqtt_Port);
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
    if (count > 20) ESP.restart();
  }
  Serial.println(" success");
}
void loop() {
  if (!mqtt.connected()) {
    reconnect();
  }
  //  esp_task_wdt_reset();

  mqtt.loop();
  long time_reset = millis();
  while (1) {
    while (ss.available() > 0)
    {
      if (gps.encode(ss.read()))
      {
        Serial.print(F("Location: "));
        if (gps.location.isValid())
        {
          Longitude = gps.location.lng();
          Latitude = gps.location.lat();
          data_send["latitude"] = Latitude;
          data_send["longitude"] = Longitude;
          Serial.print("Latitude: "); Serial.println(Latitude);
          Serial.print("Longitude: "); Serial.println(Longitude);
          break;
        }
        if (millis() - time_reset > 120000) break;
        ////
      }
    }
    if (millis() - time_reset > 120000) break;
  }

  ///

  //      humidity = dht.readHumidity();
  //      float temp = dht.readTemperature();
  //      data_send["latitude"] = Latitude;
  //      data_send["longitude"] = Longitude;
  for (int i = 0; i < 1000; i++) {
    humidity = dht.readHumidity();
    float temp = dht.readTemperature();
    if (!isnan(humidity) && !isnan(temperature)) {
      temperature = round(temp * 100) / 100.0;
      Serial.print("Humidity: "); Serial.println(humidity);
      Serial.print("Temperature: "); Serial.println(temperature);
      sprintf(temperature_str, "%.2f", temperature);
      data_send["humidity"] = humidity;
      data_send["temperature"] = temperature_str;
      break;
    }
  }
  data_send["battery_vol"] = receivedString;
  String Jdata;
  serializeJson(data_send, Jdata);
  //  //gửi json đến topic esp32/json
  mqtt.publish("v1/devices/me/telemetry", Jdata.c_str());
  delay(1000);
  Serial.println("MQTT Send Data");
  Serial.println("###############################################");
  esp_task_wdt_reset();

  esp_deep_sleep_start();


}

//    }
//  }
//}
