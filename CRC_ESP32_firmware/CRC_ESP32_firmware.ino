#include <ESP8266WiFi.h>

IPAddress local_IP(192,168,200,8);
IPAddress gateway(192,168,200,126);
IPAddress subnet(255,255,255,0); 

// WiFi credentials
const char* network_ssid = "GalaxyA52";
const char* network_password = "wkci5158";

// TCP server config
WiFiServer server(1234);

#define BAUDRATE 9600

void setup() {
  // UART0 Setup
  Serial.begin(BAUDRATE);

  //WiFi Setup
  WiFi.begin(network_ssid, network_password);

  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // Start TCP server
  server.begin();
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    while (client.connected()) {
      // receiving data from STM32 if available and sending to ESP32
      while (Serial.available() > 0) {
        uint8_t stm32_received = Serial.read();
        client.write(stm32_received);
      }

      // receiving data by ESP32 and sending to STM32
      while (client.available()) {
        int gs_received = client.read();
        if (gs_received >= 0 && gs_received <= 13) {
          Serial.write((uint8_t)gs_received);
        }
      }
    }
    client.stop();
  }
}

