
#include <WiFi.h>
#include <ArduinoJson.h>

// use WEMOS D1 MINI ESP32

#define PROTOCOL_TCP
#define PROTOCOL_UDP
bool debug = false;

char esp_id[30] = { 0 };
char device_name[30] = "Nuro";
char device_id[30] = "no id";

// For STATION mode:
const char* ssid = "bst";                 // Your ROUTER SSID
const char* pw = "billd*ngN3roShooters";  // and WiFi PASSWORD

/*************************  COM Port 0 *******************************/
#define UART_BAUD 115200     // Baudrate UART0
#define DEBUG_TCP_PORT 8880  // Wifi Port UART0
#define ESP_TCP_PORT 8881    // Wifi Port UART0
#define STATUS_PORT 8888     //

#define BUFFER_SIZE 1024 * 10
#define MAX_NMEA_CLIENTS 4
//////////////////////////////////////////////////////////////////////////

#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer debug_server(DEBUG_TCP_PORT);
WiFiServer esp_server(ESP_TCP_PORT);
WiFiClient debug_clients[MAX_NMEA_CLIENTS];
WiFiClient esp_clients[MAX_NMEA_CLIENTS];
#endif

#ifdef PROTOCOL_UDP
#include <WiFiUdp.h>
WiFiUDP udp;
IPAddress broadcastIp(10, 0, 0, 255);
#endif

uint8_t buf1[BUFFER_SIZE];
uint16_t i1 = 0;

char buf2[BUFFER_SIZE];
uint16_t i2 = 0;



void broadcast_status(char* status);

void esp_uart_message(String msg);
void debug_uart_message(String msg);

HardwareSerial mySerial0(0);
HardwareSerial mySerial1(1);


void setup() {
  /*
  delay(5000);

  Serial.begin(UART_BAUD);
  Serial.println("Serial0 initialized");
  Serial.setRxBufferSize(BUFFER_SIZE);
  Serial.println("Serial0 BUFFER_SIZE");
  Serial.setPins(21, 22, -1, -1);
  Serial.println("Serial0 pins changed");
  delay(5000);
  Serial1.begin(UART_BAUD);
  Serial.println("Serial1 initialized");
  Serial1.setRxBufferSize(BUFFER_SIZE);
  Serial.println("Serial1 BUFFER_SIZE");
  Serial.println("Serial1 pins changed");
  Serial1.setPins(5, 7, -1, -1);
*/

  mySerial0.setRxBufferSize(BUFFER_SIZE);
  mySerial0.begin(UART_BAUD, SERIAL_8N1, 21, 22);

  mySerial1.setRxBufferSize(BUFFER_SIZE);
  mySerial1.begin(UART_BAUD, SERIAL_8N1, 32, 10);



  Serial2.setRxBufferSize(BUFFER_SIZE);
  Serial2.begin(UART_BAUD);

  if (debug) mySerial0.println("\n\nLK8000 WiFi serial bridge V1.00");

  if (debug) mySerial0.println("Open ESP Station mode");
  // STATION mode (ESP connects to router and gets an IP)
  // Assuming phone is also connected to that router
  // from RoboRemo you must connect to the IP of the ESP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  String espString = WiFi.macAddress();
  espString.toCharArray(esp_id, 30);

  if (debug) mySerial0.println("try to Connect to Wireless network: ");
  if (debug) mySerial0.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (debug) mySerial0.print(".");
  }
  if (debug) mySerial0.println("\nWiFi connected");
  if (debug) mySerial0.println(esp_id);
  if (debug) mySerial0.println(WiFi.localIP());



  if (debug) mySerial0.println("Starting Debug Server");
  debug_server.begin();  // start TCP server
  debug_server.setNoDelay(true);

  if (debug) mySerial0.println("Starting ESP Server");
  esp_server.begin();  // start TCP server
  esp_server.setNoDelay(true);

  if (debug) mySerial0.println("Starting UDP Server 1");
  udp.begin(STATUS_PORT);  // start UDP server

  broadcast_status("restart");
}

#define BROADCAST_PERIOD_MS 1000
uint32_t time_since_of_broadcast_ms = 0;
void loop() {

  if (millis() - time_since_of_broadcast_ms > BROADCAST_PERIOD_MS) {
    if (debug) mySerial0.println("broadcast IP");
    broadcast_status("running");
    time_since_of_broadcast_ms = millis();
  }

  if (debug_server.hasClient()) {
    for (byte i = 0; i < MAX_NMEA_CLIENTS; i++) {
      //find free/disconnected spot
      if (!debug_clients[i] || !debug_clients[i].connected()) {
        if (debug_clients[i]) debug_clients[i].stop();
        debug_clients[i] = debug_server.available();
        if (debug) mySerial0.print("New client for Debug Serial");
        if (debug) mySerial0.print(0);
        if (debug) mySerial0.print('/');
        if (debug) mySerial0.println(i);
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = debug_server.available();
    TmpserverClient.stop();
  }

  if (esp_server.hasClient()) {
    for (byte i = 0; i < MAX_NMEA_CLIENTS; i++) {
      //find free/disconnected spot
      if (!esp_clients[i] || !esp_clients[i].connected()) {
        if (esp_clients[i]) esp_clients[i].stop();
        esp_clients[i] = esp_server.available();
        if (debug) mySerial0.print("New client for ESP Serial");
        if (debug) mySerial0.print(0);
        if (debug) mySerial0.print('/');
        if (debug) mySerial0.println(i);
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = esp_server.available();
    TmpserverClient.stop();
  }


  // read debug inputs from debug TCP clients and write to debug uart
  for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++) {
    if (debug_clients[cln]) {
      while (debug_clients[cln].available()) {
        buf1[i1] = debug_clients[cln].read();  // read char from client (LK8000 app)
        if (i1 < BUFFER_SIZE - 1) {
          i1++;
        }
      }

      mySerial0.write(buf1, i1);  // now send to UART(num):
      i1 = 0;
    }
  }

  // send debug serial data to debug TCP clients
  if (mySerial0.available())
  {
    memset(buf2, 0, sizeof(buf2));
    int index = 0;
    while (mySerial0.available() && index < BUFFER_SIZE - 1)
    {
      char byte = (char)mySerial0.read();
      buf2[index] = byte;  // read char from UART(num)

      if(index > 0 && buf2[index] == '\n' && buf2[index-1] == '\r')
      {
        debug_uart_message(String(buf2));
        memset(buf2, 0, sizeof(buf2));
        index = 0;
      }
      else
      {
        index++;
      }
    }
    if(strlen(buf2) > 0)
    {
      debug_uart_message(String(buf2));
    }
    
  }

  // send debug serial data to debug TCP clients
  if (mySerial1.available())
  {
    memset(buf2, 0, sizeof(buf2));
    int index = 0;
    while (mySerial1.available() && index < BUFFER_SIZE - 1)
    {
      char byte = (char)mySerial1.read();
      buf2[index] = byte;  // read char from UART(num)

      if(index > 0 && buf2[index] == '\n' && buf2[index-1] == '\r')
      {
        esp_uart_message(String(buf2));
        memset(buf2, 0, sizeof(buf2));
        index = 0;
      }
      else
      {
        index++;
      }
      
    }
    if(strlen(buf2) > 0)
    {
      esp_uart_message(String(buf2));
    }
    
  }

  // send debug serial data to debug TCP clients
  if (Serial2.available())
  {
    memset(buf2, 0, sizeof(buf2));
    int index = 0;
    while (Serial2.available() && index < BUFFER_SIZE - 1)
    {
      char byte = (char)Serial2.read();
      buf2[index] = byte;  // read char from UART(num)

      if(index > 0 && buf2[index] == '\n' && buf2[index-1] == '\r')
      {
        esp_uart_message(String(buf2));
        memset(buf2, 0, sizeof(buf2));
        index = 0;
      }
      else
      {
        index++;
      }
    }
    if(strlen(buf2) > 0)
    {
      esp_uart_message(String(buf2));
    }
    
  }
}


void esp_uart_message(String msg) {

  for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++) {
    if (esp_clients[cln]) {
      esp_clients[cln].print(msg);
    }
  }
}

void debug_uart_message(String msg) {

  for (byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++) {
    if (debug_clients[cln]) {
      debug_clients[cln].print(msg);
    }
  }
}

void broadcast_status(char* status) {
  char status_msg[256] = { 0 };
  sprintf(status_msg, "{\"type\":\"status\", \"status\":\"%s\", \"esp_id\":\"%s\", \"name\":\"%s\", \"device_id\":\"%s\", \"ports\":{\"debug\":%d, \"esp\":%d}}", status, esp_id, device_name, device_id, DEBUG_TCP_PORT, ESP_TCP_PORT);
  udp.beginPacket(broadcastIp, STATUS_PORT);
  udp.write((const uint8_t*)status_msg, strlen(status_msg));
  udp.endPacket();
}