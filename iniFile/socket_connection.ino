#include <ArduinoJson.h>      // Json Library to encode data
#include <WiFi.h>             // ESP32 WiFi Library
#include <WebServer.h>        // WebServer Library for ESP32
#include <WebSocketServer.h>  // WebSocket Library for WebSocket

#define N_FINGER 5

const String finger_names[N_FINGER] = {"first","second","third","fourth","fifth"};
const uint8_t json_doc_size = JSON_OBJECT_SIZE(N_FINGER+2);
/*
const char* ssid = "smart_glove";
const char* password =  "smart_glove";
*/
const bool debug = false;


const char* ssid = "Galaxy A7 (2018)770E";
const char* password =  "samuele1997";

StaticJsonDocument<json_doc_size> json_data;

int analogPin = A2;

WiFiServer server(80);
WebSocketServer webSocketServer;

void setup() {

  Serial.begin(9600);

  //WiFi.mode(WIFI_AP);
  //WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  WiFi.begin(ssid, password);

  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());

  server.begin();
  delay(100);
}

void loop() {
  Serial.println("Inizio ciclo");
  WiFiClient client = server.available();

  if (client.connected() && webSocketServer.handshake(client))
  {
    Serial.println("Connected to websocket client");
    int k = 0;
    while (client.connected()) {
      //data = webSocketServer.getData();

        //Serial.println(data);
        /* Iterate finger name, sample, and fill data field */
        
        for (int finger = 0;  finger < N_FINGER ; finger++)
        {
          if(debug)
          {
            Serial.println(finger_names[finger]);          
          }  
          json_data[finger_names[finger]] = analogRead(analogPin);
        }
        
        String data_to_send; 
        serializeJson(json_data, data_to_send);
        webSocketServer.sendData(data_to_send);
        if(debug)
          Serial.println(String(data_to_send));
      
      Serial.println("Loop");
      delay(10); // Delay needed for receiving the data correctly
    }

    Serial.println("The client disconnected");
    delay(100);
  }

  delay(100);
}
