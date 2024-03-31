#include <WiFi.h>
#include <ThingsBoard.h>
#include <PubSubclient.h>
int RelayPin = 18;

WiFiClient wifiClient;

ThingsBoard tb(wifiClient);

const char* ssid = "Sudhir";
const char* password = "12345677";
const char* tbHost = "demo.thingsboard.io";
const char* tbToken = "Ec0h7Wkj3yL4QkB1zYk6";

void setup() {
  pinMode(RelayPin, OUTPUT);
 
  Serial.begin(115200); 
  Serial.println("Connecting to Wi-Fi...");
 
 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }

  Serial.println("Connected to Wi-Fi");
  Serial.println("Connecting to ThingsBoard...");
  if (!tb.connect(tbHost, tbToken)) {
    Serial.println("Failed to connect to ThingsBoard");
    while (1);
  }

  Serial.println("Connected to ThingsBoard");
}

void loop() {
   int moist_value = analogRead(35);
   int moisture = map(moist_value,4095,25,0,100);
   if (moisture >=30)
  {
   digitalWrite(RelayPin,HIGH);  
  }
  else
  {
    digitalWrite(RelayPin,LOW); 
  }
  
  Serial.print("Soil moisture: ");
  Serial.print(moisture);
  Serial.println("%");
  tb.sendTelemetryFloat("Soil moisture", moisture);
  delay(1000);
}
