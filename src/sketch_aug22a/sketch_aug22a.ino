#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// moiture sensor
#define AOUT_PIN A0  // Arduino pin that connects to AOUT pin of moisture sensor


// WiFi
const char *ssid = "xxx";  // Enter your WiFi name
const char *password = "xxx";    // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "192.168.1.2";
const char *topic = "esp8266";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;
unsigned long previousMillis = 0;
const long interval = 5000;


WiFiClient espClient;
PubSubClient client(espClient);

//setup wifi connection
void setup_wifi() {
  delay(3);
  // Serial.print("\nConnecting to ")
  Serial.printf("\nConnecting to:%s\n", ssid);
  // WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  // Serial.printf("Connected to the WiFi network with IP '%s'\n", WiFi.localIP());
  Serial.println("Connected to the WiFi network with IP");
  Serial.println(WiFi.localIP());
}

//Connect to mqtt
void reconnect() {
  while (!client.connected()) {
    String client_id = "esp8266-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Mqtt broker connected");
      client.subscribe("led_state");  //if i need to subscribe to topic
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

//callback to receivie msqs and do some logic work
void callback(char *topic, byte *payload, unsigned int length) {
  String incommingMessage = "";
  for (int i = 0; i < length; i++) incommingMessage += (char)payload[i];
  Serial.println("Message arrived [" + String(topic) + "]" + incommingMessage);
  if (strcmp(topic, "led_state") == 0) {
    if (incommingMessage.equals("1")) digitalWrite(BUILTIN_LED, LOW);  // Turn the LED on (Note that LOW is the voltage level
    else digitalWrite(BUILTIN_LED, HIGH);                              // Turn the LED off by making the voltage HIGH
  }
}

void publishMessage(const char *topic, String payload, boolean retained) {
  if (client.publish(topic, payload.c_str(), true)) Serial.println("Message published [" + String(topic) + "]: " + payload);
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);  // Initialize the BUILTIN_LED pin as an output

  // Set software serial baud to 115200;
  Serial.begin(115200);
  // connecting to a WiFi network
  setup_wifi();

  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
}

float convert_to_moisture_percentage(int sensor_value, int min_value, int max_value, float voltage) {
    int inverted_value = max_value - sensor_value;
    float moisture_percentage = ((float)inverted_value / (max_value - min_value)) * 100.0;

    // Adjust moisture percentage for voltage
    moisture_percentage *= voltage / 3.3; // Adjust for your actual voltage

    return moisture_percentage;
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  //record random value from A0, A1 and A2
  int Rvalue = analogRead(A0);
  float moisture_percentage = convert_to_moisture_percentage(Rvalue, 240, 1024, 3.3);
    
  DynamicJsonDocument doc(1024);
  doc["deviceID"] = "WeMOS d1";
  doc["sideID"] = "my demo project";
  doc["moisture"] = moisture_percentage;
  doc["moisture_raw"] = Rvalue;

  char mqtt_message[128];
  serializeJson(doc, mqtt_message);
  publishMessage(topic, mqtt_message, true);

  delay(3000);
}