#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// moiture sensor
#define AOUT_PIN A0           // Arduino pin that connects to AOUT pin of moisture sensor
#define moisture_powerpin D8  // Arduino pin that connects to AOUT pin of gas sensor
#define pump_powerpin D2      //
// #define light_sensor D4  //

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
      client.subscribe(topic);        //if i need to subscribe to topic
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

  // Create a JSON document
  DynamicJsonDocument doc(1024);

  // // Read the payload from the MQTT message
  deserializeJson(doc, payload, length);

  float temperature_percentage = doc["moisture"];
  String device_id = doc["deviceID"];
  Serial.println("in callback moisture eaqual");
  Serial.println(temperature_percentage);
  if (temperature_percentage < 50) {
    Serial.println("turning pump ON");

    digitalWrite(pump_powerpin, LOW);  //TURN POWER ON
    delay(1000);
    digitalWrite(pump_powerpin, HIGH);  //TURN POWER OFF
    Serial.println("turning pump OFF after delay");
  } else {
    Serial.println("turning pump OFF");
    digitalWrite(pump_powerpin, HIGH);  //TURN POWER OFF
  }
  //simplified form
  // if (temperature_percentage < 50) digitalWrite(pump_powerpin, HIGH);  //TURN POWER ON
  // else digitalWrite(pump_powerpin, LOW); //TURN POWER OFF
}

void publishMessage(const char *topic, String payload, boolean retained) {
  if (client.publish(topic, payload.c_str(), true)) Serial.println("Message published [" + String(topic) + "]: " + payload);
}

void setup() {
  //we need to add pin mode
  pinMode(BUILTIN_LED, OUTPUT);        // Initialize the BUILTIN_LED pin as an output
  pinMode(moisture_powerpin, OUTPUT);  // Initialize  pin as an output
  pinMode(pump_powerpin, OUTPUT);      // Initialize  pin as an output
                                       // pinMode(light_sensor, OUTPUT);  // Initialize  pin as an output

  //turn pump off
  digitalWrite(pump_powerpin, HIGH);  //TURN POWER of pump OFF


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
  moisture_percentage *= voltage / 3.3;  // Adjust for your actual voltage

  return moisture_percentage;
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  //record random value from A0, A1 and A2
  digitalWrite(moisture_powerpin, HIGH);  //TURN POWER ON
  // Serial.println("moisture sensor ON");
  delay(10);
  int Rvalue = analogRead(A0);
  digitalWrite(moisture_powerpin, LOW);  //TURN POWER OFF
  // Serial.println("moisture sensor OFF");
  // int Gas_value = analogRead(A0);
  //light sensor
  // int light_data = digitalRead(D4);


  float moisture_percentage = convert_to_moisture_percentage(Rvalue, 240, 1024, 3.3);

  DynamicJsonDocument doc(1024);
  doc["deviceID"] = "WeMOS d1";
  doc["sideID"] = "my demo project";
  doc["moisture"] = moisture_percentage;
  doc["moisture_raw"] = Rvalue;
  // doc["gas_sensor"] = Gas_value;
  // doc["light_sensor"] = light_data;

  char mqtt_message[128];
  serializeJson(doc, mqtt_message);
  publishMessage(topic, mqtt_message, true);

  //pump
  // digitalWrite(pump_powerpin, LOW); //TURN POWER ON
  // Serial.println("pump on");
  // delay(3000);
  // digitalWrite(pump_powerpin, HIGH); //TURN POWER OFF
  // Serial.println("pump off");

  delay(3000);
}