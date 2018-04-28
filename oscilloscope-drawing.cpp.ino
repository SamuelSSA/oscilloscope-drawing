#include <WiFi.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <driver/dac.h>

#include "image.h"

#define SMOOTHSTEP(x) ((x) * (x) * (3 - 2 * (x))) //SMOOTHSTEP expression.

const char* ssid = "TrojanHorse.zip";
const char* password = "31101994";
//const char* mqtt_server = "test.mosquitto.org";
const char* mqtt_server = "192.168.43.213";
//const char* mqtt_server = "10.11.0.101";
//const char* mqtt_server = "rpi-speak.local";

long int tempo_anterior = 0;
uint8_t XYZ_points[500][3];


int j = 0;             //Just an Iterator.
int i = 0;             //Just another Iterator.
float A = 0.0;         //Input Min Value
float B = 255.0;       //Input Max Value
float C = 0.0;         //Input Min Value
float D = 255.0;       //Input Max Value
int N = 1;       //Input number of steps for transition
float X;               //final smoothstepped value
float v;               //smoothstep expression variable


WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

const int TAM_IMG = (sizeof(image));

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  static int var=0;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  memcpy(XYZ_points, payload, length);
  //Serial.println(XYZ_points);
  Serial.println(length);

  var++;

  for(int i =0; i<(length/3); i++)
  {
    var = XYZ_points[i][0];
    Serial.println(var);
  }

  Serial.println(var);
  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    uint64_t chipid = ESP.getEfuseMac(); // MAC address of ESP32
    uint16_t chip = (uint16_t)(chipid>>32);
    char clientid[25];
    //snprintf(clientid,25,"WIFI-Display-%04X",chip);
    if (client.connect(clientid)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("Say", "-t 'hello world'");
      // ... and resubscribe
      client.subscribe("frame");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setOutput(uint8_t x, uint8_t y) {
  dac_out_voltage(DAC_CHANNEL_1, x); // chanel 1 X
  dac_out_voltage(DAC_CHANNEL_2, y); //channel 2 Y
}

void keepClient() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void setup()
{
 
  Serial.begin(921600);
  //setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(DAC_CHANNEL_1, OUTPUT);
  pinMode(DAC_CHANNEL_2, OUTPUT);
}

void loop() 
{

  double tick = 0;
  uint8_t x_out;
  uint8_t y_out = 0;
  int k,z;
  int contador1,contador2;

/*  //keepClient();
  while(1) {

    tick = tick + 1;
    double t = tick / 100.0;

    uint8_t x_out = (sin(t) + 1) * 100 + 10;
    uint8_t y_out = (cos(t) + 1) * 100 + 10;

    setOutput(x_out, y_out);
  }
*/
  for(k=0,z=1; ((k+2) < TAM_IMG) && ((z+2) < TAM_IMG); k+=2, z+=2)
  {   
    contador1++;

    while (j < N)                      // Keep looping until we hit the pre-defined max number                                   // of steps
    {
      contador2++;
        v = j / N;                    // Iteration divided by the number of steps.
        v = SMOOTHSTEP(v);            // Run the smoothstep expression on v.

        x_out = (image[k+2] * v) + (image[k] * (1 - v));          //smoothstep result.
        y_out = (image[z+2] * v) + (image[z] * (1 - v)); 

        //Serial.print("  ");        
        //Serial.println(x_out);            // prints the soothstepped value
        //Serial.println(y_out);

        dac_out_voltage(DAC_CHANNEL_1, x_out); // chanel 1 X
        dac_out_voltage(DAC_CHANNEL_2, -y_out);

        j++;                    
    }
    j=0;

    Serial.print("contador1:");
    Serial.println(contador1);

    Serial.print("contador2:");
    Serial.println(contador2);

  }

    /*while(1)
    {
      dac_out_voltage(DAC_CHANNEL_1, 0); // chanel 1 X
      dac_out_voltage(DAC_CHANNEL_2, 0);
    }
*/
  /*if((millis() - tempo_anterior) >= 1000)
  {
    tempo_anterior = millis();
   client.publish("display1", "-t 'samuel_test'");
  }
  */
} 
