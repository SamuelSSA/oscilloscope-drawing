#include <WiFi.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <driver/dac.h>

#define SMOOTHSTEP(x) ((x) * (x) * (3 - 2 * (x))) //SMOOTHSTEP expression.
#define TAM_IMG

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
float N = 1000.0;       //Input number of steps for transition
float X;               //final smoothstepped value
float v;               //smoothstep expression variable


WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  
  Serial.begin(921600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(DAC_CHANNEL_1, OUTPUT);
  pinMode(DAC_CHANNEL_2, OUTPUT);
}

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

void loop() {

  double tick = 0;


  keepClient();
  while(1) {

    tick = tick + 1;
    double t = tick / 100.0;

    uint8_t x_out = (sin(t) + 1) * 100 + 10;
    uint8_t y_out = (cos(t) + 1) * 100 + 10;

    setOutput(x_out, y_out);
  }


  uint8_t x_out;
  uint8_t y_out = 0;

  //dac_out_voltage(DAC_CHANNEL_1, x_out); // chanel 1 X
  dac_out_voltage(DAC_CHANNEL_2, y_out); //channel 2 Y

/*  while(1)
  {
    dac_out_voltage(DAC_CHANNEL_1, 255*cos(2*PI)); //channel 2 Y
    dac_out_voltage(DAC_CHANNEL_2, 255*sin()); //channel 2 Y    
  }
*/

  if (j < N)                      // Keep looping until we hit the pre-defined max number 
                                  // of steps
  {
    v = j / N;                    // Iteration divided by the number of steps.
    v = SMOOTHSTEP(v);            // Run the smoothstep expression on v.
    x_out = (B * v) + (A * (1 - v));  // Run the linear interpolation expression using the current 
                                  //smoothstep result.
    for ( i=0; i < X ; i++)       // This loop could the relevant code for each time your 
                                  //motor steps. 
    {
      Serial.print("1");          //Prints the number "1" for each step.           
    }
    Serial.print("  ");           //Puts a space between each line of steps and their 
                                  //corresponding  float value
    //Serial.println(x_out);            // prints the soothstepped value

    //dac_out_voltage(DAC_CHANNEL_1, x_out); // chanel 1 X
  
    Serial.println("CLICK!!!");   // this could be where you trigger your timelapse shutter 
    j++;                          // Increments j by 1.
  }

  /*if((millis() - tempo_anterior) >= 1000)
  {
    tempo_anterior = millis();
   client.publish("display1", "-t 'samuel_test'");
  }
  */
} 
