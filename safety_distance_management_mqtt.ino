#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <SPI.h>


#include "Arduino_secrets.h"

//mosquitto_sub -h test.mosquitto.org -t arduino/topic (Mosquito code to acess topic data)
// Sensor pins
//#define sensorPower 7
#include <Servo.h>
#define SERVO_PIN   11  //servo to D11
#define LPT 2          
#define BUZZ_PIN  13  //buzzer to D13

#define IN1  7    //Right motor(K1/K2) direction Pin 7
#define IN2  8    //Right motor(K1/K2) direction Pin 8
#define IN3  9    //Left motor(K3/K4) direction Pin 9
#define IN4  10   //Left motor(K3/K4) direction Pin 10
#define ENA  5    //D5 connect to ENA PWM speed pin for Right motor(K1/K2)
#define ENB  6    //D6 connect to ENB PWM speed pin for Left motor(K3/K4)

#define Echo_PIN    2   // Ultrasonic Echo pin connect to D2
#define Trig_PIN    3   // Ultrasonic Trig pin connect to D3

#define TURN_TIME  10 
#define FAST_SPEED  30 
#define SPEED  100   
#define TURN_SPEED  30
#define BACK_SPEED1  40
#define BACK_SPEED2  50 
#define MOVE_TIME  10   

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                                // your network key Index number
int status = WL_IDLE_STATUS;                     // the Wifi radio's status

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "test.mosquitto.org";
int        port     = 1883;
const char topic[]  = "arduino/topic";
const char topic2[]  = "arduino/topic";
const char topic3[]  = "arduino/topic";

//set interval for sending messages (milliseconds)
const long interval = 8000;
unsigned long previousMillis = 0;

int count = 0;


//Value for storing distance
const int distancelimit = 15; //distance limit for obstacles in front           
int distance;
int safty_distance=distancelimit;
int state=0;
int numcycles = 0;
Servo head;



void printCurrentNet() {

  Serial.print("You're connected to : ");
  Serial.println(WiFi.SSID());
  byte bssid[6];
  WiFi.BSSID(bssid);
  long rssi = WiFi.RSSI();
  byte encryption = WiFi.encryptionType();
  Serial.println();
}



int watch(){
  long echo_distance;
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN,LOW);
  echo_distance=pulseIn(Echo_PIN,HIGH);
  echo_distance=echo_distance*0.01657; //how far away is the object in cm
  return round(echo_distance);
}


void go_Advance()  //motor rotate clockwise -->robot go ahead
{
  
  digitalWrite(IN4,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN2,HIGH );
  digitalWrite(IN1,LOW);
    set_Motorspeed(SPEED,SPEED);

}
void go_Back() //motor rotate counterclockwise -->robot go back
{

 
  
  digitalWrite(IN4,LOW);
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN2,LOW);
  digitalWrite(IN1,HIGH);
  set_Motorspeed(SPEED,SPEED);
 
  //motor reverse -->robot reverse for 0.5 sec
  delay(500);

  stop_Stop();
}

/*set motor speed */
void set_Motorspeed(int lspeed,int rspeed) //change motor speed
{
  analogWrite(ENB,lspeed);//lspeed:0-255
  analogWrite(ENA,rspeed);//rspeed:0-255   
}




void stop_Stop() //motor brake -->robot stop
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,LOW); 
  set_Motorspeed(0,0);
  //motor brake -->robot stop for 2 sec
  delay(2000);

}

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
//Set pins
     pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 
  pinMode(ENA, OUTPUT);  
  pinMode(ENB, OUTPUT);

  stop_Stop();//stop move
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN,INPUT); 
  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);  
 
  digitalWrite(Trig_PIN,LOW);
  /*init servo*/
  head.attach(SERVO_PIN); 
  head.write(90);
  delay(500);
  


  if (WiFi.status() == WL_NO_MODULE) {

    Serial.println("Communication with WiFi module failed!");

    // don't continue

    while (true);

  }

  String fv = WiFi.firmwareVersion();

  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {

    Serial.println("Please upgrade the firmware");

  }

  // attempt to connect to Wifi network:

  while (status != WL_CONNECTED) {

    Serial.print("Attempting to connect to WEP network, SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, keyIndex, pass);

    // wait 1 seconds for connection:

    delay(1000);

  }

  // once you are connected :

  Serial.print("");
//  printCurrentNet();



 
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();


  
}








void loop() {

  // check the network connection once every 0.5 seconds:

  delay(500);
//  get state;
int dist=watch();
   Serial.println(dist);
if (dist<distancelimit) {
 digitalWrite(BUZZ_PIN, LOW);
  go_Back();
  state=-1;
 
  }
  else 
  { digitalWrite(BUZZ_PIN, HIGH);
    if ( dist> distancelimit && dist <100 )
     {go_Advance();    
     state=1;} 
    else  {stop_Stop() ;}
    
  }

 
 

  // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();


  


  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    //record value from sensors
    int Rvalue = dist;
    int Rvalue2 = state;
    int Rvalue3 = safty_distance;

    Serial.print("Distance: ");
    Serial.println();
    Serial.println(Rvalue);

    Serial.print("State: ");
    Serial.println();
    Serial.println(Rvalue2);

    Serial.print("Safety_distance: ");
    Serial.println();
    Serial.println(Rvalue3);

// print distance to serial monitor
 Serial.print("Distance: ");
  Serial.print(dist);

  
    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print("Distance: ");
    mqttClient.print(Rvalue);
    mqttClient.print("CM");
    mqttClient.endMessage();

    mqttClient.beginMessage(topic2);
    mqttClient.print("State: ");
    mqttClient.print(Rvalue2);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic3);
    mqttClient.print("Safety_distance: ");
    mqttClient.print(Rvalue3);
    mqttClient.print("CM");
    mqttClient.endMessage();

    Serial.println();
  }
}
