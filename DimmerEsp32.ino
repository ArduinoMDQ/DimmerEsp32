#include <WiFi.h>
#include <PubSubClient.h>
#include <String.h>
#include <stdio.h>

#define HOME "casa"
#define ID "esp-03"
#define SITE "living"
#define FLOOR "piso-0"
#define DEPARTMENT "dto-0"
#define MQTT_SERVER_WAN "giovanazzi.dynu.net"

////////////////////////////////////
const char* ssid_etb = "Consola";
const char* password_etb =  "tyrrenal";
const char* mqttServer_etb = "giovanazzi.dynu.net";
int mqttPort_etb = 8083;

int intentos_con=0;
boolean flag=true;

const char* ssid = "Red Virtual 2";
const char* password =  "2410meridian";
const char* mqttServer = "192.168.1.107";


int mqttPort = 1883;
const char* mqttUser = "diego";
const char* mqttPassword = "24305314";
static boolean control =false;

WiFiClient espClient;
PubSubClient client(espClient);

hw_timer_t * timer = NULL;

const int pin_controlDrimer = 14;
const int pin_zeroCross = 12;
String inString="";
volatile int porcentaje = 50;
volatile int timing;

//////   *********   RGB    **************
const int led_green=25;
const int led_blue=26;
const int led_red=27;

#define BUFFER_SIZE 100

int r = 0;
int g = 0;
int b = 0;

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0
#define LEDC_CHANNEL_1     1
#define LEDC_CHANNEL_2     2
// use 13 bit precission for LEDC timer
#define LEDC_TIMER_8_BIT  8

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000

int brightness = 0;    // how bright the LED is


/////////7 **********  FIN RGB ****************

void task1( void * parameter ){
 client.publish("casa/dimmerEsp32/latidos/confirm", "modo latidos ON");
 Serial.println(" modo latidos ON");    
 while(control){
 for (int fadeValue = 4 ; fadeValue < 50; fadeValue += 1) {
    // sets the value (range from 0 to 255):
     porcentaje= fadeValue;
    // wait for 30 milliseconds to see the dimming effect
    delay(50);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 49 ; fadeValue > 4; fadeValue -= 1) {
     porcentaje= fadeValue;
    delay(40);
  }  
 }
 client.publish("casa/dimmerEsp32/latidos/confirm", "modo latidos OFF");
 Serial.println(" modo latidos OFF");
 porcentaje=0;
 vTaskDelete( NULL );
}

void IRAM_ATTR Dimmer(){
  timerStop(timer);
  digitalWrite(pin_controlDrimer,HIGH);
}

void setup() {
  
  Serial.begin(115200);
  //////////////////  wifi mqtt 
  WiFi.begin(ssid, password);
  
  while ((WiFi.status() != WL_CONNECTED) && flag) {
      delay(1000);
     intentos_con ++;
    
      Serial.println("Connecting to WiFi .." + String(intentos_con));
    
     if (intentos_con==4){
   //    flag=false
        intentos_con=0;
        ssid=ssid_etb;
        password=password_etb;
        mqttServer=mqttServer_etb;
        mqttPort=mqttPort_etb;
        WiFi.begin(ssid, password);
         
      }
   }
   
  Serial.println("Connected to the WiFi network");
  Serial.println("mqttServer " + String(mqttServer));
  Serial.println("mqttPort " + String(mqttPort));
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(1000);
     }
  }
  client.publish("casa/dimmerEsp32/confirm", "Engancho");
  client.subscribe("casa/dimmerEsp32");
  client.subscribe("casa/dimmerEsp32/latidos");
  client.subscribe("casa/rgb");
  
  //////////////////////////////////////////
 
  
  pinMode(pin_controlDrimer, OUTPUT);
 
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(led_green, LEDC_CHANNEL_0);
  ledcAttachPin(led_blue, LEDC_CHANNEL_1);
  ledcAttachPin(led_red, LEDC_CHANNEL_2);
  
  digitalWrite(pin_controlDrimer,LOW);
  
  pinMode(pin_zeroCross, INPUT);
  
  noInterrupts();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &Dimmer, true);
  attachInterrupt(digitalPinToInterrupt(pin_zeroCross),zeroCross,RISING );
  interrupts(); 
}

void loop() {
  
  client.loop();
  
  while (Serial.available() > 0) {
    porcentaje=0;
     int inChar = Serial.read();
     if (isDigit(inChar)) {     
        inString += (char)inChar;
       }
    if (inChar == '\r') {
        porcentaje=inString.toInt();   
        Serial.println(porcentaje);
        inString = "";
       }
  }
}

void zeroCross(){
 pin_controlDrimer,LOW
 noInterrupts();
 controlPorcentaje();
 interrupts();
}

void controlPorcentaje(){

    if (porcentaje < 3){
        digitalWrite(pin_controlDrimer,LOW);
    }else{
        if (porcentaje > 97){
            digitalWrite(pin_controlDrimer,HIGH);
        }else{
            digitalWrite(pin_controlDrimer,LOW);
            timing = 10000 -(porcentaje*100);
            timerAlarmWrite(timer, timing, true);
            timerAlarmEnable(timer);
            timerRestart(timer);
        }
      }  

 }

void callback(char* topic, byte* payload, unsigned int length) {
  String topicStr = topic; 
  String dato ;
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
     dato += (char)payload[i];
  }
  Serial.println();
 if(topicStr == "casa/dimmerEsp32"){
     control=false;
     client.publish("casa/dimmerEsp32/confirm",(char*)dato.c_str());
     porcentaje=dato.toInt();
     Serial.println(" El dato es: "+String(porcentaje));  
  }

 if(topicStr == "casa/dimmerEsp32/latidos"){

     if(payload[0]=='1'){
      control=true;
      xTaskCreate( task1,"Task1",10000,NULL,1,NULL);
      }else{
         control=false;
        }
   }

  if(topicStr == "casa/rgb"){
    String dataSt = dato;
    r = dataSt.substring(dataSt.indexOf('(')+1).toInt();
    g = dataSt.substring(dataSt.indexOf(',')+1,dataSt.lastIndexOf(',')).toInt();
    b = dataSt.substring(dataSt.lastIndexOf(',')+1).toInt();
   Serial.println("red: "+String(r));
   Serial.println("green: "+String(g));
   Serial.println("blue: "+String(b));

     ledcWrite(LEDC_CHANNEL_0, g);
     ledcWrite(LEDC_CHANNEL_1, b);
    ledcWrite(LEDC_CHANNEL_2, r);
 
   }
 }


