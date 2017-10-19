#include <WiFi.h>
#include <PubSubClient.h>
#include <String.h>
#include <stdio.h>
#include <driver/gpio.h>
#include <driver/adc.h>


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
int pulsos=0;
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
/// declaracion **********  ACS*****////////
/*
Measuring AC Current Using ACS712
*/
//const int sensorIn = A0;
double mVperAmp = 0.185;//185.0; // use 100 for 20A****66 for 30A **** 185 for 5A
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
int muestrasPromedio=0;
const double vEsc = 3.3/4096.0 ;
const float minUmbral =0.020;
const float correccion =0.020;
////******** FIN ACS712////////

/////////////******* LED TOUCH /////////

/*Touch Sensor Pin Layout
   T0 = GPIO4
   T1 = GPIO0
   T2 = GPIO2
   T3 = GPIO15
   T4 = GPIO13
   T5 = GPIO12
   T6 = GPIO14
   T7 = GPIO27
   T8 = GPIO33
   T9 = GPIO32 */
 /*  
int ledTouch = 2;
int SensorTactil=32;//T9

int buff(int pin){

  int out = (40 - touchRead(pin));                         //  Scale by n, value very sensitive currently
  // change to adjust sensitivity as required
  if (out > 0 )
  {
    return (out + 2);
  }
  else
  {
    return 0;                                        //Else, return 0
  }

}
*/
int threshold = 3;
bool touch1detected = false;

void gotTouch1(){
 touch1detected = true;
 
}
/////////*****  FIN LED TUCH


//////   *********   RGB    **************
const int led_green=25;
const int led_blue=26;
const int led_red=27;
static boolean control_RGB =false;
static int velocidad = 20;

const int analogPin = 35;  // Analog input pin 
int sensorValue = 0;        // value read from the adc
#define ADC1_TEST_CHANNEL (7)
#define BUFFER_SIZE 100


// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0 //verde
#define LEDC_CHANNEL_1     1 // azul
#define LEDC_CHANNEL_2     2 // rojo
#define LEDC_CHANNEL_3     3 // redBuilIn= 2
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

void task_RGB_1( void * parameter ){
 
 while(control_RGB){
  
  for (int fadeValue = 0 ; fadeValue < 255; fadeValue += 1) {
    // sets the value (range from 0 to 255):
      ledcWrite(LEDC_CHANNEL_0, fadeValue);//verde
      if(control_RGB==false){
        break;
        }
    // wait for 30 milliseconds to see the dimming effect
     delay(velocidad);
  }
  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue > 0; fadeValue -= 1) {
     ledcWrite(LEDC_CHANNEL_0, fadeValue);//verde
      if(control_RGB==false){
        break;
        }
     delay(velocidad);
  }  
   
  for (int fadeValue = 0 ; fadeValue < 255; fadeValue += 1) {
    // sets the value (range from 0 to 255):
      ledcWrite(LEDC_CHANNEL_1, fadeValue);//verde
    // wait for 30 milliseconds to see the dimming effect
     if(control_RGB==false){
        break;
        }
    delay(velocidad);
  }
  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue > 0; fadeValue -= 1) {
     ledcWrite(LEDC_CHANNEL_1, fadeValue);//verde
      if(control_RGB==false){
        break;
        }
     delay(velocidad);
  }  
    
  for (int fadeValue = 0 ; fadeValue < 255; fadeValue += 1) {
    // sets the value (range from 0 to 255):
      ledcWrite(LEDC_CHANNEL_2, fadeValue);//verde
    // wait for 30 milliseconds to see the dimming effect
     if(control_RGB==false){
        break;
        }
    delay(velocidad);
  }
  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue > 0; fadeValue -= 1) {
     ledcWrite(LEDC_CHANNEL_2, fadeValue);//verde
      if(control_RGB==false){
        break;
        }
    delay(velocidad);
  }  
 
  if(control_RGB==false){
        break;
        }
 }
 
 vTaskDelete( NULL );
}

void task_RGB_2( void * parameter ){
 
 vTaskDelete( NULL );
}

void task_RGB_3( void * parameter ){
 
 while(control_RGB){
    
    ledcWrite(LEDC_CHANNEL_0, 240);//verde
    ledcWrite(LEDC_CHANNEL_1, 240);//azul
    ledcWrite(LEDC_CHANNEL_2, 240);//rojo

    delay(velocidad*10);
     if(control_RGB==false){
        break;
        }

    ledcWrite(LEDC_CHANNEL_0, 0);//verde
    ledcWrite(LEDC_CHANNEL_1, 0);//azul
    ledcWrite(LEDC_CHANNEL_2, 0);//rojo

    delay(velocidad*10);
    
 }
 vTaskDelete( NULL );

}

void task_ADC( void * parameter ){
  
  while(1){
    // Serial.println("valor medio " + String(analogRead(analogPin)));
    Serial.println("****************************************");
    Serial.print("Valor Escalon 3.3 / 4096.0 : ");Serial.println(vEsc,8);
    float valor = TrueRMSMuestras();
    AmpsRMS=valor/mVperAmp - correccion;
    Serial.print("AmpsRMS medido: "); Serial.println(AmpsRMS,3);
    if(AmpsRMS < minUmbral){  AmpsRMS=0;}  
    String Potencia = String(220*AmpsRMS);
    String Corriente = String(AmpsRMS);
    Serial.print("AmpsRMS RMS: "); Serial.println(AmpsRMS,3);
    Serial.println("POWER RMS: " + String(Potencia));
    client.publish("casa/adc/potencia", (char*)Potencia.c_str());
    client.publish("casa/adc/corriente", (char*)Corriente.c_str());
    //vTaskDelay(3000/portTICK_PERIOD_MS);
    delay(3000);
  }
}

void IRAM_ATTR Dimmer(){
  timerStop(timer);
  digitalWrite(pin_controlDrimer,HIGH);
}

void setup() {
  
  Serial.begin(115200);
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
  client.subscribe("casa/rgb/red");
  client.subscribe("casa/rgb/green");
  client.subscribe("casa/rgb/blue");
  client.subscribe("casa/rgb/secuencia");
  client.subscribe("casa/rgb/velocidad");
  
  //////////////////////////////////////////
   
  pinMode(pin_controlDrimer, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_blue, OUTPUT);

  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);// verde
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);// azul
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);// rojo
  
  ledcAttachPin(led_green, LEDC_CHANNEL_0);
  ledcAttachPin(led_blue, LEDC_CHANNEL_1);
  ledcAttachPin(led_red, LEDC_CHANNEL_2);

  xTaskCreate( task_ADC,"ADC",10000,NULL,1,NULL);
  
  //touchAttachInterrupt(T8, gotTouch1, threshold);
 
  digitalWrite(pin_controlDrimer,LOW);
  
  pinMode(pin_zeroCross, INPUT);
  
  noInterrupts();
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &Dimmer, true);
  attachInterrupt(digitalPinToInterrupt(pin_zeroCross),zeroCross,RISING );
  interrupts(); 
}

void loop() {
  if(touch1detected){
    touch1detected = false;
    pulsos++;
    Serial.println("Tactil detectado: "+String(pulsos));
    
  }
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
      xTaskCreate( task1,"Task1",1000,NULL,2,NULL);
      }else{
         control=false;
        }
   }

  if(topicStr == "casa/rgb/red"){
    String dataSt = dato;
    int color =dataSt.toInt();
    
    Serial.println("rojo: "+ dataSt);
    ledcWrite(LEDC_CHANNEL_2,color );
   }

  if(topicStr == "casa/rgb/green"){
    String dataSt = dato;
    int color =dataSt.toInt();
    Serial.println("verde: "+ dataSt);
    ledcWrite(LEDC_CHANNEL_0, color);
  }
   
  if(topicStr == "casa/rgb/blue"){
   String dataSt = dato;
   int color =dataSt.toInt();
   Serial.println("azul: "+ dataSt);
   ledcWrite(LEDC_CHANNEL_1,  color);
  }

  if(topicStr == "casa/rgb/velocidad"){
   String dataSt = dato;
   int vel =dataSt.toInt();
   Serial.println("velocidad: "+ dataSt);
   velocidad= vel;
  }

  if(topicStr == "casa/rgb/secuencia"){
   String dataSt = dato;
   int secuencia =dataSt.toInt();
   Serial.println("secuencia: "+ dataSt);

   if(payload[0]=='0'){
      control_RGB=false;
    
    }
  
   if(payload[0]=='1'){
      control_RGB=true;
      xTaskCreate( task_RGB_1,"RGB",1000,NULL,2,NULL);
      }
   
   if(payload[0]=='2'){
      control_RGB=true;
      xTaskCreate( task_RGB_2,"RGB",1000,NULL,2,NULL);
      }
     if(payload[0]=='3'){
      control_RGB=true;
      xTaskCreate( task_RGB_3,"RGB",1000,NULL,2,NULL);
      }
     
  }
      
 }

float TrueRMSMuestras(){
  
  double result =0,conv=0,Acumulador=0,suma=0;
 int readValue =0;             //value read from the sensor
 int Count =0;
 double promedio =0;
 int sumatoria =0;
 const int n = 320;
 int diferencia =0;
 double Vo =0;
 for(int i=0;i<n;i++){
    //vTaskDelay(1/portTICK_PERIOD_MS);
    delay(1);
    sumatoria = sumatoria + analogRead(analogPin);
  } 
 promedio=sumatoria/n;
 Vo=promedio*vEsc;
 Serial.println("Tension Promedio : "+String(Vo));
 uint32_t start_time = millis();
 while((millis()-start_time )< 320){   
     Count++;
     readValue =  analogRead(analogPin);
    
     conv=(readValue - promedio)*vEsc;
     Acumulador=Acumulador+sq(conv);  
     }
   suma=Acumulador/Count;
   result=sqrt(suma);
   Serial.println(String(n)+" Muestras Promedio : "+String(promedio));
  return result;
  }

