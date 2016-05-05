

//#define NDEBUG                        // enable local debugging information

#define SKETCH_NAME "RGB LedStrip"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"
#define NODE_ID 29 //or AUTO to let controller assign   


// Load mysensors library	
#include "MySensor.h"	
// Load Serial Peripheral Interface library  
#include <SPI.h>
#include <Encoder.h>
#include <Bounce2.h>
#include <RGBConverter.h>


double hsv[3];
double savedhsv[3];


// Arduino pin attached to driver pins
#define RED_PIN 3 
#define WHITE_PIN 9	// this is not a pwm pin! change it if you want pwm
#define GREEN_PIN 6
#define BLUE_PIN 5
#define NUM_CHANNELS 3 // how many channels, RGBW=4 RGB=3...

#define SENSOR_ID 30
#define REBOOT_CHILD_ID                       100

// Smooth stepping between the values
#define STEP 1
#define INTERVAL 3 // 10
const int pwmIntervals = 255; //255;
float R; // equation for dimming curve


#define KNOB_ENC_PIN_1 14    // Rotary encoder input pin 1 (A0)
#define KNOB_ENC_PIN_2 7    // Rotary encoder input pin 2
#define KNOB_BUTTON_PIN 8   // Rotary encoder button pin 

#define KNOBCOLOR_ENC_PIN_1 16    // Rotary encoder input pin 1 (A0)
#define KNOBCOLOR_ENC_PIN_2 18    // Rotary encoder input pin 2
#define KNOBCOLOR_BUTTON_PIN 15   // Rotary encoder button pin 

#define NOCONTROLLER_MODE_PIN 17

//#define FADE_DELAY 10       // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim)
#define SEND_THROTTLE_DELAY 400 // Number of milliseconds before sending after user stops turning knob

#define KNOBUPDATE_TIME 500

#define RADIO_RESET_DELAY_TIME 25 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения
#define DATASEND_DELAY  10
#define GWSTATUSCHECK_TIME 150000


boolean gotAck=false; //подтверждение от гейта о получении сообщения 

int iCount = MESSAGE_ACK_RETRY_COUNT;


Encoder knob(KNOB_ENC_PIN_2, KNOB_ENC_PIN_1);  
Encoder knobcolor(KNOBCOLOR_ENC_PIN_1, KNOBCOLOR_ENC_PIN_2);  

Bounce debouncer = Bounce(); 
Bounce debouncercolor = Bounce(); 
byte oldButtonVal;
byte oldColorButtonVal;
boolean whiteColor=false;

long lastencoderValue=0;
long lastcolorencoderValue=0;

boolean bGatewayPresent = true;
byte bNoControllerMode = HIGH;
unsigned long previousStatMillis = 0;

MySensor gw;	
   
// Stores the current color settings
byte channels[4] = {RED_PIN, GREEN_PIN, BLUE_PIN, WHITE_PIN};
byte values[4] = {100, 100, 100, 100};
byte target_values[4] = {100, 100, 100, 100}; 


// stores dimming level
byte dimming = 100;
byte target_dimming = 100;

// tracks if the strip should be on of off
boolean isOn = true;

// time tracking for updates
unsigned long lastupdate = millis();
 

// update knobs changed
unsigned long lastKnobsChanged;
boolean bNeedToSendUpdate = false;
Bounce debouncernocontroller = Bounce(); 

RGBConverter RGBConv;  

MyMessage msgLedStripStatus(SENSOR_ID, V_STATUS);
MyMessage msgLedStripColor(SENSOR_ID, V_RGBW);

void setup() 
{

  // Set knob button pin as input (with debounce)
  pinMode(KNOB_BUTTON_PIN, INPUT);
  digitalWrite(KNOB_BUTTON_PIN, HIGH);
  debouncer.attach(KNOB_BUTTON_PIN);
  debouncer.interval(5);
  oldButtonVal = debouncer.read();

  pinMode(KNOBCOLOR_BUTTON_PIN, INPUT);
  digitalWrite(KNOBCOLOR_BUTTON_PIN, HIGH);  
  debouncercolor.attach(KNOBCOLOR_BUTTON_PIN);
  debouncercolor.interval(5);
  oldColorButtonVal = debouncercolor.read();

    pinMode(NOCONTROLLER_MODE_PIN,INPUT);
    digitalWrite(NOCONTROLLER_MODE_PIN,HIGH);
    debouncernocontroller.attach(NOCONTROLLER_MODE_PIN);
    debouncernocontroller.interval(5);
    bNoControllerMode = debouncernocontroller.read();

  // Set all channels to output (pin number, type)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(channels[i], OUTPUT);
  }

  // set up dimming
  R = (pwmIntervals * log10(2))/(log10(255));


   if (bNoControllerMode != LOW)
   {

  // Initializes the sensor node (with callback function for incoming messages)
  gw.begin(incomingMessage, NODE_ID, false);	// 123 = node id for testing	
       
  // Present sketch (name, version)			
   gw.sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER"."SKETCH_MINOR_VER);      
  // Register sensors (id, type, description, ack back)
  gw.present(SENSOR_ID, S_RGBW_LIGHT, "RGBW test light");

        //reboot sensor command
    gw.wait(RADIO_RESET_DELAY_TIME);
    gw.present(REBOOT_CHILD_ID, S_BINARY); //, "Reboot node sensor", true); 




    gw.wait(RADIO_RESET_DELAY_TIME);
    gw.request(SENSOR_ID, V_RGBW);
  	gw.wait(1000); 


    gw.wait(RADIO_RESET_DELAY_TIME);
    gw.request(SENSOR_ID, V_STATUS);

  }
  else
  {

    inputToRGBW("029377"); 

  }

TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); //disable FastPWM on Timer0. mills lasts twice longer

  // init lights
  updateLights();
 
  #ifdef NDEBUG 
  // debug
  if (isOn) {
    Serial.println("RGBW is running...");
  }
 
  Serial.println("Waiting for messages...");  
  #endif	


  wdt_enable(WDTO_8S);

}

void loop()
{


  debouncernocontroller.update();
  byte switchState = debouncernocontroller.read();

  if (switchState != bNoControllerMode) 
  {
      #ifdef NDEBUG
          Serial.print("Controller mode ");
          Serial.println(switchState);          
        #endif
      
      bNoControllerMode = switchState;
      wdt_enable(WDTO_30MS);
        while(1) {};
  }


     if (bNoControllerMode != LOW  && bGatewayPresent )
     { 
        // Process incoming messages (like config and light state from controller) - basically keep the mysensors protocol running
        gw.process();		
      }

  // and set the new light colors
  if (millis() > lastupdate + INTERVAL) {
    updateLights();
    lastupdate = millis();
  } 

checkButtonClick();

checkColorButtonClick();

checkRotaryEncoder();

checkRotaryEncoderColor();

updateBrightness();

checkGatewayStatus();

    //reset watchdog timer
    wdt_reset();   

}

// callback function for incoming messages
void incomingMessage(const MyMessage &message) {

  // acknoledgment
  if (message.isAck())
  {
    gotAck = true;
    return;
  }
 
    if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true && strlen(message.getString())>0 ) 
    {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }

  // new dim level
  else if (message.type == V_DIMMER) {
      target_dimming = message.getByte();
  }

  // on / off message
  else if (message.type == V_STATUS) {

    isOn = message.getInt();

    #ifdef NDEBUG
    if (isOn) {
      Serial.println("on");
    } else {
      Serial.println("off");
    }
    #endif
  }

  // new color value
  else if (message.type == V_RGBW) {    
    const char * rgbvalues = message.getString();
    inputToRGBW(rgbvalues);   


  }  
}

// this gets called every INTERVAL milliseconds and updates the current pwm levels for all colors
void updateLights() {  


  // for each color
  for (int v = 0; v < NUM_CHANNELS; v++) {

    if (values[v] < target_values[v]) {
      values[v] += STEP;
      if (values[v] > target_values[v]) {
        values[v] = target_values[v];
      }
    }

    if (values[v] > target_values[v]) {
      values[v] -= STEP;
      if (values[v] < target_values[v]) {
        values[v] = target_values[v];
      }
    }
  }

  // dimming
  if (dimming < target_dimming) {
    dimming += STEP;
    if (dimming > target_dimming) {
      dimming = target_dimming;
    }
  }
  if (dimming > target_dimming) {
    dimming -= STEP;
    if (dimming < target_dimming) {
      dimming = target_dimming;
    }
  }



  // set actual pin values
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (isOn) {

      // normal fading
       //analogWrite(channels[i], dimming / 100 * values[i]);
      // non linear fading, idea from https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
      analogWrite(channels[i], pow (2, (values[i] / R)) - 1);
    } else {
      analogWrite(channels[i], 0);
    }
  }
}

// converts incoming color string to actual (int) values
// ATTENTION this currently does nearly no checks, so the format needs to be exactly like domoticz sends the strings
void inputToRGBW(const char * input) {


  if (strlen(input) == 6) {

    target_values[0] = fromhex (& input [0]);
    target_values[1] = fromhex (& input [2]);
    target_values[2] = fromhex (& input [4]);
    target_values[3] = 0;


  } else if (strlen(input) == 9) {
    Serial.println("new rgbw value");
    target_values[0] = fromhex (& input [1]); // ignore # as first sign
    target_values[1] = fromhex (& input [3]);
    target_values[2] = fromhex (& input [5]);
    target_values[3] = fromhex (& input [7]);
  } else {
   // Serial.println("Wrong length of input");
  }  

    RGBConv.rgbToHsv(target_values[0], target_values[1], target_values[2], hsv);


   
   lastencoderValue = hsv[2]*100;

   if (lastencoderValue < 18)
   {
    lastencoderValue = 18;
   }
  
   if (lastencoderValue > 100)
   {
    lastencoderValue = 100;
   }

  knob.write(lastencoderValue);



lastcolorencoderValue = hsv[0]*100;     

   if (lastcolorencoderValue < 0)
   {
    lastcolorencoderValue = 0;
   }
  
   if (lastcolorencoderValue > 100)
   {
    lastcolorencoderValue = 100;
   }

knobcolor.write(lastcolorencoderValue);


  #ifdef NDEBUG 
  Serial.print("New encoder value: ");
  Serial.println(lastencoderValue);

  Serial.print("New color values: ");
  Serial.println(input);
  #endif


}

// converts hex char to byte
byte fromhex (const char * str)
{
  char c = str [0] - '0';
  if (c > 9)
    c -= 7;
  int result = c;
  c = str [1] - '0';
  if (c > 9)
    c -= 7;
  return (result << 4) | c;
}





  void checkButtonClick() {
  debouncer.update();
  byte buttonVal = debouncer.read();
  byte newLevel = 0;
  if (buttonVal != oldButtonVal && buttonVal == LOW) {

  	if (isOn)
  	{
  		isOn = false;
  	}
  	else
  	{
  		isOn = true;
  	}

     if (bNoControllerMode != LOW  && bGatewayPresent )
     {
     		   			//Отсылаем состояние переключателя
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            gw.send(msgLedStripStatus.set(isOn?"1":"0"), true);
			                    gw.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

          if ( iCount == 0 )
          {

              bGatewayPresent = false;

                  #ifdef NDEBUG 
                  Serial.println("No gateway present"); 
                  #endif 
          }
                 
      }

  }
  oldButtonVal = buttonVal;
}



  void checkColorButtonClick() {
  debouncercolor.update();
  byte buttonVal = debouncercolor.read();
  byte newLevel = 0;
  if (buttonVal != oldColorButtonVal && buttonVal == LOW) {

  #ifdef NDEBUG 
  Serial.println(buttonVal);
  #endif

  if ( !whiteColor )
  {
    whiteColor= true;
 
     savedhsv[0] = hsv[0];
     savedhsv[1] = hsv[1];
     //savedhsv[2] = hsv[2];
     savedhsv[2] = (double)lastcolorencoderValue;

     hsv[0]=0.0;
     hsv[1]=0.0;
  }
  else
  {
    whiteColor= false;
 
     hsv[0] = savedhsv[0];
     hsv[1] = savedhsv[1];
     lastcolorencoderValue = savedhsv[2] * 100 /100;
     knobcolor.write(lastcolorencoderValue);

  }

 RGBConv.hsvToRgb(hsv[0], hsv[1], hsv[2], target_values);

  }
  oldColorButtonVal = buttonVal;
}


void checkRotaryEncoder() {
  long encoderValue = knob.read();

  if (encoderValue != lastencoderValue)
  {
        #ifdef NDEBUG 
  			Serial.print("Encoder value: ");
  			Serial.println(encoderValue);
        #endif

  			lastencoderValue=encoderValue;



   hsv[2]=(double)encoderValue/100.0;



	RGBConv.hsvToRgb(hsv[0], hsv[1], hsv[2], target_values);

	lastKnobsChanged = millis();
	bNeedToSendUpdate = true;


  }

  if (encoderValue > 100) {
    encoderValue = 100;
    knob.write(100);
  } else if (encoderValue < 18) {
    encoderValue = 18;
    knob.write(18);
  }

 
}

void checkRotaryEncoderColor() {
  long encoderValue = knobcolor.read();

  if (encoderValue != lastcolorencoderValue && !whiteColor)
  {

        #ifdef NDEBUG 
        Serial.print("Encoder color value: ");
        Serial.println(encoderValue);
        lastcolorencoderValue=encoderValue;
        #endif


   hsv[0]=(double)encoderValue/100.0;
   hsv[1] = 1;


  RGBConv.hsvToRgb(hsv[0], hsv[1], hsv[2], target_values);

  lastKnobsChanged = millis();
  bNeedToSendUpdate = true;

  }

  if (encoderValue > 100) {
    encoderValue = 0;
    knobcolor.write(0);
  } else if (encoderValue < 0) {
    encoderValue =100;
    knobcolor.write(100);
  }

 
}


void updateBrightness()
{

 unsigned long currentTempMillis = millis();
    if((currentTempMillis - lastKnobsChanged ) > KNOBUPDATE_TIME && bNeedToSendUpdate)

	{

			bNeedToSendUpdate = false;

      #ifdef NDEBUG 
      Serial.print("H: ");
      Serial.print(hsv[0]);
      Serial.print(" S: ");
      Serial.print(hsv[1]);
      Serial.print(" V: ");
      Serial.println(hsv[2]);    
      #endif
 
 sendColorState();

	}

}


void sendColorState()
{

double iHue = modifiedMap(hsv[0], 0.0, 1.0, 0.0, 360.0);     
 double iSat = modifiedMap(hsv[1], 0.0, 1.0, 0.0, 100.0); 
 double iBr = modifiedMap(hsv[2], 0.0, 1.0, 0.0, 100.0); 

char cMsg[20];
char cH[8];
char cS[8];
char cB[8];


      #ifdef NDEBUG 
      Serial.print("H1: ");
      Serial.print(iHue);
      Serial.print(" S1: ");
      Serial.print(iSat);
      Serial.print(" V1: ");
      Serial.println(iBr); 
      #endif


dtostrf(iHue, 4, 2, cH);
dtostrf(iSat, 4, 2, cS);
dtostrf(iBr, 4, 2, cB);


sprintf(cMsg,"%s-%s-%s",cH,cS,cB);
      //Serial.println(cMsg); 

     if (bNoControllerMode != LOW  && bGatewayPresent )
     {      

                  iCount = MESSAGE_ACK_RETRY_COUNT;

                    while( !gotAck && iCount > 0 )
                      {
            
                      gw.send(msgLedStripColor.set(cMsg), true);
                       gw.wait(RADIO_RESET_DELAY_TIME);
                        iCount--;
                       }

                      gotAck = false;    

          if ( iCount == 0 )
          {

              bGatewayPresent = false;
                  #ifdef NDEBUG 
                  Serial.println("No gateway present"); 
                  #endif 
          }

      }                  

}


double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max)
{
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 temp = (int) (4*temp + .5);
 return (double) temp/4;
}


void checkGatewayStatus()
{

if ( bNoControllerMode != LOW )
{
    unsigned long currentStatMillis = millis();
    if((currentStatMillis - previousStatMillis ) > GWSTATUSCHECK_TIME ) 
      {


        // Save the current millis
        previousStatMillis = currentStatMillis;

                //Отсылаем состояние переключателя
                  iCount = MESSAGE_ACK_RETRY_COUNT;

                    while( !gotAck && iCount > 0 )
                      {
            
                        gw.send(msgLedStripStatus.set(isOn?"1":"0"), true);
                          gw.wait(RADIO_RESET_DELAY_TIME);
                        iCount--;
                       }

                      gotAck = false; 


      if ( iCount > 0 )
      {

          bGatewayPresent = true;
            #ifdef NDEBUG 
            Serial.println("Gateway present"); 
            #endif          

      }
      else
      {

          bGatewayPresent = false;
            #ifdef NDEBUG 
            Serial.println("No gateway present"); 
            #endif   
      }

      }


}
}
