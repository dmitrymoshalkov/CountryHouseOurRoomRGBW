/**
Based on the MySensors Project: http://www.mysensors.org

This sketch controls a (analog)RGBW strip by listening to new color values from a (domoticz) controller and then fading to the new color.

Version 0.9 - Oliver Hilsky

TODO
safe/request values after restart/loss of connection
*/

#define NDEBUG                        // enable local debugging information

#define SKETCH_NAME "RGBW Led strip"
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



// Arduino pin attached to driver pins
#define RED_PIN 3 
#define WHITE_PIN 9	// this is not a pwm pin! change it if you want pwm
#define GREEN_PIN 6
#define BLUE_PIN 5
#define NUM_CHANNELS 3 // how many channels, RGBW=4 RGB=3...

#define SENSOR_ID 30

// Smooth stepping between the values
#define STEP 1
#define INTERVAL 3 // 10
const int pwmIntervals = 255; //255;
float R; // equation for dimming curve


#define KNOB_ENC_PIN_1 14    // Rotary encoder input pin 1 (A0)
#define KNOB_ENC_PIN_2 7    // Rotary encoder input pin 2
#define KNOB_BUTTON_PIN 8   // Rotary encoder button pin 
//#define FADE_DELAY 10       // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim)
#define SEND_THROTTLE_DELAY 400 // Number of milliseconds before sending after user stops turning knob

#define KNOBUPDATE_TIME 500

#define RADIO_RESET_DELAY_TIME 50 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения
#define DATASEND_DELAY  10

boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;


Encoder knob(KNOB_ENC_PIN_2, KNOB_ENC_PIN_1);  
Bounce debouncer = Bounce(); 
byte oldButtonVal;
long lastencoderValue=0;

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


RGBConverter RGBConv;  

MyMessage msgLedStripStatus(SENSOR_ID, V_STATUS);


void setup() 
{

  // Set knob button pin as input (with debounce)
  pinMode(KNOB_BUTTON_PIN, INPUT);
  digitalWrite(KNOB_BUTTON_PIN, HIGH);
  debouncer.attach(KNOB_BUTTON_PIN);
  debouncer.interval(5);
  oldButtonVal = debouncer.read();

  // Initializes the sensor node (with callback function for incoming messages)
  gw.begin(incomingMessage, NODE_ID, false);	// 123 = node id for testing	
       
  // Present sketch (name, version)			
   gw.sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER"."SKETCH_MINOR_VER);      
  // Register sensors (id, type, description, ack back)
  gw.present(SENSOR_ID, S_RGBW_LIGHT, "RGBW test light", true);

  // request current values from gateway/controller
  //gw.request(SENSOR_ID, S_RGBW_LIGHT);

  // Set all channels to output (pin number, type)
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(channels[i], OUTPUT);
  }

  // set up dimming
  R = (pwmIntervals * log10(2))/(log10(255));


    gw.request(SENSOR_ID, V_RGBW);
  	gw.wait(1000); 



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

}

void loop()
{
  // Process incoming messages (like config and light state from controller) - basically keep the mysensors protocol running
  gw.process();		

  // and set the new light colors
  if (millis() > lastupdate + INTERVAL) {
    updateLights();
    lastupdate = millis();
  } 

  checkButtonClick();

checkRotaryEncoder();
}

// callback function for incoming messages
void incomingMessage(const MyMessage &message) {

  // acknoledgment
  if (message.isAck())
  {
    gotAck = true;
    return;
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
    //Serial.println("Got RGBW command");
    inputToRGBW(rgbvalues);   


  }  
}

// this gets called every INTERVAL milliseconds and updates the current pwm levels for all colors
void updateLights() {  



  // update pin values -debug
  //Serial.println(greenval);
  //Serial.println(redval);
  //Serial.println(blueval);
  //Serial.println(whiteval);

  //Serial.println(target_greenval);
  //Serial.println(target_redval);
  //Serial.println(target_blueval);
  //Serial.println(target_whiteval);
  //Serial.println("+++++++++++++++");

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

  /*
  // debug - new values
  Serial.println(greenval);
  Serial.println(redval);
  Serial.println(blueval);
  Serial.println(whiteval);

  Serial.println(target_greenval);
  Serial.println(target_redval);
  Serial.println(target_blueval);
  Serial.println(target_whiteval);
  Serial.println("+++++++++++++++");
  */

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
  //Serial.print("Got color value of length: "); 
  //Serial.println(strlen(input));
  
    //Serial.print("New color values recv: ");
  //Serial.println(input);

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


   knob.write(hsv[2]*100);
   lastencoderValue = hsv[2]*100;

  Serial.print("New color values: ");
  Serial.println(input);
  



  for (int i = 0; i < NUM_CHANNELS; i++) {
  //  Serial.print(target_values[i]);
  //  Serial.print(", ");
  }
 
 // Serial.println("");
  //Serial.print("Dimming: ");
  //Serial.println(dimming);
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

     		   			//Отсылаем состояние переключателя
			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            gw.send(msgLedStripStatus.set(isOn?"1":"0"), true);
			                    gw.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	


  }
  oldButtonVal = buttonVal;
}





void checkRotaryEncoder() {
  long encoderValue = knob.read();

  if (encoderValue != lastencoderValue)
  {
  			Serial.print("Encoder value: ");
  			Serial.println(encoderValue);
  			lastencoderValue=encoderValue;



   hsv[2]=(double)encoderValue/100.0;



	RGBConv.hsvToRgb(hsv[0], hsv[1], hsv[2], target_values);

	lastKnobsChanged = mills();
	bNeedToSendUpdate = true;

    //RGBConv.hsvToRgb(hsv[0], hsv[1], hsv[2], target_values);
  }

  if (encoderValue > 100) {
    encoderValue = 100;
    knob.write(100);
  } else if (encoderValue < 18) {
    encoderValue = 18;
    knob.write(18);
  }

 
}

void updateBrightness()
{

 unsigned long currentTempMillis = millis();
    if((currentTempMillis - lastKnobsChanged ) > KNOBUPDATE_TIME && bNeedToSendUpdate) {

	{

			bNeedToSendUpdate = false;

	}

}






