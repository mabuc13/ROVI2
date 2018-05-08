
/***************************************************************************
# SDU computer controlled tx firmware
# Copyright (c) 2018, Mark Buch <mabuc13@student.sdu.dk> <markstedalb@gmail.com>
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# This library is based on the source example Generate_PPM_signal_V0.2 obtained
# from https://code.google.com/archive/p/generate-ppm-signal/downloads
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
This firmware is developed for the v03-2018 version of the SDU UAS Transmitter

Revision

****************************************************************************/
/* parameters */



/****************************************************************************/
/* input defines */
#define PIN_LEFT_X A5
#define PIN_LEFT_Y A4
#define PIN_RIGHT_X A3
#define PIN_RIGHT_Y A2
#define PIN_3_POS_SW_LEFT A7
#define PIN_3_POS_SW_RIGHT A6
#define PIN_LEFT_BUTTON 3
#define PIN_RIGHT_BUTTON 4
#define PIN_2_POS_SW_LEFT 7
#define PIN_2_POS_SW_RIGHT 8
#define PIN_POT A1

#define PIN_TX 6
#define PIN_AUDIO 11
#define PIN_BATT_VOLT A0
#define PIN_BUZZER 5
#define PIN_LED_RED 9
#define PIN_LED_GREEN 10

/* ppm defines */
#define ppm_number 8  //set the number of ppm chanels
#define analog_number 8  //set the number of ppm chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
//#include <SoftwareSerial.h>

//SoftwareSerial portOne(10, 11);
//#include <String>
/****************************************************************************/
/* variables */
/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
long ppm[ppm_number];
long analog[analog_number];
short count;
boolean led_state;
boolean buzzer_state;
int serialCount;
int thr = 0;
int roll = 0;
int pitch = 0;
int yaw = 0;
boolean initPPM = false;
/****************************************************************************/
void setup()
{  
  // setup digital output pins
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(PIN_LED_RED, OUTPUT); 
  pinMode(PIN_LED_GREEN, OUTPUT); 
  pinMode(PIN_BUZZER, OUTPUT); 

  // enable pull-up resistor on digital input pins 
  digitalWrite (PIN_LEFT_BUTTON, HIGH);
  digitalWrite (PIN_RIGHT_BUTTON, HIGH);
  digitalWrite (PIN_2_POS_SW_LEFT, HIGH);
  digitalWrite (PIN_2_POS_SW_RIGHT, HIGH);

   Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //portOne.begin(115200);
  //initiallize default ppm values
  for(int i=0; i<ppm_number; i++)
  {
    ppm[i]= default_servo_value;
  }
  thr = 0;
  serialCount = 0;
  pinMode(PIN_TX, OUTPUT);
  pinMode(PIN_AUDIO, OUTPUT);
  digitalWrite(PIN_TX, !onState);  //set the PPM signal pin to the default state (off)
  digitalWrite(PIN_AUDIO, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}
/****************************************************************************/
ISR(TIMER1_COMPA_vect)
{
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state)
  {  //start pulse
    digitalWrite(PIN_TX, onState);
    digitalWrite(PIN_AUDIO, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else
  {  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(PIN_TX, !onState);
    digitalWrite(PIN_AUDIO, !onState);
    state = true;

    if(cur_chan_numb >= ppm_number)
    {
      digitalWrite(PIN_TX, !onState);
      digitalWrite(PIN_AUDIO, !onState);
       cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else
    {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

int trimInputs(int min, int max, int neutral, bool reverse, int analog)
{
  int FCMax = 1900;
  int FCMin = 1170;

  //Scaler til en int mellem 0 og 1000
   

  //scaler til mellem FCMax og FCMin
}

void killPPM()
{
  ppm[0] = 1170;
  ppm[1] = 1170;
  ppm[2] = 1170;
  ppm[3] = 1170;
}

void updatePPM() //The standard message will be in the format TTTTRRRRPPPPYYYY
                 // The serial index                         :0123456789 etc
{
  String temp = "";
  temp = Serial.readString();
   thr += (temp[0]-48)*1000;
   thr += (temp[1]-48)*100;
   thr += (temp[2]-48)*10;
   thr += (temp[3]-48);
   roll += (temp[4]-48)*1000;    
   roll += (temp[5]-48)*100;
   roll += (temp[6]-48)*10;
   roll += (temp[7]-48);
   pitch += (temp[8]-48)*1000;
   pitch += (temp[9]-48)*100;
   pitch += (temp[10]-48)*10;
   pitch += (temp[11]-48);
   yaw += (temp[12]-48)*1000;
   yaw += (temp[13]-48)*100;
   yaw += (temp[14]-48)*10;
   yaw += (temp[15]-48);
   
   ppm[0] = thr;
   ppm[1] = roll;
   ppm[2] = pitch;
   ppm[3] = yaw;
   thr = 0;
   roll = 0;
   yaw = 0;
   pitch = 0;
    


  
  /*Serial.print(temp);
  Serial.print(" ");
  Serial.print(thr, DEC);
  Serial.print(" ");
  Serial.print(ppm[0], DEC);*/
  
  
}

/****************************************************************************/
void loop()
{
  //mySerial.println("Hello, world?");
  //portOne.listen();
  //put main code here
  static int val = 1;
  count ++;
  if (!initPPM)
  {
    killPPM();
    initPPM = true;
  }
  // update LED
  if (count % 200 == 0)
    digitalWrite(PIN_LED_GREEN, HIGH);
  else
    digitalWrite(PIN_LED_GREEN, LOW);

  if (Serial.available() > 0)
  {
    Serial.println("I actually recieved something");
    //char dummy = Serial.read();
     updatePPM();
  }
  
  
  // read analog input
  analog[0] = analogRead(PIN_LEFT_Y);
  analog[1] = analogRead(PIN_LEFT_X);
  analog[2] = analogRead(PIN_RIGHT_Y);
  analog[3] = analogRead(PIN_RIGHT_X);
  analog[4] = analogRead(PIN_3_POS_SW_LEFT);
  analog[5] = analogRead(PIN_3_POS_SW_RIGHT);
  analog[6] = analogRead(PIN_POT);
  analog[7] = analogRead(PIN_BATT_VOLT);
  /**/
  // map to ppm output The basic options in cleanflight is that the minimum value is 1170 on all input channels and maximun is 1900
  //ppm[0] = (analog[0]-55)*730/(993-55) + 1170 - 10; // throttle  
  //ppm[1] =  (1008-70-(analog[3]-70))*(1900-1170)/(1008-70)+1170; // roll (aileron) This one needs to be inversed  
  //ppm[2] = (analog[2]-58)*730/(993-55) + 1170; // pitch (elevator) 
  //ppm[3] = (1019-62-(analog[1]-62))*(1900-1170)/(1019-62)+1170; // yaw (rudder) This one needs to be reversed aswell //(pot_range -(analog-analog_min))*output_rang/input_rangee+min_output

  // handle special case of arming AutoQuad
  if (analog[0] < 450 && analog[1] > 1000)
  {
    ppm[0] = 1150;
    ppm[3] = 1850;  
  }

  // handle left 3-way switch
  if (analog[4] < 300)
    ppm[4] = 1150;
  else if (analog[4] < 700)
    ppm[4] = 1500;
  else
    ppm[4] = 1850;
  
  // handle right 3-way switch
  if (analog[5] < 300)
    ppm[5] = 1150;
  else if (analog[5] < 700)
    ppm[5] = 1500;
  else
    ppm[5] =1850; 

  // unused for now 
  ppm[6] = default_servo_value;
  ppm[7] = default_servo_value;
  //ppm[0] = 1;
  // debug: output analog and digital input values to the serial port
  Serial.print(ppm[0]); //Throttle
 //mySerial.write (" ");
  Serial.println(" ");
  Serial.print(analog[4]);
  Serial.println(" ");
  /*Serial.write (ppm[1]);
  mySerial.write (" ");
  mySerial.write (ppm[2]);
  mySerial.write (" ");
  mySerial.write (ppm[3]);
  mySerial.write (" ");
  mySerial.write (analog[4]);
  mySerial.write (" ");
  mySerial.write (analog[5]);
  mySerial.write (" ");
  mySerial.write (analog[6]);
  mySerial.write (" ");
  mySerial.write (analog[7]);
  mySerial.write (" ");
  mySerial.write (digitalRead(PIN_LEFT_BUTTON));
  mySerial.write (" ");
  mySerial.write (digitalRead(PIN_RIGHT_BUTTON));
  mySerial.write (" ");
  mySerial.write (digitalRead(PIN_2_POS_SW_LEFT));
  mySerial.write (" ");
  mySerial.write (digitalRead(PIN_2_POS_SW_RIGHT)); */
  Serial.flush();
  delay(10);
}
/****************************************************************************/

/* switch(serialCount)
  {
    case 0:
      temp = Serial.read();
      thr += temp.toInt()*1000;
      Serial.print(thr, DEC);
      Serial.print(" ");
      Serial.print(temp);
      
      break;
    case 1:
      temp = Serial.read();
      thr += temp.toInt()*100;
      break;
    case 2:
      temp = Serial.read();
      thr += temp.toInt()*10;
      break;
    case 3:
      temp = Serial.read();
      thr += temp.toInt();
      break;
    case 4:
      temp = Serial.read();    
      roll += temp.toInt()*1000;
      break;
    case 5:
      temp = Serial.read();    
      roll += temp.toInt()*100;
      break;
    case 6:
      temp = Serial.read();    
      roll += temp.toInt()*10;
      break;
    case 7:
      temp = Serial.read();    
      roll += temp.toInt();
      break;
    case 8:
      temp = Serial.read();    
      pitch += temp.toInt()*1000;
      break;
    case 9:
      temp = Serial.read();    
      pitch += temp.toInt()*100;
      break;
    case 10:
      temp = Serial.read();    
      pitch += temp.toInt()*10;
      break;
    case 11:
      temp = Serial.read();    
      pitch += temp.toInt();
      break;
    case 12:
      temp = Serial.read();    
      yaw += temp.toInt()*1000;
      break;
    case 13:
      temp = Serial.read();    
      yaw += temp.toInt()*100;
      break;
    case 14:
      temp = Serial.read();    
      yaw += temp.toInt()*10;
      break;
    case 15:
      temp = Serial.read();
      yaw += temp.toInt();
      ppm[0] = thr;
      ppm[1] = roll;
      ppm[2] = pitch;
      ppm[3] = yaw;
      thr = 0;
      roll = 0;
      yaw = 0;
      pitch = 0;
      break;    
    default:
      break;
  }*/
