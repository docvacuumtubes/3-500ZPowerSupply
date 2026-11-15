//  GNU General Public License 3.0
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, version 3 of the License.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You can review the GNU General Public License at:
//     see <https://www.gnu.org/licenses/>.
//    NOTE this includes COPYLEFT. 
//




//////////////////LIBRARIES TO INCLUDE/////////////////////////////////
// include the library for the Liquid Crystal display
#include <LiquidCrystal.h>    // standard lib for liquid crystal
#include <Arduino.h>          // basic Arduino functions
#include <avr/wdt.h>          // Allows us to have a watchdog timer


/////////////ARDUINO  OUTPUTS//actual board//////////////////////////
#define TRANSMITOUTPUT 3 // D3 turns on relays to move into transmit
#define SENDINPUT     4 //  D4  receives the send input 
#define ALCOUTPUT     5 //  D5 (PWM) creates alc output via averaging 
                        //  NOTE: D5 is a PWM output @ 980 Hz
#define ALCTRIGGER    6 //  D6  shows when TX is on and powered normally
#define HVLOST        7 //  D7  shows during the time that delay is active
#define ALCLEVEL     A0 // A0 allows reading desired power level
#define HVSENSE     A1 // A1 allows reading desired delay before output power after SEND
#define SLOWSTART   A2 // A2 can turn on the relay output
#define KILLOUTPUT    A3 // A3  quickly brings output to zero
#define GRIDCURRENT   A7  //  A7 reads grid current induced voltage across 0.25 ohm resistor
#define SLOWSTARTDELAY  3000 // Mininum of 3000 msec delay for startup
#define HVMULTIPLIER    18    // Actual HV is 0-1024 measureent times this number

#define RX            0
#define TX            1
#define HVMISSING     0
#define HVPRESENT     1



////////////////////////STATES//////////////////////////////////
int   send_state; // 0 = RX; 1 = TX
int   tx_state;   // 0 = RX; 1 = TX
int   HV_state;   //  0 = no high voltage;  1 = normal high voltage
int   prev_state;

                        
///////////////////////////STRUCTURES & VARIABLES ///////////////////////////////////////
unsigned long   current_milliseconds;         // the current number of milliseconds since SEND output
int             delay_milliseconds;           // the desired delay
int             gridcurrentlimitsetting=200;     // max grid current limit, in mA
int             screencount=0;                // avoids flickering screen
int             HighVoltageAvg=0;              // high voltage in volts
int             GridCurrentAvgmA=0;            // calculated grid current in mA
int             reading1,reading2;
int   highvoltage[10];   // in volts, an array for high voltage for averaging (over 5)
                        // to smooth out interference
int   gridcurrentlimit; // in milliamperes, set by analot potentiometer,
                        // which also adjusts the feedback gain at same time
int   gridcurrent[30];  // An array for grid current since we need to average
                        // over 10
                        // to deal with interfering signals
int ALCDivisor=10;      // divisor that sets slope of ALC response (negative feedback gain)                        

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);  // The GLG board -- sequencer, rotator controller, arduino winkeyer etc

//LiquidCrystal lcd(13,12,11,10,9,8);  // when using the GAINESVILLE VENTILATOR Ashhar 1.0 Board
// This means that D8goes to RS, D9 to en, D10-D13 are the 4 bit parallel port info
// to the lcd display, which is connected exactly like both the UF venntilator
// and the uBitx Raduino display.






/////////////////// SET UP ROUTINE (EXECUTED ONCE ON STARTUP ////////////////////////////////////////////
void setup() {

  int i;  // general purpose counter
  int j;  // another 

  pinMode(TRANSMITOUTPUT, OUTPUT);  // HI moves relays into transmit
  pinMode(SENDINPUT, INPUT_PULLUP);  // send input from transmitter
  pinMode(ALCOUTPUT, OUTPUT);
  pinMode(ALCTRIGGER, OUTPUT);
  pinMode(HVLOST, OUTPUT);
  pinMode(HVSENSE, INPUT); // analog pins can be used as digital inputs--but not sure if this is a good idea
  pinMode(SLOWSTART, OUTPUT);  // not sure if needed; asking analog pin to be a digital output
  pinMode(KILLOUTPUT, OUTPUT);  // not sure if needed; asking analog pin to be a digital output
    
  analogReference(DEFAULT); // sets 5V as the top of reference
  digitalWrite(SLOWSTART, LOW); // turn OFF the relay output, so charges through resistor
  analogWrite(ALCOUTPUT,200);  // set the ALC output to maximum silence output (gets inverted to -4V)
  digitalWrite(KILLOUTPUT,HIGH);  // turn off the output quickly
  digitalWrite(HVLOST,LOW);  // turn off that LED
  digitalWrite(ALCTRIGGER,LOW); // turn off the delay LED
// set all the states to RECEIVE
  tx_state = RX;
   
  cli();
  wdt_reset();
  /*
    WDTCSR configuration:
    WDIE = 1: Interrupt Enable
    WDE = 1 :Reset Enable
    See table for time-out variations:
    WDP3 = 0 :For 1000ms Time-out
    WDP2 = 1 :For 1000ms Time-out
    WDP1 = 1 :For 1000ms Time-out
    WDP0 = 0 :For 1000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Set Watchdog settings:
  WDTCSR = (1 << WDIE) | (1 << WDE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
  // trying to set it for 2 seconds
  sei();  // R-enable the interrups

  Serial.begin(115200);   // this sets up the USB to give us diagnostic info
                          // at baud rate 115200 (fast)

  lcd.begin(16, 2);       // initialize the lcd display
  lcd.setCursor(0,1);     // ;set cursor to 0th column, 1st row (2nd line on ours)
  Lcd_Clear_Line();
  lcd.setCursor(0,1);
  
  lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
  Lcd_Clear_Line();
  lcd.setCursor(0,0);
  lcd.print(F("GNU GPL v3.0"));
  MyDelay(1000);
  lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
  Lcd_Clear_Line();
  lcd.setCursor(0,0);
  lcd.print(F("3-500Z Power"));

  lcd.setCursor(0,1);
  Lcd_Clear_Line();
  lcd.setCursor(0,1);
  lcd.print(F("Slow-Start.."));
  wdt_reset();
  
  MyDelay(SLOWSTARTDELAY);    // Delay to allow capacitors to charge and filaments to light...
  // NOW finished with charging delay. 
  // Turn everything Back On
  lcd.setCursor(0,1);
  Lcd_Clear_Line();
  wdt_reset();
  digitalWrite(SLOWSTART, HIGH); // turn ON the relay output, so charges fully
  analogWrite(ALCOUTPUT,0);  // set the ALC output to allow full output
  digitalWrite(KILLOUTPUT,LOW);  // allow otuput
  digitalWrite(HVLOST,LOW);  // turn off that LED
  digitalWrite(ALCTRIGGER,LOW); // turn off the ALC-trigered LED

  lcd.setCursor(0,0);     // set the cursor at 0th collumn, 0th row
  Lcd_Clear_Line();
  lcd.setCursor(0,0);
  lcd.print(F("Full Voltage"));
   MyDelay(1000);    // Delay so this can be READ 

  // Need to turn off all the current systems and measure the offset voltages...

  Serial.print("PowerLevel:  ");
  Serial.println(analogRead(A0));
  
  MyDelay(500);
   wdt_reset();
  MyDelay(500);
  wdt_reset();
 // Fill up the highvoltage[] and gridcurrent[] arrays
 for(i=0;i<5;i++)  highvoltage[i] = analogRead(HVSENSE) * 18 ;
 for(i=0;i<10;i++) gridcurrent[i] = analogRead(GRIDCURRENT) * 50;
 
return;
}  //end of SETUP

////////////////////////////READ grid current limit LEVEL////////////////////////////////
// Reads the potentiometer to find desired maximum grid current
// Also adjusts the gain of the feedback loop
// Note that we make ALC decisions based on the AVERAGED grid current

void readgridlimit() {
int powersetting;  // 0 = maximum output
// read the 0-5V level of the power level adjustment potentiometer
powersetting = analogRead(ALCLEVEL);   // reads 0-1023 for 0 - 5 VDC
gridcurrentlimitsetting = powersetting/3;  // converts to 0-340mA, this will need adjusting.
// make the divisor for calculating the ALC output voltage become smaller if the milliampere limit is lower
// Start with 25 (100mA over ==> 4 volts) at top of range
// End up with 10 (40 mA over ==> 4 volts) at bottom of range
ALCDivisor =  10 + (gridcurrentlimitsetting/23);   
}
////////////////////////////////Read Actual Grid Current //////////////////////////////
void readgridcurrent() {
  int i;  // general variable
  // 0.25 ohm resistor in the Loudenboomer, so V = I * R
  // mA of grid current = 1000 * 0.25 * V = 250 * V
  // V =   reading (0-1023)/5  =   0.2 * reading
  // mA of grid current =  0.2 * 250 * reading =  50 * reading
  for(i=0;i<9;i++)  gridcurrent[i] = gridcurrent[i+1];  // shift array downwards
  gridcurrent[9] = analogRead(GRIDCURRENT) * 50;  // fill in new top value
  GridCurrentAvgmA = 0;  // set up averaging sum
  for(i=0;i<10;i++) GridCurrentAvgmA = GridCurrentAvgmA + gridcurrent[i];
  GridCurrentAvgmA = GridCurrentAvgmA/10;  // compute the average

 //Serial.print("gridcurrent[9] =  ");
 //Serial.println(gridcurrent[9];
// Serial.print("GridCurrentAvgmA:  ");
 //Serial.println(GridCurrentAvgmA);

return;  
}

//////////////////////////////READ HV ///////////////////////////////////
void readHV() {
// read the 0-5V setting of the portion of HV
int i;  // general variable

for(i=0;i<4;i++)  highvoltage[i] = highvoltage[i+1];  // shift array downwards
  highvoltage[4] = analogRead(HVSENSE) * 18;  // fill in new top value
                // reads 0-1023 for 0-1023 for 0-5 volts where approx 1 V = 4000
  HighVoltageAvg = 0;  // set up averaging sum
  for(i=0;i<5;i++) HighVoltageAvg = HighVoltageAvg + highvoltage[i];
  HighVoltageAvg = HighVoltageAvg/5;  // compute the average
  if(HighVoltageAvg>5000)  HighVoltageAvg = 4999;
return;
}


///////////////////////READ SEND/////////////////////////////
void readsend()
{
//int reading1;
//int reading2;
reading1 = digitalRead(SENDINPUT);
MyDelay(2);
reading2 = digitalRead(SENDINPUT);
// A simpleminded "debounce"
if(reading1 == reading2) {
    if((reading1==LOW) &&(tx_state==RX)) tx_state=TX;
    if ((reading1==HIGH) &&(tx_state!=RX))tx_state=RX;
    }
return;
}




///////////////////////////LOOP ROUTINE ///////////////////////////////////////////
// Meat of the program: executed over and over again
void loop() {
  char  buffer[20];
  int   outputvoltage;  // voltage to send to inverting system
  // put your main code here, to run repeatedly:
   wdt_reset();      // Reset the watchdog timer -- it it doesn't get reset within 2 seconds,
                    // it will restart the software
  // Make important measurements and carry out important actions
  // 1 FIRST PRIORITY is getting relays switched ASAP to avoid hot-switch
  // Check for SEND input requiring transition into or out of transmit, if adequate high voltage seen previously
  readsend();   // readsend() sets tx_state appropriately based on ground-to-transmit input
  if ((tx_state==RX) )digitalWrite(TRANSMITOUTPUT,LOW);  // switch to receive any time requested
  // To avoid any protection delays, we make DECISIONS based on the single last measurement of high voltage
  if ((tx_state==TX) && (highvoltage[4] > 1000) )digitalWrite(TRANSMITOUTPUT,HIGH);
            // switch to transmit if requested and HV present at very last measurement
           

  // 2 Second Priority, Check for HV loss Now check the high voltage
  readHV();
  if(highvoltage[4] < 1000)     {   // decide based on the very last voltage measured
    analogWrite(ALCOUTPUT,255);  // set the ALC output to maximum silence output (gets inverted to -4V)
    digitalWrite(ALCTRIGGER,HIGH);  // turn on the ALC trigger to indicate the command
    digitalWrite(TRANSMITOUTPUT, LOW); // move the relays to BYPASS the amplifier, protecting the grid
    digitalWrite(HVLOST,HIGH); // turn on that LED
    // do NOT switch the resistors in series with the primary because they can overheat
    // put ALERT msg on screen
          lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
          Lcd_Clear_Line();
          lcd.setCursor(0,0);
          sprintf(buffer,"****HV LOST****");
          lcd.print(buffer);
          MyDelay(200);
          wdt_reset(); 
  }
  if(HighVoltageAvg> 1000)     digitalWrite(HVLOST,LOW);  // turn off that LED

  // 1 FIRST PRIORITY is getting relays switched ASAP to avoid hot-switch
  // Check for SEND input requiring transition into or out of transmit, if adequate high voltage seen previously
  readsend();   // readsend() sets tx_state appropriately based on ground-to-transmit input
  if ((tx_state==RX) )digitalWrite(TRANSMITOUTPUT,LOW);  // switch to receive any time requested
  // To avoid any protection delays, we make DECISIONS based on the single last measurement of high voltage
  if ((tx_state==TX) && (highvoltage[9] > 1000) )digitalWrite(TRANSMITOUTPUT,HIGH);
            // switch to transmit if requested and HV present at very last measurement

  // 3 THIRD PRIORITY  Read the grid current and determine if an ALC output needs to occur

  readgridlimit();    // returns int gridcurrentlimitsetting
  readgridcurrent();  // returns int GridCurrentAvgmA which is an average of past 10 values

  if(GridCurrentAvgmA <= gridcurrentlimitsetting){
    analogWrite(ALCOUTPUT,0);
    digitalWrite(ALCTRIGGER,LOW);
  }
  if(GridCurrentAvgmA > gridcurrentlimitsetting) {
  // generate a commensurate voltage;  difference could be 0-100 mA or so?
    outputvoltage= (GridCurrentAvgmA-gridcurrentlimitsetting)/ALCDivisor;
        // ALCDivisor ranges from 10 to about 25 setting the gain so tighter control if level 
        // is set lower
    if(outputvoltage>4) outputvoltage=4;
    analogWrite(ALCOUTPUT, outputvoltage * 200);
    // turn on the LED
    digitalWrite(ALCTRIGGER,HIGH);
  }

 // 1 FIRST PRIORITY is getting relays switched ASAP to avoid hot-switch
  // Check for SEND input requiring transition into or out of transmit, if adequate high voltage seen previously
  readsend();   // readsend() sets tx_state appropriately based on ground-to-transmit input
  if ((tx_state==RX) )digitalWrite(TRANSMITOUTPUT,LOW);  // switch to receive any time requested
  // To avoid any protection delays, we make DECISIONS based on the single last measurement of high voltage
  if ((tx_state==TX) && (highvoltage[9] > 1000) )digitalWrite(TRANSMITOUTPUT,HIGH);
            // switch to transmit if requested and HV present at very last measurement

  // 4 LAST Priority is sending output to the LCD Screen to keep the user aware
    screencount++;
    if(screencount==25){
      if(highvoltage[4] < 1000)     {   // decide based on the very last voltage measured
        // put ALERT msg on screen
          lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
          Lcd_Clear_Line();
          lcd.setCursor(0,0);
          sprintf(buffer,"****HV LOST****");
          lcd.print(buffer);
      }
      else {
      lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
      Lcd_Clear_Line();
      lcd.setCursor(0,0);
     
      sprintf(buffer,"READY TX:%d",tx_state);
      //IN%4d Ig%4d", reading1,GridCurrentAvgmA);
      lcd.print(buffer);
      }
     
      lcd.setCursor(0,1);
      Lcd_Clear_Line();
      lcd.setCursor(0,1);
      if(highvoltage[4] > 1000) sprintf(buffer,"Ig%4d Lim%4d", GridCurrentAvgmA,gridcurrentlimitsetting);
      else sprintf(buffer,"EST HV:%4d",HighVoltageAvg);
      lcd.print(buffer);
      
      screencount = 0;
    }
                    



   
}  // END OF LOOP

//////////////////////////supporting subroutines //////////////////////////////////////


///////////////////////////CLEAR LINE////////////////////////////////////////////////
void Lcd_Clear_Line()
{
  lcd.print(F("                "));  // should be exactly 16 spaces
}
//////////////////////////PRINT TO THE SCREEN ///////////////////////////////////////
void  lcd_display(char *s1,char *s2, int dtime)
{
  // dtime= milliseconds to delay
  lcd.setCursor(0,0);     // set the cursor at 0th column, 0th row
  // make sure the strings are null terminated after the 16th character.  1st char = *s1
  *(s1+15) = 0;
  *(s2+15) = 0;
  lcd.print(s1);
  lcd.setCursor(0,1);     // set cursor to 0th column, 1st row (2nd line on ours)
  lcd.print(s2);
  // call the delay function, while handling the watchdog
  wdt_reset();      // Reset the watchdog timer -- it it doesn't get reset within 2 seconds,
                    // it will restart the software
  MyDelay(dtime);
  wdt_reset();      // Reset the watchdog timer -- it it doesn't get reset within 2 seconds,
                    // it will restart the software
}
//////////////////////MICROSECOND DELAY NON BLOCKING /////////////////////////////////
void MyMicroSecondsDelay(int udelay)
{
  int microseconds;
  unsigned long current_microseconds, next_microseconds;
  
  current_microseconds = micros();
  next_microseconds = current_microseconds + (unsigned long) udelay;
    while(micros()<next_microseconds);
  return;
  
}

/////////////////////////////MILLISECOND DELAY NON BLOCKING///////////////////
void MyDelay(int msec)
{
  
  unsigned long local_current_milliseconds, next_milliseconds, intermediate_milliseconds;
  int x;
  int thousands;
  wdt_reset();
  local_current_milliseconds = millis();
  next_milliseconds = local_current_milliseconds + (unsigned long) msec;
  thousands = msec/1000;            // integer division
  if(thousands>=1) {
      for(x=1;x<= thousands; x++){
          intermediate_milliseconds = local_current_milliseconds + (unsigned long) (x*1000);
          while (millis() < intermediate_milliseconds);
          wdt_reset();
          }
    }
  // now finish out the remainder, which should be less than 1000 milliseconds
   while (millis()<next_milliseconds);
   wdt_reset();
  return;
}
