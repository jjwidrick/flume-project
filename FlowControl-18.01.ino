#include <DallasTemperature.h>   // to communicate with temperature probe
#include <OneWire.h>             // to read DS18B20 temperature probe
//---------------------------------------------------------------------------------------- 
// FlowControl: code for Arduino controlled zebrafish flume       
// author: Jeffrey J. Widrick       
// last modified: March 23, 2018
   char ver[ ] = "version 18.01";
//
// 
// MIT License
//
// Copyright (c) 2018 Jeffrey J. Widrick
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
//----------------------------------------------------------------------------------------  
//----------------------------------------------------------------------------------------
// INITIAL SET-UP 
//
//----------------------------------------------------------------------------------------
// 1. Diameter of the working section.
//----------------------------------------------------------------------------------------  
     float pipe_dia_in = 1.00;       // inches (2 decimal pts.)
// 
//----------------------------------------------------------------------------------------
// 2. PWM stall value.
//----------------------------------------------------------------------------------------
    int PWM_stall = 235;   // slower than this PWM value causes pump to stall (integer)
//
//----------------------------------------------------------------------------------------
// 3. Flow meter calibrations.
//----------------------------------------------------------------------------------------
// Flow meter 1 calibration INFO, SLOPE, and Y_INTERCEPT.
     char cal_info_1[ ] = "meter1";
     float cal_slope_1 = 2.44;        // ml per pulse
     float cal_yint_1 = 0.79;            // ml
//           
// Flow meter 2 calibration INFO, SLOPE, and Y_INTERCEPT.
     char cal_info_2[ ] = "meter2";
     float cal_slope_2 = 2.38;        // ml per pulse
     float cal_yint_2 = 1.55;            //ml
//
//----------------------------------------------------------------------------------------
// 4. Set-up information for mode 3 (PWM x flow calibration).
//---------------------------------------------------------------------------------------- 
     int cal_array[]{220, 200, 180, 160, 140, 120, 100, 80, 60, 40};   // PWM values (integers) at each step: 255 (slowest) to 0 (fastest)        
     int cal_interval = 30;            // duration of each stage in seconds (integer)
//
//
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
// PRE-EXPERIMENT SET-UP
//
//----------------------------------------------------------------------------------------
// 1. Information about this session.
//----------------------------------------------------------------------------------------
// Your NAME.
        char oper[ ] = "name";      
// The DATE.
        char date[ ] = "yymmdd";    // yymmdd format         
// A short DESCRIPTION of your project.
        char proj[ ] = "project";        
//
//----------------------------------------------------------------------------------------
// 2. Coefficients for the PWM x flow calibration.
//----------------------------------------------------------------------------------------       
      float first_order  =  -6.265;   // first order regression coefficient
      float second_order =  -1.002;   // second order regression coefficient      
      float intercept    = 236.8;     // intercept of regression
//
//----------------------------------------------------------------------------------------
// 3. Custom protocol. 
//---------------------------------------------------------------------------------------- 
// protocol A
//        float flow_array[]{2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0, 10.5, 11.0, 11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0};        // flow rates at each stage, in cm/s (1 decimal pt.)                                                            
//          int time_array[]{ 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30 };                // time at each stage, in seconds (integer)                    
//
// protocol B     
      float flow_array[]{20.0, 10.0, 12.5, 15.0, 17.5, 20.0, 22.5, 25.0, 27.5, 30.0, 32.5, 35.0, 37.5, 40.0, 42.5, 45.0, 47.5, 50.0, 52.5, 55.0, 57.5, 60.0, 62.5, 65.0, 67.5, 70.0, 72.5};        // flow rates at each stage, in cm/s (1 decimal pt.)                                                            
        int time_array[]{   5,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   30,   120};                // time at each stage, in seconds (integer)                    
//
//
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
// Reference: pin connections.
//
//      pin AIO-0.....read potentiometer   
//      pin DIO-1.....read temperature probe    
//      pin DIO-2.....read flow meter 1
//      pin DIO-3.....write PWM to pump 1 (connect to BLUE pump wire)
//      pin DIO-4.....read flow meter 2     
//      pin DIO-5.....write PWM to pump 2 (connect to BLUE pump wire)
//      pin DIO-6.....read rocker switch
//      3.3 V.........power pump electronics (connect to the GREEN pump wires)
//      5 V...........power temperature probe, flow meters 
//      A 12 V DC power supply is required to power the motors of each pump
// 
//----------------------------------------------------------------------------------------                                                                                                         
//----------------------------------------------------------------------------------------
// Do not modify anything below here unless 
// you intend to alter the function of the program
//----------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------
//
int pot_value;
int PWM_pot;
int PWM; 
int PWM_1;    // PWM sent to pump 1
int PWM_2;    // PWM sent to pump 2
int mode;
int PWM_value;
int stage;
int original_on_off_status;
int new_on_off_status;
int i;

const int pot_pin = 0;
const int temp_pin = 1;
const int flow_pin_1 = 2;
const int pwm_pin_1 = 3; 
const int flow_pin_2 = 4;                           
const int pwm_pin_2 = 5; 
const int toggle_pin = 6;  // toggle switch

int flow_array_size = sizeof(flow_array)/sizeof(float);
int time_array_size = sizeof(time_array)/sizeof(int);
int cal_array_size = sizeof(cal_array)/sizeof(int);
 
volatile float flow_counts_1;
volatile float flow_counts_2;

float r;
float water_CSA;

float flow_hz_1;
float ml_per_s_1;

float flow_hz_2;
float ml_per_s_2;

float ml_per_s_total;
float cm_per_s;

float start_ms;
int stage_time;
int total_time;

float time1;
float time2;
float elapsed_time;
float target;

char rx_byte;
String exp_ID;
int manual_time;      // seconds

OneWire oneWire(temp_pin);            //setup oneWire to communicate w/sensor
DallasTemperature sensors(&oneWire);  //pass oneWire refernce to DallasTemperatue sensor
 
//////// define functions //////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void read_ID()
{

rx_byte = 0;
exp_ID = " ";

while (Serial.available() >0)            // clear the serial monitor buffer
    {
      Serial.read();
    }


while(Serial.available() == 0)          // wait for serial monitor input
    {
        Serial.println();
        Serial.println();
        Serial.println(" Enter experiment ID number");
        delay(2000);     
    }

    
while(Serial.available()>0)               // read input, one char at a time until \n
    {
        rx_byte = Serial.read();              // read character
        if(rx_byte != '\n')                   // not the end of line?
              {                                // if not the end of line go here
                  exp_ID = exp_ID + rx_byte;     // build up string 
              }
        else
              {                               
                  // do nothing here
              }
    } 
    
exp_ID.trim();      // trims any white space from string

}                          
//////////////////////////////////////////////////////////////////////////////////////
void read_manual_time()
{

rx_byte = 0;
String manual_min = ("");

while (Serial.available() >0)            // clear the serial monitor buffer
    {
      Serial.read();
    }


while(Serial.available() == 0)          // wait for serial monitor input
    {
        Serial.println();
        Serial.println();
        Serial.println(" Enter minutes to stay in manual mode (as integer)");
        delay(2000);     
    }

    
while(Serial.available()>0)               // read input, one char at a time until \n
    {
        rx_byte = Serial.read();              // read character
        if(rx_byte != '\n')                   // not the end of line?
              {                                // if not the end of line go here
                  manual_min = manual_min + rx_byte;     // build up string 
              }
        else
              {                               
                  // do nothing here
              }
    }
   
   manual_time = manual_min.toInt();
   manual_time = manual_time * 60;
}                                     
//////////////////////////////////////////////////////////////////////////////////////
void water_CSA_calc()     
{
  // calculate CSA of water in working section
  r = (pipe_dia_in * 2.54)/2;      // tube radius in cm (hypothenuse of triangle)
  water_CSA = 3.14 * r * r;       // CSA of water in working section, in cm^2
}
//////////////////////////////////////////////////////////////////////////////////////////
void count_samples_1()   
{
  flow_counts_1++;
}
//////////////////////////////////////////////////////////////////////////////////////////
void count_samples_2()   
{
  flow_counts_2++;
}
///////////////////////////////////////////////////////////////////////////////////////////
void calc_flow()
{
      if(flow_hz_1 == 0)
      {
        ml_per_s_1 = 0;
      }
      if(flow_hz_1 >0)
      {
        ml_per_s_1 = (flow_hz_1 * cal_slope_1) + cal_yint_1;         // pulse/s * ml/pulse = ml/s
      }

      if(flow_hz_2 == 0)
      {
        ml_per_s_2 = 0;
      }
      if(flow_hz_2 >0)
      {
        ml_per_s_2 = (flow_hz_2 * cal_slope_2) + cal_yint_2;         // pulse/s * ml/pulse = ml/s
      }

      ml_per_s_total = ml_per_s_1 + ml_per_s_2; 
      cm_per_s = ml_per_s_total / water_CSA; 
}

////////////////////////////////////////////////////////////////////////////////////////////
void read_temp()
{    
  sensors.requestTemperatures();
}

///////////////////////////////////////////////////////////////////////////////////////////
void PWM_calc()
{
  PWM_1 = PWM;         // PWM value for pump 1 
  PWM_2 = PWM; 
/*
  if(PWM >= 225)        
      {
        PWM_2 = 255;   // no flow   
      }
  else
      {
        PWM_2 = PWM * 1.1;  // factor for incrementing pump 2   
      }
*/
    
}

///////////////////////////////////////////////////////////////////////////////////////////
void print2()  
{ 
  // print out when entering protocol mode (mode 1) 
  Serial.print("invest");
  Serial.print("\t");
  Serial.println(oper);
   
  Serial.print("date");
  Serial.print("\t");
  Serial.println(date);
   
  Serial.print("project");
  Serial.print("\t");
  Serial.println(proj); 

  Serial.print("genotype");
  Serial.print("\t");
  Serial.println("NA");

  Serial.print("group");
  Serial.print("\t");
  Serial.println("NA");
  
  Serial.print("info1");
  Serial.print("\t");
  Serial.println("NA");

  Serial.print("info2");
  Serial.print("\t");
  Serial.println("NA");

  Serial.print("age");
  Serial.print("\t");
  Serial.println("NA");

  Serial.print("sex");
  Serial.print("\t");
  Serial.println("NA");

  Serial.print("Lstd_mm");
  Serial.print("\t");
  Serial.println("NA");

  Serial.print("mass_g");
  Serial.print("\t");
  Serial.println("NA");

  Serial.print("software");
  Serial.print("\t");
  Serial.println(ver);
   
  Serial.print("Flow meter 1 cal: slope = ");
  Serial.print(cal_slope_1);
  Serial.print(" ml/pulse");
  Serial.print(", y-int = ");
  Serial.print(cal_yint_1);
  Serial.println(" ml");
    
  Serial.print("Flow meter 2 cal: slope = ");
  Serial.print(cal_slope_2);
  Serial.print(" ml/pulse");
  Serial.print(", y-int = ");
  Serial.print(cal_yint_2);
  Serial.println(" ml");
  
  Serial.print("CSA of water in working section = ");
  Serial.print(water_CSA,2);
  Serial.println(" cm^2");

  Serial.print("flow vs. PWM cal: ");
  Serial.print("1st order = ");
  Serial.print(first_order,4);
  Serial.print(", 2nd order = ");
  Serial.print(second_order,4); 
  Serial.print(", y-int = ");
  Serial.print(intercept,2);
  Serial.println("");
  Serial.println("");
}

///////////////////////////////////////////////////////////////////////////////////////////
void print5a()   
{
  // print out when entering manual mode (mode 2)
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("# Entering manual mode");
  Serial.println("# Adjust flow using poteniometer");
  Serial.println("# ");
  Serial.print("# Manual mode will last for ");
  Serial.print(manual_time);
  Serial.println(" sec");
  Serial.println("# ");
  Serial.println("# Note: there is no total timer or target flow rate when in manual mode.");
  Serial.println("# ");
  read_temp();
}

///////////////////////////////////////////////////////////////////////////////////////////////
void print6a()
{
  // print out when entering flow vs. PWM calibration (mode 3)  
  Serial.println("");
  Serial.println("# Entering auto calibration mode"); 
  Serial.print("# Flow meter 1 cal: slope = ");
  Serial.print(cal_slope_1);
  Serial.print(" ml/pulse");
  Serial.print(", y-int = ");
  Serial.print(cal_yint_1);
  Serial.println(" ml.");  
  Serial.print("# Flow meter 2 cal: slope = ");
  Serial.print(cal_slope_2);
  Serial.print(" ml/pulse");
  Serial.print(", y-int = ");
  Serial.print(cal_yint_2);
  Serial.println(" ml.");    
  Serial.print("# CSA of water in working section = ");
  Serial.print(water_CSA);
  Serial.println(" cm^2.");
  Serial.println("# ");
}

/////////////////////////////////////////////////////////////////////////////////////////////
void print3()  
{
  // print out header for results
  Serial.print("date");
  Serial.print("\t");
  Serial.print("ID");
  Serial.print("\t");
  Serial.print("stage");
  Serial.print("\t");
  Serial.print("sec");
  Serial.print("\t");
  Serial.print("total");
  Serial.print("\t");
  Serial.print("tempC");
  Serial.print("\t");
  Serial.print("target");
  Serial.print("\t");
  Serial.print("PWM1");
  Serial.print("\t");
  Serial.print("PWM2");
  Serial.print("\t");
  Serial.print("Hz1");
  Serial.print("\t");
  Serial.print("Hz2");
  Serial.print("\t");
  Serial.print("mls1");
  Serial.print("\t");
  Serial.print("mls2");
  Serial.print("\t");
  Serial.print("mls");
  Serial.print("\t");
  Serial.println("cms");
}
 
/////////////////////////////////////////////////////////////////////////////////////////////
void print4()    // print out results  
{
  Serial.print(date);
  Serial.print("\t");
  Serial.print(exp_ID);
  Serial.print("\t");
  Serial.print(stage);
  Serial.print("\t");
  Serial.print(stage_time);
  Serial.print("\t");
  Serial.print(total_time);
  Serial.print("\t");
  Serial.print(sensors.getTempCByIndex(0),1);
  Serial.print("\t");
  Serial.print(target);
  Serial.print("\t");
  Serial.print(PWM_1);
  Serial.print("\t");
  Serial.print(PWM_2);
  Serial.print("\t");
  Serial.print(flow_hz_1);
  Serial.print("\t");
  Serial.print(flow_hz_2);
  Serial.print("\t");
  Serial.print(ml_per_s_1);
  Serial.print("\t");
  Serial.print(ml_per_s_2);
  Serial.print("\t");
  Serial.print(ml_per_s_total);
  Serial.print("\t");
  Serial.println(cm_per_s); 
  
}

/////////////////////////////////////////////////////////////////////////////////


//////// define pins, attach interrupt, open serial communication ////////////////////////////////
void setup()    
{ 
  pinMode(temp_pin, INPUT);     //temperatue probe
  pinMode(pot_pin, INPUT);      //potentiometer for user input of PWM level
  pinMode(flow_pin_1, INPUT);     //flow sensor 1
  pinMode(flow_pin_2, INPUT);     //flow sensor 2
  pinMode(pwm_pin_1, OUTPUT);     //PWM output to pump 1
  pinMode(pwm_pin_2, OUTPUT);     //PWM output to pump 2
  attachInterrupt(2,count_samples_1, RISING);  // interrupt for flow_meter_1
  attachInterrupt(4,count_samples_2, RISING);   // interrupt for flow meter_2
  Serial.begin(19200);
  sensors.begin();
  total_time = 0;               // reset total time

}




//////// main loop /////////////////////////////////////////////////////////////////////////////////
void loop()  
{                            
  PWM_1 = 255;                                  // turn pump 1 off
  PWM_2 = 255;                                  // turn pump 2 off
  analogWrite(pwm_pin_1, PWM_1);                  // send pwm to pump
  analogWrite(pwm_pin_2, PWM_2);                  // send pwm to pump
  water_CSA_calc();                             // calculate water CSA - used in all modes
  original_on_off_status = digitalRead(toggle_pin);      // read status of on-off toggle pin
  mode = Serial.read();                         // read mode entered by operator
  total_time = 0;


  
  //////////////// MODE 1 - run protocol ////////////////////////////////////////////  
  if(mode == 49)   // note: serial reads in DEC, where a 1 entered is converted to 49) 
  {               
    read_ID();
    Serial.println();
    Serial.println();
    print2();                                              
    print3();  

    for(i = 0; i < time_array_size; )        
    {                                            
        stage = i + 1;                                          
        stage_time = 0;
        target = flow_array[i];     
        PWM = (second_order*flow_array[i]*flow_array[i]) + (first_order*flow_array[i]) + intercept;
        PWM_calc();  // calculate PWM values to send to pump 1 and 2 
    
        while (stage_time < time_array[i])   // if stage_time is less than value in time array, run loop 
          {                   
           analogWrite(pwm_pin_1, PWM_1);   // output PWM_1 value at pwm_pin_1 (DIO-3)
           analogWrite(pwm_pin_2, PWM_2);   // output PWM_2 value at pwm_pin_2 (DIO-5)           
           flow_counts_1 = 0;
           flow_counts_2 = 0;          
           start_ms = millis();
           interrupts();                             
           delay(4000);                 // count pulses for 4 sec
           noInterrupts();
           flow_hz_1 = flow_counts_1/4;     // calculate pulses per sec
           flow_hz_2 = flow_counts_2/4;     // calculate pulses per sec
           calc_flow(); 
           read_temp();           
           delay(5000 - (millis() - start_ms));            
           stage_time = stage_time + 5; 
           total_time = total_time + 5;     
           print4();
           
           new_on_off_status = digitalRead(toggle_pin);       // test to determine whether to continue or exit
           if(new_on_off_status == original_on_off_status)   // if toggle switch has not changed, continue with data collection 
              {   
                      // go back to while loop and continue     
              }          
           if(new_on_off_status != original_on_off_status)  // if toggle switch has changed, exit
              {               
               stage_time = time_array[i] + 1;            // force exit from while loop (stage time counter) 
               i = time_array_size - 1;                       // force exit from if loop (no. stage counter)
              } 

          }  // back to while loop

          i = i + 1;   // increment to next value in array
          Serial.println("");    // print blank line between stages
          
      } // back to for loop
      
      Serial.println("---------------------------------------------------------------------------------------------------------------------");
      Serial.println("protocol complete");
      Serial.println("");
      
    }   // back to mode 1 loop




    
  //////// MODE 2 - manual control (potentiometer) //////////////////////////////////////////    
  else if(mode == 50)                    
    {
    read_manual_time();
    print5a();
    print3();
    target = 0;
    stage_time = 0;
    while(stage_time < manual_time)
        { 
          stage = 1;
          pot_value = analogRead(pot_pin);  // obtain PWM value from potentiometer, 0-1023 value     
          PWM_pot = pot_value/4;    // divide by 4 to convert pot value to 0-255 scale
                //// pump motor is configured so PWM of 255 is off and PWM of 0 is max speed, therefore
                //// reverse pot scaling so 0 pot_PWM_value yields PWM_value of 255 (for off) and
                //// 255 pot_PWM_value yields a PWM_value of 0 (for max speed) 
          PWM = (PWM_pot*-1)+255;
          if(PWM >= PWM_stall) // prevents PMW going so high (causing pump to go slow) that pump stalls
                   {
                   PWM = PWM_stall;   
                   }
           PWM_1 = PWM;         // PWM value for pump 1
           PWM_calc();
           // output PWM_1 and PWM_2 to corresponding pins
           // re-set flow_counts to 0 and set start_ms to current millisec
           // call count_samples_1 interrupt, read pulses (flow_counts) for 4 sec, close interrupt
           // calculate flow rate and water temp
           // delay for whatever is needed to make total delay 5 sec
           // print results
           // repeat as long as you have not exceeded manual time duration        
           analogWrite(pwm_pin_1, PWM_1);   // output PWM_1 at pwm_pin_1 (DIO-3)
           analogWrite(pwm_pin_2, PWM_2);   // output PWM_2 at pwm_pin_2 (DIO-5)         
           flow_counts_1 = 0;
           flow_counts_2 = 0;
           start_ms = millis();
           interrupts();                             
           delay(4000);                 // count pulses for 4 sec
           noInterrupts();
           flow_hz_1 = flow_counts_1/4;     // calculate pulses per sec for flow meter 1
           flow_hz_2 = flow_counts_2/4;     // calculate pulses per sec for flow meter 1
           calc_flow(); 
           read_temp();           
           delay(5000 - (millis() - start_ms));            
           stage_time = stage_time + 5;
           print4();
           
          // test to determine whether to continue or exit back to options
          new_on_off_status = digitalRead(toggle_pin);
          if(new_on_off_status == original_on_off_status)   // if toggle switch has not changed, continue with data collection 
              {
               stage_time = stage_time;          
              }          
          if(new_on_off_status != original_on_off_status)  // if toggle switch has changed, exit
              {               
               stage_time = manual_time + 1000;    // force time to expire, causing exit back to options window
              }            
        }          
      Serial.println("---------------------------------------------------------------------------------------------------------------------");
      Serial.println("manual control ends");
      Serial.println("");
      }




  //////// MODE 3 - PWM calibration ////////////////////////////////////////////////////////
  else if(mode == 51)             
  {                             
    print6a();
    print3();
    //// loop through for number of stages                                      
    for (int i = 0; i <= cal_array_size-1; )       
    {                                            
        stage = i + 1;                                          
        stage_time = 0;                
        PWM = cal_array[i];
        PWM_calc();
        //// now, loop as many times as necessary (5 sec reads) for each stage
        while (stage_time < cal_interval)    
          {            
           // output PWM value (from array) to pin
           // re-set flow_counts to 0 and set start_ms to current millisec
           // call count_samples_1 interrupt, read pulses (flow_counts) for 4 sec, close interrupt
           // calculate flow rate and water temp
           // delay for whatever is needed to make total delay equal to 5 sec
           // print results (will print every 5 sec)
           // repeat as long as you have not exceeded stage time duration        
           analogWrite(pwm_pin_1, PWM_1);   // output PWM_1 value at pwm_pin_1 (DIO-3)
           analogWrite(pwm_pin_2, PWM_2);   // output PWM_2 value at pwm_pin_2 (DIO-5)
           flow_counts_1 = 0;
           flow_counts_2 = 0;
           start_ms = millis();
           interrupts();                             
           delay(4000);                 // count pulses for 4 sec
           noInterrupts();
           flow_hz_1 = flow_counts_1/4;     // calculate pulses per sec
           flow_hz_2 = flow_counts_2/4;
           calc_flow(); 
           read_temp();           
           delay(5000 - (millis() - start_ms));            
           stage_time = stage_time + 5;
           print4(); 

           new_on_off_status = digitalRead(toggle_pin);       // test to determine whether to continue or exit
           if(new_on_off_status == original_on_off_status)   // if toggle switch has not changed, continue with data collection 
              {   
                      // go back to while loop and continue     
              }          
           if(new_on_off_status != original_on_off_status)  // if toggle switch has changed, exit
              {               
               stage_time = stage_time + time_array[i] + 1;   // force exit from while loop 
               i = i + time_array_size + 1;                       // force exit from if loop
              } 

          }  // back to while loop

          i = i + 1;   // increment to next value in array
          Serial.println("");    // print blank line between stages
          
      } // back to for loop
      
    Serial.println("---------------------------------------------------------------------------------------------------------------------");
    Serial.println("flow x PWM calibration complete");
    Serial.println("");
    }




  /////// repeat betwen modes ///////////////////////////////////////////////////////////
  else                  
    {
      time1 = millis();
      read_temp();
      Serial.println(" Choose one of the following options:");    
      Serial.println("      run protocol    -> 1");
      Serial.println("      manual control  -> 2");
      Serial.println("      PWM x flow cal. -> 3");
      Serial.println("");
      Serial.print(" Water temperature = ");
      Serial.print(sensors.getTempCByIndex(0),1);
      Serial.println(" degC");
      Serial.println("");
      Serial.println("");
      Serial.println("");
      Serial.println("");
      time2 = millis();
      elapsed_time = time2-time1;
      delay(5000 - elapsed_time);
    }
}



