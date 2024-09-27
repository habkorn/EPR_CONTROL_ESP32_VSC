#include <Arduino.h>

// #include "movingAvgFloat.h"   

#include "MovingAverageFilter.h"

int analogPressPin = A0;

// Define the onboard LED pin

// RGB pins:
// Red, r_pin   = 14
// Green, g_pin = 15
// Blue, b_pin  = 16

double pAvg;
int pvAvg;
double dtAvg;

MovingAverageFilter avgGen1(10);
MovingAverageFilter avgPressureFilter(10);
MovingAverageFilter avgGen3(10);



// movingAvgFloat avgGen1(10);  
// movingAvgFloat avgGen2(10);  
// movingAvgFloat avgGen3(10);  


// PID parameters

uint64_t currentMicros = esp_timer_get_time(); // used by publish
uint64_t currentMicrosInterval = esp_timer_get_time(); // used by publish


double dc_set=0.;

double pRaw=0;
int pvRaw=0;



double set_p = 0.; // bar
double p_in=8.;// bar

double current_pressure_value = 0;

double prev_error = 0;
double integral = 0;


double Kp =2;//3.8; //3
double Ti_Inv = 0.041;// 0.003;// 0.032;
double Td = 0.02;//0.01;

double gamma_=0.2;//0.;



const int ledPinRed = 14; // Onboard LED 
const int ledPinGreen = 15; // Onboard LED 
const int ledPinBlue = 16; // Onboard LED 


// PWM Configuration

const int pwmChannelValve[] = {4, 5, 6, 7, 8, 9};     // Choose a PWM channel (0-15)
const int valvePins[]       = {4, 5, 6, 7, 8, 9};

// pinout: https://docs.arduino.cc/resources/datasheets/ABX00083-datasheet.pdf
const int pwmFrequencyValve = 1000; // Set frequency 
const int pwmResolutionValve = 14;   // Set resolution, max: 14 bits

uint32_t dc_u_int_max_val=pow(2, pwmResolutionValve)-1;//2^14-1=16383;
uint32_t dc_u_int;

const int pwmChannelRedLED = 0;     // Choose a PWM channel (0-15)
const int pwmChannelGreenLED = 1;     // Choose a PWM channel (0-15)
const int pwmChannelBlueLED = 2;     // Choose a PWM channel (0-15)

const int pwmFrequencyLED = 500; // Set frequency to 500 Hz
const int pwmResolutionLED = 8;   // Set resolution to 8 bits (values from 0 to 255)

int i=0;

int valveNr=1;

void setup() 
{

  Serial.begin(115200);

  // delay(10);



  // Initialize the LED pin as an output
  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinBlue, OUTPUT);
  
  // Configure the PWM channel with the chosen frequency and resolution
  ledcSetup(pwmChannelRedLED, pwmFrequencyLED, pwmResolutionLED);
  ledcSetup(pwmChannelGreenLED, pwmFrequencyLED, pwmResolutionLED);
  ledcSetup(pwmChannelBlueLED, pwmFrequencyLED, pwmResolutionLED);

  for(i=0;i<sizeof(pwmChannelValve)/sizeof(int);i++)
  {
      ledcSetup(pwmChannelValve[i], pwmFrequencyValve, pwmResolutionValve); 
      ledcAttachPin(valvePins[i], pwmChannelValve[i]); 
  }

  ledcAttachPin(ledPinRed, pwmChannelRedLED);
  ledcAttachPin(ledPinGreen, pwmChannelGreenLED);
  ledcAttachPin(ledPinBlue, pwmChannelBlueLED);  

  currentMicros = esp_timer_get_time();


}



double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int getPMOccurancesInStr(String str)
{
  
  int count = 0;

  for (uint8_t i=0; i<str.length(); i++)
    if (str[i] == '+') count++;
    else if(str[i] == '-') count--;

  return count;
}

void loop() 
{


  // Write the duty cycle to the PWM pin
  //analogWrite(pwmPin, dutyCycle);

  pvRaw = analogRead(analogPressPin);  // read the input pin

  pvAvg = avgGen1.process(pvRaw); 

  // output = movingAverageFilter.process(input); 
 

  // pRaw=mapf(pvRaw,105,600,0,5.93);  // pressure in bar(g)

  pRaw=mapf(pvRaw,620,4095,0,7);  // pressure in bar(g)


  if (pRaw<0) pRaw=0;

  pAvg = avgPressureFilter.process(pRaw); 



  // Read the analog sensor value
  current_pressure_value = pAvg;

  double error = set_p - current_pressure_value;

  double dt=dtAvg; //(millis()-currentMilliSRunning)/1000.;
  
  double derivative = (error - prev_error)/dt;
  prev_error = error;

  //Ti_Inv=0.;
  //Td=0.;
  
  integral += error * dt;


  double i_Part_unrest=Kp * Ti_Inv * integral;

  double i_Part=i_Part_unrest;

  double u_max=p_in;//p_in * Kp;

  // if (i_Part<0) i_Part=0;
  // if (i_Part>u_max) i_Part=u_max;

  // Anti-Windup 

  double i_Part_korr=(i_Part - i_Part_unrest)*gamma_;

  integral += i_Part_korr * dt;

  i_Part=Kp * Ti_Inv * integral;
 
  
  double p_Part = Kp * error;
  double d_Part= Kp * Td * derivative;

  //d_Part=0;

  double u = p_Part + i_Part + d_Part ;


  // Stellgrößenbeschränkung
  if (u<0) u=0;
  // if (u>u_max) u=u_max;

  // Map the control output to PWM range (0-100) for analogWrite
  //uint16_t dc_u_int = mapf(u, 0, p_in*Kp+1., 0, ICR1);

// Set the duty cycle (0 to max_val)

  // u=u_max/3.;
  dc_u_int = mapf(u, 0, u_max, 0, dc_u_int_max_val);

  if (dc_u_int>dc_u_int_max_val) dc_u_int=dc_u_int_max_val;


  // dc_u_int=0.7*dc_u_int_max_val;

  
  // dc_u_int= dc_set/100.*dc_u_int_max_val;

  ledcWrite(pwmChannelValve[3],  dc_u_int);

  //Serial.println("Hallo, Serial Monitor!");

  while(esp_timer_get_time()-currentMicrosInterval<100.){} // slow down
  
  
  dtAvg = avgGen3.process(esp_timer_get_time()-currentMicrosInterval) / 1000000.; 

  
  // while(millis()-currentMillisRunning<1);

  currentMicrosInterval=esp_timer_get_time();

    // while (true) 
    // {
    // // Infinite loop to halt the program
    // delay(1000);
    // Serial.println(esp_timer_get_time()-currentMillis);
    // Serial.println(dc_u_int);
    // Serial.println("");
    // }

  if (esp_timer_get_time()-currentMicros>100000)
  {

    Serial.print(">");
    Serial.print("u:");
    Serial.print(u);

    Serial.print(",error:");
    Serial.print(error);     

    Serial.print(",set_p:");   
    Serial.print(set_p);   

    Serial.print(",pAvg:");
    Serial.print(pAvg);  

    Serial.print(",pRaw:");
    Serial.print(pRaw);    

    Serial.print(",pvRaw:"); 
    Serial.print(pvRaw);     

    Serial.print(",pvAvg:");
    Serial.print(pvAvg);    

    Serial.print(",dc:");
    // Serial.print(((double)dc_u_int)/max_val,3);    
    Serial.print(((double)dc_u_int)/((double)dc_u_int_max_val),3);    

    Serial.print(",dc_u_int:");
    Serial.print(dc_u_int);   

    Serial.print(",u_max:");
    Serial.print(u_max);   
    
    Serial.print(",dtAvg:");
    Serial.print(dtAvg*1000.,1);   
    
    Serial.print(",dt:");
    Serial.print(dt*1000.,1);      

    Serial.print(",Kp:");
    Serial.print(Kp);   

    Serial.print(",Ti_Inv:"); 
    Serial.print(Ti_Inv,3);   

    Serial.print(",Td:");  
    Serial.print(Td,3);   
    
    Serial.print(",gamma_:");  
    Serial.print(gamma_,2);  

    Serial.print(",p_Part:"); 
    Serial.print(p_Part); 
    
    Serial.print(",i_Part:"); 
    Serial.print(i_Part); 
    
    Serial.print(",d_Part:"); 
    Serial.print(d_Part); 
    
    Serial.print(",Ti:"); 
    Serial.println(1/Ti_Inv); 
    

    currentMicros=esp_timer_get_time();

  }



  if(Serial.available())
  {

    String inputStr = Serial.readStringUntil('\n');

    if(inputStr.startsWith("c"))
    {
      dc_set= (inputStr.substring(1)).toDouble();
      // Accuracy @500 Hz and @ 2000 Hz: 0.005 %
      Serial.print("dc_set%:");
      Serial.println(dc_set,3);
    }
      
    if(inputStr.startsWith("g"))
      gamma_+=0.1*getPMOccurancesInStr(inputStr);

    if(inputStr.startsWith("p"))
      Kp+=0.1*getPMOccurancesInStr(inputStr);

    else if(inputStr.startsWith("i"))
      Ti_Inv+=0.0025*getPMOccurancesInStr(inputStr);

    else if(inputStr.startsWith("d"))
      Td+=0.01*getPMOccurancesInStr(inputStr);

    else if(inputStr.startsWith("t"))
      set_p+=0.1*getPMOccurancesInStr(inputStr);      

    else if(inputStr.startsWith("s"))
      set_p=(inputStr.substring(1)).toDouble();

       		
  }





}
