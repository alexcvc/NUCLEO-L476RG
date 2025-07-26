/***********************************************************
 *                    TEMPERATURE SENSOR
 *                    ==================
 * This program reads the analog temperature and sends         
 * it to the Nucleo-L476RG board over serial UART
 * 
 * Author: Dogan Ibrahim
 * Date  : October, 2020
 * File  : ArdTemp.c
 ***********************************************************/
#include <SoftwareSerial.h>
SoftwareSerial MySerial(6, 7);             // RX, TX
int TempPin = 0;
int val, temp;
float mv, temperature;
String tempstr;

void setup() 
{Serial.begin(9600);
  MySerial.begin(9600);  
}

void loop() 
{
  val = analogRead(TempPin);
  mv = val * 5000.0 / 1024.0;
  temperature = (mv - 500.0) / 10.0;
  temp = (int)temperature;
  
  if(temp < 10)
      tempstr = "0" + String(temp);
  else
      tempstr = String(temp);
      
  MySerial.print(tempstr);
  Serial.print(tempstr);
  delay(1000);  
}
