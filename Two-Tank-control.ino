#include <Wire.h>
#define I2C_ADDR 0x27
#define BACKLIGHT_PIN 3
#include <LiquidCrystal_I2C.h>

//LCD condition
LiquidCrystal_I2C lcd(0x27, 16, 2);// LCD condition

//Time stamp

  int t = 0;
  
//Ultrasonic conditions 1

  int pingPin = 13; //Trig
  int inPin = 12;
  float duration, cm,cm1, cm2, lv;
  
//Ultrasonic conditions 2

  int pingPin2 = 7; //Trig
  int inPin2 = 6;
  float duration2, CM,CM1, CM2, LV;
  
//Motor A connections

  int enA = 11;
  int in1 = 10;
  int in2 = 9;
  int input_PWM = 0;
  int spd = 0;
  
//Control variables

  float h1 = 0;
  float h2 = 0;
  float h1s = 20.98;
  float h2s = 20.87;
  float x[]={h1,h2};
  float K_component1;
  float K_component2;
  float xQx[10];
  float u_cal;
  float us = 39.84;
  float u_real;
  int i;
  int j;
  int index;
  
//Inverst Q

  float Q1[ 2 ][ 2 ] = { { 0.0009, 0.0001 }, { 0.0001, 0.0011 } };
  float Q2[ 2 ][ 2 ] = { { 0.00010, 0.0001 }, { 0.0001, 0.0011 } };
  float Q15[ 2 ][ 2 ] = { { 0.0015, 0.0002 }, { 0.0002, 0.0017 } };
  float Q30[ 2 ][ 2 ] = { { 0.0025, 0.0003 }, { 0.0003 , 0.0030 } };
  float Q45[ 2 ][ 2 ] = { { 0.0043, 0.0004 }, { 0.0004, 0.0051 } };
  float Q60[ 2 ][ 2 ] = { { 0.0072, 0.0007 }, { 0.0007, 0.0086 } };
  float Q90[ 2 ][ 2 ] = { { 0.0242, 0.0027 }, { 0.0027, 0.0300 } };
  float Q120[ 2 ][ 2 ] = { { 0.0627, 0.0086 }, { 0.0086, 0.0949 } };
  float Q160[ 2 ][ 2 ] = { { 0.1807, 0.0343 }, { 0.0343, 0.4801 } };
  float Q200[ 2 ][ 2 ] = { { 0.5995, 0.1219 }, { 0.1219, 2.5959 } };
  
//Control K

  float K[10][2] = { {-0.1444, -0.1413},  //1
                   {-0.1471, -0.1442},    //2
                   {-0.1787, -0.1787},    //15
                   {-0.2414, -0.2238},    //30
                   {-0.3314, -0.2603},    //45
                   {-0.4587, -0.2844},    //60
                   {-0.9184, -0.2863},    //90
                   {-1.5004, -0.2912},     //120
                   {-2.5495, -0.4882},    //160
                   {-4.5873, -0.9052}};   //200

void setup() 
{
  
//Set all the motor control pins

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
//Turn off motors - Initial state

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

//LCD   

  lcd.init();
  lcd.backlight();

//Set serial monitor signal port  

  Serial.begin(9600);

  delay(5000);
}

void loop()
{

//UL1

pinMode(pingPin, OUTPUT);
digitalWrite(pingPin, LOW);
delayMicroseconds(2);
digitalWrite(pingPin, HIGH);
delayMicroseconds(5);
digitalWrite(pingPin, LOW);
pinMode(inPin, INPUT);
duration = pulseIn(inPin, HIGH);
cm = microsecondsToCentimeters(duration);

//UL2

pinMode(pingPin2, OUTPUT);
digitalWrite(pingPin2, LOW);
delayMicroseconds(2);
digitalWrite(pingPin2, HIGH);
delayMicroseconds(5);
digitalWrite(pingPin2, LOW);
pinMode(inPin2, INPUT);
duration2 = pulseIn(inPin2, HIGH);
CM = microsecondsToCentimeters2(duration2);

//Out FN

cm2 = firstUL(cm);
CM2 = secondUL(CM);

/*
 // Manual Control of spd
 * 
    while (Serial.available() > 0) 
  {
    String spd_input = Serial.readString();// s1 is String type variable.
    Serial.print("Received Data => ");

    spd = spd_input.toInt();
    Serial.println(spd);
    //Serial.println(s1);//display same received Data back in serial monitor.
    //delay(500);
  }
  */

  h1 = cm2-h1s;
  h2 = CM2-h2s;

  float x[]={h1,h2};

  // Control action

  xQx[0] = (x[0]*((x[0]*Q1[0][0])+(x[1]*Q1[0][1])))+(x[1]*((x[0]*Q1[1][0])+(x[1]*Q1[1][1])));
  xQx[1] = (x[0]*((x[0]*Q2[0][0])+(x[1]*Q2[0][1])))+(x[1]*((x[0]*Q2[1][0])+(x[1]*Q2[1][1])));
  xQx[2] = (x[0]*((x[0]*Q15[0][0])+(x[1]*Q15[0][1])))+(x[1]*((x[0]*Q15[1][0])+(x[1]*Q15[1][1])));
  xQx[3] = (x[0]*((x[0]*Q30[0][0])+(x[1]*Q30[0][1])))+(x[1]*((x[0]*Q30[1][0])+(x[1]*Q30[1][1])));
  xQx[4] = (x[0]*((x[0]*Q45[0][0])+(x[1]*Q45[0][1])))+(x[1]*((x[0]*Q45[1][0])+(x[1]*Q45[1][1])));
  xQx[5] = (x[0]*((x[0]*Q60[0][0])+(x[1]*Q60[0][1])))+(x[1]*((x[0]*Q60[1][0])+(x[1]*Q60[1][1])));
  xQx[6] = (x[0]*((x[0]*Q90[0][0])+(x[1]*Q90[0][1])))+(x[1]*((x[0]*Q90[1][0])+(x[1]*Q90[1][1])));
  xQx[7] = (x[0]*((x[0]*Q120[0][0])+(x[1]*Q120[0][1])))+(x[1]*((x[0]*Q120[1][0])+(x[1]*Q120[1][1])));
  xQx[8] = (x[0]*((x[0]*Q160[0][0])+(x[1]*Q160[0][1])))+(x[1]*((x[0]*Q160[1][0])+(x[1]*Q160[1][1])));
  xQx[9] = (x[0]*((x[0]*Q200[0][0])+(x[1]*Q200[0][1])))+(x[1]*((x[0]*Q200[1][0])+(x[1]*Q200[1][1])));

for (i=1 ; i<10 ;i++) // i repersent to Q-th
  {
    if (xQx[i] > 1)
    {
     K_component1 = K[i][0];
     K_component2 = K[i][1];    
      index = i-1;
      break;
    }
  }

  u_cal = (x[0]*K_component1)+(x[1]*K_component2);
  u_real = u_cal + us;

  if (u_real > 40)
  {
    input_PWM = (u_real - 37.754)/0.0288;
    if (input_PWM > 255)
    {
      input_PWM = 255;
    }
  }
  else if (u_real > 33)
  {
    input_PWM = (u_real-30.386)/0.1159;
  }
  else
  {
   input_PWM = (u_real-0.9031)/1.4265; 
  }
  
  spd = input_PWM;

//Serial.print("Tank1 = ");
Serial.print("\t");
Serial.print(abs(cm2));
Serial.print("\t");

//Serial.print("Tank2 = ");
Serial.print(abs(CM2));
Serial.print("\t");

//Serial.print("h1 ");
Serial.print(h1);
Serial.print("\t");

//Serial.print("h2 ");
Serial.print(h2);
Serial.print("\t");

//Serial.print("U cal = ");
Serial.print(abs(u_cal));
Serial.print("\t");

//Serial.print("u_real = ");
Serial.print(u_real);
Serial.print("\t");

//Serial.print("index = ");
Serial.print(index);
Serial.print("\t");

//Serial.print("PWM = ");
Serial.println(spd);

// spd convert
  
  analogWrite(enA, spd);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  //LCD coding
  lcd.setCursor(0, 0);
  lcd.print(cm2);
  lcd.setCursor(0, 1);
  lcd.print(CM2);
  delay(1000);

  lcd.clear();
}

//UL1 FN

float microsecondsToCentimeters(float microseconds)
  {
return microseconds / 29 / 2;
  }

float firstUL (float cm)
{
    if (cm>= 21)
{
  cm2 = 0;
}
else if (cm >= 20)
{
  cm1 = cm - (0.01*cm);
  cm2 = 22.5 - cm1;
}
else if (cm >= 18.5)
{
  cm1 = cm - (0.067*cm);
  cm2 = 22.5 - cm1;
}
else if (cm >= 16.5)
{
  cm1 = cm - (0.061*cm);
  cm2 = 22.5 - cm1;
}
else if (cm >= 14.5)
{
  cm1 = cm - (0.1462*cm);
  cm2 = 22.5 - cm1;
  }
else if (cm >= 12.5)
{
  cm1 = cm + (0.004*cm);
  cm2 = 22.5 - cm1;
}
else if (cm >= 10.5)
{
  cm1 = cm + (0.038*cm);
  cm2 = 22.5 - cm1;
}     
else if (cm >= 8.5)
{
  cm1 = cm + (0.028*cm);
  cm2 = 22.5 - cm1;
}
else if (cm >= 6.5)
{
  cm1 = cm - (0.158*cm);
  cm2 = 22.5 - cm1;
}
else if (cm >= 4.5)
{
  cm1 = cm - (0.115*cm);
  cm2 = 22.5 - cm1;
}
else if (cm >= 2.5)
{
  cm1 = cm - (0.33*cm);
  cm2 = 22.5 - cm1;
}

return cm2;
  }

  // UL2 FN

  float microsecondsToCentimeters2(float microseconds)
  {
return microseconds / 29 / 2;
  }

  float secondUL (float cm)
  {
if (CM >= 20)
{
  CM2 = 0;
}
else if (cm >= 18 )
{
  CM1 = CM + (0.066 * CM);
  cm2 = 21 -cm1;
}
else if (CM >= 12 )
{
  CM1 = CM + (0.02 * CM);
  CM2 = 21 -CM1;
}
else if (CM >= 9 )
{
  CM1 = CM - (0.01 * CM);
  CM2 = 21 -CM1;
}
else if (CM >= 6)
{
  CM1 = CM - (0.04 * CM);
  CM2 = 21 -CM1;
}
else if (CM >= 3 )
{
  CM1 = CM - (0.16 * CM);
  CM2 = 21 -CM1;
}
else if (CM < 3 )
{
  CM1 = CM + (0.55 * CM);
  CM2 = 21 -CM1;
}
return CM2;
}
  
  
