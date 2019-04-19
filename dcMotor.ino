#define P1A 2 // define pin 10as for P1A
#define P2A 4// define pin 11 as for P2A
#define EN12 3 // define pin 9 as for 1,2EN enable
int trigpin=9;
int echopin=10;
const int speedStep =15;
const int speedDelay = 1000;// delay between speed increment
 double kp=0;
double kd=10;
double ki=0;
double input=180;
int output;
int setpoint=12;
double error;
int count=0;
double P;
double I;
double D;
double integral;
double derivative;
int lastinput=0;
double maxvalue[10][10];



void setup() {
  // L293 Motor Control Code by Robojax.com 20180811
  Serial.begin(9600);// setup Serial Monitor to display information
  pinMode(P1A, OUTPUT);// define pin as OUTPUT for P1A
  pinMode(P2A, OUTPUT);// define pin as OUTPUT for P2A
  pinMode(EN12, OUTPUT);// define pin as OUTPUT for 1,2EN
  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  //OCR2A = 180;
pinMode(trigpin,OUTPUT);
pinMode(echopin,INPUT);
  Serial.println("Robojax");
  Serial.println("L293D Motor Speed Control");
  
  // L293 Motor Control Code by Robojax.com 20180811   
}

void loop() {
  long duration,distance;
 
  // L293d Motor Control Code by Robojax.com 20180811

kp=analogRead(A0)/100;
ki=analogRead(A1)/100;
setpoint=linearIntra2(analogRead(A2));
  
    digitalWrite(trigpin,HIGH);
delayMicroseconds(1000);
digitalWrite(trigpin,LOW);
duration=pulseIn(echopin,HIGH);
output=(duration/2)/29.1;

//delay(500);
if(output>100){
  output=3;
  }
error=setpoint-output;






  P=kp*error;

 integral += error * (double)(1);
 I=ki*integral;

 derivative=(error - lastinput)  / (double)(1);
 D=(-kd*derivative);
  

  lastinput=error;

input=(double)P+D+I;


if(input<0){
  input*=-1;
  }
if(input>maxvalue[(int)ki][(int)kd]){
  maxvalue[(int)ki][(int)kd]=input;
  }
  Serial.println(maxvalue[(int)ki][(int)kd]);
 // Serial.print(input);
  //Serial.print("      ");
   
input=linearIntra(input,maxvalue[(int)ki][(int)kd]);

//Serial.print(input);
  //Serial.print("      ");
  
//Serial.print(maxvalue);
  
   //Serial.println(error);
  
    //Serial.print(P);
 //Serial.print(",");
     //Serial.println(I);
  
   // Serial.print(setpoint);
 
 

//if(error==1 || error==-1 || error==0){
//Serial.print("SetPoint Reached");
  //analogWrite(EN12 ,0);
 //}


 if(error<0){
  analogWrite(EN12 ,input);// Enable 1A and 2A 
    digitalWrite(P1A,HIGH);// send PWM with spd value to P1A
    digitalWrite(P2A,LOW);// LOW singal to P2A  

   
  }
else if(error>0){
  analogWrite(EN12 ,input);// Enable 1A and 2A 
    digitalWrite(P2A,HIGH);// send PWM with spd value to P2A
    digitalWrite(P1A,LOW);// LOW singal to P1A  

  }




 
}




/*
 * L293D(char dir,int spd, int en)
 * dir is character either L for CW direction
 *  or R for CCW direction
 *  en is integer 1 to totate, 0 for stop
 *  spd is the speed value from 0 to 255
 */


int linearIntra(double x,double x2){
  int y=((230*x)+(25*x2))/x2  ;

  return y;
  }

 int linearIntra2(double x){
  int y=((30*x)+2670)/990 ;

  return y;
  }
