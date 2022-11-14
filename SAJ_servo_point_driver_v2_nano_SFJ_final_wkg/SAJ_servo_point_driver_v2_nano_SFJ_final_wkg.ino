// INCLUDE LIBRARIES
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//SET PCA9685 BOARD ADDRESS
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //setup the board address - defaults to 0x40 if not specified

// code modified from slow motion servo Sketch (Rob @ Little Wicket Railway)

  int setStatus[5]; //table to hold the status of each device
  int closeValue[5] = {1075,1250,1060,1350,1485}; //table of closed point values
  int throwValue[5] = {1750,1850,1585,1850,1985}; //table of thrown point values
  int currentPosition[5]; //table storing current position
  int stepSize[5] = {5,5,5,5,5}; //step increment per servo
  int delayTime[5] = {30,30,30,30,30}; //delay for each step increment per servo
  int servoConnection; //output on servo driver board
  int inputStatus; //status of point, 0 or 1, to change
  unsigned long previousMillis[5]; // tracking variable for time elapsed
  
// code modified from route control panel (Ron @ Ron's Trains and Things)

  int pointState[5] = {0, 0, 0, 0, 0}; // declare point initial state
  int selectButton[8] = {5, 6, 7, 8, 9, 10, 11, 12}; // declare button pins
  int btn00New; // button states
  int btn00Old = 1;
  int btn01New;
  int btn01Old = 1;
  int btn02New;
  int btn02Old = 1;
  int btn03New;
  int btn03Old = 1;
  int btn04New;
  int btn04Old = 1;
  int btn05New;
  int btn05Old = 1;
  int btn06New;
  int btn06Old = 1;
  int btn07New;
  int btn07Old = 1;


// code modified from slow motion servo Sketch (Rob @ Little Wicket Railway)

// slow motion servo routine
void moveServos(int inputStatus, int servoConnection)
{
  unsigned long currentMillis = millis();
  if ((inputStatus != setStatus[servoConnection]) && (currentMillis - previousMillis[servoConnection] >= delayTime[servoConnection]))
  {
    previousMillis[servoConnection] = currentMillis;
    if (inputStatus == 1)
    {
      if (currentPosition[servoConnection] < throwValue[servoConnection])
      {
        currentPosition[servoConnection] = currentPosition[servoConnection] + stepSize[servoConnection];
        pwm.writeMicroseconds(servoConnection, currentPosition[servoConnection]);
      }
      else
      {
          setStatus[servoConnection]=1;
      }
    }
    else
    {
      if (currentPosition[servoConnection] > closeValue[servoConnection])
      {
        currentPosition[servoConnection] = currentPosition[servoConnection] - stepSize[servoConnection];
        pwm.writeMicroseconds(servoConnection, currentPosition[servoConnection]);
      }
      else
      {
          setStatus[servoConnection]=0;
      }
    }
  }
}

void setup()
{

// code modified from slow motion servo Sketch (Rob @ Little Wicket Railway)
  
  for (int i = 0; i < 5; i++)  //point states initialised to closed
  {
     currentPosition[i]=closeValue[i];
  }
  pwm.begin(); //sets PWM output on servo driver board
  pwm.setPWMFreq(50);

// code modified from route control panel (Ron @ Ron's Trains and Things)

  for (int p=0; p<5; p++)    // initialise output array
    {
      pointState[p] = 0;     // point states initailised to closed
      //pinMode ((p + 3), OUTPUT); // LED indicators      
    }
    
  for (int b=0; b<8; b++)    // initialise button array
    {
      pinMode (selectButton[b], INPUT_PULLUP);
    }

}

void loop()
{

// code modified from route control panel (Ron @ Ron's Trains and Things)

  btn00New=digitalRead(selectButton[0]); // checks for each button press and sets outputs
  if (btn00Old==0 && btn00New==1)
    {
      pointState[0] = 1;
      pointState[1] = 0;
    }
  btn00Old=btn00New;

  btn01New=digitalRead(selectButton[1]);
  if (btn01Old==0 && btn01New==1)
    {
      pointState[0] = 0;
      pointState[1] = 1;
    }
  btn01Old=btn01New;

  btn02New=digitalRead(selectButton[2]);
  if (btn02Old==0 && btn02New==1)
    {
      pointState[2] = 1;
    }
  btn02Old=btn02New;

  btn03New=digitalRead(selectButton[3]);
  if (btn03Old==0 && btn03New==1)
    {
      pointState[2] = 0;
      pointState[3] = 0;
      pointState[4] = 0;
    }
  btn03Old=btn03New;

  btn04New=digitalRead(selectButton[4]);
  if (btn04Old==0 && btn04New==1)
    {
      pointState[2] = 0;
      pointState[3] = 0;
      pointState[4] = 1;
    }
  btn04Old=btn04New;

  btn05New=digitalRead(selectButton[5]);
  if (btn05Old==0 && btn05New==1)
    {
      pointState[2] = 0;
      pointState[3] = 1;
    }
  btn05Old=btn05New;

  btn06New=digitalRead(selectButton[6]);
  if (btn06Old==0 && btn06New==1)
    {
     //point change statements
    }
  btn06Old=btn06New;

  btn07New=digitalRead(selectButton[7]);
  if (btn07Old==0 && btn07New==1)
    {
      //point change staements 
    }
  btn07Old=btn07New;

// code modified from slow motion servo Sketch (Rob @ Little Wicket Railway)
  
  moveServos(pointState[0],0);
  moveServos(pointState[1],1);  
  moveServos(pointState[2],2);
  moveServos(pointState[3],3);
  moveServos(pointState[4],4);
  
}
