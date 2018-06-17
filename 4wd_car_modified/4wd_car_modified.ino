#include <LiquidCrystal_I2C.h>   //including libraries of I2C-LCD1602 liquid crystal 
#include <Wire.h>              //including libraries of I2C
#define SensorLeft    6   //sensor left pin of line tracking module 
#define SensorMiddle  9   //sensor middle pin of line tracking module
#define SensorRight   11  //sensor right pin of line tracking module

unsigned char SL;        //state of left sensor of line tracking module
unsigned char SM;        //state of middle sensor of line tracking module
unsigned char SR;        //state of right sensor of line tracking module
int inputPin = A0; // ultrasonic module   ECHO to A0
int outputPin = A1; // ultrasonic module  TRIG to A1
unsigned char Bluetooth_val;       // ultrasonic module  TRIG to A1
LiquidCrystal_I2C lcd(0x27, 16, 2); //defining liquid crystal

#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  10    //pin of controlling speed---- ENB of motor driver board

int pinLB = 2;           //pin of controlling turning---- IN1 of motor driver board
int pinLF = 4;           //pin of controlling turning---- IN2 of motor driver board
int pinRB = 7;          //pin of controlling turning---- IN3 of motor driver board
int pinRF = 8;          //pin of controlling turning---- IN4 of motor driver board
unsigned char Lpwm_val = 150; //initialized left wheel speed at 250
unsigned char Rpwm_val = 150; //initialized right wheel speed at 250
int Car_state = 0;           //the working state of car
int servopin = 3;            //defining digital port pin 3, connecting to signal line of servo motor
int myangle;                //defining variable of angle
int pulsewidth;              //defining variable of pulse width
unsigned char DuoJiao = 103;  //initialized angle of motor at 90°

void Sensor_IO_Config()     //IO initialized function of three line tracking , all setting at input
{
  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);
}

void Sensor_Scan(void) //function of reading-in signal of line tracking module
{
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
}

void servopulse(int servopin, int myangle) //defining a function of pulse
{
  pulsewidth = (myangle * 11) + 500; //converting angle into pulse width value at 500-2480
  digitalWrite(servopin, HIGH); //increasing the level of motor interface to upmost
  delayMicroseconds(pulsewidth); //delaying microsecond of pulse width value
  digitalWrite(servopin, LOW); //decreasing the level of motor interface to the least
  delay(20 - pulsewidth / 1000);
}

void Set_servopulse(int set_val)
{
  for (int i = 0; i <= 10; i++) //giving motor enough time to turn to assigning point
  {
    servopulse(servopin, set_val); //invokimg pulse function
    delay(100);
  }
}

void M_Control_IO_config(void)
{
  pinMode(pinLB, OUTPUT); // /pin 2
  pinMode(pinLF, OUTPUT); // pin 4
  pinMode(pinRB, OUTPUT); // pin 7
  pinMode(pinRF, OUTPUT); // pin 8
  pinMode(Lpwm_pin, OUTPUT); // pin 11 (PWM)
  pinMode(Rpwm_pin, OUTPUT); // pin10(PWM)
}

void Set_Speed(unsigned char Left, unsigned char Right) //function of setting speed
{
  analogWrite(Lpwm_pin, Left);
  analogWrite(Rpwm_pin, Right);
}

void turnR()    //  turning right(dual wheel)
{
  digitalWrite(pinRB, LOW); // making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW); // making motor move towards left rear
  digitalWrite(pinLF, HIGH);
  Car_state = 1;
  show_state();
}

void advance()        //going forward
{
  digitalWrite(pinRB, LOW); //making motor move towards right rear
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW); //making motor move towards left front
  Car_state = 4;
  show_state();
}

void back()         //back up
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW );  //making motor move towards right front
  digitalWrite(pinLB, LOW);  //making motor move towards left rear
  digitalWrite(pinLF, HIGH);
  Car_state = 3;
  show_state();
}

void stopp()        //stop
{
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
  Car_state = 5;
  show_state();
}

void turnL()         //turning left(dual wheel)
{
  digitalWrite(pinRB, HIGH); //making motor move towards right rear
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH); //making motor move towards left rear
  digitalWrite(pinLF, LOW);
  Car_state = 2;
  show_state() ;
}

void show_state(void)
{
  lcd.setCursor(0, 1);
  switch (Car_state)
  {
    case 4: lcd.print(" Go  "); Serial.print(" \r\n GO");
      break;
    case 3: lcd.print("Back "); Serial.print(" \r\n Back");
      break;
    case 2: lcd.print("Left "); Serial.print(" \r\n Left");
      break;
    case 1: lcd.print("Right"); Serial.print(" \r\n Right");
      break;
    case 5: lcd.print("Stop "); Serial.print(" \r\n Stop");
      break;
    default:
      break;
  }
}

void LCD1602_init(void)            //including initialized function of liquid crystal
{
  lcd.init();  //invoking initialized function of LCD in LiquidCrystal_I2C.h
  delay(10);  //delaying for 10 millisecond
  lcd.backlight(); //open backlight of LCD1602
  lcd.clear();    //clear screen
}

void Show_V(unsigned char V)
{
  lcd.setCursor(11, 0);
  lcd.print("V=    ");
  lcd.setCursor(13, 0);
  lcd.print(V, DEC);
  Serial.print("\n Speed = ");
  Serial.print(V, DEC);
}

void Show_DuoJiao(unsigned char Jiao)
{
  lcd.setCursor(6, 1);
  lcd.print("C=    ");
  lcd.setCursor(8, 1);
  lcd.print(Jiao, DEC);
  Serial.print("\n JiaoDu = ");
  Serial.print(Jiao, DEC);
}

void LineTracking_Mode(void) //function of line tracking
{
  lcd.setCursor(0, 0);  //setting cursor in the first row and column
  lcd.print("LineTracking_Mode           ");
  Sensor_Scan();
  if (SM == HIGH)// middle sensor in black area
  {
    if (SL == LOW & SR == HIGH) // black on left, white on right, turn left
    {
      turnR();
    }
    else if (SR == LOW & SL == HIGH) // white on left, black on right, turn right
    {
      turnL();
    }
    else // white on both sides, going forward
    {
      advance();
    }
  }
  else // middle sensor on white area
  {
    if (SL == LOW & SR == HIGH) // black on left, white on right, turn left
    {
      turnR();
    }
    else if (SR == LOW & SL == HIGH) // white on left, black on right, turn right
    {
      turnL();
    }
    else // all white, stop
    {
      back();
      delay(100);
      stopp() ;
    }
  }
}

void Self_Control(void)//self-going, ultrasonic obstacle avoidance
{
  int H;
  lcd.setCursor(0, 0);  //setting cursor in the first row and column
  lcd.print("Self_Ctr        ");
  Show_V(Lpwm_val);
  
  Set_servopulse(DuoJiao);

  int range = Ultrasonic_Ranging(1);
  Serial.print("\ndistance : ");
  Serial.print(range);

  if(range>60 | range<0) {
    switch(DuoJiao) {
      case 103:
        advance();
        break;
      case 45:
        turnR();
        DuoJiao = 103;
        break;
      case 135:
        turnL();
        DuoJiao = 103;
        break;
      case 180:
        turnL();
        DuoJiao = 135;
        break;
      case 0:
        turnR();
        DuoJiao = 45;
        break;
    }
  } else {
    stopp();
    switch(DuoJiao) {
      case 103:
        DuoJiao = 45;
        break;
      case 45:
        DuoJiao = 135;
        break;
      case 135:
        DuoJiao = 180;
        break;
      case 180:
        DuoJiao = 0;
        break;
      case 0:
        DuoJiao = 103;
        break;
    }
  }
}


int Ultrasonic_Ranging(unsigned char Mode)//function of ultrasonic distance detecting ，MODE=1，displaying，no displaying under other situation
{
  int old_distance;
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  int distance = pulseIn(inputPin, HIGH);  // reading the duration of high level
  distance = distance / 58; // Transform pulse time to distance
  if (Mode == 1) {
    lcd.setCursor(11, 1);
    lcd.print("H=    ");
    lcd.setCursor(13, 1);
    lcd.print(distance, DEC);
    Serial.print("\n H = ");
    Serial.print(distance, DEC);
    return distance;
  }
  else  return distance;
}

void Bluetooth_Control() //Bluetooth remote control
{
  lcd.setCursor(0, 0);  //setting cursor in the first row and column
  lcd.print("BluetoothControl");

  if (Serial.available()) //to judge whether serial port receive data
  {
    Bluetooth_val = Serial.read(); //reading value of Bluetooth serial port, giving the value to val
    switch (Bluetooth_val)
    {
      case 'U': advance(); //UP
        break;
      case 'D': back();   //back
        break;
      case 'L': turnL();  //Left
        break;
      case 'R': turnR(); //Right
        break;
      case 'S': stopp();   //stop
        break;
    }
  }
}

void setup()
{
  pinMode(servopin, OUTPUT); //setting motor interface as output
  LCD1602_init();            //initializing 1602
  M_Control_IO_config();     //motor controlling the initialization of IO
  Set_Speed(Lpwm_val, Rpwm_val); //setting initialized speed
  Set_servopulse(DuoJiao);       //setting initialized motor angle
  Sensor_IO_Config();            //initializing IO of line tracking module
  pinMode(inputPin, INPUT);      //starting receiving IR remote control signal
  pinMode(outputPin, OUTPUT);    //IO of ultrasonic module
  Serial.begin(9600);            //initialized serial port , using Bluetooth as serial port, setting baud
  lcd.setCursor(0, 0);           //setting cursor at 0.0
  lcd.print("  Wait  Signal  ");  //LCD printing character string
  stopp();                       //stop
}
void loop()
{
  Self_Control();
  delay(10);
}
