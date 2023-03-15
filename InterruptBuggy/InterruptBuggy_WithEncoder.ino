#include <SPI.h>
#include <WiFiNINA.h>

//PID constants
double kp = 2;
double ki = 5;
double kd = 1;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

const char SSID[] = "X1C";  //Hotspot name and password
const char PASSWORD[] = "supersonic";
WiFiServer server(4800);  //Server port
bool start = false;       //Updates via wifi to start and stop the buggy

const int FORWARD = 4;   //1A and 3A
const int BACKWARD = 12;  //2A and 4A
const int L_SPEED = 17;   //12EN
const int R_SPEED = 16;    //34EN
const int L_EYE = 2;      //Left IR sensor
const int R_EYE = 3;      //Right IR sensor
const int TRIG = 14;      //Ultrasonic trigger
const int ECHO = 15;      //Ultrasonic echo

int Rstate;  //global variables used to update speed via PWM on EN pins
int Lstate;
volatile bool Lbool;  //global variables corresponding to IR sensors updated via interrupts
volatile bool Rbool;  //Must be volatile if changed within interrupts

void setup() {
  setPoint = 0;
  Serial.begin(9600);
  WiFi.beginAP(SSID, PASSWORD);
  delay(5000);  //Make sure wifi is connected
  IPAddress ip = WiFi.localIP();
  Serial.print(ip);  //In case IP address changes
  server.begin();

  pinMode(FORWARD, OUTPUT);  //output because sending data out to h-bridge
  pinMode(BACKWARD, OUTPUT);
  pinMode(L_SPEED, OUTPUT);
  pinMode(R_SPEED, OUTPUT);

  pinMode(L_EYE, INPUT);  //input because receiving data to arduino
  pinMode(R_EYE, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_EYE), LirSensor, FALLING);
  attachInterrupt(digitalPinToInterrupt(R_EYE), RirSensor, FALLING);
  //creates an interrupt function L/RirSensor which is run when the value from
  //L/R_EYE falls from HIGH to LOW
  //interrupts only found at pins 3 and 4

  Rstate = HIGH;  //initialise speed
  Lstate = HIGH;

  Lbool = false;  //initialise IR booleans
  Rbool = false;

  pinMode(TRIG, OUTPUT);  //Ultrasonic pins
  pinMode(ECHO, INPUT);
}

void loop() {
  WiFiClient client = server.available();  //create client object for processing

  input = analogRead(A0);                //read from rotary encoder connected to A0
  output = computePID(input);
  delay(100);
  analogWrite(3, output);                //control the motor based on PID value

  if (client.connected()) {  //read a character from processing
    char ch = client.read();
    if (ch == 'w') {  //determine if buggy should start or stop
      start = true;
    }

    else if (ch == 's')
      start = false;
  }

  int dist = distance();      //read distance from function
  if (dist < 10 || !start) {  //if object seen or told to stop
    Lstate = 0;
    Rstate = 0;  //stop the buggy
  }

  else if (digitalRead(L_EYE) && digitalRead(R_EYE)) {  //if IR detects nothing
    Lstate = 150;
    Rstate = 150;
    Lbool = false;
    Rbool = false;
  }

  if (Lbool && start && dist > 9) {  //if left IR triggers
    Lstate = 30; //differential steering
    Rstate = 150;
  }

  if (Rbool && start && dist > 9) {  //if right IR triggers
    Lstate = 150;
    Rstate = 30;
  }

  if((Lbool && start && dist > 9) && (Rbool && start && dist > 9)) {
    Lstate = 0;
    Rstate = 0;
  }

  server.write(dist);  //send distance data to processing

  digitalWrite(FORWARD, HIGH);  //move forward
  digitalWrite(BACKWARD, LOW);
  analogWrite(L_SPEED, Lstate);  //use PWM to vary speed
  analogWrite(R_SPEED, Rstate);
  delay(200);  //delay for interrupt recovery
}

int distance() {  //return distance using Ultrasonic
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);  //send ping
  delayMicroseconds(10);
  int duration = pulseIn(ECHO, HIGH);  //listen for response
  return duration / 56; //convert time to distance
}

void LirSensor() {  //Left sensor interrupt function
  Lbool = true;
}

void RirSensor() {  //Right sensor interrupt function
  Rbool = true;
}

double computePID(double inp){     
        currentTime = millis();                                    //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        error = setPoint - inp;                                    //determine error
        cumError += error * elapsedTime;                           //compute integral
        rateError = (error - lastError)/elapsedTime;               //compute derivative
        double out = kp*error + ki*cumError + kd*rateError;        //PID output               
        lastError = error;                                         //remember current error
        previousTime = currentTime;                                //remember current time
        return out;                                                //have function return the PID output
}
