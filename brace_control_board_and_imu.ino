//TITLE: brace_control_panel_activation
//AUTHOR: Duncan Stevenson
//PURPOSE: script for controlling the pressure system via control board and IMU detection.

//Control board has 3 LEDs, 1 button, 1 potentiometer.
//3 LEDs are used to show binary number of selection from 1-8.
//Poentiometer controls selection number.
//Button allows for IMU system input to activate the brace, and resets step timer for activation to zero.
//Selection number is configurable through minOutput and stepOutput.

//minOutput is solenoid open time at potentiometer position 0.
//stepOutput is increase for each subsequent potentiometer position.
float minOutput = .1;
float stepOutput = .1;

//Input from the IMU system comes in the form of gait input - integer values for different parts of gait.
//Arduino script counts the steps to give actuation on a regular step.
//This step counter resets to 0 when stopping or standing.
//activateStep configures the consecutive same-foot step after which the device activates.
int activateStep = 4;


int pin_led1 = 3;
int pin_led2 = 4;
int pin_led3 = 5;
int pin_button = 2;
int pin_relay = 6;
int pin_pot = A0;
int rawPotVal = 0;
int pPotVal = 0;
int pPotVal_last = 0;
float solenoidOpen = 0;
int activate = LOW;
int b1 = LOW;
int b2 = LOW;
int b3 = LOW;
int stepCounter = 0;
int byteGaitInput = 0;
int lastGaitByte = 0;
int lastGaitByte2 = 0;
int lastGaitByte3 = 0;


void setup() {
  pinMode(pin_led1, OUTPUT);
  pinMode(pin_led2, OUTPUT);
  pinMode(pin_led3, OUTPUT);
  pinMode(pin_button, INPUT);
  pinMode(pin_relay, OUTPUT);
  Serial.begin(256000);

}

void loop() {
  //sets solenoid open time using the potentiometer
  rawPotVal = analogRead(pin_pot);
  pPotVal = map(rawPotVal, 30, 1000, 0, 7);
  solenoidOpen = minOutput + (pPotVal * stepOutput);

  
  //uses binary to display value from 1-8
  if (pPotVal != pPotVal_last){
    binaryLED(pPotVal);
  }
  pPotVal_last = pPotVal;


  //start activation run when pressed
  //resets step counter and flips switch variable to true
  int buttonState = digitalRead(pin_button);
  if (buttonState == HIGH) {
    activate = HIGH;
    stepCounter = 0;
  }


  //check for new gait inputs
  //probably better to do with an array but oh well
  if(Serial.available() > 0){
    lastGaitByte3 = lastGaitByte2;
    lastGaitByte2 = lastGaitByte;
    lastGaitByte = byteGaitInput;
    byteGaitInput = Serial.read();
  }


  //detect change from 0 to 1, increment step counter
  if((lastGaitByte == 0) && (byteGaitInput == 1)){
    stepCounter = stepCounter + 1;
  }


  //if switch is flipped to true and the activate step is reached,
  //then activate the solenoid and reset switch and stepcounter
  if ((stepCounter >= activateStep) && (activate == HIGH)){
    activateSolenoid(solenoidOpen*1000);
    activate = LOW;
    stepCounter = 0;
  }
}













void activateSolenoid(int openTime){
  digitalWrite(pin_relay, HIGH);
  delay(openTime);
  digitalWrite(pin_relay, LOW);
}






void binaryLED(int n1) {
  b1 = LOW;
  b2 = LOW;
  b3 = LOW;

  if (n1==1 || n1==3 || n1==5 || n1==7){
    b1=HIGH;
  }
  if (n1==2 || n1==3 || n1==6 || n1==7){
    b2=HIGH;
  }
  if (n1==4 || n1==6 || n1==5 || n1==7){
    b3=HIGH;
  }

  digitalWrite(pin_led1, b1);
  digitalWrite(pin_led2, b2);
  digitalWrite(pin_led3, b3);
  
}
