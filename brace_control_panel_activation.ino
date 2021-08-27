//TITLE: brace_control_panel_activation
//AUTHOR: Duncan Stevenson
//PURPOSE: script for controlling the pressure system via proprietary control board.

//Control board has 3 LEDs, 1 button, 1 potentiometer.
//3 LEDs are used to show binary number of selection from 1-8.
//Poentiometer controls selection number.
//Button activates the solenoid valve.
//Selection number is configurable through minOutput and stepOutput.

//minOutput is solenoid open time at potentiometer position 0.
//stepOutput is increase for each subsequent potentiometer position.
float minOutput = .1;
float stepOutput = .1;

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

//Setup all devices
void setup() {
  pinMode(pin_led1, OUTPUT);
  pinMode(pin_led2, OUTPUT);
  pinMode(pin_led3, OUTPUT);
  pinMode(pin_button, INPUT);
  pinMode(pin_relay, OUTPUT);

}



void loop() {
  //Start each loop by reading button, not used until after checking pot value
  int buttonState = digitalRead(pin_button);

  //sets solenoid open time using the potentiometer
  rawPotVal = analogRead(pin_pot);
  pPotVal = map(rawPotVal, 30, 1000, 0, 7);
  solenoidOpen = minOutput + (pPotVal * stepOutput);

  //uses binary to display value from 1-8
  if (pPotVal != pPotVal_last){
    binaryLED(pPotVal);
  }
  pPotVal_last = pPotVal;

  //activate brace if button is pressed
  if(buttonState == HIGH){
    activate = HIGH;
  } else {
    if (activate == HIGH){
      delay(500);
      activateSolenoid(solenoidOpen*1000); //convert s to ms
      activate = LOW;
    }
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
