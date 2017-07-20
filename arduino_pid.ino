/*
 *  //////////////////////////////////////////////////
 *  ////// Pireus University of Applied Sciences /////
 *  ////////// APPLIED INFORMATION SYSTEMS ///////////
 *  ///////// Integrated Industrial Control //////////
 *  //////////////////////////////////////////////////
 *  **************************************************
 *  //////////////////////////////////////////////////
 *  //// Smarth Home with Arduino PID Controllers ////
 *  //////////////////////////////////////////////////
 *  **************************************************
 *  @authors: Alonefti Maria - Grigoropoulos Nikolaos 
 *  **************************************************
 *  //////////////////////////////////////////////////
*/

#include <PID_v1.h>

/*LedController Variables*/
const int photores = A0;                      // LDR input pin
const int pot = A1;                           // Potentiometer input pin
const int bedLed = 9;                         // LED output pin
double lightLevel;                            // Indirectly store the light level
  /*Tuning parameters*/
    float Kp = 0;                             // Proportional gain
    float Ki = 10;                            // Integral gain
    float Kd = 0;                             // Differential gain
      /*Record the set point as well as the controller input and output*/
      double Setpoint, Input, Output;
        /*Create a controller that is linked to the specified Input, Ouput and Setpoint*/
          PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
          const int sampleRate = 1;           // Time interval of the PID control
          const long serialPing = 500;        // How often data is recieved from the Arduino
          unsigned long now = 0;              // Store the time that has elapsed
          unsigned long lastMessage = 0;      // The last time that data was recieved

/*Temperature Variables*/
float tempC;                                  // create variable to store the temperature in.
const int tempPin = A3;                       // Attach vout to analog pin 3.
int led3 = 12;                                // attach led to pin 12.
int fan = 5;                                  // attach base of transistor to digital pin 5.
unsigned long previousMillis = 0;             // will store last time temperature was updated
const long interval = 3000;                   // interval at which to measure temperature (milliseconds)

/*Alarm Variables*/
int ledPin = 13;                              // choose the pin for the LED
int inputPin = 2;                             // choose the input pin (for PIR sensor)
int pirState = LOW;                           // we start, assuming no motion detected
int val = 0;                                  // variable for reading the pin status
int pinSpeaker = 11;                          //Set up a speaker on a PWM pin (digital 9, 10, or 11)

/*Led Variables*/
int led2 = 7;                                 // choose the pin for the LED
int inPin = 8;                                // choose the input pin (for PIR sensor)
int pir2State = LOW;                          // we start, assuming no motion detected
int val2 = 0;                                 // variable for reading the pin status

void setup()                                  // Will execute once at the start of the code.
{
  Serial.begin(9600);                         // opens serial port, sets data rate to 9600 bps
   
/*Alarm Setup*/
  pinMode(ledPin, OUTPUT);                    // declare LED as output
  pinMode(inputPin, INPUT);                   // declare sensor as input
  pinMode(pinSpeaker, OUTPUT);                // declare speaker as output
             
/*Led Setup*/
  pinMode(led2, OUTPUT);                      // declare LED as output
  pinMode(inPin, INPUT);                      // declare sensor as input

/*Temperature Setup*/
  pinMode (led3, OUTPUT);                     // sets the led pin 12 up as an output.
  pinMode (fan, OUTPUT);                      // sets the fan1 pin 5 up as an output.
  
/*LedController Setup*/
  lightLevel = analogRead(photores);          // Read the set point
  /*Arduino has an analogueRead() resolution of 0-1023 and an analogueWrite() resolution of 0-255*/
   Input = map(lightLevel, 0, 1023, 0, 255);  // Scale the input
   Setpoint = map(analogRead(pot), 0, 1023, 0, 255);  // Scale the set point
   myPID.SetMode(AUTOMATIC);                   // Turn on the PID control
   myPID.SetSampleTime(sampleRate);            // Assign the sample rate of the control
   //Serial.println("Begin");                  // Let the user know that the set up s complete
   lastMessage = millis();                     // Serial data will be recieved relative to this first point 
}

void loop()                                   // code here will continue to replay nutil powered off.
{    
          functionAlarm();  
          functionLed();        
          functionTemperature(); 
          functionLedController();   
}

void functionTemperature(){
  /*Temperature Loop*/
  tempC = analogRead(tempPin);                // read the analog value from the lm35 sensor.
  tempC = (5.0 * tempC * 100.0)/1024.0;       // convert the analog input to temperature in centigrade.
  unsigned long currentMillis = millis();
  //Serial.println((byte)tempC);              // send the data to the computer.
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (tempC > 30)                           // creates bool expression for analyzation. if it evaluates to true,
    {                                         // the body of the if statement will execute.
      digitalWrite (led3, HIGH);              // turns on led.
      digitalWrite (fan, HIGH);               // turns on fan1.      
    }
    else                                      // if the if equation evaluates to false the else statement will execute.
    {
      digitalWrite (led3, LOW);               // turns off led.
      digitalWrite (fan, LOW);                // turns off fan1.
    }
 }                             
}

void functionAlarm(){
 /*Alarm Loop*/
  val = digitalRead(inputPin);                // read input value
  if (val == HIGH) {                          // check if the input is HIGH
    digitalWrite(ledPin, HIGH);               // turn LED ON
    playTone(300, 160);
    //delay(150);
    if (pirState == LOW) {                    // we have just turned on
      //Serial.println("Motion detected!");
      pirState = HIGH;                        // We only want to print on the output change, not state
    }
  } else {
      digitalWrite(ledPin, LOW);              // turn LED OFF
      playTone(0, 0);
     // delay(300);    
      if (pirState == HIGH){                  // we have just turned off
        //Serial.println("Motion ended!");    
        pirState = LOW;                       // We only want to print on the output change, not state
      }
    }
}

void playTone(long duration, int freq) {      //Duration in mSecs, frequency in hertz
    duration *= 1000;
    int period = (1.0 / freq) * 1000000;
    long elapsed_time = 0;
    while (elapsed_time < duration) {
        digitalWrite(pinSpeaker,HIGH);
        delayMicroseconds(period / 2);
        digitalWrite(pinSpeaker, LOW);
        delayMicroseconds(period / 2);
        elapsed_time += (period);
    }
}

void functionLed(){
  /*Led Loop*/ 
    val2 = digitalRead(inPin);                // read input value
    if (val2 == HIGH) {                       // check if the input is HIGH
    digitalWrite(led2, HIGH);                 // turn LED ON
    if (pir2State == LOW) {                   // we have just turned on
      pir2State = HIGH;                       // We only want to print on the output change, not state
    }
  } else {
      digitalWrite(led2, LOW);                // turn LED OFF 
      if (pir2State == HIGH){                 // we have just turned off     
        pir2State = LOW;                      // We only want to print on the output change, not state
      }
    }
}

void functionLedController(){
  /*Photoresistor Loop*/
  Setpoint = map(analogRead(pot), 0, 1023, 0, 255);  // Continue to read and scale the set point
  lightLevel = analogRead(photores);                 // Read the light level
  Input = map(lightLevel, 0, 900, 0, 255);           // Scale the input to the PID
  myPID.Compute();                                   // Calculates the PID output at a specified sample time
  analogWrite(bedLed, Output);                       // Power the LED
  now = millis();                                    // Keep track of the elapsed time
  if(now - lastMessage > serialPing)                 // If enough time has passed send data
  {
    Serial.print("Setpoint = ");
    Serial.print(Setpoint);
    Serial.print(" Input = ");
    Serial.print(Input);
    Serial.print(" Output = ");
    Serial.print(Output);
    Serial.print("\n");
/*The tuning parameters can be retrieved by the Arduino from the serial monitor: 0,0.5,0 set Ki to 0.5. - Commas are ignored by the Serial.parseFloat() command*/
    if (Serial.available() > 0)
    {
      for (int x = 0; x < 4; x++)
      {
        switch(x)
        {
          case 0:
            Kp = Serial.parseFloat();
          break;
          case 1:
            Ki = Serial.parseFloat();
          break;
          case 2:
            Kd = Serial.parseFloat();
          break;
          case 3:
          for (int y = Serial.available(); y == 0; y--)
          {
            Serial.read();
          }
          break;
        }
      }
      Serial.print(" Kp,Ki,Kd = ");                 // Display the new parameters
      Serial.print(Kp);
      Serial.print(" ,");
      Serial.print(Ki);
      Serial.print(" ,");
      Serial.print(Kd);
      myPID.SetTunings(Kp, Ki, Kd);                 // Set the tuning of the PID loop
    }
    lastMessage = now;                              // Reference the next serial communication to this point
 }
}
