#include <Arduino.h>
//#include <driver/dac.h> // Remove unused include

#define _TIMERINTERRUPT_LOGLEVEL_   4
#include <ESP32TimerInterrupt.h>

#define TIMER0_INTERVAL_MS 1

// put function declarations here:

int analogPin1 = GPIO_NUM_32; // potentiometer wiper (middle terminal) connected to analog pin 32
int analogPin2 = GPIO_NUM_25;
int buttonPin = GPIO_NUM_4; // push button connected to digital pin 4

int potVal1 = 0; //AMP
int potVal2 = 0; //FEQ
int pVal = 0;
unsigned long timeDelay = 37;
unsigned long bpm = 0;

enum monState {AR, DEAD, HB};
enum monState state = HB;

                    // outside leads to ground and +5V
//int val = 0;  // variable to store the value read
  static uint8_t points[] = {
65,
65,
65,
65,
70,
76,
74,
70,
65,
63,
65,
65,
65,
65,
48,
230,
15,
65,
65,
65,
74,
90,
100,
102,
100,
95,
80,
70,
65,
65,
65,
65
};
#include <vector>

std::vector<uint8_t> points2(points + 12, points + 16);
ESP32Timer ITimer0(0);
bool IRAM_ATTR TimerHandler0(void * timerNo){
  static int index = 0;
  static int counter = timeDelay;
  static monState lastState = state;
  if(lastState != state){
    index = 0;
    counter = timeDelay;
    lastState = state;
  }

  if(counter <= 0){
    if(state == HB){
      pVal = points[index];
      if(index >= sizeof(points)){
      index = 0;
      }
    }else if(state == AR){
      pVal = points2[index];
      if(index >= sizeof(points2)){
      index = 0;
      }
    }else if (state == DEAD){
      pVal = 65;
      index = 0;
    }
    index++;
    counter = timeDelay;
  }else{
    pVal=65;
  }
    counter--;
  return true;
  }

void setup() {
  Serial.begin(115200);           //  setup serial
  delay(1000);
	if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
	{
		Serial.print(F("Starting  ITimer0 OK, millis() = "));
		Serial.println(millis());
	}
  pinMode(buttonPin, INPUT_PULLUP);
}
unsigned long lastButtonPressTime = 0;
const unsigned long debounceDelay = 150;
void loop() {
  static unsigned long lastTimer = 0;
  potVal1 = analogRead(analogPin1);
  long mappedVal1 = map(potVal1,0,4095,0,100);
  
  potVal2 = analogRead(analogPin2);
  timeDelay = map(potVal2,0,4095,9,48);
  bpm = map(potVal2,0,4095,220,40);
  if(pVal != 0){
    Serial.print(pVal * (mappedVal1/100.0)+65*(1-(mappedVal1/100.0)));
    Serial.print(",");
    Serial.println(bpm);
    pVal = 0;
  }
  
  // Check for button press with debounce
  if (millis() - lastButtonPressTime >= debounceDelay) {
    if(digitalRead(buttonPin) == LOW){
      if(state == HB){
        state = AR;
      }
      else if(state == AR){
        state = DEAD;
      }
      else if(state == DEAD){
        state = HB;
      }
      lastButtonPressTime = millis();
    }
  }
}