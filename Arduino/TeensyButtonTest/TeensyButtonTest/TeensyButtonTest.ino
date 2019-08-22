// Pin Assignments
const int UP = 4;
const int DOWN = 3;
const int SELECT = 8;

#include <Wire.h>


#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);
#define BOTTOM 55


void setup() {
  RTC_CR = 0; // disable RTC
  delay(100);
  //Serial.println(RTC_CR,HEX);
  // change capacitance to 26 pF (12.5 pF load capacitance)
  RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P; 
  delay(100);
  RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P | RTC_CR_OSCE;
  delay(100);

  analogReference(DEFAULT);
    
  Serial.begin(115200);
  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);
  pinMode(SELECT, INPUT_PULLUP);
  Wire.begin();

  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
//  delay(140);
//  cDisplay();
//  display.println("Loggerhead");
//  display.display();
  
}

void loop() {
  if(digitalRead(UP)==0) Serial.println("Up");
  delay(100);
}

void cDisplay(){
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
}
