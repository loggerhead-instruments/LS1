
/* DISPLAY FUNCTIONS
 *  
 */

time_t autoStartTime;
 
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  display.print(":");
  printZero(digits);
  display.print(digits);
}

void printZero(int val){
  if(val<10) display.print('0');
}

#define noSet 0
#define setRecDur 1
#define setRecSleep 2
#define setYear 3
#define setMonth 4
#define setDay 5
#define setHour 6
#define setMinute 7
#define setSecond 8
#define setMode 9
#define setStartHour 10
#define setStartMinute 11
#define setEndHour 12
#define setEndMinute 13

void manualSettings(){
  boolean startRec = 0, startUp, startDown;
  readEEPROM();

  autoStartTime = getTeensy3Time();
  
  // make sure settings valid (if EEPROM corrupted or not set yet)
  if (rec_dur < 0 | rec_dur>100000) rec_dur = 60;
  if (rec_int<0 | rec_int>100000) rec_int = 60;
  if (startHour<0 | startHour>23) startHour = 0;
  if (startMinute<0 | startMinute>59) startMinute = 0;
  if (endHour<0 | endHour>23) endHour = 0;
  if (endMinute<0 | endMinute>59) endMinute = 0;
  if (recMode<0 | recMode>1) recMode = 0;

  // get free space on cards
    cDisplay();
    display.print("LS1 Init");
    display.setTextSize(1);
    display.setCursor(0, 16);
    display.println("Card Free/Total MB");
    for (int n=0; n<4; n++){
      freeMB[n] = 0; //reset
      Serial.println(); Serial.println();
      Serial.print("Card:"); Serial.println(n + 1);
  
      display.print(n + 1); display.print("    ");
      display.display();
      
      // Initialize the SD card
      SPI.setMOSI(7);
      SPI.setSCK(14);
      SPI.setMISO(12);
    
      if(card.init(SPI_FULL_SPEED, chipSelect[n])){
         if(!volume.init(card)){
        Serial.println("could not find fat partition");
        }
        uint32_t freeSpace;
        uint32_t volumeMB = volumeInfo(&freeSpace);
        Serial.print("Free space (MB): ");
        Serial.println((uint32_t) freeSpace);
  
        freeMB[n] = freeSpace - 200; // take off 200 MB to be safe
        if (freeMB[n] < 0) freeMB[n] = 0;
        display.print(freeMB[n]);
        display.print("/");
        display.println(volumeMB);
        display.display();
      }
      else{
        Serial.println("Unable to access the SD card");
        //Serial.println(card.errorCode());
       // Serial.println(card.errorData());
        display.println("  None");
        display.display();
    }
  }

  // set back to card 1
  if(!card.init(SPI_FULL_SPEED, chipSelect[0])){
       Serial.println("Unable to access the SD card in slot 1");
       cDisplay();
       display.println("Error");
       display.print("Card 1 failed");
       display.display();
       while(1);
  }
  

  delay(2000);
  cDisplay();
  display.display();
  delay(600);
  
  while(startRec==0){
    static int curSetting = noSet;
    static int newYear, newMonth, newDay, newHour, newMinute, newSecond, oldYear, oldMonth, oldDay, oldHour, oldMinute, oldSecond;
    
    // Check for mode change
    boolean selectVal = digitalRead(SELECT);
    if(selectVal==0){
      curSetting += 1;
      if((recMode==MODE_NORMAL & curSetting>8)) curSetting = 0;
    }

    cDisplay();

    t = getTeensy3Time();

    if (t - autoStartTime > 600) startRec = 1; //autostart if no activity for 10 minutes
    switch (curSetting){
      case noSet:
        if (settingsChanged) {
          writeEEPROM();
          settingsChanged = 0;
          autoStartTime = getTeensy3Time();  //reset autoStartTime
        }
        display.print("UP+DN->Rec"); 
        
         // Check for start recording
        startUp = digitalRead(UP);
        startDown = digitalRead(DOWN);
        if(startUp==0 & startDown==0) {
          cDisplay();
          writeEEPROM(); //save settings
          display.print("Starting..");
          display.display();
          delay(1500);
          startRec = 1;  //start recording
        }
        break;
      case setRecDur:
        rec_dur = updateVal(rec_dur, 1, 3600);
        display.print("Rec:");
        display.print(rec_dur);
        display.println("s");
        break;
      case setRecSleep:
        rec_int = updateVal(rec_int, 0, 3600 * 24);
        display.print("Slp:");
        display.print(rec_int);
        display.println("s");
        break;
      case setYear:
        oldYear = year(t);
        newYear = updateVal(oldYear,2000, 2100);
        if(oldYear!=newYear) setTeensyTime(hour(t), minute(t), second(t), day(t), month(t), newYear);
        display.print("Year:");
        display.print(year(getTeensy3Time()));
        break;
      case setMonth:
        oldMonth = month(t);
        newMonth = updateVal(oldMonth, 1, 12);
        if(oldMonth != newMonth) setTeensyTime(hour(t), minute(t), second(t), day(t), newMonth, year(t));
        display.print("Month:");
        display.print(month(getTeensy3Time()));
        break;
      case setDay:
        oldDay = day(t);
        newDay = updateVal(oldDay, 1, 31);
        if(oldDay!=newDay) setTeensyTime(hour(t), minute(t), second(t), newDay, month(t), year(t));
        display.print("Day:");
        display.print(day(getTeensy3Time()));
        break;
      case setHour:
        oldHour = hour(t);
        newHour = updateVal(oldHour, 0, 23);
        if(oldHour!=newHour) setTeensyTime(newHour, minute(t), second(t), day(t), month(t), year(t));
        display.print("Hour:");
        display.print(hour(getTeensy3Time()));
        break;
      case setMinute:
        oldMinute = minute(t);
        newMinute = updateVal(oldMinute, 0, 59);
        if(oldMinute!=newMinute) setTeensyTime(hour(t), newMinute, second(t), day(t), month(t), year(t));
        display.print("Minute:");
        display.print(minute(getTeensy3Time()));
        break;
      case setSecond:
        oldSecond = second(t);
        newSecond = updateVal(oldSecond, 0, 59);
        if(oldSecond!=newSecond) setTeensyTime(hour(t), minute(t), newSecond, day(t), month(t), year(t));
        display.print("Second:");
        display.print(second(getTeensy3Time()));
        break;
    }
    displaySettings();
    displayClock(getTeensy3Time(), BOTTOM, 1);
    display.display();
    delay(200);
  }
}

void setTeensyTime(int hr, int mn, int sc, int dy, int mh, int yr){
  tmElements_t tm;
  tm.Year = yr - 1970;
  tm.Month = mh;
  tm.Day = dy;
  tm.Hour = hr;
  tm.Minute = mn;
  tm.Second = sc;
  time_t newtime;
  newtime = makeTime(tm); 
  Teensy3Clock.set(newtime); 
  autoStartTime = getTeensy3Time();
}
  
int updateVal(long curVal, long minVal, long maxVal){
  boolean upVal = digitalRead(UP);
  boolean downVal = digitalRead(DOWN);
  static int heldDown = 0;
  static int heldUp = 0;
  if(upVal==0){
    settingsChanged = 1;
      if (heldUp > 10) {
        curVal = (int) (curVal / 10) * 10;
        curVal += 10;
        if (heldUp > 20) curVal += 100;
      }
      else curVal += 1;
      heldUp += 1;
    }
    else heldUp = 0;
    
    if(downVal==0){
      settingsChanged = 1;
      if (heldDown > 10) {
        curVal = (int) (curVal / 10) * 10;
        curVal -= 10;
        if (heldDown > 20) curVal -= 100;
      }
      else
        curVal -= 1;
      heldDown += 1;
    }
    else heldDown = 0;

    if (curVal < minVal) curVal = maxVal;
    if (curVal > maxVal) curVal = minVal;
    return curVal;
}

void cDisplay(){
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
}

void displaySettings(){
  //t = Teensy3Clock.get();
  t = getTeensy3Time();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 18);
//  display.print("Mode:");
//  if (recMode==MODE_NORMAL) display.println("Normal");
//  if (recMode==MODE_DIEL) {
//    display.println("Diel");
//  }
  display.print("Rec:");
  display.print(rec_dur);
  display.println("s");
  display.print("Sleep:");
  display.print(rec_int);
  display.println("s");
  if (recMode==MODE_DIEL) {
    display.print("Active: ");
    printZero(startHour);
    display.print(startHour);
    printDigits(startMinute);
    display.print("-");
    printZero(endHour);
    display.print(endHour);
    printDigits(endMinute);
    display.println();
  }
  display.setTextSize(1);
  uint32_t totalRecSeconds = 0;
  uint32_t totalSleepSeconds = 0;
  float fileBytes = (2 * rec_dur * audio_srate) + 44;
  float fileMB = fileBytes / 1024 / 1024;
  for(int n=0; n<4; n++){
    filesPerCard[n] = 0;
    if(freeMB[n]==0) filesPerCard[n] = 0;
    else{
      filesPerCard[n] = (uint32_t) floor(freeMB[n] / fileMB);
    }
    totalRecSeconds += (filesPerCard[n] * rec_dur);
    totalSleepSeconds += (filesPerCard[n] * rec_int);
    //display.setCursor(60, 18 + (n*8));  // display file count for debugging
    //display.print(n+1); display.print(":");display.print(filesPerCard[n]); 
  }

  float recDraw = ((float) rec_dur * (mAmpRec + ((float) camFlag * mAmpCam)));
  float sleepDraw = ((float) rec_int * mAmpSleep) ;
  float avgCurrentDraw = (recDraw + sleepDraw) / (float) (rec_dur + rec_int);
  //Serial.print(recDraw); Serial.print(" "); Serial.print(sleepDraw); Serial.print(" ");
  //Serial.println(avgCurrentDraw);
  uint32_t powerSeconds = uint32_t (3600.0 * (mAhTotal / avgCurrentDraw));

  if(powerSeconds < (totalRecSeconds + totalSleepSeconds)){
    displayClock(getTeensy3Time() + powerSeconds, 45, 0);
    display.setCursor(0, 36);
    display.print("Battery Limit:");
    display.print(powerSeconds / 86400);
    display.print("d");
  }
  else{
    displayClock(getTeensy3Time() + totalRecSeconds + totalSleepSeconds, 45, 0);
    display.setCursor(0, 36);
    display.print("Memory Limit:");
    display.print((totalRecSeconds + totalSleepSeconds) / 86400);
    display.print("d");
  }
  
//  if(camFlag){
//    display.setCursor(70, 18);
//    display.print("Cam On");
//  }
}

void displayClock(time_t t, int loc, boolean showSeconds){
  display.setTextSize(1);
  display.setCursor(0,loc);
  display.print(year(t));
  display.print('-');
  display.print(month(t));
  display.print('-');
  display.print(day(t));
  display.print("  ");
  printZero(hour());
  display.print(hour(t));
  printDigits(minute(t));
  if(showSeconds) printDigits(second(t));
}

void printTime(time_t t){
  Serial.print(year(t));
  Serial.print('-');
  Serial.print(month(t));
  Serial.print('-');
  Serial.print(day(t));
  Serial.print(" ");
  Serial.print(hour(t));
  Serial.print(':');
  Serial.print(minute(t));
  Serial.print(':');
  Serial.println(second(t));
}

void readEEPROM(){
  rec_dur = readEEPROMlong(0);
  rec_int = readEEPROMlong(4);
  startHour = EEPROM.read(8);
  startMinute = EEPROM.read(9);
  endHour = EEPROM.read(10);
  endMinute = EEPROM.read(11);
  recMode = EEPROM.read(12);
}

union {
  byte b[4];
  long lval;
}u;

long readEEPROMlong(int address){
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address + 1);
  u.b[2] = EEPROM.read(address + 2);
  u.b[3] = EEPROM.read(address + 3);
  return u.lval;
}

void writeEEPROMlong(int address, long val){
  u.lval = val;
  EEPROM.write(address, u.b[0]);
  EEPROM.write(address + 1, u.b[1]);
  EEPROM.write(address + 2, u.b[2]);
  EEPROM.write(address + 3, u.b[3]);
}

void writeEEPROM(){
  writeEEPROMlong(0, rec_dur);  //long
  writeEEPROMlong(4, rec_int);  //long
  EEPROM.write(8, startHour); //byte
  EEPROM.write(9, startMinute); //byte
  EEPROM.write(10, endHour); //byte
  EEPROM.write(11, endMinute); //byte
  EEPROM.write(12, recMode); //byte
}

uint32_t volumeInfo(uint32_t *freeSpace){
  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();
  
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  if (volumesize < 8388608ul) {
    Serial.print("Volume size (bytes): ");
    Serial.println(volumesize * 512);        // SD card blocks are always 512 bytes
  }
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 2;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  
  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);
  uint32_t usedSpace = root.ls(LS_R | LS_DATE | LS_SIZE);
  Serial.print("Used Space: "); Serial.println(usedSpace);
  if(volumesize ==0) *freeSpace = 0;
    else
    *freeSpace = volumesize - usedSpace;
  return volumesize;
}
