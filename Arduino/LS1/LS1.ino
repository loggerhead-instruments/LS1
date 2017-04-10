//
// LS1 acoustic recorder
//
// Loggerhead Instruments
// 2017
// David Mann
// 
// Modified from PJRC audio code and Snap code
// http://www.pjrc.com/store/teensy3_audio.html
//

// Compile with 48 MHz Optimize Speed

//#include <SerialFlash.h>
#include <Audio.h>  //comment out includes SD.h from play_sd_
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include "amx32.h"
#include <Snooze.h>  //using https://github.com/duff2013/Snooze; uncomment line 62 #define USE_HIBERNATE
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
//#include <TimerOne.h>

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define BOTTOM 57

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

//*********************************************************
//
static boolean printDiags = 0;  // 1: serial print diagnostics; 0: no diagnostics
int camFlag = 1;
//
//***********************************************************

static uint8_t myID[8];
unsigned long baud = 115200;

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400

#define MODE_NORMAL 0
#define MODE_DIEL 1

// GUItool: begin automatically generated code
AudioInputI2S            i2s2;           //xy=105,63
AudioRecordQueue         queue1;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

const int myInput = AUDIO_INPUT_LINEIN;
int gainSetting = 4; //default gain setting; can be overridden in setup file

// LS1 Pins
const int hydroPowPin = 2;
const int UP = 4;
const int DOWN = 3; 
const int SELECT = 8;
const int displayPow = 6;
const int CAM_SW = 5;
const int vSense = 16; 

// microSD chip select pins
#define CS1 10
#define CS2 15
#define CS3 20
#define CS4 21
int chipSelect[4];
uint32_t freeMB[4];
uint32_t filesPerCard[4];
int currentCard = 0;
boolean newCard = 0;

SdFat sd;
SdFile file;

// Pins used by audio shield
// https://www.pjrc.com/store/teensy3_audio.html
// MEMCS 6
// MOSI 7
// BCLK 9
// SDCS 10
// MCLK 11
// MISO 12
// RX 13
// SCLK 14
// VOL 15
// SDA 18
// SCL 19
// TX 22
// LRCLK 23

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording, 2=playing
time_t startTime;
time_t stopTime;
time_t t;
time_t burnTime;
byte startHour, startMinute, endHour, endMinute; //used in Diel mode

boolean CAMON = 0;
boolean audioFlag = 1;

boolean LEDSON=1;
boolean introperiod=1;  //flag for introductory period; used for keeping LED on for a little while

float audio_srate = 44100.0;

float audioIntervalSec = 256.0 / audio_srate; //buffer interval in seconds
unsigned int audioIntervalCount = 0;

int recMode = MODE_NORMAL;
long rec_dur = 10;
long rec_int = 30;
int wakeahead = 15;  //wake from snooze to give hydrophone and camera time to power up
int snooze_hour;
int snooze_minute;
int snooze_second;
int buf_count;
long nbufs_per_file;
boolean settingsChanged = 0;

float mAmpRec = 70;
float mAmpSleep = 4;
float mAmpCam = 600;
float mAhTotal = 12000 * 4; // assume 12Ah per battery pack

long file_count;
char filename[25];
char dirname[7];
int folderMonth;
//SnoozeBlock snooze_config;
SnoozeAlarm alarm;
SnoozeAudio snooze_audio;
SnoozeBlock config_teensy32(snooze_audio, alarm);

// The file where data is recorded
File frec;

typedef struct {
    char    rId[4];
    unsigned int rLen;
    char    wId[4];
    char    fId[4];
    unsigned int    fLen;
    unsigned short nFormatTag;
    unsigned short nChannels;
    unsigned int nSamplesPerSec;
    unsigned int nAvgBytesPerSec;
    unsigned short nBlockAlign;
    unsigned short  nBitsPerSamples;
    char    dId[4];
    unsigned int    dLen;
} HdrStruct;

HdrStruct wav_hdr;
unsigned int rms;
float hydroCal = -170;

unsigned char prev_dtr = 0;


void setup() {
  read_myID();

  chipSelect[0] = CS1;
  chipSelect[1] = CS2;
  chipSelect[2] = CS3;
  chipSelect[3] = CS4;

  Serial.begin(baud);
  delay(500);
  Wire.begin();

  pinMode(hydroPowPin, OUTPUT);
  pinMode(displayPow, OUTPUT);
  pinMode(CAM_SW, OUTPUT);

  pinMode(vSense, INPUT);
  analogReference(DEFAULT);

  digitalWrite(hydroPowPin, LOW);
  digitalWrite(displayPow, HIGH);
  digitalWrite(CAM_SW, LOW);
  
  //setup display and controls
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(SELECT, INPUT);
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(SELECT, HIGH);

  delay(500);    

  setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time
  t = getTeensy3Time();
  if (t < 1451606400) Teensy3Clock.set(1451606400);


  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
  delay(100);
  cDisplay();
  display.println("Loggerhead");
//  Serial.println("Loggerhead");
//  display.println("USB <->");
  display.display();
  // Check for external USB connection to microSD
// while(digitalRead(usbSense)){
//    delay(500);
//  }

  // Power down USB if not using Serial monitor
//  if (printDiags==0){
//      usbDisable();
//  }

  cDisplay();
  display.println("Loggerhead");
  display.display();
  
//  delay(200);
//  // Initialize the SD card
//  SPI.setMOSI(7);
//  SPI.setSCK(14);
//  if (!SD.begin(chipSelect[currentCard])) {
//    // stop here if no SD card, but print a message
//    Serial.println("Unable to access the SD card");
//    
//    while (1) {
//      cDisplay();
//      display.println("SD error. Restart.");
//      displayClock(getTeensy3Time(), BOTTOM, 1);
//      display.display();
//      delay(20000);  
//      //resetFunc();
//    }
//  }

  manualSettings();
  SdFile::dateTimeCallback(file_date_time);
  
  digitalWrite(hydroPowPin, LOW); // make sure hydrophone powered off when in manual settings in case of accidental reset
  
  // disable buttons; not using any more
  digitalWrite(UP, LOW);
  digitalWrite(DOWN, LOW);
  digitalWrite(SELECT, LOW);
  pinMode(UP, OUTPUT);
  pinMode(DOWN, OUTPUT);
  pinMode(SELECT, OUTPUT);
  
  cDisplay();

  int roundSeconds = 300;//modulo to nearest x seconds
  t = getTeensy3Time();
  startTime = getTeensy3Time();
  startTime -= startTime % roundSeconds;  
  startTime += roundSeconds; //move forward
  stopTime = startTime + rec_dur;  // this will be set on start of recording
  
 // if (recMode==MODE_DIEL) checkDielTime();  
  
  nbufs_per_file = (long) (rec_dur * audio_srate / 256.0);
  long ss = rec_int - wakeahead;
  if (ss<0) ss=0;
  snooze_hour = floor(ss/3600);
  ss -= snooze_hour * 3600;
  snooze_minute = floor(ss/60);
  ss -= snooze_minute * 60;
  snooze_second = ss;
  Serial.print("Snooze HH MM SS ");
  Serial.print(snooze_hour);
  Serial.print(snooze_minute);
  Serial.println(snooze_second);

  Serial.print("rec dur ");
  Serial.println(rec_dur);
  Serial.print("rec int ");
  Serial.println(rec_int);
  Serial.print("Current Time: ");
  printTime(t);
  Serial.print("Start Time: ");
  printTime(startTime);
  
  // Sleep here if won't start for 60 s
  long time_to_first_rec = startTime - t;
  Serial.print("Time to first record ");
  Serial.println(time_to_first_rec);

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  AudioMemory(100);
  AudioInit(); // this calls Wire.begin() in control_sgtl5000.cpp

  cam_wake();
  
  digitalWrite(hydroPowPin, HIGH);
  mode = 0;

  // create first folder to hold data
  folderMonth = -1;  //set to -1 so when first file made will create directory
}

//
// MAIN LOOP
//

int recLoopCount;  //for debugging when does not start record
  
void loop() {
  // Standby mode
  if(mode == 0)
  {
      t = getTeensy3Time();
      cDisplay();
      display.println("Next Start");
      display.setTextSize(1);
      display.setCursor(0, 18);
      display.print("Card:");
      display.print(currentCard + 1);
      display.print(" ");
      display.print(filesPerCard[currentCard]);
      // display.print("Free:");
      // display.print();
      displayClock(startTime, 40, 1);
      displayClock(t, BOTTOM, 1);
      display.display();

      if((t >= startTime - 1) & CAMON==1){ //start camera 1 second before to give a chance to get going
        if (camFlag)  cam_start();
      }
      if(t >= startTime){      // time to start?
        Serial.println("Record Start.");
        
        stopTime = startTime + rec_dur;
        startTime = stopTime + rec_int;
      //  if (recMode==MODE_DIEL) checkDielTime();

        Serial.print("Current Time: ");
        printTime(getTeensy3Time());
        Serial.print("Stop Time: ");
        printTime(stopTime);
        Serial.print("Next Start:");
        printTime(startTime);

//        cDisplay();
//        display.println("Rec");
//        display.setTextSize(1);
//        display.print("Stop Time: ");
//        displayClock(stopTime, 30);
//        display.display();

        mode = 1;
  
        display.ssd1306_command(SSD1306_DISPLAYOFF); // turn off display during recording
        startRecording();
      }
  }


  // Record mode
  if (mode == 1) {
    continueRecording();  // download data  

    /*
     // update clock while recording
      recLoopCount++;
      if(recLoopCount>50){
        recLoopCount = 0;
        t = getTeensy3Time();
        cDisplay();
        if(rec_int > 0) {
          display.println("Rec");
          displayClock(stopTime, 20);
        }
        else{
          display.println("Rec Contin");
          display.setTextSize(1);
          display.println(filename);
        }
        displayClock(t, BOTTOM);
        display.display();
      }
      */
      
    if(buf_count >= nbufs_per_file){       // time to stop?
      if(rec_int == 0){
        frec.close();
        checkSD();
        FileInit();  // make a new file
        buf_count = 0;
      }
      else{
        stopRecording();
        cam_stop();
        checkSD();
        
        long ss = startTime - getTeensy3Time() - wakeahead;
        if (ss<0) ss=0;
        snooze_hour = floor(ss/3600);
        ss -= snooze_hour * 3600;
        snooze_minute = floor(ss/60);
        ss -= snooze_minute * 60;
        snooze_second = ss;
        
        if( (snooze_hour * 3600) + (snooze_minute * 60) + snooze_second >=10){
            if (printDiags) Serial.println("Shutting bits down");
            digitalWrite(hydroPowPin, LOW); //hydrophone off
            cam_off(); //camera off
            if (printDiags) Serial.println("hydrophone off");
            audio_power_down();
            if (printDiags) Serial.println("audio power down");

            if(printDiags){
              Serial.print("Snooze HH MM SS ");
              Serial.print(snooze_hour);
              Serial.print(snooze_minute);
              Serial.println(snooze_second);
            }
            delay(100);
            Serial.println("Going to Sleep");
            delay(100);
  
           // AudioNoInterrupts();
  
            //snooze_config.setAlarm(snooze_hour, snooze_minute, snooze_second);
            //delay(100);
            //Snooze.sleep( snooze_config );
            //Snooze.deepSleep(snooze_config);
            //Snooze.hibernate( snooze_config);
  
            alarm.setAlarm(snooze_hour, snooze_minute, snooze_second);
            Snooze.sleep(config_teensy32);
  
            
            /// ... Sleeping ....
            
            // Waking up
           // if (printDiags==0) usbDisable();
            
            digitalWrite(hydroPowPin, HIGH); // hydrophone on
   
          //  audio_enable();
          //  AudioInterrupts();
            cam_wake();
            audio_power_up();
            //sdInit();  //reinit SD because voltage can drop in hibernate
         }
         
        //digitalWrite(displayPow, HIGH); //start display up on wake
        //delay(100);
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
        mode = 0;  // standby mode
      }
    }
  }
}

void startRecording() {
  if (printDiags)  Serial.println("startRecording");
  FileInit();
  buf_count = 0;
  queue1.begin();
  if (printDiags)  Serial.println("Queue Begin");
}

void continueRecording() {
  if (queue1.available() >= 2) {
    byte buffer[512];
    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
    //digitalWrite(ledGreen, HIGH);
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer+256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    frec.write(buffer, 512); //audio to .wav file
      
    buf_count += 1;
    audioIntervalCount += 1;
//    
//    if(printDiags){
//      Serial.print(".");
//   }
  }
}

void stopRecording() {
  if (printDiags) Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  if (printDiags) Serial.print("Audio Memory Max");
  if (printDiags) Serial.println(maxblocks);
  byte buffer[512];
  queue1.end();
  //queue1.clear();
  AudioMemoryUsageMaxReset();
  //frec.timestamp(T_WRITE,(uint16_t) year(t),month(t),day(t),hour(t),minute(t),second);
  frec.close();
  delay(100);
}



/*
void sdInit(){
     if (!(SD.begin(10))) {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    
    while (1) {
      cDisplay();
      display.println("SD error. Restart.");
      displayClock(getTeensy3Time(), BOTTOM);
      display.display();
      delay(1000);
      
    }
  }
}
*/

void FileInit()
{
   t = getTeensy3Time();
   
   if ((folderMonth != month(t)) | newCard){
    newCard = 0;
    if(printDiags) Serial.println("New Folder");
    folderMonth = month(t);
    sprintf(dirname, "%04d-%02d", year(t), folderMonth);
    //file.dateTimeCallback(file_date_time);
    sd.mkdir(dirname);
   }
   pinMode(vSense, INPUT);  // get ready to read voltage

   // get new filename
   int filenameIncrement = 0;
   sprintf(filename,"%s/%02d%02d%02d%02d.wav", dirname, day(t), hour(t), minute(t), second(t));  //filename is DDHHMM
   while (sd.exists(filename)){
    filenameIncrement++;
    sprintf(filename,"%s/%02d%02d%02d%02d_%d.wav", dirname, day(t), hour(t), minute(t), second(t), filenameIncrement);  //filename is DDHHMM
   }

   // log file
   //SdFile::dateTimeCallback(file_date_time);

   float voltage = readVoltage();
   File logFile;
   if(logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      logFile.print(filename);
      logFile.print(',');
      for(int n=0; n<8; n++){
        logFile.print(myID[n]);
      }
      logFile.print(',');
      logFile.print(gainSetting); 
      logFile.print(',');
      logFile.println(voltage); 
      if(voltage < 3.0){
        logFile.println("Stopping because Voltage less than 3.0 V");
        logFile.close();  
        // low voltage hang but keep checking voltage
        while(readVoltage() < 3.0){
            delay(30000);
        }
      }
      logFile.close();
   }
   else{
    if(printDiags) Serial.print("Log open fail.");
    // resetFunc();
   }
    
   frec = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
   Serial.println(filename);
   
   while (!frec){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count in root directory
    logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE);
    logFile.print("File open failed. Retry: ");
    logFile.println(filename);
    logFile.close();
    frec = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
//    if(file_count>1000) {
//      currentCard += 1; // try next card after many tries
//      if(currentCard==4) resetFunc(); // try starting all over
//      if(sd.begin(chipSelect[currentCard], SD_SCK_MHZ(50))) {
//        newCard = 1;
//      }
    }

    //intialize .wav file header
    sprintf(wav_hdr.rId,"RIFF");
    wav_hdr.rLen=36;
    sprintf(wav_hdr.wId,"WAVE");
    sprintf(wav_hdr.fId,"fmt ");
    wav_hdr.fLen=0x10;
    wav_hdr.nFormatTag=1;
    wav_hdr.nChannels=1;
    wav_hdr.nSamplesPerSec=audio_srate;
    wav_hdr.nAvgBytesPerSec=audio_srate*2;
    wav_hdr.nBlockAlign=2;
    wav_hdr.nBitsPerSamples=16;
    sprintf(wav_hdr.dId,"data");
    wav_hdr.rLen = 36 + nbufs_per_file * 256 * 2;
    wav_hdr.dLen = nbufs_per_file * 256 * 2;
  
    frec.write((uint8_t *)&wav_hdr, 44);

  Serial.print("Buffers: ");
  Serial.println(nbufs_per_file);
}

void checkSD(){
  filesPerCard[currentCard] -= 1;

  if(printDiags){
    Serial.print("Files per card: ");
    Serial.println(filesPerCard[currentCard]);
  }
  
  // find next card with files available
  while(filesPerCard[currentCard] == 0){
    currentCard += 1;
    newCard = 1;
    if(currentCard == 4)  // all cards full
    {
      if(printDiags) Serial.println("All cards full");
      while(1);
    }

  if(!sd.begin(chipSelect[currentCard], SD_SCK_MHZ(50))){
       if(printDiags){
        Serial.print("Unable to access the SD card: ");
        Serial.println(currentCard + 1);
        }
    }
    else
      break;
  }

  if(printDiags){
    Serial.print("Current Card: ");
    Serial.println(currentCard + 1);
  }
}


//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  t = getTeensy3Time();
  *date=FAT_DATE(year(t),month(t),day(t));
  *time=FAT_TIME(hour(t),minute(t),second(t));
}

void AudioInit(){
    // Enable the audio shield, select input, and enable output
 // sgtl5000_1.enable();

 // Instead of using audio library enable; do custom so only power up what is needed in sgtl5000_LHI
  audio_enable();
 
  //sgtl5000_1.inputSelect(myInput);
  //sgtl5000_1.volume(0.0);
  sgtl5000_1.lineInLevel(gainSetting);  //default = 4
  // CHIP_ANA_ADC_CTRL
// Actual measured full-scale peak-to-peak sine wave input for max signal
//  0: 3.12 Volts p-p
//  1: 2.63 Volts p-p
//  2: 2.22 Volts p-p
//  3: 1.87 Volts p-p
//  4: 1.58 Volts p-p (0.79 Vpeak)
//  5: 1.33 Volts p-p
//  6: 1.11 Volts p-p
//  7: 0.94 Volts p-p
//  8: 0.79 Volts p-p (+8.06 dB)
//  9: 0.67 Volts p-p
// 10: 0.56 Volts p-p
// 11: 0.48 Volts p-p
// 12: 0.40 Volts p-p
// 13: 0.34 Volts p-p
// 14: 0.29 Volts p-p
// 15: 0.24 Volts p-p
  //sgtl5000_1.autoVolumeDisable();
 // sgtl5000_1.audioProcessorDisable();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1451606400; // Jan 1 2016
} 
  
// Calculates Accurate UNIX Time Based on RTC Timestamp
unsigned long RTCToUNIXTime(TIME_HEAD *tm){
    int i;
    unsigned const char DaysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    unsigned long Ticks = 0;

    long yearsSince = tm->year + 30; // Years since 1970
    long numLeaps = yearsSince >> 2; // yearsSince / 4 truncated

    if((!(tm->year%4)) && (tm->month>2))
            Ticks+=SECONDS_IN_DAY;  //dm 8/9/2012  If current year is leap, add one day

    // Calculate Year Ticks
    Ticks += (yearsSince-numLeaps)*SECONDS_IN_YEAR;
    Ticks += numLeaps * SECONDS_IN_LEAP;

    // Calculate Month Ticks
    for(i=0; i < tm->month-1; i++){
         Ticks += DaysInMonth[i] * SECONDS_IN_DAY;
    }

    // Calculate Day Ticks
    Ticks += (tm->day - 1) * SECONDS_IN_DAY;

    // Calculate Time Ticks CHANGES ARE HERE
    Ticks += (ULONG)tm->hour * SECONDS_IN_HOUR;
    Ticks += (ULONG)tm->minute * SECONDS_IN_MINUTE;
    Ticks += tm->sec;

    return Ticks;
}

void resetFunc(void){
  EEPROM.write(20, 1); // reset indicator register
  CPU_RESTART
}


void read_EE(uint8_t word, uint8_t *buf, uint8_t offset)  {
  noInterrupts();
  FTFL_FCCOB0 = 0x41;             // Selects the READONCE command
  FTFL_FCCOB1 = word;             // read the given word of read once area

  // launch command and wait until complete
  FTFL_FSTAT = FTFL_FSTAT_CCIF;
  while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF))
    ;
  *(buf+offset+0) = FTFL_FCCOB4;
  *(buf+offset+1) = FTFL_FCCOB5;       
  *(buf+offset+2) = FTFL_FCCOB6;       
  *(buf+offset+3) = FTFL_FCCOB7;       
  interrupts();
}

    
void read_myID() {
  read_EE(0xe,myID,0); // should be 04 E9 E5 xx, this being PJRC's registered OUI
  read_EE(0xf,myID,4); // xx xx xx xx
}

float readVoltage(){
   float  voltage = 0;
   for(int n = 0; n<8; n++){
    voltage += (float) analogRead(vSense) / 1024.0;
    delay(2);
   }
   voltage = 5.9 * voltage / 8.0;   //fudging scaling based on actual measurements; shoud be max of 3.3V at 1023
   return voltage;
}


void cam_wake() {
  digitalWrite(CAM_SW, HIGH);
  delay(2000); //power on camera (if off)
  digitalWrite(CAM_SW, LOW);     
  CAMON = 1;   
}

void cam_start() {
  digitalWrite(CAM_SW, HIGH);
  delay(500);  // simulate  button press
  digitalWrite(CAM_SW, LOW);      
  CAMON = 2;
}

void cam_stop(){
  digitalWrite(CAM_SW, HIGH);
  delay(100);  // simulate  button press
  digitalWrite(CAM_SW, LOW);  
}

void cam_off() {
  digitalWrite(CAM_SW, HIGH);
  delay(3000); //power down camera (if still on)
  digitalWrite(CAM_SW, LOW);      
  CAMON = 0;
}
