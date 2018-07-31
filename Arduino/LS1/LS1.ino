//
// LS1 acoustic recorder
//
// Loggerhead Instruments
// 2017, 2018
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
char codeVersion[14] = "2018-07-31";
static boolean printDiags = 0;  // 1: serial print diagnostics; 0: no diagnostics
int camFlag = 1;
long rec_dur = 60;
long rec_int = 60;
int roundSeconds = 60;//modulo to nearest x seconds
float hydroCal = -170.0;
int wakeahead = 20;  //wake from snooze to give hydrophone and camera time to power up
int noDC = 0; // 0 = freezeDC offset; 1 = remove DC offset
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
const int briteLED = 0;
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

int snooze_hour;
int snooze_minute;
int snooze_second;
int buf_count;
long nbufs_per_file;
boolean settingsChanged = 0;

float mAmpRec = 50;
float mAmpSleep = 3;
float mAmpCam = 600;
byte nBatPacks = 4;
float mAhPerBat = 12000.0; // assume 12Ah per battery pack

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

unsigned char prev_dtr = 0;

float gainDb;

void setup() {
  read_myID();

  chipSelect[0] = CS1;
  chipSelect[1] = CS2;
  chipSelect[2] = CS3;
  chipSelect[3] = CS4;

  Serial.begin(baud);
  HWSERIAL.begin(baud);
  delay(500);

  Wire.begin();

  pinMode(hydroPowPin, OUTPUT);
  pinMode(displayPow, OUTPUT);
  pinMode(CAM_SW, OUTPUT);
  pinMode(briteLED, OUTPUT);
  cam_wake();

  pinMode(vSense, INPUT);
  analogReference(DEFAULT);

  digitalWrite(hydroPowPin, LOW);
  digitalWrite(displayPow, HIGH);
  digitalWrite(CAM_SW, LOW);
  digitalWrite(briteLED, LOW);
  
  //setup display and controls
  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);
  pinMode(SELECT, INPUT_PULLUP);

  delay(500);    

  //setSyncProvider(getTeensy3Time); //use Teensy RTC to keep time
  t = getTeensy3Time();
  if (t < 1451606400) Teensy3Clock.set(1451606400);
  Serial.println("Clock Check");
  t = getTeensy3Time();
  Serial.println(t);
  delay(1000);
  t = getTeensy3Time();
  Serial.println(t);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
  delay(100);
  cDisplay();
  display.println("Loggerhead");
  display.display();

  cDisplay();
  display.println("Loggerhead");
  display.display();

  digitalWrite(hydroPowPin, HIGH);
  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  AudioMemory(100);
  AudioInit(); // this calls Wire.begin() in control_sgtl5000.cpp

  manualSettings();
  setGain();
  SdFile::dateTimeCallback(file_date_time);
  
  cDisplay();
  
  t = getTeensy3Time();
  startTime = getTeensy3Time();
  startTime -= startTime % roundSeconds;  
  startTime += roundSeconds; //move forward
  stopTime = startTime + rec_dur;  // this will be set on start of recording
  
  if (recMode==MODE_DIEL) checkDielTime();  
  
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
  mode = 0;

  // create first folder to hold data
  folderMonth = -1;  //set to -1 so when first file made will create directory
  logFileHeader();
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
        
        if(noDC==0) {
          audio_freeze_adc_hp(); // this will lower the DC offset voltage, and reduce noise
          noDC = -1;
        }
        Serial.println("Record Start.");
        
        stopTime = startTime + rec_dur;
        startTime = stopTime + rec_int;
        if (recMode==MODE_DIEL) checkDielTime();

        Serial.print("Current Time: ");
        printTime(getTeensy3Time());
        Serial.print("Stop Time: ");
        printTime(stopTime);
        Serial.print("Next Start:");
        printTime(startTime);

        mode = 1;
  
        display.ssd1306_command(SSD1306_DISPLAYOFF); // turn off display during recording
        startRecording();
      }
  }


  // Record mode
  if (mode == 1) {
    continueRecording();  // download data  

    if(digitalRead(UP)==0 & digitalRead(DOWN)==0){
      // stop recording
      queue1.end();
      // update wav file header
      wav_hdr.rLen = 36 + buf_count * 256 * 2;
      wav_hdr.dLen = buf_count * 256 * 2;
      frec.seek(0);
      frec.write((uint8_t *)&wav_hdr, 44);
      frec.close();
      display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
      delay(100);
      cDisplay();
      display.println("Stopped");
      display.setTextSize(1);
      display.println("Safe to turn off");
      display.display();
      while(1);
    }
      
    if(buf_count >= nbufs_per_file){       // time to stop?
      if(rec_int == 0){
        frec.close();
        checkSD();
        FileInit();  // make a new file
        buf_count = 0;
      }
      else{
        stopRecording();
        
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
            if(camFlag) cam_off(); //camera off
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
  
            alarm.setAlarm(snooze_hour, snooze_minute, snooze_second);
            Snooze.sleep(config_teensy32);
            
            /// ... Sleeping ....
            
            digitalWrite(hydroPowPin, HIGH); // hydrophone on
   
            if(camFlag) cam_wake();
            audio_power_up();
            //sdInit();  //reinit SD because voltage can drop in hibernate
         }
         else{
          if(camFlag) cam_stop();
         }
         
        //digitalWrite(displayPow, HIGH); //start display up on wake
        //delay(100);
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
        mode = 0;  // standby mode
      }
    }
  }
  asm("wfi"); // reduce power between interrupts
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
      logFile.print(gainDb); 
      logFile.print(',');
      logFile.print(voltage); 
      logFile.print(',');
      logFile.println(codeVersion);
      if(voltage < 3.1){
        if (camFlag){
          cam_off();
          camFlag = 0;
          logFile.println("Camera off.");
        }
      }
      if(voltage < 3.0){
        logFile.println("Stopping because Voltage less than 3.0 V");
        logFile.close();  
       
        // low voltage hang but keep checking voltage
        while(readVoltage() < 3.2){
          digitalWrite(hydroPowPin, LOW);
          alarm.setAlarm(0, 0, 20); // sleep 20 seconds
          Snooze.sleep(config_teensy32);
          delay(10);
        }
        digitalWrite(hydroPowPin, HIGH);
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

void logFileHeader(){
  if(File logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      logFile.println("filename, ID, gain (dB), Voltage, Version");
      logFile.close();
  }
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
  setGain();

}

void setGain(){
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
  switch(gainSetting){
    case 0: gainDb = -20 * log10(3.12 / 2.0); break;
    case 1: gainDb = -20 * log10(2.63 / 2.0); break;
    case 2: gainDb = -20 * log10(2.22 / 2.0); break;
    case 3: gainDb = -20 * log10(1.87 / 2.0); break;
    case 4: gainDb = -20 * log10(1.58 / 2.0); break;
    case 5: gainDb = -20 * log10(1.33 / 2.0); break;
    case 6: gainDb = -20 * log10(1.11 / 2.0); break;
    case 7: gainDb = -20 * log10(0.94 / 2.0); break;
    case 8: gainDb = -20 * log10(0.79 / 2.0); break;
    case 9: gainDb = -20 * log10(0.67 / 2.0); break;
    case 10: gainDb = -20 * log10(0.56 / 2.0); break;
    case 11: gainDb = -20 * log10(0.48 / 2.0); break;
    case 12: gainDb = -20 * log10(0.40 / 2.0); break;
    case 13: gainDb = -20 * log10(0.34 / 2.0); break;
    case 14: gainDb = -20 * log10(0.29 / 2.0); break;
    case 15: gainDb = -20 * log10(0.24 / 2.0); break;
  }
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
   float vDivider = 1.9; //when using 3.3 V ref R9 100K
   float fudgeFactor = 0.85;
   //float vDivider = 4.5;  // when using 1.2 V ref R9 301K
   float vRef = 3.3;
   pinMode(vSense, INPUT);  // get ready to read voltage
   if (vRef==1.2) analogReference(INTERNAL); //1.2V ref more stable than 3.3 according to PJRC
   int navg = 32;
   for(int n = 0; n<navg; n++){
    voltage += (float) analogRead(vSense);
   }
   voltage = fudgeFactor * vDivider * vRef * voltage / 1024.0 / navg;  
   pinMode(vSense, OUTPUT);  // done reading voltage
   return voltage;
}


void cam_wake() {
  digitalWrite(CAM_SW, HIGH);
  delay(2000); //power on camera (if off)
  digitalWrite(CAM_SW, LOW);     
  CAMON = 1;   
}

void cam_start() {
  digitalWrite(briteLED, HIGH);
  digitalWrite(CAM_SW, HIGH);
  delay(200);  // simulate  button press
  digitalWrite(CAM_SW, LOW);      
  CAMON = 2;
}

void cam_stop(){
  digitalWrite(CAM_SW, HIGH);
  delay(200);  // simulate  button press
  digitalWrite(CAM_SW, LOW);  
  delay(100);
  CAMON = 1;   
  digitalWrite(briteLED, LOW);
}

void cam_off() {
  digitalWrite(CAM_SW, HIGH);
  delay(3000);
  digitalWrite(CAM_SW, LOW);  
  digitalWrite(briteLED, LOW);    
  CAMON = 0;
}

void checkDielTime(){
  unsigned int startMinutes = (startHour * 60) + (startMinute);
  unsigned int endMinutes = (endHour * 60) + (endMinute );
  unsigned int startTimeMinutes =  (hour(startTime) * 60) + (minute(startTime));
  
  tmElements_t tmStart;
  tmStart.Year = year(startTime) - 1970;
  tmStart.Month = month(startTime);
  tmStart.Day = day(startTime);
  // check if next startTime is between startMinutes and endMinutes
  // e.g. 06:00 - 12:00 or 
  if(startMinutes<endMinutes){
     if ((startTimeMinutes < startMinutes) | (startTimeMinutes > endMinutes)){
       // set startTime to startHour startMinute
       tmStart.Hour = startHour;
       tmStart.Minute = startMinute;
       tmStart.Second = 0;
       startTime = makeTime(tmStart);
       Serial.print("New diel start:");
       printTime(startTime);
       if(startTime < getTeensy3Time()) startTime += SECS_PER_DAY;  // make sure after current time
       Serial.print("New diel start:");
       printTime(startTime);
       }
     }
  else{  // e.g. 23:00 - 06:00
    if((startTimeMinutes<startMinutes) & (startTimeMinutes>endMinutes)){
      // set startTime to startHour:startMinute
       tmStart.Hour = startHour;
       tmStart.Minute = startMinute;
       tmStart.Second = 0;
       startTime = makeTime(tmStart);
       Serial.print("New diel start:");
       printTime(startTime);
       if(startTime < getTeensy3Time()) startTime += SECS_PER_DAY;  // make sure after current time
       Serial.print("New diel start:");
       printTime(startTime);
    }
  }
}
