#include <SI4735.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "AiEsp32RotaryEncoder.h"
#include "si5351.h"
#include "Wire.h"


//------------------------------- Switches Init ------------------------------//
#define SW_4_PIN 19
#define SW_3_PIN 16
#define SW_2_PIN 4
#define SW_1_PIN 15

void setupSwitches(void)
{
  pinMode(SW_1_PIN,INPUT);  
  pinMode(SW_2_PIN,INPUT);  
  pinMode(SW_3_PIN,INPUT);  
  pinMode(SW_4_PIN,INPUT);  
}

//------------------------------- TFT Display Init ------------------------------//
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
void setupDisplay(void)
{
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_WHITE);
}

//------------------------------- Encoder Init ------------------------------//
#define ROTARY_ENCODER_A_PIN 34
#define ROTARY_ENCODER_B_PIN 35
#define ROTARY_ENCODER_BUTTON_PIN 32
#define ROTARY_ENCODER_STEPS 4
#define ROTARY_ENCODER_VCC_PIN -1
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);
long lastEncoderReading = 0;

void IRAM_ATTR readEncoderISR()
{
	rotaryEncoder.readEncoder_ISR();
}

void setupEncoder(void)
{
  	//we must initialize rotary encoder
	rotaryEncoder.begin();
	rotaryEncoder.setup(readEncoderISR);
	rotaryEncoder.setAcceleration(0); //250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
}

//------------------------------- Si4735 Init ------------------------------//
#include <patch_init.h> // SSB patch for whole SSBRX initialization string

const uint16_t size_content = sizeof ssb_patch_content; // see ssb_patch_content in patch_full.h or patch_init.h

#define AM_FUNCTION 1
#define RESET_PIN 13

#define LSB 1
#define USB 2

bool disableAgc = true;
bool avc_en = true;

int currentBFO = 0;

// Some variables to check the SI4735 status
uint16_t currentFrequency=7131;
uint16_t lastFrequency=7131;
uint8_t currentStep = 1;
uint16_t currentBFOStep = 100;

uint8_t bandwidthIdx = 2;
const char *bandwidth[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};

//uint16_t *BFO_Step = {100,200,500,1000};


long et1 = 0, et2 = 0;

typedef struct
{
  uint16_t minimumFreq;
  uint16_t maximumFreq;
  uint16_t currentFreq;
  uint16_t currentStep;
  uint8_t currentSSB;
} Band;

Band band[] = {
    {520, 2000, 810, 1, LSB},
    {3500, 4000, 3510, 1, LSB},
    {5258, 5406, 5400, 1, USB},
    {7000, 7200, 7146, 1, LSB},
    {11700, 12000, 11940, 1, USB},
    {14000, 14300, 14200, 1, USB},
    {18000, 18300, 18100, 1, USB},
    {21000, 21400, 21200, 1, USB},
    {24890, 25000, 24940, 1, USB},
    {27000, 27700, 27300, 1, USB},
    {28000, 28500, 28400, 1, USB}};

const int lastBand = (sizeof band / sizeof(Band)) - 1;
// int currentFreqIdx = 9;
int currentFreqIdx = 3;
uint8_t currentAGCAtt = 0;

uint8_t rssi = 0;


SI4735 si4735;



void loadSSB()
{
  si4735.setI2CFastModeCustom(500000); // Increase the transfer I2C speed
  si4735.loadPatch(ssb_patch_content, size_content); // It is a legacy function. See loadCompressedPatch 
  si4735.setI2CFastModeCustom(100000); // Set standard transfer I2C speed
}



void setup()
{
   Serial.begin(115200);
   while(!Serial);

  setupDisplay();
  setupEncoder();

// Gets and sets the Si47XX I2C bus address
  int16_t si4735Addr = si4735.getDeviceI2CAddress(RESET_PIN);
  if ( si4735Addr == 0 ) {
    Serial.println("Si473X not found!");
    Serial.flush();
    while (1);
  } else {
    Serial.print("The Si473X I2C address is 0x");
    Serial.println(si4735Addr, HEX);
  }


  si4735.setup(RESET_PIN, AM_FUNCTION);

  delay(10);
  Serial.println("SSB patch is loading...");
  et1 = millis();
  loadSSB();
  et2 = millis();
  Serial.print("SSB patch was loaded in: ");
  Serial.print( (et2 - et1) );
  Serial.println("ms");
  delay(100);
  si4735.setTuneFrequencyAntennaCapacitor(1); // Set antenna tuning capacitor for SW.
  si4735.setSSB(band[currentFreqIdx].minimumFreq, band[currentFreqIdx].maximumFreq, band[currentFreqIdx].currentFreq, band[currentFreqIdx].currentStep, band[currentFreqIdx].currentSSB);
  delay(100);
  currentFrequency = si4735.getFrequency();
  si4735.setAvcAmMaxGain(90); // Sets the maximum gain for automatic volume control on AM/SSB mode (from 12 to 90dB)
  si4735.setVolume(60);
  si4735.setAutomaticGainControl(0, currentAGCAtt);
  si4735.setSSBAudioBandwidth(bandwidthIdx);
  if (bandwidthIdx == 0 || bandwidthIdx == 4 || bandwidthIdx == 5)
        si4735.setSSBSidebandCutoffFilter(0);
  else
        si4735.setSSBSidebandCutoffFilter(1);
  
  
  tft.setTextColor(TFT_WHITE,TFT_BLACK );  
  tft.drawString("Freq ",5,30,4);  
  tft.setTextColor(TFT_BLACK,TFT_WHITE );  
  String freq = String(float(currentFrequency)/1000.0,3)+ "  ";
  tft.drawString(freq,70,30,4);  

  si4735.setSsbAgcOverrite 	(1,0,0);

  setupSwitches();

  si4735.setAutomaticGainControl(1,0);

  si4735.setSSBAutomaticVolumeControl(1);


}

void showSeparator()
{
  Serial.println("\n**************************");
}

void showStatus()
{
  showSeparator();
  Serial.print("SSB | ");

  si4735.getAutomaticGainControl();
  si4735.getCurrentReceivedSignalQuality();
    
  Serial.print((si4735.isAgcEnabled()) ? "AGC ON " : "AGC OFF");
  Serial.print(" | LNA GAIN index: ");
  Serial.print(si4735.getAgcGainIndex());
  Serial.print("/");
  Serial.print(currentAGCAtt);
  
  Serial.print(" | BW :");
  Serial.print(String(bandwidth[bandwidthIdx]));
  Serial.print("kHz");
  Serial.print(" | SNR: ");
  Serial.print(si4735.getCurrentSNR());
  Serial.print(" | RSSI: ");
  Serial.print(si4735.getCurrentRSSI());
  Serial.print(" dBuV");
  Serial.print(" | Volume: ");
  Serial.println(si4735.getVolume());
  //showFrequency();
}

// Main
void loop()
{ 
  long currentReading = rotaryEncoder.readEncoder();
  long change =  currentReading -lastEncoderReading;
  bool update = false;  
  static int lastState = 0;

  if(change !=0){
    if(change > 0 ) change =1;
    if(change < 0 ) change =-1;

    //currentFrequency = currentFrequency + change;

    currentBFO = currentBFO - change*currentBFOStep;

    if(currentBFO < -16000){
      currentFrequency = currentFrequency + 16;
      currentBFO = 0;
    }

    if(currentBFO > 16000){
      currentFrequency = currentFrequency - 16;
      currentBFO = 0;
    }


    lastEncoderReading = currentReading;
    update = true;  

    //si4735.setSSBAudioBandwidth(2);
    }


  


  int bState1=digitalRead(SW_1_PIN);
  int bState2=digitalRead(SW_2_PIN);
  int bState3=digitalRead(SW_3_PIN);
  int bState4=digitalRead(SW_4_PIN);

  if(bState1==1)
  {
    bandwidthIdx++;
    if(bandwidthIdx >5)
      bandwidthIdx=0;
    si4735.setSSBAudioBandwidth(bandwidthIdx);
    do{
      delay(100);
    }
    while(digitalRead(SW_1_PIN)==1);
  }

  if(bState2==1)
  {
    if(currentBFOStep==100)
      currentBFOStep=1000;
    else
      currentBFOStep=100;
    do{
      delay(100);
    }
    while(digitalRead(SW_2_PIN)==1);
  }

  if(bState3==1)
  {
    currentFreqIdx++;
    if(currentFreqIdx==lastBand)
      currentFreqIdx=0;

    si4735.setSSB(band[currentFreqIdx].minimumFreq, band[currentFreqIdx].maximumFreq, band[currentFreqIdx].currentFreq, band[currentFreqIdx].currentStep, band[currentFreqIdx].currentSSB);
    do{
      delay(100);
    }
    while(digitalRead(SW_3_PIN)==1);

    currentFrequency = si4735.getFrequency();
    update = true; 
    currentBFO=0; 
  }

  if(bState4==1)
  {
    showStatus();

    do{
      delay(100);
    }
    while(digitalRead(SW_4_PIN)==1);

  }

  if(update==true){
    String freq = String(float(currentFrequency-(currentBFO/1000.0)),3) + "   ";
    tft.drawString(freq,0,30,4)+ "   ";

    String bfo = String(float(currentBFO),3) + "   ";
    tft.drawString(bfo,70,80,4)+ "   ";

    si4735.setSSBBfo(currentBFO);
    if(lastFrequency != currentFrequency)
      si4735.setFrequency(currentFrequency);
    lastFrequency = currentFrequency;
  }



  

    
}
