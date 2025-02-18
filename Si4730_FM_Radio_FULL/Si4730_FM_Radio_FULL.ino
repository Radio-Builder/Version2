#include <SI4735.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "AiEsp32RotaryEncoder.h"
#include <TFT_eWidget.h>  
#include <EEPROM.h>
int eeprom_address = 0;
#define EEPROM_SIZE 8


// These are the default frequencies ( Changes these for your area )
uint16_t Preset1=8910;
uint16_t Preset2=10580;
uint16_t Preset3=10390;
uint16_t Preset4=9620;



void plotNeedle(int value, byte ms_delay);
void analogMeter();
#define M_SIZE 0.667

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
	rotaryEncoder.setAcceleration(250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
}

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

//------------------------------- Si4735 Init ------------------------------//
#define RESET_PIN 13
#define FM_FUNCTION 0

uint16_t currentFrequency = 8910;
uint16_t previousFrequency;
SI4735 rx;

void setupSi4735(void)
{
  digitalWrite(RESET_PIN, HIGH);

  // Look for the Si47XX I2C bus address
  int16_t si4735Addr = rx.getDeviceI2CAddress(RESET_PIN);
  if ( si4735Addr == 0 ) {
    Serial.println("Si473X not found!");
    Serial.flush();
    while (1);
  } else {
    Serial.print("The SI473X / SI474X I2C address is 0x");
    Serial.println(si4735Addr, HEX);
  }

  delay(500);
  rx.setup(RESET_PIN, FM_FUNCTION);
  rx.setFM(8400, 10800, Preset1, 10);
  //rx.setAM(100, 510, 198,1);

  delay(500);

  currentFrequency = previousFrequency = rx.getFrequency();

  Serial.println("currentFrequency : " +String(rx.getFrequency()));

  rx.setVolume(60);
}

void writeEepromInt(uint16_t value, int location){
  EEPROM.write(location, value);
  EEPROM.write(location + 1, value >> 8);
  EEPROM.commit();
}

int readEepromInt(int location){
  uint16_t val;

  val = (EEPROM.read(location + 1) << 8);
  val |= EEPROM.read(location);

  return val;
}



void setup()
{
   Serial.begin(115200);
   while(!Serial);

  EEPROM.begin(EEPROM_SIZE);

  Preset1 = readEepromInt(0);
  Preset2 = readEepromInt(2);
  Preset3 = readEepromInt(4);
  Preset4 = readEepromInt(6);

  if(Preset1 < 8800 || Preset1 > 108000)
    Preset1 < 8800;
  
  if(Preset2 < 8800 || Preset2 > 108000)
    Preset2 < 8800;

  if(Preset3 < 8800 || Preset3 > 108000)
    Preset3 < 8800;
  
  if(Preset4 < 8800 || Preset4 > 108000)
    Preset4 < 8800;

  Serial.println("EEPROM Read 0 " + String((uint16_t)readEepromInt(0)));
  Serial.println("EEPROM Read 2 " + String((uint16_t)readEepromInt(2)));
  Serial.println("EEPROM Read 4 " + String((uint16_t)readEepromInt(4)));
  Serial.println("EEPROM Read 6 " + String((uint16_t)readEepromInt(6)));

  setupSi4735();

  setupDisplay();
  setupEncoder();
  

  analogMeter();

  tft.drawString("P1",10,110,2);
  tft.drawString("P2",50,110,2);
  tft.drawString("P3",95,110,2);
  tft.drawString("P4",135,110,2);

  tft.drawRoundRect(5,110,25,15, 3, TFT_BLACK);
  tft.drawRoundRect(45,110,25,15, 3, TFT_BLACK);
  tft.drawRoundRect(90,110,25,15, 3, TFT_BLACK);
  tft.drawRoundRect(130,110,25,15, 3, TFT_BLACK);


  tft.drawString("FM Receiver",40,90,2);

  long unsigned int value =  (currentFrequency -8800)/20;   
  plotNeedle((int)value, 0);

  tft.drawCentreString("  "+String(currentFrequency)+"  ", M_SIZE*120, M_SIZE*75, 2); // Comment out to avoid font 4

}

// Main
void loop()
{ 
  long currentReading = rotaryEncoder.readEncoder();
  long change =  currentReading -lastEncoderReading;
  bool update = false;  
  static int lastState = 0;
  static uint16_t P1_PressCount=0;
  static uint16_t P2_PressCount=0;
  static uint16_t P3_PressCount=0;
  static uint16_t P4_PressCount=0;


  if(change !=0){
    currentFrequency = currentFrequency + change*10;
    if(currentFrequency > 10880)
      currentFrequency = 10880;
    if(currentFrequency < 8800)
      currentFrequency = 8800;

    lastEncoderReading = currentReading;
    update = true;  
    }

  int bState1=digitalRead(SW_1_PIN);
  int bState2=digitalRead(SW_2_PIN);
  int bState3=digitalRead(SW_3_PIN);
  int bState4=digitalRead(SW_4_PIN);
  

  // Nothing Pressed
  if(bState1==0 && bState2==0 && bState3==0 && bState4==0)
  {
    if(P1_PressCount >500){
      writeEepromInt(currentFrequency,0);
      Preset1 = currentFrequency;
      for(int i=0;i<10;i++){
        tft.drawRoundRect(5,110,25,15, 3, TFT_RED);
        delay(100);
        tft.drawRoundRect(5,110,25,15, 3, TFT_BLACK);
        }
    }

    if(P2_PressCount >500){
      writeEepromInt(currentFrequency,2);
      Preset2 = currentFrequency;
      for(int i=0;i<10;i++){
        tft.drawRoundRect(45,110,25,15, 3, TFT_RED);
        delay(100);
        tft.drawRoundRect(45,110,25,15, 3, TFT_BLACK);
        delay(100);
        }
    }

    if(P3_PressCount >500){
      writeEepromInt(currentFrequency,4);
      Preset3 = currentFrequency;
      for(int i=0;i<10;i++){
        tft.drawRoundRect(90,110,25,15, 3, TFT_RED);
        delay(100);
        tft.drawRoundRect(90,110,25,15, 3, TFT_BLACK);
        delay(100);
        }
    }

    if(P4_PressCount >500){
      writeEepromInt(currentFrequency,6);
      Preset4 = currentFrequency;
      for(int i=0;i<10;i++){
        tft.drawRoundRect(130,110,25,15, 3, TFT_RED);
        delay(100);
        tft.drawRoundRect(130,110,25,15, 3, TFT_BLACK);
        delay(100);
        }

    }


    if(P1_PressCount >10 && P1_PressCount <500){
      currentFrequency = Preset1;
      update = true;  
      }
    if(P2_PressCount >10 && P2_PressCount <500){
      currentFrequency = Preset2;
      update = true;  
      }
    if(P3_PressCount >10 && P3_PressCount <500){
      currentFrequency = Preset3;
      update = true;  
      }
    if(P4_PressCount >10 && P4_PressCount <500){
      currentFrequency = Preset4;
      update = true;  
      }
  }


  if(bState1==1)
    P1_PressCount++;
  else
    P1_PressCount=0;

  if(bState2==1)
    P2_PressCount++;
  else
    P2_PressCount=0;

  if(bState3==1)
    P3_PressCount++;
  else
    P3_PressCount=0;

  if(bState4==1)
    P4_PressCount++;
  else
    P4_PressCount=0;

    if(update==true){
    String freq = String(float(currentFrequency)/100.0) + "   ";
    //tft.drawString(freq,70,30,4)+ "   ";
    rx.setFrequency(currentFrequency);
    Serial.println("currentFrequency : " +String(currentFrequency));
    // Serial.print(" [SNR:");
    // Serial.print(rx.getCurrentSNR());
    // Serial.print("dB");

   // long unsigned int value =  ((currentFrequency)*100)/200000;  
    long unsigned int value =  (currentFrequency -8800)/20;   
    Serial.println("Value : " +String(value)); 
    plotNeedle((int)value, 0);
    tft.drawCentreString("        ", M_SIZE*120, M_SIZE*75, 2); // Comment out to avoid font 4
    tft.drawCentreString(String(currentFrequency), M_SIZE*120, M_SIZE*75, 2); // Comment out to avoid font 4

  }

  delay(10);
    
}


#define TFT_GREY 0x5AEB
#define TFT_ORANGE      0xFD20      /* 255, 165,   0 */
//#define M_SIZE 0.667

float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = M_SIZE*120, osy = M_SIZE*120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update

int old_analog =  -999; // Value last displayed

int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;
// #########################################################################
//  Draw the analogue meter on the screen
// #########################################################################
void analogMeter()
{

  // Meter outline
  tft.fillRect(0, 0, M_SIZE*239, M_SIZE*131, TFT_GREY);
  tft.fillRect(1, M_SIZE*3, M_SIZE*234, M_SIZE*125, TFT_WHITE);

  tft.setTextColor(TFT_BLACK);  // Text colour

  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (M_SIZE*100 + tl) + M_SIZE*120;
    uint16_t y0 = sy * (M_SIZE*100 + tl) + M_SIZE*150;
    uint16_t x1 = sx * M_SIZE*100 + M_SIZE*120;
    uint16_t y1 = sy * M_SIZE*100 + M_SIZE*150;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (M_SIZE*100 + tl) + M_SIZE*120;
    int y2 = sy2 * (M_SIZE*100 + tl) + M_SIZE*150;
    int x3 = sx2 * M_SIZE*100 + M_SIZE*120;
    int y3 = sy2 * M_SIZE*100 + M_SIZE*150;

    //Yellow zone limits
    if (i >= -50 && i < -30) {
     tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
     tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    }

    // Short scale tick length
    if (i % 25 != 0) tl = 8;

    // Recalculate coords incase tick lenght changed
    x0 = sx * (M_SIZE*100 + tl) + M_SIZE*120;
    y0 = sy * (M_SIZE*100 + tl) + M_SIZE*150;
    x1 = sx * M_SIZE*100 + M_SIZE*120;
    y1 = sy * M_SIZE*100 + M_SIZE*150;

    // Draw tick
    tft.drawLine(x0, y0, x1, y1, TFT_BLACK);

    // Check if labels should be drawn, with position tweaks
    if (i % 25 == 0) {
      // Calculate label positions
      x0 = sx * (M_SIZE*100 + tl + 10) + M_SIZE*120;
      y0 = sy * (M_SIZE*100 + tl + 10) + M_SIZE*150;
      switch (i / 25) {
        case -2: tft.drawCentreString("88", x0+4, y0-4, 1); break;
        case -1: tft.drawCentreString("93", x0+2, y0, 1); break;
        case 0: tft.drawCentreString("98", x0, y0, 1); break;
        case 1: tft.drawCentreString("103", x0, y0, 1); break;
        case 2: tft.drawCentreString("108", x0-2, y0-4, 1); break;
      }
    }

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * M_SIZE*100 + M_SIZE*120;
    y0 = sy * M_SIZE*100 + M_SIZE*150;
    // Draw scale arc, don't draw the last part
    if (i < 50) tft.drawLine(x0, y0, x1, y1, TFT_BLACK);
  }

 // tft.drawString("%RH", M_SIZE*(3 + 230 - 40), M_SIZE*(119 - 20), 2); // Units at bottom right
  //tft.drawCentreString("%RH", M_SIZE*120, M_SIZE*75, 4); // Comment out to avoid font 4
  tft.drawRect(1, M_SIZE*3, M_SIZE*236, M_SIZE*126, TFT_BLACK); // Draw bezel line

  plotNeedle(0, 0); // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void plotNeedle(int value, byte ms_delay)
{
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  //char buf[8]; dtostrf(value, 4, 0, buf);
  //tft.drawRightString(buf, 33, M_SIZE*(119 - 20), 2);

  if (value < -10) value = -10; // Limit value to emulate needle end stops
  if (value > 110) value = 110;

  // Move the needle until new value reached
  while (!(value == old_analog)) {
    if (old_analog < value) old_analog++;
    else old_analog--;

    if (ms_delay == 0) old_analog = value; // Update immediately if delay is 0

    float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
    // Calculate tip of needle coords
    float sx = cos(sdeg * 0.0174532925);
    float sy = sin(sdeg * 0.0174532925);

    // Calculate x delta of needle start (does not start at pivot point)
    float tx = tan((sdeg + 90) * 0.0174532925);

    // Erase old needle image
    tft.drawLine(M_SIZE*(120 + 24 * ltx) - 1, M_SIZE*(150 - 24), osx - 1, osy, TFT_WHITE);
    tft.drawLine(M_SIZE*(120 + 24 * ltx), M_SIZE*(150 - 24), osx, osy, TFT_WHITE);
    tft.drawLine(M_SIZE*(120 + 24 * ltx) + 1, M_SIZE*(150 - 24), osx + 1, osy, TFT_WHITE);

    // Re-plot text under needle
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    //tft.drawCentreString("%RH", M_SIZE*120, M_SIZE*75, 4); // // Comment out to avoid font 4

    // Store new needle end coords for next erase
    ltx = tx;
    osx = M_SIZE*(sx * 98 + 120);
    osy = M_SIZE*(sy * 98 + 150);

    // Draw the needle in the new postion, magenta makes needle a bit bolder
    // draws 3 lines to thicken needle
    tft.drawLine(M_SIZE*(120 + 24 * ltx) - 1, M_SIZE*(150 - 24), osx - 1, osy, TFT_RED);
    tft.drawLine(M_SIZE*(120 + 24 * ltx), M_SIZE*(150 - 24), osx, osy, TFT_MAGENTA);
    tft.drawLine(M_SIZE*(120 + 24 * ltx) + 1, M_SIZE*(150 - 24), osx + 1, osy, TFT_RED);

    // Slow needle down slightly as it approaches new postion
    if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;

    // Wait before next update
    delay(ms_delay);
  }
}
