// RadioLab Superheterodyne Radio Sketch

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "si5351.h"
#include "Wire.h"

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

//------------------------------- Si5351 Init ------------------------------//
Si5351 si5351(0x60);
//#define IF_FILTER_PEAK (4914875 - 800)
//#define IF_FILTER_PEAK (9218616)
//#define IF_FILTER_PEAK (9216283)

#define IF_FILTER_PEAK (9215285)


int slelectedClock = 0;
uint64_t clk_1_frequency = IF_FILTER_PEAK + 7123000; 
uint64_t clk_2_frequency = IF_FILTER_PEAK;
uint64_t clk_3_frequency = 0;

void setupSi5351()
{
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found)
  {
    Serial.println("Device not found on I2C bus!");
  }
  else
    Serial.println("Device found on I2C bus!");  

  si5351.set_correction(191000, SI5351_PLL_INPUT_XO);

  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_2MA);

  si5351.set_freq(clk_1_frequency*100, SI5351_CLK0);
  si5351.set_freq(clk_2_frequency*100, SI5351_CLK1);

  si5351.update_status();
}


void updateTFT()
{

  String freq = String(clk_1_frequency-IF_FILTER_PEAK);
  tft.drawString(freq+"+IF",60,30,2);

  freq = String(clk_2_frequency);
  tft.drawString(freq,60,50,2);

  freq = String(clk_3_frequency);
  tft.drawString(freq,60,70,2);  

  tft.drawString("  ",145,30,2);
  tft.drawString("  ",145,50,2);
  tft.drawString("  ",145,70,2);

  switch(slelectedClock)
  {
  case 0: tft.drawString("<-",145,30,2);  break;
  case 1: tft.drawString("<-",145,50,2);  break;
  case 2: tft.drawString("<-",145,70,2);  break;
  }
}

#define SW_4_PIN 19
#define SW_3_PIN 16
#define SW_2_PIN 4
//#define SW_ENCODER_PIN 32

void setup()
{
   Serial.begin(115200);
   while(!Serial);

  setupDisplay();
  setupEncoder();
  setupSi5351();
 
  tft.fillRect(0,0, 160, 128, TFT_BLACK);    

  tft.drawString(" www.RADIOBUILDER.org",5,5,2);

  tft.drawString(" OSC-1 ",5,30,2);
  tft.drawString(" OSC-2 ",5,50,2);
  tft.drawString(" OSC-3 ",5,70,2);

  tft.fillRoundRect(5, 108,30,15,3, TFT_BLUE); 
  tft.fillRoundRect(45, 108,30,15,3, TFT_RED); 
  tft.fillRoundRect(85, 108,30,15,3, TFT_GREEN); 
  tft.fillRoundRect(125, 108,30,15,3,TFT_YELLOW);

  updateTFT();

  pinMode(SW_4_PIN, INPUT);
}

float value = 0.0f;
int sign = 1;


#define IDLE      0
#define RAMP_UP   1
#define STEADY    2
#define RAMP_DOWN 3
#define QSK       4


int state = IDLE;
float amplitude = 0.0f;
float volume = 0.5;
int debounce = 0;
int qsk_counter = 0;


#define RAMP_VALUE    0.005
#define MAX_VALUE     2*3.14
#define SMAPLE_SIZE   0.13;
#define DEBOUNCE_TIME 1000
#define QSK_TIME      50000


void loop() {
	if (rotaryEncoder.encoderChanged())
	{
  Serial.print("Encoder Changed ");

    long currentReading = rotaryEncoder.readEncoder();
    long change =  currentReading -lastEncoderReading;

    lastEncoderReading = currentReading;

    switch(slelectedClock)
    {
    case 0: clk_1_frequency += change*100;  break;
    case 1: clk_2_frequency += change*100;  break;
    case 2: clk_3_frequency += change*10000;  break;
    }
    updateTFT();
    si5351.set_freq(clk_1_frequency*100, SI5351_CLK0);
    si5351.set_freq(clk_2_frequency*100, SI5351_CLK1);
    si5351.set_freq(clk_3_frequency*100, SI5351_CLK2);    
    
	}


	if (rotaryEncoder.isEncoderButtonClicked(1))
	{
		Serial.print("Button Clicked ");
    slelectedClock++;
    if(slelectedClock>2)
    slelectedClock = 0;

    updateTFT();
	}

  // bool KeyDown = digitalRead(SW_4_PIN);
  // //Serial.print("KeyDown : " + String(KeyDown));


  // float f;
  // switch(state){
  //   case IDLE     : if(KeyDown == true)
  //                     if(debounce++ >= DEBOUNCE_TIME){
  //                       state = RAMP_UP; 
  //                       amplitude = 0.0f;
  //                       value = 0.0f;
  //                       si5351.set_freq(0, SI5351_CLK1);
  //                       }
                    
  //                   break;
  //   case RAMP_UP  : debounce = 0;
  //                   f = (sin(((float)value)) );
  //                   f = (f * amplitude* 127.5 * volume) + 127.5;
  //                   dacWrite(25, (uint8_t)f);
  //                   amplitude = amplitude + RAMP_VALUE;
  //                   if(amplitude >= 1.0f)
  //                     state = STEADY;
  //                   value = value + SMAPLE_SIZE;
  //                   if(value >=MAX_VALUE) value = 0;
                    
  //                   break;
  //   case STEADY   : f = (sin(((float)value)));
  //                   f = (f * amplitude* 127.5 * volume)  + 127.5;
  //                   dacWrite(25, (uint8_t)f);
  //                   if(KeyDown == false)
  //                     state = RAMP_DOWN;
  //                   value = value + SMAPLE_SIZE;
  //                   if(value >=MAX_VALUE) value = 0;

  //                   break;
  //   case RAMP_DOWN: f = (sin(((float)value)) );
  //                   f = (f * amplitude* 127.5 * volume)  + 127.5;
  //                   dacWrite(25, (uint8_t)f);
  //                   amplitude = amplitude - RAMP_VALUE;
  //                   if(amplitude <= 0.0f){
  //                     state = QSK;
  //                   }
  //                   value = value + SMAPLE_SIZE;
  //                   if(value >=MAX_VALUE) value = 0;

  //                   break;
  //   case QSK      : qsk_counter++;
  //                   if(qsk_counter >= QSK_TIME){
  //                     si5351.set_freq(clk_2_frequency*100, SI5351_CLK1);
  //                     state = IDLE;
  //                     qsk_counter = 0;
  //                     }
  //                   if(KeyDown == true){
  //                     state = IDLE;
  //                     qsk_counter = 0;
  //                     }

  //                   break;



  //   };

  // bool KeyDown = digitalRead(SW_4_PIN);
  // if(KeyDown == true)
  //   si5351.set_freq(0, SI5351_CLK1);
  // else
  //   si5351.set_freq(clk_2_frequency*100, SI5351_CLK1);


 }


