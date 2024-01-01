#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "AiEsp32RotaryEncoder.h"
#include "si5351.h"
#include "Wire.h"
#include "esp32/ulp.h"
#include "driver/adc.h"     

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

//------------------------------- Si5351 Init ------------------------------//
Si5351 si5351(0x60);
uint64_t start_frequency = 6000000;
uint64_t stop_frequency =  8000000;

// uint64_t start_frequency = 9213000;
// uint64_t stop_frequency =  9221000;


uint64_t step_frequency = (stop_frequency-start_frequency)/160;
uint64_t clk_1_frequency = start_frequency; // 7MHz
uint64_t marker_1_frequency = (start_frequency+stop_frequency)/2;

void setupSi5351()
{
// Start serial and initialize the Si5351
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found)
  {
    Serial.println("Device not found on I2C bus!");
  }
  else
    Serial.println("Device found on I2C bus!");  

  si5351.set_correction(146999, SI5351_PLL_INPUT_XO);

  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);
  si5351.set_freq(clk_1_frequency*100, SI5351_CLK0);

  si5351.update_status();
}

void updateTFT()
{

  tft.setTextColor(TFT_YELLOW,TFT_BLUE );  
  String freq = String(clk_1_frequency);
  tft.drawString(freq,60,30,2);

}


#define SW_1_PIN 19
#define SW_2_PIN 16
#define SW_3_PIN 4
#define SW_4_PIN 15

#define SW_ENCODER_PIN 32


int mode = 0;
int gain = 10;

#define MODE_MARKER 0
#define MODE_GAIN 1
#define MODE_BW 2

void setup()
{
   Serial.begin(115200);
   while(!Serial);

  setupDisplay();
  setupEncoder();
  setupSi5351();
 
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_ulp_enable();


  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_WHITE,TFT_BLACK );  
  tft.drawString(" 7.000MHz ",45,0,2);

  pinMode(SW_1_PIN, INPUT);
  pinMode(SW_2_PIN, INPUT);
  pinMode(SW_3_PIN, INPUT);
  pinMode(SW_4_PIN, INPUT);

  uint64_t bandwidth = stop_frequency - start_frequency;
  float f = ((float)bandwidth)/1000000.0f;

  tft.setTextColor(TFT_BLACK,TFT_WHITE );  
  tft.drawString("SP "+String(f,1),3,3,1);

  tft.setTextColor(TFT_BLACK,TFT_WHITE );  
  tft.drawString("G "+String(gain)+" ",127,3,1);
}




// Main
void loop()
{ 

  static bool draw_marker = true;

    int sw1 = digitalRead(SW_1_PIN);
    if(sw1==1){
      Serial.println("SW1 Pressed");
      mode = MODE_GAIN;
    }

    int sw2 = digitalRead(SW_2_PIN);
    if(sw2==1){
      Serial.println("SW2 Pressed");
      mode =  MODE_BW;
    }

    int sw3 = digitalRead(SW_3_PIN);
    if(sw3==1){
      Serial.println("SW3 Pressed");
      mode = MODE_MARKER;
    }

    switch(mode){
      case MODE_MARKER :  tft.setTextColor(TFT_BLUE,TFT_WHITE );  
                tft.drawString("Marker",30,115,2);
                tft.setTextColor(TFT_BLACK,TFT_WHITE );  
                tft.drawString("SP",85,115,2);
                tft.setTextColor(TFT_BLACK,TFT_WHITE );  
                tft.drawString("Gain",115,115,2);
                break;
      case  MODE_BW :  tft.setTextColor(TFT_BLACK,TFT_WHITE ); 
                tft.drawString("Marker",30,115,2);
                tft.setTextColor(TFT_BLUE,TFT_WHITE );  
                tft.drawString("SP",85,115,2);
                tft.setTextColor(TFT_BLACK,TFT_WHITE );  
                tft.drawString("Gain",115,115,2);
                break;
      case  MODE_GAIN :  tft.setTextColor(TFT_BLACK,TFT_WHITE );  
                tft.drawString("Marker",30,115,2);
                tft.setTextColor(TFT_BLACK,TFT_WHITE );  
                tft.drawString("SP",85,115,2);
                tft.setTextColor(TFT_BLUE,TFT_WHITE );  
                tft.drawString("Gain",115,115,2);
                break;


    };


 	if (rotaryEncoder.encoderChanged())
	{
		Serial.print("Value: ");
		Serial.println(rotaryEncoder.readEncoder());

    long currentReading = rotaryEncoder.readEncoder();
    long change =  currentReading -lastEncoderReading;

    lastEncoderReading = currentReading;

    if(mode == 0){
      marker_1_frequency += step_frequency * change;
      float f = ((float)marker_1_frequency)/1000000.0f;
      tft.setTextColor(TFT_WHITE,TFT_BLACK );  
      tft.drawString(" "+String(f,3)+"MHz ",45,0,2);
    }

    if(mode == 1){
      gain += change;
      if(gain <= 1) gain = 1;
      if(gain >= 20) gain = 20;
      tft.setTextColor(TFT_BLACK,TFT_WHITE );  
      tft.drawString("G "+String(gain)+" ",127,3,1);
    }

    if(mode == 2){
      start_frequency -= change*50000;
      stop_frequency  += change*50000;
      step_frequency = (stop_frequency-start_frequency)/160;
      uint64_t bandwidth = stop_frequency - start_frequency;
      if(bandwidth <=0){
      start_frequency = 7000000-500;
      stop_frequency  = 7000000+500;;
      step_frequency = (stop_frequency-start_frequency)/160;
      uint64_t bandwidth = stop_frequency - start_frequency;

      }
      float f = ((float)bandwidth)/1000000.0f;
      tft.setTextColor(TFT_BLACK,TFT_WHITE );  
      tft.drawString("SP "+String(f,1),3,3,1);
    }

	}

    static int x_count = 0;
    
    si5351.set_freq(clk_1_frequency*100, SI5351_CLK0);   

    clk_1_frequency += step_frequency;
    if(clk_1_frequency > stop_frequency){
      clk_1_frequency = start_frequency;
      x_count = 0;
      draw_marker = true;
    }

    delay(1);

    int value = adc1_get_raw(ADC1_CHANNEL_0);

    for(int i =13;i<115;i++)
    {
      tft.drawPixel(x_count,i , tft.color565(50,50,50));
    }

    int y_value = 115 - ((value)/(21-gain));
    if(y_value<=13)
      {    
      y_value=14;
      tft.drawPixel(x_count,y_value , tft.color565(0,0,255));
      tft.drawPixel(x_count,y_value-1 , tft.color565(0,0,255));
      }
      else
      {
      tft.drawPixel(x_count,y_value , tft.color565(255,255,0));
      tft.drawPixel(x_count,y_value-1 , tft.color565(255,255,0));
      }

    if((clk_1_frequency / marker_1_frequency) == 1)
      if(draw_marker == true)
      {
        for(int i =13;i<115;i++)
        {
          tft.drawPixel(x_count,i , tft.color565(255,255,255));
        }
        draw_marker = false;
      }

    x_count++;
}

// Pseudocode

// CurrentFrequency = Lower Limit

// Do Forever :
//   Set Frequency to CurrentFrequency
//   Read Voltage Level
//   Plot Voltage
//   Increment CurrentFrequency
//   If CurrentFrequency > Upper Limit:
//     CurrentFrequency = Lower Limit





