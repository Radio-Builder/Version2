#include "FreqCountESP.h"
#include "si5351.h"
#include "Wire.h"
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include <TFT_eWidget.h>  

TFT_eSPI tft = TFT_eSPI(); 
#define TFT_GREY 0x5AEB // New colour


//------------------------------- TFT Display Init ------------------------------//
//TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
//MeterWidget   FQ  = MeterWidget(&tft);
void setupDisplay(void)
{
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_GREY); 
  tft.setTextColor(TFT_BLACK,TFT_GREY);  tft.setTextSize(2);

  tft.drawString("FREQUENCY",10,5,2);
  tft.drawString(" COUNTER ",10,35,2);


   tft.setTextColor(TFT_RED,TFT_GREY);  tft.setTextSize(2);
    
}


Si5351 si5351(0x60);

uint64_t clk_1_frequency = 10000000; 
uint64_t clk_2_frequency = 20000000;
uint64_t clk_3_frequency = 1000000;

void setupSi5351()
{
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found)
  {
    Serial.println("Device not found on I2C bus!");
  }
  else
    Serial.println("Device found on I2C bus!");  

  si5351.set_correction(2184*100, SI5351_PLL_INPUT_XO);

  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK1,SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK2,SI5351_DRIVE_2MA);

  si5351.set_freq(clk_1_frequency*100, SI5351_CLK0);
  si5351.set_freq(clk_2_frequency*100, SI5351_CLK1);
  si5351.set_freq(clk_3_frequency*100, SI5351_CLK2);

  si5351.update_status();

  setupDisplay();
}


void setup()
{
  int inputPin = 13;
  int timerMs = 1000;
  FreqCountESP.begin(inputPin, timerMs);
  Serial.begin(115200);          
  
  Serial.println("Frequency Counter");
  setupSi5351();
}



void loop()
{
  if (FreqCountESP.available())
  {
    uint32_t frequency = FreqCountESP.read();

    float CalibratedFrequency = frequency*(float)(0.1*10000000.0/999966.0);
    Serial.println(CalibratedFrequency);

    //String output = "Frequency : " +String(CalibratedFrequency,0)+ "  ";
    String output = String(CalibratedFrequency,0)+ " Hz   ";
    
    Serial.println(output);

    tft.drawString(output,5,64,2);
  }
}