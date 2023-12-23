
#include "si5351.h"
#include "Wire.h"

//------------------------------- Si5351 Init ------------------------------//
Si5351 si5351(0x60);

//uint64_t clk_1_frequency = 7000000; // 7MHz
uint64_t clk_1_frequency = 10000000; // 

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

  si5351.set_correction(0*100, SI5351_PLL_INPUT_XO);

  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);
  si5351.set_freq(clk_1_frequency*100, SI5351_CLK0);

  si5351.update_status();
}


void setup() {
  Serial.begin(115200);
    while(!Serial);

  setupSi5351();
}

void loop() {
  // put your main code here, to run repeatedly:

}
