#include <Adafruit_NeoPixel.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include <LMP91000.h>
#include <Wire.h>
#include <SPI.h>

#include "ads8353_read_write.pio.h"
#include "ads8353_conversion.pio.h"

#include "proteuse.h"
#include "ads8353.h"

#include "DAC80502.h"

#define PROTEUS_CTS 6
#define PROTEUS_RTS 7
#define PROTEUS_TX 8
#define PROTEUS_RX 9
#define PROTEUS_RESET 10

#define LMP_EN_PIN 26
#define SDA_PIN 4
#define SCL_PIN 5

#define SCLK_PIN 18
#define MISO_PIN 20
#define MOSI_PIN 19

#define DAC80502_CS 1
#define DAC80501_CS 0

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ADS8353 adc(ADS8353_SDI, ADS8353_SDO_BASE, ADS8353_SIDE_BASE);
ProteusE ble(PROTEUS_RESET);
LMP91000 pStat = LMP91000();
DAC80502 dac(DAC80502_CS, 2500);
DAC80502 dac2(DAC80501_CS, 2500);

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  while(!Serial);

  // Set up NeoPixel status
  pixels.begin();
  pixels.setBrightness(20); // not so bright
  
  // Red for just Serial
  pixels.fill(0x00FD1616);
  pixels.show();

  // BLE
  if (!ble.init(PROTEUS_TX, PROTEUS_RX, PROTEUS_CTS, PROTEUS_RTS)) {
    // Orange for BLE
    pixels.fill(0x00ECBD23);
    pixels.show();
    Serial.println("BLE Online");
  }

  pinMode(DAC805021_CS, OUTPUT);
  digitalWrite(DAC80501_CS, HIGH);
  pinMode(DAC80502_CS, OUTPUT);
  digitalWrite(DAC80502_CS, HIGH);

  SPI.setSCK(SCLK_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setRX(MISO_PIN);
  SPI.begin();
  //dac.init();
  //dac2.init();

  // ADC
  uint sm = pio_claim_unused_sm(pio0, true);
  adc.init_pio(pio0, sm);
  if (!adc.init_adc()) {
    // Teal for ADC
    pixels.fill(0x003ADCE7);
    pixels.show();
    Serial.println("ADC Online");
  }
  adc.set_mode(true);

  // LMP
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();

  pStat.setMENB(LMP_EN_PIN);
  pStat.enable();
  if (pStat.isReady()) {
    Serial.println(pStat.getIntZ());
    Serial.println("LMP Online");
  }

  
  // DAC80501
}

// the loop routine runs over and over again forever:
void loop() {
  uint16_t data;
  byte dataHigh = data >> 8;  // high 8 bits
  byte dataLow = data & 0xff; // low 8 bits
  SPI.beginTransaction(SPISettings(1000, MSBFIRST, SPI_MODE1));
  digitalWrite(cs, LOW);
  SPI.transfer(command);
  SPI.transfer(dataHigh);
  SPI.transfer(dataLow);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  dac2.setA(1000);
  adc.do_conversion(1);
  delay(100);
}