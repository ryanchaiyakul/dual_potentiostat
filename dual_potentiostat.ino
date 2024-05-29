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

#include "DPV.h"
#include "SWV.h"

#include "lmp_constants.h"

// Bounds

// Modes
#define SERIAL_MODE 0
#define EXP_MODE 1
#define DEB_MODE 2

#define MINMV -750           // WE-RE lowest voltage
#define MAXMV 750            // WE-RE highest voltage

#define SAMPLE_COUNT 32

// DPV Setting Sources (And Electrochemical Workstation)
// https://pineresearch.com/shop/kb/software/methods-and-techniques/voltammetric-methods/differential-pulse-voltammetry-dpv/

// DPV Settings

// All in mV
int dpvStartMV = 100;
int dpvEndMV = -250;
int dpvIncrE = 5; // Sign calculated later
int dpvAmplitude = 25;

// 10ms granularity (us)
unsigned long dpvPulseWidth = 100000;
unsigned long dpvPulsePeriod = 500000; // Includes pulse
unsigned long dpvQuietTime = 2000000;  // Hold reference at dpvTrueStartMV for x us
unsigned long dpvRelaxTime = 2000000;  // Hold reference at dpvTrueEndMV for x us

unsigned long dpvSamplingOffsetUS = 1670;

// SWV Setting Sources
// https://pineresearch.com/shop/kb/software/methods-and-techniques/voltammetric-methods/square-wave-voltammetry/

// SWV Settings

// All in mV
int swvStartMV = -70;
int swvVerticesMVs[4] = {0}; // up to 4 vertices
int swvEndMV = -500;
int swvIncrE = 1;
int swvAmplitude = 25;

int swvVertices = 0;

// 10ms granularity (us)
unsigned long swvFrequency = 45;     // Hz <= 200 Hz
unsigned long swvQuietTime = 2000000; // Hold reference at swvTrueStartMV for x us
unsigned long swvRelaxTime = 2000000; // Hold reference at swvTrueEndMV for x us
unsigned long swvSamplingOffsetUS = 1000;

// PINS

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

// Loop variables
int8_t state = 0;
Voltammetry *method;
UpdateStatus status;
double mv[2];
double zero_v[2];
bool high_low = true;  // false on low cycle
bool is_ready = false;
int i = 0;

// Debug
bool debug_f = false;
unsigned long start_time;
unsigned count;


Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
ADS8353 adc(ADS8353_SDI, ADS8353_SDO_BASE, ADS8353_SIDE_BASE);
ProteusE ble(PROTEUS_RESET);
LMP91000 pStat = LMP91000();
DAC80502 dac(DAC80502_CS, 2500);
DAC80502 dac2(DAC80501_CS, 2500);

DPV dpv(dpvStartMV,
        dpvEndMV,
        dpvIncrE,
        dpvAmplitude,
        dpvPulseWidth,
        dpvPulsePeriod,
        dpvSamplingOffsetUS,
        dpvQuietTime,
        dpvRelaxTime,
        MINMV,
        MAXMV);

SWV swv(swvStartMV,
        swvEndMV,
        swvVerticesMVs,
        swvVertices,
        swvIncrE,
        swvAmplitude,
        swvFrequency,
        swvSamplingOffsetUS,
        swvQuietTime,
        swvRelaxTime,
        MINMV,
        MAXMV);

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  while(!Serial);

  // Set up NeoPixel status
  pixels.begin();
  pixels.setBrightness(20); // not so bright
  
  // BLE
  if (!ble.init(PROTEUS_TX, PROTEUS_RX, PROTEUS_CTS, PROTEUS_RTS)) {
    // Orange for BLE
    Serial.println("BLE Online");
  }

  // DAC
  SPI.setSCK(SCLK_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setRX(MISO_PIN);
  SPI.begin();

  dac.init();
  dac2.init();

  dac2.setA(2000);

  // ADC
  uint sm = pio_claim_unused_sm(pio0, true);
  adc.init_pio(pio0, sm);
  if (!adc.init_adc()) {
    // Teal for ADC
    Serial.println("ADC Online");
  }
  adc.set_mode(true);

  // LMP
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();

  pStat.setMENB(LMP_EN_PIN);
  pStat.enable();
  delay(50);
  pStat.standby();
  delay(50);
  pStat.disableFET();
  pStat.setGain(6); //set the internal gain as zero, using the zero gain resistor as gain instead
  pStat.setRLoad(0); // 100 ohms load, by default
  pStat.setRefSource(1); // set the reference as the external souce
  pStat.setBiasSign(0); // negative bias
  pStat.setBias(0);
  pStat.setIntZ(1); // set internal reference as 50%
  pStat.setThreeLead();
  delay(2000); //warm-up time for the gas sensor

  if (pStat.isReady()) {
    Serial.println("LMP Online");
  }
  selectSWV();
}

double setLMP91000(LMP91000 pStat, int16_t mv) {
  if (abs(mv) < MIN_MV || abs(mv) > MAX_MV) {
    return 0;
  }
  uint bias = OPTIMAL_BIAS[abs(mv)-MIN_MV];
  uint16_t bin = DAC_BINARY[abs(mv)-MIN_MV];
  pStat.setBias(bias);
  dac2.DAC_WR(DAC80502_A, bin);
  return 2.5 * bin / 65536; // Pre divided by 2
}

double setLMP91000Debug(int16_t mv) {
  if (abs(mv) < MIN_MV || abs(mv) > MAX_MV) {
    return 0;
  }
  uint bias = OPTIMAL_BIAS[abs(mv)-MIN_MV];
  uint16_t bin = DAC_BINARY[abs(mv)-MIN_MV];
  Serial.print("Reconstructed Voltage: ");
  Serial.println(TIA_BIAS[bias] * 5 * bin / 65536);
  return 0.0;
}

void selectDPV()
{
  pixels.fill(0x003ADCE7);
  pixels.show();
  method = &dpv;
}

void selectSWV()
{
  pixels.fill(0x009BFD16);
  pixels.show();
  method = &swv;
}

void serialCMD(byte cmd)
{
  // Single character corrresponds with a command
  switch (cmd)
  {
  // Begin experiment
  case 'b':
  case 'B':
    if (debug_f)
    {
      state = DEB_MODE;
      start_time = micros();
    }
    else
    {
      state = EXP_MODE;
      start_time = micros();
    }
    high_low = true;
    method->reset();
    break;
  // DPV mode
  case 'd':
  case 'D':
    selectDPV();
    break;
  // SWV mode
  case 's':
  case 'S':
    selectSWV();
    break;
  case 'v':
  case 'V':
    debug_f = !debug_f;
    if (debug_f) {
      Serial.println("Debugging mode enabled");
    }
    break;
  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
    pStat.setGain(int(cmd)-0x30);
    break;
  default:
    break;
  }
}

void potentiostatMain() {
  switch (state)
  {
  case SERIAL_MODE:
  {
    if (Serial.available())
    {
      serialCMD(Serial.read());
    }
    break;
  }
  case EXP_MODE:
  {
    status = method->update();
    switch (status)
    {
    case NONE:
      // TODO: Implement working burst mode
      if (is_ready) {
        double mv_low = mv[0] - zero_v[0];
        double mv_high = mv[1] - zero_v[1];
        Serial.println(mv_high - mv_low, 4);
        is_ready = false;
      } 
      
      break;
    case SHIFTV:
      i = 1;
      if (high_low) {
        i = 0;
      }
      zero_v[i] = setLMP91000(pStat, method->getVoltage());
      break;  
    case SAMPLE:
      if (high_low) {
        mv[0]  = adc.do_conversion(SAMPLE_COUNT).A;
        high_low = !high_low;
      } else {
        mv[1]  = adc.do_conversion(SAMPLE_COUNT).A;
        is_ready = true;
        high_low = !high_low;
      }
      break;
    case DONE:
      // Send last sample if there was not time to send it
      state = SERIAL_MODE;
      break;
    }
    break;
  }
  case DEB_MODE:
  {
    // Does not actuate DAC or ADC, just reports what should happen when
    status = method->update();
    switch (status)
    {
    case NONE:
      break;
    case SHIFTV:
      Serial.print(micros() - start_time);
      Serial.print(',');
      Serial.println(method->getVoltage());
      setLMP91000Debug(method->getVoltage());
      break;
    case SAMPLE:
      // Count # of sent samples and timing
      Serial.print(micros() - start_time);
      Serial.print(',');
      Serial.println("Sample");
      count++;
      break;
    case DONE:
      state = SERIAL_MODE;
      break;
    }
  }
  break;
  }
}

// the loop routine runs over and over again forever:
void loop() {
  potentiostatMain();
}