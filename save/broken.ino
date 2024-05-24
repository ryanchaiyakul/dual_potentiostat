#include <Adafruit_NeoPixel.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "ads8353_read_write.pio.h"
#include "ads8353_conversion_2.pio.h"

#define PROTEUS_CTS 6
#define PROTEUS_RTS 7
#define PROTEUS_TX 8
#define PROTEUS_RX 9
#define PROTEUS_RESET 10

// Neopixel on KB2040
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

/**
Proteus.h
*/

#define PROTEUS_DEFAULT_BAUD 115200

// TODO: Make it memory safe
class ProteusECommand {
  public:
    ~ProteusECommand();
    ProteusECommand(int8_t, uint16_t);
    ProteusECommand& operator=(const ProteusECommand&);
    int8_t type;
    uint16_t length;
    uint8_t* payload;
};

ProteusECommand::~ProteusECommand() {
  delete[] payload;
}

ProteusECommand::ProteusECommand(int8_t type, uint16_t length) {
  this->type = type;
  this->length = length;
  this->payload = new uint8_t[length];
}


ProteusECommand& ProteusECommand::operator=(const ProteusECommand& src) {
  if (&src == this) {
    return *this;
  }
  delete[] payload;
  this->type = src.type;
  this->length = src.length;
  this->payload = new uint8_t[length];
  for (uint16_t i = 0; i < length; i++) {
    this->payload[i] = src.payload[i];
  }
  return *this;
}

class ProteusE {
public:
  ProteusE(pin_size_t);
  void init(pin_size_t, pin_size_t, pin_size_t, pin_size_t);
  void reset();
  int readCMD();
private:
  pin_size_t reset_pin;
  long baud;
};

ProteusE::ProteusE(pin_size_t reset) {
  this->reset_pin = reset;
  this->baud = PROTEUS_DEFAULT_BAUD;
  pinMode(reset_pin, OUTPUT);
}

void ProteusE::init(pin_size_t tx, pin_size_t rx, pin_size_t cts, pin_size_t rts) {
  Serial2.setTX(PROTEUS_TX);
  Serial2.setRX(PROTEUS_RX);
  Serial2.setCTS(PROTEUS_CTS);
  Serial2.setRTS(PROTEUS_RTS);
  Serial2.begin(baud);
  while(!Serial2);
  reset();
}

void ProteusE::reset() {
  digitalWrite(reset_pin, LOW);
  delay(1);                       // manual pg.30
  digitalWrite(reset_pin, HIGH);
  if(!readCMD()) {
    Serial.println("ProteusE Online");
  }
}

int ProteusE::readCMD() {
  // Start byte must be 0x02
  while(!Serial2.available());
  int test_byte =  Serial2.read();
  if (test_byte != 0x02) {
    return 1;
  }
  byte cs = 0x02;
  
  // Get cmd byte
  while(!Serial2.available());
  test_byte = Serial2.read();
  cs ^= test_byte;
 
  // Get LSB length
  while(!Serial2.available());
  uint16_t length = Serial2.read();
  while(!Serial2.available());
  length += ((uint) Serial2.read()) << 8;
  cs ^= (length & 0xFF) ^ (length >> 8);

  ProteusECommand temp(test_byte, length);
  for (uint16_t i = 0; i < length; i++) {
    while(!Serial2.available());
    temp.payload[i] = (uint8_t) Serial2.read();
    cs ^= temp.payload[i];
  }
  while(!Serial2.available());
  if (cs != Serial2.read()) {
    return 3;
  }
  // TODO: Return copy of ProteusECommand
  return 0;
}

/**
end Proteus.h
*/

/**
start ADS8353.h
*/

class ADS8353 {
public:
  ADS8353(uint8_t);
};

#define ADS8353_SDI 27
#define ADS8353_SIDE_BASE 28
#define ADS8353_SDO_BASE 2
#define ADS8353_2N 65536

ProteusE ble(PROTEUS_RESET);
uint sm, sm_2, sm_3;

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

  ble.init(PROTEUS_TX, PROTEUS_RX, PROTEUS_CTS, PROTEUS_RTS);
  // Orange for BLE
  pixels.fill(0x00ECBD23);
  pixels.show();

  // Load ads8353_read_write to configure registers
  uint offset = pio_add_program(pio0, &ads8353_read_write_program);
  sm = pio_claim_unused_sm(pio0, true); 
  ads8353_read_write_init(pio0, sm, offset, ADS8353_SDI, ADS8353_SDO_BASE+1, ADS8353_SIDE_BASE);

  // set CFR register
  //pio_sm_put_blocking(pio0, sm, 0b0000001001000001);
  pio_sm_put_blocking(pio0, sm, 0b0000001001000001);
  pio_sm_get_blocking(pio0, sm);
  // Read CFR register
  pio_sm_put_blocking(pio0, sm, 0b0000000000001100);
  pio_sm_get_blocking(pio0, sm);
  // Dummy command to get output of CRF read
  pio_sm_put_blocking(pio0, sm, 0);
  uint32_t ret = pio_sm_get_blocking(pio0, sm);
  if (ret == 0b001001000000) {
    // Final Color
    pixels.fill(0x003ADCE7);
    pixels.show();
    Serial.println("ADC Online");
  };

  // TODO: Fix conversion loading
  pio_sm_set_enabled(pio0, sm, false);
  offset = pio_add_program(pio0, &ads8353_conversion_sclk_program);
  ads8353_conversion_sclk_init(pio0, sm, offset, ADS8353_SIDE_BASE);

  offset = pio_add_program(pio0, &ads8353_conversion_sdo_program);
  sm_2 = pio_claim_unused_sm(pio0, true);
  sm_3 = pio_claim_unused_sm(pio0, true);
  ads8353_conversion_sdo_init(pio0, sm_2, sm_3, offset, ADS8353_SDO_BASE+1, ADS8353_SDO_BASE);
  pio_enable_sm_mask_in_sync (pio0, 0b1111);
}

float ads8353_bin_to_float(const uint16_t bin) {
  return 5.0 - (5.0 * bin / ADS8353_2N);
}


// the loop routine runs over and over again forever:
void loop() {

  pio_sm_put_blocking(pio0, sm, 1);
  Serial.println(pio_sm_get_rx_fifo_level(pio0, sm));
  Serial.println(pio_sm_get_tx_fifo_level(pio0, sm));
  uint32_t bin_a = pio_sm_get(pio0, sm_2);
  uint32_t bin_b = pio_sm_get(pio0, sm_3);
  Serial.println(pio_sm_get_rx_fifo_level(pio0, sm_2));
  Serial.println(pio_sm_get_tx_fifo_level(pio0, sm_2));
  Serial.println(bin_a);
  Serial.println(bin_b);

  double a = ads8353_bin_to_float(bin_a);
  double b = ads8353_bin_to_float(bin_b);
  Serial.print(a, 4);
  Serial.print("\t");
  Serial.println(b, 4);
  delay(500);
}