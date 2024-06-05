#pragma once

#define VREF 2.5
#define ADS8353_SDI 27
#define ADS8353_SIDE_BASE 28
#define ADS8353_SDO_BASE 2
#define ADS8353_2N 65536

class ADS8353Conversion {
  public:
    ADS8353Conversion(double A, double B) : A(A), B(B) {};
    double A;
    double B;
};

class ADS8353 {
public:
  ADS8353(uint8_t, uint8_t, uint8_t);
  void init_pio(PIO, uint);
  int init_adc();
  void set_mode(bool);
  ADS8353Conversion do_conversion(uint32_t count);
  float bin_to_float(const uint16_t bin);
  uint16_t bin_to_mv(const uint16_t bin);
private:
  PIO pio;
  uint sm;
  pio_sm_config read_write_config, conversion_config;
  uint read_write_offset, conversion_offset;
  uint8_t sdi;
  uint8_t sdo_b; // b -> a
  uint8_t side; // CS -> SCLK
};

ADS8353::ADS8353(uint8_t sdi, uint8_t sdo_b, uint8_t side) {
  this->sdi = sdi;
  this->sdo_b = sdo_b;
  this->side = side;
}

void ADS8353::init_pio(PIO pio, uint sm) {
  read_write_offset = pio_add_program(pio0, &ads8353_read_write_program);
  read_write_config = ads8353_read_write_init(pio, sm, read_write_offset, sdi, sdo_b+1, side);
  conversion_offset = pio_add_program(pio0, &ads8353_conversion_program);
  conversion_config = ads8353_conversion_init(pio, sm, conversion_offset, sdo_b, side);
  this->pio = pio;
  this->sm = sm;
}

int ADS8353::init_adc() {
  set_mode(false);
    // set CFR register
  pio_sm_put_blocking(pio, sm, 0b0000001001000001);
  //pio_sm_put_blocking(pio, sm, 0b0000000001000001);
  pio_sm_get_blocking(pio, sm);

  // Read CFR register
  pio_sm_put_blocking(pio, sm, 0b0000000000001100);
  pio_sm_get_blocking(pio, sm);

  // Dummy command to get output of CRF read
  pio_sm_put_blocking(pio, sm, 0);
  uint32_t ret = pio_sm_get_blocking(pio, sm);
  //if (ret == 0b001000000000) {
  if (ret == 0b001001000000) {
    return 0;
  }
  return 1;
}

void ADS8353::set_mode(bool is_conversion) {
  pio_sm_set_enabled(pio, sm, false);

  uint offset = read_write_offset;
  pio_sm_config c = read_write_config;
  if (is_conversion) {
    offset = conversion_offset;
    c = conversion_config;
  }

  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);
}

float ADS8353::bin_to_float(const uint16_t bin) {
  return 5.0 - 5.0 * bin / ADS8353_2N;
}

uint16_t ADS8353::bin_to_mv(const uint16_t bin) {
  return 50000 - 5000 * bin / ADS8353_2N;
}

ADS8353Conversion ADS8353::do_conversion(uint32_t count) {
  pio_sm_put_blocking(pio, sm, count+1);
  double a_sum = 0;
  double b_sum = 0;
  for (int i = 0; i < count; i++) {
    uint32_t ret = pio_sm_get_blocking(pio, sm);
    uint16_t bin_a, bin_b;
    bin_a = bin_b = 0;
    for (uint i = 0; i < 16; i++) {
      bin_b |= ((ret >> 2*i) & 0x01) << i;
      bin_a |= ((ret >> (2*i + 1)) & 0x01) << i;
    }
    a_sum += bin_to_float(bin_a);
    b_sum += bin_to_float(bin_b);
    
  }
  return ADS8353Conversion(a_sum / count, b_sum / count);
}