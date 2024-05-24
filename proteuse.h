#pragma once

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
  int init(pin_size_t, pin_size_t, pin_size_t, pin_size_t);
  int reset();
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

int ProteusE::init(pin_size_t tx, pin_size_t rx, pin_size_t cts, pin_size_t rts) {
  Serial2.setTX(tx);
  Serial2.setRX(rx);
  Serial2.setCTS(cts);
  Serial2.setRTS(rts);
  Serial2.begin(baud);
  while(!Serial2);
  return reset();
}

int ProteusE::reset() {
  digitalWrite(reset_pin, LOW);
  delay(1);                       // manual pg.30
  digitalWrite(reset_pin, HIGH);
  return readCMD();
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