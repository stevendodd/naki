/*
  payload.h
*/
#ifndef payload_h
#define payload_h

class payload
{
  void setup();
  void loop();
  void rtty_delay();
  void rtty_send_string();
  void rtty_init_char();
  void rtty_transmit_char();
  void rtty_stopbit();
  void rtty_txbit(int);
  void rtty_initialise_interrupt();
  void generateTelemetry();
  uint16_t CRC16_checksum(char*);
  void gps_initialise();
  void gps_read();
  void sendUBX(uint8_t*, uint8_t);
  boolean getUBX_ACK(uint8_t*);
  unsigned int getAnalogValue();
  void getTemp();
  void getVoltage();
};

#endif
