#include <avr/wdt.h>
#include <avr/io.h>
#include <util/crc16.h>
#include <TinyGPS.h> 
#include <avr/interrupt.h>
#include <SoftwareSerial.h>
#include <./payload.h>

// --------------------------------
// define methods
// --------------------------------
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
void CRC16_checksum();
void gps_initialise();
void gps_read();
void sendUBX(uint8_t*, uint8_t);
boolean getUBX_ACK(uint8_t*);
unsigned int getAnalogValue();
void getTemp();
void getVoltage();
 
SoftwareSerial Logger(4, 5);  // Log debug over software serial RX (Digital port 4) & TX (Digital port 5)
 
const float referenceVolts = 3.3;  // The voltage of the ADREF pin

const int SAMPLE_SIZE = 25;    // Number of samples from ADC readings

// --------------------------------
// RTTY data and control variables
// -------------------------------- 
#define RADIOPIN 5              // Output pin for RTTY communication
 
#define ASCII 7                 // Number of ASCII bits for each char
#define STOPBITS 2              // Number of stop bits to send
#define TXDELAY 2               // Number of seconds to delay between string transmissions
#define RTTY_BAUD 300           // Baud rate to transmit at

// Data and control variables for RTTY transmission
char tx_str[80];                // A copy of the string to store while transmitting
volatile int tx_status=1;       // Transmit status
volatile int tx_strlength=0;    // The length string to transmit
volatile char txc;              // The char to transmit
volatile int txi;               // Compare counter for ASCII length
volatile int txj;               // Index of character in transmission string & delay counter

// --------------------------------
// GPS data and control variables
// --------------------------------
TinyGPS gps;
boolean gpsDisconnected = 0;

// Tiny GPS Struct for storing raw input from the GPS
typedef struct
{
  float flat, flon, falt;
  unsigned long age, time, date;
} gpsd;
gpsd g; // instantiate above struct

// TX sentence Struct for storing formatted input ready to be added to the telemetry string
typedef struct
{
  unsigned int id;                              // Message Id
  int hour, minute, second;                     // Time
  char latbuf[12], lonbuf[12];                  // Latitude and longitude
  char external_temp[10], internal_temp[10];    // Internal and external temperature   TO DO: Reduce size of these char*
  char voltage[10];                             // Voltage to two decimal places
  long int alt;                                 // Altitude
  int sats;                                     // Number of GPS satellites available
} sent;
sent s; // instantiate above struct

// --------------------------------
//   Telemetry String variables 
// --------------------------------
char telemetry_str[80];   // The data string to log and transmit over radio

volatile int trackerMode=1;        // Mode 1 = Generate / Mode 2 = Transmit

unsigned int count=0;  // The number of transmissions sent over radio

// --------------------------------
//   Temperature variables
// --------------------------------
const int internalTempPin = 4;
const int externalTempPin = 5;

// --------------------------------
//   Voltage variables
// --------------------------------
const int batteryPin = 3;

// Voltage divider resistors
const float R1 = 14600;
const float R2 = 2367;
const float resistorFactor = R2/(R1 + R2);

// --------------------------------
//   Reset the chip via watchdog timeout
//   Added this to prevent a failure on startup related to powering off the chip
// --------------------------------
#define Reset_AVR() wdt_enable(WDTO_30MS); while(1) {}
#define MCUCSR MCUSR       // TO DO: Check to see if this is correct MCU[C]SR?

// --------------------------------
//   Initialise on power on
// --------------------------------
void setup()
{
  // clear WDRF, then turn of watchdog as recommended in datasheet
  MCUCSR &= ~(1<<WDRF);
  wdt_disable();
  
  // Set up the digital output pin for Tx to N2TX radio module
  pinMode(RADIOPIN,OUTPUT);

  //pinMode(internalTempPin,INPUT);
  //pinMode(externalTempPin,INPUT);
  //pinMode(batteryPin,INPUT);

  // Setup the software serial logging and print 1st message
  Logger.begin(9600);
  delay(100);
  Logger.println(F("\r\n\r\n\r\nsetup() Project NAKI"));
  
  gps_initialise();
  rtty_initialise_interrupt();
} 
 
// --------------------------------
//   Endless loop while AVR is powered on
// -------------------------------- 
void loop()
{
   if (trackerMode == 1)            // If we can read sensors and generate new telemetry string do so
     generateTelemetry();
   
   delay(500);                    // Don't go around this loop more than once 1/2 a second

   // TO DO: Consider implementing a watchdog timeout
}

// --------------------------------
//   RTTY Methods
// --------------------------------

// This is the optional delay between transmissions.
void rtty_delay()
{
    txj++;
    if(txj>(TXDELAY*RTTY_BAUD))
    {
      tx_status=1;       // Flag delay complete for next interrupt
    }
}

// Initialise transmission
void rtty_send_string()
{
    strcpy(tx_str,telemetry_str);  // Take a copy of the string so it doesn't change mid transmission
    tx_strlength=strlen(tx_str);   // Set the string length for indexing during transmission

    txj=0;                         // Set index to the 1st char in the string
    tx_status=2;                   // Flag string as ready for next interrupt
    rtty_init_char();              // Queue the 1st char for transmission
}

// If there are more characters to send grab a char and start the transmission
void rtty_init_char()
{
    if ( txj < tx_strlength)      // If there are more chars in the string
    {
      txc = tx_str[txj];          // Queue the next char

      txj++;                      // Move to the next char in the string
      tx_status=3;                // Flag the char as ready for transmission on next interrupt

      rtty_txbit (0);             // Send start bit
      txi=0;                      // Set index to the 1st bit of the char
    }
    else
    {
      tx_status=0;                // Flag we have finished transmitting the string for the next interrupt
      txj=0;                      // Reset counter ready for the optional delay
      trackerMode=1;          	  // Allow sensors to be read and new telemetry string to be created
    }  
}

// Transmit the character bit by bit and then the 1st stop bit.
void rtty_transmit_char()
{
    if(txi<ASCII)                 // If we have not transmitted all the bits in the expected ASCII char
    {
      txi++;                      // Move to the next bit
      if (txc & 1) rtty_txbit(1); // If there are more bits in the char and 1st bit is a 1 send a HIGH  TO DO: Better comment
      else rtty_txbit(0);         // Else send a LOW
      txc = txc >> 1;             // Shift the remaining bits to the left        TO DO: Better comment
    }
    else
    {
		rtty_txbit (1);             // Send stop Bit
		tx_status=4;                // Flag char transmission complete
		txi=0;                      // Reset the bit index for the next transmission
    } 
}

// Send the second stop bit if required and then initialise the next character.
void rtty_stopbit()
{
    if(STOPBITS==2)
    {
    	rtty_txbit (1); // Send stop bit if there are 2 stopbits required
    }
    
    rtty_init_char();               // Get ready to transmit the next char
}

// Transmit the rtty bit
void rtty_txbit (int bit)
{
	if (bit)
	{
		digitalWrite(RADIOPIN, HIGH);  // Set voltage high on the radio output pin
	}
	else
	{
		digitalWrite(RADIOPIN, LOW);   // Set voltage low on the radio output pin
	}
}

// RTTY background interrupt
ISR(TIMER1_COMPA_vect)
{
	if (trackerMode == 2)
	{
	  switch(tx_status)
	  {
		  case 0: // This is the optional delay between transmissions.
			rtty_delay();
			break;
		  case 1: // Initialise transmission.
			rtty_send_string();
			break;
		  case 2: // If there are more characters to send grab a char and start the transmission.
			rtty_init_char();
			break;
		  case 3: // Transmit the character bit by bit and then the 1st stop bit.
			rtty_transmit_char();
			break;
		  case 4: // Send the second stop bit if required and then initialise the next character.
			rtty_stopbit();
			break;
	  }
	}
}

void rtty_initialise_interrupt()
{
  cli();                                   // disable global interrupts
  TCCR1A = 0;                              // Zero the Timer/Counter Control Registers
  TCCR1B = 0;
  OCR1A = ((F_CPU / 1024) / (RTTY_BAUD));  // Set the Output Compare Register to depending on the Baud rate we want to trasmit at
  TCCR1B |= 1<<CS10 | 1<<CS12 | 1<<WGM12;  // Set Clock/1024 from prescaler and Compare Match (CTC mode)
  TIMSK1 |= 1<<OCIE1A;                     // Enable Compare Match interupt to execute
  sei();                                   // enable global interrupts
}

// --------------------------------
//   Telemetry Methods variables 
// --------------------------------
void generateTelemetry ()
{
  unsigned int last_id = s.id;      // Get the last sentence id
  while (s.id == last_id)           // Attempt to read the GPS until we get a good sentence
  {
    gps_read();                     // Read GPS and store output in sentence struct
  }

  cli();                            // Disable interrupts to read analog sensors
  getTemp();                        // Read temperature sensors and store them in sentence struct
  getVoltage();                     // Read battery sensors and store them in sentence struct
  sei();                            // Re-enable interrupts
   
  // $$CALLSIGN,sentence_id,time,latitude,longitude,altitude,external temperature,internal temperature,voltage,number of satellites*CHECKSUM\n
  // $$NAKI,00338,14:38:04,51.3254,-0.7730,119,-42,24,6.28,10*8F6B
  sprintf(telemetry_str,"$$NAKI,%05u,%02u:%02u:%02u,%s,%s,%ld,%s,%s,%s,%d",
                count,
                s.hour, s.minute, s.second,
                s.latbuf, s.lonbuf, s.alt,
                s.external_temp, s.internal_temp,
                s.voltage,
                s.sats
  ); // Create the telemetry_str

  CRC16_checksum();  // Add a checksum to telemetry_str
  
  count++;                                        // Increment the Message Id
  Logger.println(telemetry_str);                  // Print the telemetry string to Serial output
  trackerMode=2;                          				// Tell the radio we have new data to transmit
}

void CRC16_checksum ()
{
  size_t i;                                       // Index for the string
  uint16_t crc;                                   // The checksum
  uint8_t c;                                      // A character in the string
  char checksum_str[6];                           // A string representative of the checksum

  crc = 0xFFFF;                                   // Set the checksum mask
 
  for (i = 2; i < strlen(telemetry_str); i++)            // Calculate checksum ignoring the first two $s
  {
    c = telemetry_str[i];                                // The next char to add to the checksum
    crc = _crc_xmodem_update (crc, c);            // Update the checksum with a new char
  }
 
  sprintf(checksum_str, "*%04X\n", crc);          // Create a checksum string
  strcat(telemetry_str,checksum_str);                    // Add the checksum to the end of the telemetry string
}

// --------------------------------
//   ublox GPS Methods
// --------------------------------

/*
$PUBX,00,		Message Type ID
082955.00,   		UTC Time
5119.52561,	 	Latitude, Degrees + minutes
N,			N/S
00046.37466,		Longitude, Degrees + minutes
W,			E/W 
119.578,		Altitude above user datum ellipsoid
G3,			Max Status  (G3 = Stand alone 3D solution)
7.4,			Horizontal accuracy estimate
7.0,			Vertical accuracy estimate
2.079,			Speed over ground
29.36,			Course over ground
0.855,			Vertical velocity positive=downwards
,			Age of most recent DGPS corrections, empty = none available
2.72,			HDOP, Horizontal Dilution of Precision
3.81,			VDOP, Vertical Dilution of Precision
3.18,			TDOP, Time Dilution of Precision
4,			Number of GPS satellites used in the navigation solution
0,			Number of GLONASS satellites used in the navigation solution
0			DR used
*44 			Checksum
*/

void gps_initialise()
{   
  // Indicates a success response from ublox
  boolean gps_set_sucess = 0; 
  
  Logger.println(F("setup() Initialise the GPS Serial at 9600 Baud"));
  //Logger.println(F("setup() GPS is connected to hardware serial port"));
  
  Serial.begin(9600);
  delay(100);
    
  Logger.println(F("setup() Turn off GPS NMEA strings that are not required"));

  //Serial.println("$PUBX,40,DTM,0,0,0,0*46");
  //Serial.println("$PUBX,40,GBS,0,0,0,0*4D");
  //Serial.println("$PUBX,40,GPQ,0,0,0,0*5D");
  //Serial.println("$PUBX,40,GRS,0,0,0,0*5D");
  //Serial.println("$PUBX,40,GST,0,0,0,0*5B"); 
  //Serial.println("$PUBX,40,THS,0,0,0,0*54");
  //Serial.println("$PUBX,40,TXT,0,0,0,0*43");

  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  Serial.println("$PUBX,40,ZDA,0,0,0,0*44");
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");  
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  
  Logger.println(F("setup() Set ublox to Flight Mode"));
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  while(!gps_set_sucess)
  {
    Logger.println(F("setup() sendUBX Command"));
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    Logger.println(F("setup() getUBX Ack"));
    gps_set_sucess=getUBX_ACK(setNav);
    if(!gps_set_sucess)
    {
      Logger.println(F("setup() ERROR gps_set_sucess not set"));

      if(gpsDisconnected == 1)
      {
    	  gps_set_sucess=1;

      } else {
    	  Reset_AVR();
      }
    }
  }
  gps_set_sucess=0;

  Logger.println(F("setup() Finished Setup\r\n\r\n\r\n\r\n"));
  delay(2000);
}
  
void gps_read()
{
  if(gpsDisconnected == 1)
  {
	Logger.println(F("gps_read() Increment sentence id, a unique id for each sentence"));
	s.id = s.id + 1;
	s.hour = 0;
	s.minute = 0;
	s.second = 0;
	s.second = 0;
	g.flat = 51.3255;
	g.flon = -0.7734;
	dtostrf(g.flat,0,4,s.latbuf);
	dtostrf(g.flon,0,4,s.lonbuf);
	s.alt = long(100);
	s.sats = 5;
	delay(1000);
	return; // exit gps_read()
  }

  Logger.println(F("gps_read()"));
  Logger.println(F("gps_read() Request Navigation Data from GPS module"));
  Serial.println("$PUBX,00*33");  // we may not have much time to read the data before we overflow the serial buffer

  unsigned long timer_s = millis() + 3000;

  int c; // char from serial buffer

  while (true)
  {
    if (timer_s < millis())
    {
      Logger.println(""); 
      Logger.println(F("gps_read() ERROR Timed out going around while gps_read"));
      return; // exit gps_read()
    }
    
    while (!Serial.available())
    {
     if (timer_s < millis())
     {
       Logger.println(""); 
       Logger.println(F("gps_read() ERROR Timed out in Serial.available"));
       return; // exit gps_read()
     }
    }
   c = Serial.read();
   Logger.write(c); // Note this takes time, watch we dont overflow the GPS serial buffer
   if(gps.encode(c))
   {
     Logger.println(""); 
     Logger.println(F("gps_read() gps.encode() says we have a full sentence"));
     break; // Break out of this while loop
   }
  }
  
  Logger.println(F("gps_read() Increment sentence id, a unique id for each sentence"));
  s.id = s.id + 1;

  Logger.println(F("gps_read() Convert time to hhmmss"));
  gps.get_datetime(&g.date, &g.time, &g.age);
  s.hour = (g.time / 1000000);
  s.minute = (g.time - (s.hour * 1000000)) / 10000;
  s.second = (g.time - ((s.hour * 1000000) + (s.minute * 10000)));
  s.second = s.second / 100;

  Logger.println(F("gps_read() Retrieve +/- lat/long in 100000ths of a degree"));
  gps.f_get_position(&g.flat, &g.flon);
  dtostrf(g.flat,0,4,s.latbuf);
  dtostrf(g.flon,0,4,s.lonbuf);
  Logger.print("gps_read() Lat: ");
  Logger.println(s.latbuf);
  Logger.print("gps_read() Lon: ");
  Logger.println(s.lonbuf);

  Logger.print(F("gps_read() Get Altitude: "));
  g.falt = gps.f_altitude();
  s.alt = long(g.falt);
  Logger.println(s.alt);

  s.sats = gps.satellites();

  Logger.print(F("gps_read() Sats: "));
  Logger.print(s.sats, DEC);
  Logger.println("");
}  

// UBX Functions

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) 
{
  Logger.println(F("sendUBX()"));
  Logger.print(F("sendUBX() Byte: "));
  for(int i=0; i<len; i++)
  {
    Logger.print(i, DEC);
    Serial.write(MSG[i]);
  }
  Serial.println();
  Logger.println("");
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) 
{  
  Logger.println(F("getUBX_ACK() 1"));
    
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
 
  Logger.println(F("getUBX_ACK() 2 Construct the expected ACK packet"));  
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  Logger.println(F("getUBX_ACK() 3 Calculate the checksums"));
  Logger.print(F("getUBX_ACK() Checksums: "));
  for (uint8_t i=2; i<8; i++)
  {
    Logger.print(i, DEC);
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  Logger.print(ackPacket[8], HEX);
  Logger.print(ackPacket[9], HEX);
  Logger.println("");
 
  while (1)
  {
    Logger.println(F("getUBX_ACK() 4 Test for success?"));
    if (ackByteID > 9)
    {
      Logger.println(F("getUBX_ACK() 4 All packets in order!"));
      return true;
    }
 
    Logger.println(F("getUBX_ACK() 6 Timeout if no valid response in 3 seconds"));
    if (millis() - startTime > 3000)
    {
      Logger.println(F("getUBX_ACK() 7 ERROR Timeout!"));
      return false;
    }
 
    Logger.println(F("getUBX_ACK() 8 Make sure data is available to read"));
    if (Serial.available())
    {
      b = Serial.read();
 
      Logger.println(F("getUBX_ACK() 9 Check that bytes arrive in sequence as per expected ACK packet"));
      if (b == ackPacket[ackByteID])
      {
        Logger.println(F("getUBX_ACK() 10 This ackByteID Correct"));
        ackByteID++;
      } 
      else
      {
        Logger.println(F("getUBX_ACK() 11 ERROR This ackByteID Incorrect, reset and start again"));
        ackByteID = 0;	// Reset and look again, invalid order
      }
    }
  }
}

unsigned int getAnalogValue(int pin)
{
  int i;
  unsigned int val = 0;
  
  for(i=0;i<SAMPLE_SIZE;i++)    // Read the analog pin SAMPLE_SIZE times
  {
	val += analogRead(pin);     // Read the analog pin value and add it to the sample set
	delay(5);
  }

  return(val/SAMPLE_SIZE);      // Return the average of the values read
}

void getTemp()
{
  // -------------------------
  // Internal Temperature
  // -------------------------
  unsigned int val = getAnalogValue(internalTempPin);        // Get samples from analog input
  float internal_vin = referenceVolts * val / 1024.0;        // Calculate the input voltage scale
  //float internal_temp_C = (1.8663 - internal_vin) / 0.01169; // Convert to degrees Celsius
  float internal_temp_C = -1481.96+sqrt(2.1962E6+(1.8639-internal_vin)/(3.88E-6)); // Convert to degrees Celsius
  dtostrf(internal_temp_C,0,0,s.internal_temp);              // Convert to a string and store in the telemtry string struct

  Logger.print("getTemp() ");
  Logger.print(s.internal_temp);
  Logger.print(" Internal Temp degrees C (");
  Logger.print(val);
  Logger.println(")");  

  // -------------------------  
  // External Temperature
  // -------------------------
  val = getAnalogValue(externalTempPin);                      // Get samples from analog input
  float external_vin = referenceVolts * val / 1024.0;         // Calculate the input voltage scale
  //float external_temp_C = (1.8663 - external_vin) / 0.01169;  // Convert to degrees Celsius
  float external_temp_C = -1481.96+sqrt(2.1962E6+(1.8639-external_vin)/(3.88E-6));  // Convert to degrees Celsius
  dtostrf(external_temp_C,0,0,s.external_temp);               // Convert to a string and store in the telemetry string struct
  
  Logger.print("getTemp() ");
  Logger.print(s.external_temp);
  Logger.print(" External Temp degrees C (");
  Logger.print(val);
  Logger.println(")");
}

void getVoltage()
{
  unsigned int val = getAnalogValue(batteryPin);                       // Get samples from analog input

  float voltage = ((referenceVolts * val) / 1024.0) / resistorFactor;  // Calculate the input voltage scale and divide by resistor factor
  dtostrf(voltage,0,2,s.voltage);                                      // Convert to a string and store in the telemtry string struct

  Logger.print("getVoltage() ");
  Logger.print(s.voltage);
  Logger.print(" Voltage (");  
  Logger.print(val);
  Logger.print(")"); 
  Logger.print(" referenceVolts: ");
  Logger.print(referenceVolts);
  Logger.print(" resistorFactor: ");
  Logger.println(resistorFactor);
}

