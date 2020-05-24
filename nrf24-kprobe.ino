//
// nrf24-kprobe 
//
// Used for a simple battery powered K-type sensor reading temperature in Celcius and sending
// sensor readings to a NRF24 gateway.
//
// Please refer to the GitHub repository for the gateway for further details:
//   https://github.com/bjarne-hansen/nrf24-gateway
//
// There are also a template for a low-power sensor at:
//   https://github.com/bjarne-hansen/nrf24-low-power-sensor
// 

#include <LowPower.h>
#include <Adafruit_MAX31855.h>
#include <printf.h>
#include <RF24.h>

#define PIN_DO         2      // YELLOW
#define PIN_CS         3      // GREEN
#define PIN_CLK        4      // BLUE

#define PIN_RF24_CSN   9      // CSN PIN for RF24 module.
#define PIN_RF24_CE   10      // CE PIN for RF24 module.

#define NRF24_CHANNEL         1             // 0 ... 125
#define NRF24_CRC_LENGTH      RF24_CRC_16   // RF24_CRC_DISABLED, RF24_CRC_8, RF24_CRC_16 for 16-bit
#define NRF24_DATA_RATE       RF24_250KBPS  // RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
#define NRF24_PAYLOAD_SIZE    32            // Max. 32 bytes.
#define NRF24_PA_LEVEL        RF24_PA_MAX   // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX    
#define NRF24_RETRY_DELAY     5             // Delay bewteen retries, 1..15.  Multiples of 250µs.
#define NRF24_RETRY_COUNT     15            // Number of retries, 1..15.

#define PROTOCOL 0x03                       // 0x03 (byte), reading (unsigned int), vcc (unsigned int), temperature (double)
                                            // Python: <BHHf

// Switch on/off debugging via Serial.
#undef DEBUG_PRINT

#include "debug.h"
#ifdef DEBUG_PRINT
#define debug_reading(protocol, reading, vcc, celcius)    debug_print(reading); debug_print(": "); \
                                                          debug_print("temperature="); debug_print(celcius, 2); \
                                                          debug_println(); \
                                                          delay(250);
#else
#define debug_reading(protocol, reading, vcc, celcius)
#endif

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);  
byte sensor_tx_addr[6] = "4LD57";           // Transmission address for readings.
byte payload[32];

Adafruit_MAX31855 sensor(PIN_CLK, PIN_CS, PIN_DO);
unsigned int reading = 0;

void setup() 
{
  debug_begin();  
  debug_println("\n\nnrf24-kprobe sensor, version 1.0.");
  
  debug_println("Configure NRF24 ...");
  nrf24_setup();
  debug_nrf24(radio);
  
  delay(250);
}

void loop() 
{
  unsigned int vcc;
  double celcius;

  // Increase number of readings and read k-probe temperature.
  reading++;
  celcius = sensor.readCelsius();
  // TODO: Battery sensing is not yet implemented.
  vcc = 0;
  
  // Print debug information on serial.
  debug_reading(0x03, reading, vcc, celcius);
  
  // Send data via nrf24
  send_reading(PROTOCOL, reading, vcc, celcius);
  
  // Power down for 14 seconds and wait one second for sensor and ADC to settle.
  radio.powerDown();
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);  
  radio.powerUp();
  delay(1000);
}

void send_reading(byte protocol, unsigned int reading, unsigned int vcc, double celcius)
{
  int offset = 0;

  debug_print("Preparing payload: "); debug_println(reading);
  memcpy(payload + offset, (byte *)(&protocol), sizeof(protocol)); offset += sizeof(protocol); 
  memcpy(payload + offset, (byte *)(&reading), sizeof(reading)); offset += sizeof(reading);
  memcpy(payload + offset, (byte *)(&vcc), sizeof(vcc)); offset += sizeof(vcc);
  memcpy(payload + offset, (byte *)(&celcius), sizeof(celcius)); offset += sizeof(celcius);
  debug_print("Bytes packed: "); debug_println(offset);
  
  int rc = nrf24_send(payload, offset, 5);
  
  #ifdef DEBUG_PRINT
  if (rc > -1)
  {
    debug_print("Payload sent. Retries="); 
    debug_println(rc);   
  }
  else
  {
    debug_println("Failed to send payload.");
  }
  delay(250);
  #endif  
}


//
// NRF24L01 functions
//

int nrf24_send(byte *buf, int bytes, int retries)
{
  int max_retries = retries;
  
  while (retries > 0)
  {
    delay((max_retries - retries) * 50);      
    if (radio.write(payload, bytes))
      break;
    retries--;      
  }

  if (retries == 0)
    return -1;
  else
    return max_retries - retries;
}

void nrf24_setup()
{
  radio.begin();
  
  radio.setAutoAck(true);                 
  radio.enableDynamicPayloads();          
  radio.setPALevel(NRF24_PA_LEVEL);
  radio.setRetries(NRF24_RETRY_DELAY, NRF24_RETRY_COUNT);              
  
  radio.setDataRate(NRF24_DATA_RATE);          
  radio.setChannel(NRF24_CHANNEL);
  radio.setCRCLength(NRF24_CRC_LENGTH);
  radio.setPayloadSize(NRF24_PAYLOAD_SIZE);
  
  radio.openWritingPipe(sensor_tx_addr);  
  radio.stopListening();                  
}
