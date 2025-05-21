/*
 * Balloon Tracker
 * Sends the position of the instrument every 5 minutes and at that time checks the satellite
 * mailbox. If there is a valid message there we transmit it via the short range XBee to any
 * cutdown in range. Cutdowns are responsible for determining if the message is valid and
 * is intended for them.
 * 
 * The unit ID of the tracker is unrelated to the cutdown and is simply for keeping track of
 * the instruments.
 * 
 * V1.1 Sept 28 2021
 */

// Uncomment the next line to send all messages out via XBee for debugging/testing
// #define ENABLE_DEBUG

// This line should be uncomments for flight as it enables iridium, but can be disabled
// for troubleshooting to save data costs.
#define ENABLE_IRIDIUM

// Iridium library diagnostic messages
#define DIAGNOSTICS false

#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include "pins.h"

SoftwareSerial xbeeSerial(PIN_XBEE_SERIAL_RX, PIN_XBEE_SERIAL_TX);
SoftwareSerial gpsSerial(PIN_GPS_SERIAL_RX, PIN_GPS_SERIAL_TX);
TinyGPSPlus gps;
IridiumSBD modem(Serial);

uint8_t unit_id = 17;
char tx_buffer[65];
uint8_t rx_buffer[65];

// Determine the telemetry interval, 2 minutes for the first 90 minutes, then
// down to every 10 minutes.
uint32_t fast_telemetry_ms = 120000;
uint32_t slow_telemetry_ms = 600000;
uint32_t n_packets_change_rate = 90 / (fast_telemetry_ms / 60 / 1000);

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void sendUBX(uint8_t *MSG, uint8_t len)
{
  /*
   * Send a byte array of UBX protocol to the GPS.
   */
  #ifdef ENABLE_DEBUG
  xbeeSerial.println("Sending UBX Message");
  #endif
  for(int i=0; i<len; i++)
  {
    gpsSerial.write(MSG[i]);
    #ifdef ENABLE_DEBUG
    xbeeSerial.println(MSG[i], HEX);
    #endif
  }
  gpsSerial.println();
}
 
boolean getUBX_ACK(uint8_t *MSG)
{
  /*
   * Calculate expected UBX ACK packet and parse UBX response from GPS.
   */
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  #ifdef ENABLE_DEBUG
  xbeeSerial.println("Reading GPS ACK response");
  #endif
 
  // Construct the expected ACK packet    
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
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      #ifdef ENABLE_DEBUG
      xbeeSerial.println("ACK Good");
      #endif
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      #ifdef ENABLE_DEBUG
      xbeeSerial.println("ACK Timeout");
      #endif
      return false;
    }
 
    // Make sure data is available to read
    if (gpsSerial.available())
    {
      b = gpsSerial.read();
      #ifdef ENABLE_DEBUG
      xbeeSerial.println(b, HEX);
      #endif
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID])
      { 
        ackByteID++;
      } 
      else
      {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}

bool setDynamicMode6()
{
  /*
   * Sets up the GPS in high altitude dynamic mode 6.
   */
  static byte gps_set_sucess = 0 ;
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,
                      0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
                      0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  uint32_t start_millis = millis();
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
    // Timeout after 10 seconds
    if ((millis() - start_millis) > 10000)
    {
      return false;
    }
  }
  gps_set_sucess=0;
  return gps_set_sucess;
}

void clear_buffers()
{
  // Clear the tx and rx buffers
  memset(&tx_buffer[0], 0, sizeof(tx_buffer));
  memset(&rx_buffer[0], 0, sizeof(rx_buffer));
}

void update_gps()
{
  // Read in characters from the GPS and parse into nice objects!
  uint8_t read_cnt = 0;
  while(gpsSerial.available() && (read_cnt < 245))
  {
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    char c = gpsSerial.read();
    gps.encode(c);
    read_cnt += 1;
  }
  digitalWrite(PIN_LED_ACTIVITY, LOW);
}

void setup()
{
  // Pin Setup
  pinMode(PIN_LED_ACTIVITY, OUTPUT);
  pinMode(PIN_GPS_PPS, INPUT);
  pinMode(PIN_RB_NETWORK, INPUT);

  // Turn activity on to show we're booting
  //digitalWrite(PIN_LED_ACTIVITY, HIGH);

  // Serial
  Serial.begin(19200);
  xbeeSerial.begin(9600);
  gpsSerial.begin(9600);
  delay(1000);

  // Make sure the GPS is in high altitude mode - Normally we'd look for the ACK, but we're
  // missing it with SoftwareSerial I think - same code works on the control paddle. Until
  // shown to be wrong, we'll asssume this is setting things to high altitude mode.
  setDynamicMode6(); 

  // Spin for about 30 seconds to let the GPS get a lock
  uint32_t start_warmup = millis();
  while ((millis() - start_warmup) < 30000)
  {
    update_gps();
  }

  #ifdef ENABLE_DEBUG
  xbeeSerial.println("Setting up Iridium...");
  #endif

  // Iridium Setup
  #ifdef ENABLE_IRIDIUM
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // High current profile - we've got the power
  if (modem.begin() != ISBD_SUCCESS)
  {
    for (int i=0; i<50; i++)
    {
      digitalWrite(PIN_LED_ACTIVITY, HIGH);
      delay(100);
      digitalWrite(PIN_LED_ACTIVITY, LOW);
      delay(100);
    }
    resetFunc();
  }
  #endif
  

  digitalWrite(PIN_LED_ACTIVITY, LOW);

  #ifdef ENABLE_DEBUG
  xbeeSerial.println("Iridium Setup Complete");
  #endif
}

void tx_rx_tracking()
{
  int err;

  // Clear buffers
  clear_buffers();

  // If it's time to send tracking (every 5 min) gather what we need and send it
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double alt = gps.altitude.meters();
  /*
  char lat_buffer[12];
  char lon_buffer[12];
  char alt_buffer[12];
  dtostrf(lat, 6, 4, lat_buffer);
  dtostrf(lon, 6, 4, lon_buffer);
  dtostrf(alt, 5, 1, alt_buffer);
  */

  sprintf(tx_buffer, "%lu,%d,%.4f,%.4f,%.1f\r", millis(), unit_id, lat, lon, alt);
  //sprintf(tx_buffer, "%lu,%d,%s,%s,%s\r", millis(), unit_id, lat_buffer, lon_buffer, alt_buffer);
  size_t rx_buffer_size = sizeof(rx_buffer);

  #ifdef ENABLE_DEBUG
  xbeeSerial.print("Iridium sending: ");
  xbeeSerial.println(tx_buffer);
  #endif

  #ifdef ENABLE_IRIDIUM
  err = modem.sendReceiveSBDText(tx_buffer, rx_buffer, rx_buffer_size);
  if (err != ISBD_SUCCESS)
  {
    #ifdef ENABLE_DEBUG
    xbeeSerial.print("Send error ");
    xbeeSerial.println(err);
    #endif
  }
  #endif

  #ifdef ENABLE_DEBUG
  xbeeSerial.print("Iridium send complete");
  #endif

  // Transmit anything that was in the rx buffer via the xBee link - repeat it
  // 3 times to be sure it goes through in a noisy environment.
  uint8_t char_size = 0;
  for (uint8_t i=0; i < 53; i++)
  {
    if (rx_buffer[i] != 0x00)
    {
      char_size += 1;
    }
    else
    {
      break;
    }
  }

  #ifdef ENABLE_DEBUG
  xbeeSerial.print("Iridium Buffer Length ");
  xbeeSerial.println(char_size);
  #endif

  if (char_size > 4)
  {
    for (int j=0; j<3; j++)
      {
        for (size_t i=0; i<char_size; i++)
        {
          xbeeSerial.write(rx_buffer[i]);
        }
        delay(30000);
      }
  }
  else
  {
    clear_buffers();
    rx_buffer_size = 0;
  }
}

void loop()
{

  // The main loop updates the GPS and every set interval runs the satellite modem
  // transmit/receive function to send position and check messages.
  static uint32_t last_tx_millis = 0;
  static char first_tx = 1;
  static uint32_t number_packets_sent = 0;

  // Feed the GPS characters to the parser so we are always up to date
  update_gps();

  uint32_t TELEMETRY_MS = slow_telemetry_ms;
  if (number_packets_sent < n_packets_change_rate)
  {
    TELEMETRY_MS = fast_telemetry_ms;
  }

  // If it's been more than the set interval since we last sent telemetry - we do it!
  if (((millis() - last_tx_millis) > TELEMETRY_MS) || first_tx)
  {
    number_packets_sent += 1;
    last_tx_millis = millis();
    first_tx = 0;
    tx_rx_tracking();
    #ifdef ENABLE_DEBUG
    xbeeSerial.println("Done with telemetry if");
    #endif
  }

}

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  //xbeeSerial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  //xbeeSerial.write(c);
}
#endif
