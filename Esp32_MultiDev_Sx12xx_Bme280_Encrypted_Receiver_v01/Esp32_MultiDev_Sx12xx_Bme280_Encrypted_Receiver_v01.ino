
/*
Version Management

18.02.2025 V01 First version, this is based on 
               'Esp32_MultiDev_Sx12xx_Bme280_Advanced_Receiver_v04'
               Note: you have to define the same AES_KEY as used in the transmitter
               sketch for the node number AND the Base MAC address of the
               ESP32 device where the transmitter sketch is running. Using the
               same transmitter sketch on different ESP32 devices mean you have to 
               define all of them in the receiver sketch (done in 'Node_Settings.h')
*/

/**
* Please uncomment just one hardware definition file that reflects your hardware combination
* for Heltec WiFi LoRa 32 V2 boards use HELTEC_V2
* for Heltec WiFi LoRa 32 V3 boards use HELTEC_V3
* for LilyGo T3S3 LoRa boards use LILYGO_T3S3_SX1262
* for ESP32 Development boards with attached LoRa module SX1276 module and OLED use ESP32_SX1276_OLED
* for ESP32 Development boards with attached LoRa module SX1276 module and TFT use ESP32_SX1276_TFT
* for all other boards and hardware combination you should consider to modify an existing one to your needs
*
* Don't forget to change the Board in Arduino:
* for Heltec V2: Heltec WiFi LoRa 32(V2)
* for Heltec V3: Heltec WiFi LoRa 32(V3) / Wireless shell (V3) / ...
* for LilyGo T3S3 LoRa: ESP32S3 Dev Module
* or ESP32 Development Boards: ESP32-WROOM-DA Module
*
* - or in Tools menue:
* for Heltec V2: Tools - Board - esp32 - Heltec WiFi LoRa 32(V2)
* for Heltec V3: Tools - Board - esp32 - Heltec WiFi LoRa 32(V3) / Wireless shell (V3) / ...
* for LilyGo T3S3 LoRa: Tools - Board - esp32 - ESP32S3 Dev Module
* for ESP32 Development Boards: Tools - Board - esp32 - ESP32-WROOM-DA Module
*
*/

//#define HELTEC_V2
//#define HELTEC_V3
#define LILYGO_T3S3_SX1262
//#define ESP32_SX1276_OLED
//#define ESP32_SX1276_TFT

// ------------------------------------------------------------------
// include the hardware definition files depending on the uncommenting
#ifdef HELTEC_V2
#include "Heltec_V2_Hardware_Settings.h"
#endif

#ifdef HELTEC_V3
#include "Heltec_V3_Hardware_Settings.h"
#endif

#ifdef LILYGO_T3S3_SX1262
#include "LilyGo_T3S3_LoRa_SX1262_Hardware_Settings.h"
#endif

#ifdef ESP32_SX1276_OLED
#include "ESP32_SX1276_OLED_Hardware_Settings.h"
#endif

#ifdef ESP32_SX1276_TFT
#include "ESP32_SX1276_TFT_Hardware_Settings.h"
#endif

// ------------------------------------------------------------------

// when using the (default) OLED display SSD1306 128 * 64 px the maximum length is 25 chars
const String PROGRAM_VERSION = "BME280 Receiver Enc   V01";

// ------------------------------------------------------------------
// internal or external OLED SSD1306 128 * 64 px display

#ifdef IS_OLED
#include "FONT_MONOSPACE_9.h"
// For a connection via I2C using the Arduino Wire include:
#include <Wire.h>
#include "SSD1306.h"  // https://github.com/ThingPulse/esp8266-oled-ssd1306
SSD1306Wire display(OLED_I2C_ADDRESS, OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN);
#endif

#ifdef IS_TFT
// ------------------------------------------------------------------
// TFT display ST7735 1.8' 128 * 160 RGB
#include "FONT_MONOSPACE_9.h"
#include <SPI.h>
#include <Adafruit_GFX.h>                                        // Core graphics library, https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_ST7735.h>                                     // Hardware-specific library for ST7735, https://github.com/adafruit/Adafruit-ST7735-Library
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);  // hardware SPI
#endif

// vars for displaying line 1 to 5 to display in a loop
String display1 = "";
String display2 = "";
String display3 = "";
String display4 = "";
String display5 = "";
// for TFT only
String display6, display7, display8, display9, display10, display11, display12, display13;

bool showDisplay = false;
bool isDisplayOn = false;  // this bool is needed for switching the display off after timer exceeds

// -----------------------------------------------------------------------
// https://github.com/StuartsProjects/SX12XX-LoRa

#include <SPI.h>

#ifdef HELTEC_V2
#include <SX127XLT.h>  //include the appropriate library
SX127XLT LT;           //create a library class instance called LT
#endif

#ifdef HELTEC_V3
#include <SX126XLT.h>  //include the appropriate library
SX126XLT LT;           //create a library class instance called LT
#endif

#ifdef LILYGO_T3S3_SX1262
#include <SX126XLT.h>  //include the appropriate library
SX126XLT LT;           //create a library class instance called LT
#endif

#ifdef ESP32_SX1276_OLED
#include <SX127XLT.h>  //include the appropriate library
SX127XLT LT;           //create a library class instance called LT
#endif

#ifdef ESP32_SX1276_TFT
#include <SX127XLT.h>  //include the appropriate library
SX127XLT LT;           //create a library class instance called LT
#endif

#include <arrayRW.h>        // routines for reading and writing varaibles to an array
#include "LoRa_Settings.h"  // include the setttings file, LoRa frequencies, txpower etc
#include "Node_Settings.h"  // include the node/sketch specific settings

uint8_t RXPacketL;   // stores length of packet received
int16_t PacketRSSI;  // stores RSSI of received packet
int8_t PacketSNR;    // stores signal to noise ratio of received packet
uint8_t RXPayloadL;  // stores length of payload received

uint8_t loRaSpreadingFactor = SPREADING_FACTOR;  // default setting

// prepairing the buffer
const uint8_t RXBUFFER_SIZE = 16;
uint8_t RXBuffer[RXBUFFER_SIZE + 2 + 6]; // 2 crc, node, receiver, counter

// -----------------------------------------------------------------------
// PRG/Boot button
// #define BOOT_BUTTON_PIN 0 // see settings or hardware settings
boolean isBootButtonPressed = false;
uint8_t modeCounter = 0;  // just a counter

void IRAM_ATTR bootButtonPressed() {
  modeCounter++;
  isBootButtonPressed = true;
  // deactivate the interrupt to avoid bouncing
  detachInterrupt(BOOT_BUTTON_PIN);
}

// LilyGo T3S3 support for battery mode
#include "LilyGoLoRaBoard.h"

// -----------------------------------------------------------------------
// AES cryptography

//#define AES_CRYPTOGRAPHY_DEBUG

// AES-256 encryption in mode CBC
// see https://gist.github.com/cnlohr/96128ef4126bcc878b1b5a7586c624ef
#include <aes/esp_aes.h>

// for cryptographic work like SHA-256
#include "mbedtls/md.h"

const uint8_t _AES_KEY_SIZE = 32;
const uint8_t _IV_SIZE = 16;
const uint8_t _TEXT_SIZE = 16;  // plain- and ciphertext size

uint8_t _plaintext[_TEXT_SIZE];
uint8_t _ciphertext[_TEXT_SIZE];
uint8_t _aesKey[_AES_KEY_SIZE];
uint8_t _Iv[_IV_SIZE];

const uint8_t _MAC_BASE_SIZE = 6;
uint8_t rxMacBaseAddress[_MAC_BASE_SIZE] = { 0 };
uint8_t rxNodeNumber;
uint8_t rxReceiverNumber;
uint32_t rxTxPacketCounter;

void loop() {

  // the BOOT button was pressed
  if (isBootButtonPressed) {
    isBootButtonPressed = false;

    // this is left in case you want to add something on Boot button pressing...

    // activate the interrupt again
    attachInterrupt(BOOT_BUTTON_PIN, bootButtonPressed, RISING);
  }

  LT.receive((uint8_t*)&RXBuffer, RXBUFFER_SIZE, 0, NO_WAIT);  // this is a non blocking call

  while (!readDioRx()) {
    // during this idle time the other elements in loop are called here
    // the BOOT button was pressed
    if (isBootButtonPressed) {
      isBootButtonPressed = false;

      // this is left in case you want to add something on Boot button pressing...

      // activate the interrupt again
      attachInterrupt(BOOT_BUTTON_PIN, bootButtonPressed, RISING);
      // reactivate the call
      LT.receive((uint8_t*)&RXBuffer, RXBUFFER_SIZE, 0, NO_WAIT);  // this is a non blocking call
    }
  }

  // at this point we received something
  ledFlash(1, 125);
  printSeconds();

  RXPacketL = LT.readRXPacketL();
  PacketRSSI = LT.readPacketRSSI();
  PacketSNR = LT.readPacketSNR();

  if (RXPacketL == 0) {
    packetIsError();
  } else {
    packetIsOK();
  }

  ledFlash(2, 125);
  Serial.println();
}

/**
* If DIO0 (SX1276) or DIO1 (SX1262) are high a signal was received
**/
boolean readDioRx() {
  // this reads DIO0 (SX1276) or DIO1 (SX1262), depending on device
#ifdef HELTEC_V2
  return digitalRead(SX_DIO0);
#endif
#ifdef HELTEC_V3
  return digitalRead(SX_DIO1);
#endif
#ifdef LILYGO_T3S3_SX1262
  return digitalRead(SX_DIO1);
#endif
#ifdef ESP32_SX1276_OLED
  return digitalRead(SX_DIO0);
#endif
#ifdef ESP32_SX1276_TFT
  return digitalRead(SX_DIO0);
#endif
}

void packetIsOK() {
  printReceptionDetails();
  Serial.println();
  Serial.print(F("Received a packet with "));
  Serial.print(RXPacketL);
  Serial.println(F(" bytes"));
  // encrypted version
  const uint8_t expectedPacketLength = RXBUFFER_SIZE + 2 + 6;
  // 2 additional bytes with CRC and 6 for node & receiver nbr and txpacketcounter

  // at this point we want to accept packets with 18 bytes only
  if (RXPacketL != expectedPacketLength) {
    display1 = PROGRAM_VERSION;
    display2 = "Received a packet with";
    display3 = (String)RXPacketL + " bytes but waiting";
    display4 = "for a packet with";
    display5 = (String)expectedPacketLength + " bytes -> * REJECTED *";
    displayData();
    return;
  }

  // read the packet in the buffer
  // encrypted version: size is RXBUFFER_SIZE + 2 for 2 CRC bytes + 6 bytes
  LT.readPacket(RXBuffer, RXPacketL);

  Serial.print(F("Received Packet: "));
  LT.printHEXPacket(RXBuffer, RXPacketL);  // print the received array as HEX
  Serial.println();
  Serial.flush();

  // get the data from RXBuffer
  uint8_t len;
  beginarrayRW(RXBuffer, 0);              // start reading from array at location 0
  rxNodeNumber = arrayReadUint8();        // 1 byte
  rxReceiverNumber = arrayReadUint8();    // 1 byte
  rxTxPacketCounter = arrayReadUint32();  // 4 bytes
  // 16 bytes of encrypted payload
  uint8_t tempBuff[_TEXT_SIZE];
  arrayReadByteArray(tempBuff, _TEXT_SIZE);  // 16 bytes
  uint16_t txPacketCrc = arrayReadUint16();  // 2 bytes
  len = endarrayRW() + 1; 

  // advanced version: first check that the packet is for THIS receiver
  bool receiverNumberIsMatching = false;
  if (rxReceiverNumber == NODE_NUMBER) {
    receiverNumberIsMatching = true;
    //display3 = "   * Node is ALLOWED *";
    display3 = "* Receiver is MATCHING *";
    display4 = "";
    display5 = "";
  } else {
    // todo don't display the data, just the warning !
    //display3 = "    * Node REJECTED *";
    display3 = "   * Packet REJECTED *";
    display4 = "This receiver nbr:     " + String(NODE_NUMBER, HEX);
    display5 = "Received receiver nbr: " + String(rxReceiverNumber, HEX);
    displayData();
    return;
  }

  // advanced version: second check that the packet is from an allowed node
  bool nodeNumberIsAllowed = false;
  if (rxNodeNumber == ALLOWED_TX_NODE_NUMBER) {
    nodeNumberIsAllowed = true;
    display4 = "* Node is ALLOWED *";
    display5 = "";
  } else {
    // todo don't display the data, just the warning !
    display3 = "    * Node REJECTED *";
    display4 = "Allowed  node nbr:" + String(ALLOWED_TX_NODE_NUMBER, HEX);
    display5 = "Received node nbr:" + String(rxNodeNumber, HEX);
    displayData();
    return;
  }

  // we need the individual AES_KEY and Base MAC address for the transmitting node
  // this is a very simple 'database'
  if (rxNodeNumber == 0x67) {
    Serial.println(F("Transmitting Node: 0x67, use database entries"));
    for (int i = 0; i < _AES_KEY_SIZE; i++) {
      _aesKey[i] = AES_KEY_67[i];
    }
    for (int i = 0; i < _MAC_BASE_SIZE; i++) {
      rxMacBaseAddress[i] = MAC_HEL2_1[i];
    }
  }

  Serial.print(F("AES KEY: "));
  LT.printHEXPacket(_aesKey, _AES_KEY_SIZE);
  Serial.println();
  Serial.print(F("MAC ADR: "));
  LT.printHEXPacket(rxMacBaseAddress, _MAC_BASE_SIZE);
  Serial.println();
  Serial.print(F("TMP BUF: "));
  LT.printHEXPacket(tempBuff, _TEXT_SIZE);
  Serial.println();
  Serial.println(F("----- start decryption -----"));

  // step 1: copy the RXBuffer to the internal var
  // here we are using the TXBuffer
  // step 1: copy the TXBuffer to the internal var
  memset(_ciphertext, 0, _TEXT_SIZE);
  memcpy(_ciphertext, tempBuff, _TEXT_SIZE);
  // calculate the IV on some values
  generateIv();
  // now run the internal AES decryption
  esp_aes_context ctxd;
  esp_aes_init(&ctxd);
  esp_aes_setkey(&ctxd, _aesKey, 256);
  esp_aes_crypt_cbc(&ctxd, ESP_AES_DECRYPT, _TEXT_SIZE, _Iv, (uint8_t*)_ciphertext, (uint8_t*)_plaintext);
  // clear the _ciphertext
  memset(_ciphertext, 0, _TEXT_SIZE);
  Serial.print(F("plaintext:  "));
  serialPrintHexValues(_plaintext, _TEXT_SIZE);
  // do what you want to do with the _plaintext, e.g. displaying
  // here we are putting it in the tempBuff again
  memcpy(tempBuff, _plaintext, _TEXT_SIZE);
  // clear the _plaintext
  memset(_plaintext, 0, _TEXT_SIZE);

  // now we are reading the tempBuff as usual
  beginarrayRW(tempBuff, 0);                              // start reading from array at location 0
  uint8_t nodeNumber = arrayReadUint8();                  // 1 byte
  uint8_t receiverNumber = arrayReadUint8();              // 1 byte
  uint32_t txPacketCounter = arrayReadUint32();           // 4 bytes
  float temperature = arrayReadFloat();                   // 4 bytes
  float humidity = arrayReadUint16() / 100.0;             // 2 bytes, converted back to float
  uint16_t pressure = arrayReadUint16();                  // 2 bytes, no decimals
  uint8_t batteryVoltageTransmission = arrayReadUint8();  // 1 byte
  uint8_t rfu = arrayReadUint8();                         // 1 byte, RFU - reserved for future usage
  // advanced version with added 2 bytes CRC value, already read from SXBuffer
  uint8_t packetLength = endarrayRW() + 1;  // this returns the number of array bytes read

  // this can be done after decryption only
  // advanced version: third check that the packet has a valid CRC checksum
  // added 2 bytes CRC value
  // calculating our own CRC of the payload data on the first 16 bytes
  uint16_t calcCrc = LT.CRCCCITT(tempBuff, 16, 0);
  Serial.print(F("txPacketCRC: "));
  //Serial.println(String(txPacketCrc, HEX));
  Serial.print(F("calcCRC:     "));
  Serial.println(String(calcCrc, HEX));
  bool isCrcOk = false;
  if (txPacketCrc == calcCrc) {
    Serial.println(F("The received CRC matches the calculated CRC"));
    isCrcOk = true;
  } else {
    Serial.println(F("The received CRC DIFFERS from the calculated CRC"));
  }
  // advanced version with CRC check
  if (isCrcOk) {
    display5 = "* Node is ALLOWED CRC OK *";
  } else {
    display5 = "* NodeNbr OK CRC FAIL *";
    // todo don't display the data !
    displayData();
    return;
  }

  /*
    Battery Voltage for transmission is a calculated value to fit in a uint8_t var:
    Usually a Lithium Ion battery has a nominal voltage of 3.7 volts, a maximum of 
    4.2 volts and a minimum of 3 volts. As we have one byte only available we have 
    to adjust the voltage in the range of 0..255 (range of an uint8_t variable).
    We multiply the battery voltage (which has usually 2 decimals like „3.78“) 
    with 100 and get a range of 300 .. 420. Now we are subtracting 200 from the 
    value and get a range of 100..220 which is perfect for our uint8_t variable. 
    On receiver side we are working the opposite way. If we receive e.g. a value 
    of „207“ we add „200“ and the sum is „407“, divided by 100.0 we get our initial 
    value of 4.07 volts.
  */

  float batteryVoltage = (batteryVoltageTransmission + 200.0) / 100.0;

  displayData();
  //delay(10000); // debug output
  delay(1000);
  // display the received data
  display1 = PROGRAM_VERSION;
  display2 = "Node:" + String(nodeNumber, HEX) + " RECV:" + String(receiverNumber, HEX) + " TXCnt:" + (String)txPacketCounter;
  display3 = "* Receiver, Node, CRC OK *";
  display4 = "T:" + String(temperature) + " *C  | H:" + String(humidity) + " %";
  display5 = "P:" + String(pressure) + " hPa  | V: " + String(batteryVoltage, 2) + " V";
  displayData();

}

void append(char* s, char* buffer) {
  byte i;
  int bufferLen = strlen(buffer);
  int len = strlen(s);
  for (i = 0; i < bufferLen; i++) {
    s[len + i] = buffer[i];
  }
}

void packetIsError() {
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();  //get the IRQ status

  if (IRQStatus & IRQ_RX_TIMEOUT) {
    Serial.print(F(" RXTimeout"));
  } else {
    Serial.print(F(" PacketError"));
    printReceptionDetails();
    Serial.print(F("  Length,"));
    Serial.print(LT.readRXPacketL());  //get the real packet length
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
  }
}

void printReceptionDetails() {
  Serial.print(F(" RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB  "));
  display1 = PROGRAM_VERSION;
  display2 = "RXPacketL: " + (String)RXPacketL;
  display3 = "** ERROR **";
  display4 = "RSSI: " + (String)PacketRSSI + " dBm";
  display5 = "SNR : " + (String)PacketSNR + " dB";
  displayData();
}

void ledFlash(uint16_t flashes, uint16_t delaymS) {
  // run only if a LED is connected
  if (LED_PIN >= 0) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
      digitalWrite(LED_PIN, HIGH);
      delay(delaymS);
      digitalWrite(LED_PIN, LOW);
      delay(delaymS);
    }
  }
}

void printSeconds() {
  float secs;
  secs = ((float)millis() / 1000);
  Serial.print(secs, 3);
}

void setup() {
#ifdef LILYGO_T3S3_SX1262
  setupLilyGoBoard();
#else
  Serial.begin(115200);
  while (!Serial) {}
#endif

  Serial.println(F("ESP32 MD LoRa BME280 Receiver Encrypted V01"));

  // if we have a power control for devices put it on
#ifdef IS_VEXT_CONTROL
  setVextControl(true);
#endif

  if (LED_PIN >= 0) {
    pinMode(LED_PIN, OUTPUT);  // setup pin as output for indicator LED
    ledFlash(1, 125);          // two quick LED flashes to indicate program start
  }

  delay(1000);

  // this is necessary as the LilyGo T3S3 board requires a different setup
#ifdef LILYGO_T3S3_SX1262
  SPI.begin(SX_SCK, SX_MISO, SX_MOSI);
#else
  SPI.begin();
#endif

  // setup display
#ifdef IS_OLED
  if (OLED_I2C_RST_PIN >= 0) {
    pinMode(OLED_I2C_RST_PIN, OUTPUT);
    digitalWrite(OLED_I2C_RST_PIN, LOW);  // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(OLED_I2C_RST_PIN, HIGH);
    delay(50);
  }
  clearDisplayData();
  display.init();
#ifdef DISPLAY_ORIENTATION_FLIPPED
  // do nothing
#else
  display.flipScreenVertically();  // Landscape 90 degrees right rotated
#endif
  display.setFont(ArialMT_Plain_10);
  delay(50);
  display1 = PROGRAM_VERSION;
  displayData();
  delay(500);
#endif

  // init TFT display
#ifdef IS_TFT
  tft.initR(INITR_BLACKTAB);     // den ST7735S Chip initialisieren, schwarz
  tft.fillScreen(ST77XX_BLACK);  // und den Schirm mit Schwarz füllen
  tft.setTextWrap(false);        // automatischen Zeilenumbruch ausschalten
#ifdef DISPLAY_ORIENTATION_FLIPPED
  tft.setRotation(1);  // Landscape 270 degrees right rotated
#else
  tft.setRotation(3);              // Landscape 90 degrees right rotated
#endif
  Serial.println(F("Display init done"));
  display1 = PROGRAM_VERSION;
  displayData();
  delay(500);
#endif

  display2 = "Display init done";
  displayData();
  delay(1000);

#ifdef HELTEC_V2
  if (LT.begin(SX_NSS, SX_NRESET, SX_DIO0, LORA_DEVICE)) {
    Serial.println(F("LoRa Device Heltec V2 found"));
    display3 = "LoRa Device HV2 found";
    displayData();
    ledFlash(2, 125);
  } else {
    Serial.println(F("Device error"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
#endif

#ifdef HELTEC_V3
  if (LT.begin(SX_NSS, SX_NRESET, SX_RFBUSY, SX_DIO1, LORA_DEVICE)) {
    Serial.println(F("LoRa Device Heltec V3 found"));
    display3 = "LoRa Device HV3 found";
    displayData();
    ledFlash(1, 125);
  } else {
    Serial.println(F("Device error"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
  // The Heltec V3 board uses an unusual crystal voltage. Somme errors came up
  // when using Reliable communication so I'm setting the value here.
  LT.setDIO3AsTCXOCtrl(TCXO_CTRL_1_8V);
#endif

#ifdef LILYGO_T3S3_SX1262
  if (LT.begin(SX_NSS, SX_NRESET, SX_RFBUSY, SX_DIO1, LORA_DEVICE)) {
    Serial.println(F("LoRa Device LilyGo T3S3 SX1262 found"));
    display3 = "LoRa Dev LilyGoT3S3 found";
    displayData();
    ledFlash(1, 125);
  } else {
    Serial.println(F("Device error"));
    Serial.println(F("LoRa Device LilyGo T3S3 SX1262"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
  // The LilyGo T3S3 board uses an unusual crystal voltage. Somme errors came up
  // when using Reliable communication so I'm setting the value here.
  LT.setDIO2AsRfSwitchCtrl();
  LT.setDIO3AsTCXOCtrl(TCXO_CTRL_1_8V);
#endif

#ifdef ESP32_SX1276_OLED
  if (LT.begin(SX_NSS, SX_NRESET, SX_DIO0, LORA_DEVICE)) {
    Serial.println(F("LoRa Device ESP32/SX1276 found"));
    display3 = "LoRa Dev.ESP32+SX1276";
    displayData();
    ledFlash(2, 125);
  } else {
    Serial.println(F("Device error"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
#endif

#ifdef ESP32_SX1276_TFT
  if (LT.begin(SX_NSS, SX_NRESET, SX_DIO0, LORA_DEVICE)) {
    Serial.println(F("LoRa Device ESP32/SX1276 found"));
    display3 = "LoRa Dev.ESP32+SX1276";
    displayData();
    ledFlash(2, 125);
  } else {
    Serial.println(F("Device error"));
    while (1) {
      Serial.println(F("No device responding"));
      display3 = "No LoRa Device found";
      displayData();
      ledFlash(50, 50);  // long fast speed flash indicates LoRa device error
    }
  }
#endif

  // just to be for sure - use the default syncword
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);  // this is the default value
  // set the high sensitive mode
  // Sets LoRa device for the highest sensitivity at expense of slightly higher LNA current.
  // The alternative is setLowPowerReceive() for lower sensitivity with slightly reduced current.
  LT.setHighSensitivity();
  // start the device with default parameters
  LT.setupLoRa(FREQUENCY, OFFSET, loRaSpreadingFactor, BANDWIDTH, CODE_RATE, OPTIMISATION);
  Serial.println(F("LoRa setup is complete"));

  // debug information
  Serial.println();
  LT.printModemSettings();
  Serial.println();
  LT.printOperatingSettings();
  Serial.println();

  display3 = "LoRa init done";
  display4 = "";
  display5 = "Please wait ...";
  displayData();
  delay(2000);

  display2 = "Frequency: " + (String)(FREQUENCY / 1000) + " Khz";
  display3 = "Spreading Factor: " + (String)SPREADING_FACTOR;
  // this is left in case you want to add something on Boot button pressing...
  //display4 = "";
  //display5 = "";
  displayData();
  delay(1000);

  // init the mode select button
  pinMode(BOOT_BUTTON_PIN, INPUT);
  attachInterrupt(BOOT_BUTTON_PIN, bootButtonPressed, RISING);
}

void displayData() {
#ifdef IS_TFT
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setFont(NULL);  // Pass NULL to revert to 'classic' fixed-space bitmap font.
  tft.setCursor(0, 0);
  tft.print(display1);
  tft.setCursor(0, 10);
  tft.print(display2);
  tft.setCursor(0, 20);
  tft.print(display3);
  tft.setCursor(0, 30);
  tft.print(display4);
  tft.setCursor(0, 40);
  tft.print(display5);
  tft.setCursor(0, 50);
  tft.print(display6);
  tft.setCursor(0, 60);
  tft.print(display7);
  tft.setCursor(0, 70);
  tft.print(display8);
  tft.setCursor(0, 80);
  tft.print(display9);
  tft.setCursor(0, 90);
  tft.print(display10);
  tft.setCursor(0, 100);
  tft.print(display11);
  tft.setCursor(0, 110);
  tft.print(display12);
  tft.setCursor(0, 120);
  tft.print(display13);
#endif

#ifdef IS_OLED
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(Monospaced_plain_9);
  display.drawString(0, 0, display1);
  display.drawString(0, 12, display2);
  display.drawString(0, 24, display3);
  display.drawString(0, 36, display4);
  display.drawString(0, 48, display5);
  display.display();
#endif
}

void clearDisplayData() {
  display1 = "";
  display2 = "";
  display3 = "";
  display4 = "";
  display5 = "";
  display6 = "";
  display7 = "";
  display8 = "";
  display9 = "";
  display10 = "";
  display11 = "";
  display12 = "";
  display13 = "";
}

void setVextControl(boolean trueIsOn) {
#ifdef IS_VEXT_CONTROL
  if (trueIsOn) {
    pinMode(VEXT_POWER_CONTROL_PIN, OUTPUT);
    digitalWrite(VEXT_POWER_CONTROL_PIN, LOW);
  } else {
    // pulled up, no need to drive it
    pinMode(VEXT_POWER_CONTROL_PIN, INPUT);
  }
#endif
}

// -----------------------------------------------------------------------
// AES cryptography helper

void generateIv() {
  generateIv(rxMacBaseAddress, rxNodeNumber, rxReceiverNumber, rxTxPacketCounter);
}

void generateIv(uint8_t macAddress[], uint8_t nodeAddress, uint8_t receiverAddress, uint32_t txCounter) {
#ifdef AES_CRYPTOGRAPHY_DEBUG
  // printout input data
  Serial.print(F("macAddress:  "));
  serialPrintHexValues(macAddress, 6);
  Serial.print(F("nodeAddress: "));
  serialPrintHexValue(nodeAddress);
  Serial.print(F("recvAddress: "));
  serialPrintHexValue(receiverAddress);
  Serial.print(F("txCounter:   "));
  Serial.println(txCounter);
#endif
  const uint8_t tempSize = 16;
  uint8_t temp[tempSize] = { 0 };
  uint8_t i;
  for (i = 0; i < sizeof(macAddress); i++) {
    temp[i] = macAddress[i];
  }
  temp[7] = nodeAddress;
  temp[8] = receiverAddress;
  // convert uint32_t to uint8_t
  for (i = 0; i < 4; i++) {
    temp[9 + i] = ((txCounter >> i * 8) & 0xFF);
  }

#ifdef AES_CRYPTOGRAPHY_DEBUG
  Serial.print(F("dataForSha:  "));
  serialPrintHexValues(temp, tempSize);
#endif
  // generate the SHA-256
  const uint8_t shaResultSize = 32;
  uint8_t shaResult[shaResultSize];  // this is the full SHA-256 = 32 bytes

  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
  mbedtls_md_starts(&ctx);
  mbedtls_md_update(&ctx, temp, tempSize);
  mbedtls_md_finish(&ctx, shaResult);
  mbedtls_md_free(&ctx);
#ifdef AES_CRYPTOGRAPHY_DEBUG
  Serial.print(F("fullSha-256: "));
  serialPrintHexValues(shaResult, shaResultSize);
#endif
  // this is the shorted SHA-256 = 16 bytes
  const uint8_t shaResultShortSize = 16;
  for (i = 0; i < shaResultShortSize; i++) {
    _Iv[i] = shaResult[i];
  }
#ifdef AES_CRYPTOGRAPHY_DEBUG
  Serial.print(F("shortSha-256:"));
  serialPrintHexValues(_Iv, shaResultShortSize);
#endif
}

// -----------------------------------------------------------------------
// some helper

void serialPrintHexValues(uint8_t data[], const int len) {
  for (int i = 0; i < len; i++) {
    char str[3];
    sprintf(str, "%02X ", (int)data[i]);
    Serial.print(str);
  }
  Serial.println();
}

void serialPrintHexValue(uint8_t data) {
  char str[3];
  sprintf(str, "%02X ", (int)data);
  Serial.print(str);
  Serial.println();
}