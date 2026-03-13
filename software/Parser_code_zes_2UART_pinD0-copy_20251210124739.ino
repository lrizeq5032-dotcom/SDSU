// ============================================================
// ZES Mock Telemetry Parser  (using extra UART on SERCOM3: D0/D6)
// ------------------------------------------------------------
// Expects frames on mySerial (SERCOM3, pins D0=RX, D6=TX) with format:
//
//   [SYNC1 SYNC2 VER TYPE LEN_L LEN_H PAYLOAD... CRC_L CRC_H]
//
// Payload layout (514 bytes, little-endian):
//   seq   : uint16_t (2 bytes)
//   ticks : uint32_t (4 bytes)
//   magX  : int16_t  (2 bytes)
//   magY  : int16_t  (2 bytes)
//   magZ  : int16_t  (2 bytes)
//   status: uint8_t  (1 byte, present but currently ignored)
//
// CRC is CRC-16/CCITT-FALSE over header+payload:
//   poly = 0x1021, init = 0xFFFF, no xorout, no reflection.
// ============================================================

#include <Arduino.h>
#include "wiring_private.h"   // needed for SERCOM on SAMD21

// -------------------- Extra UART on SERCOM3 (D0/D6) ---------
Uart mySerial(&sercom3, 0, 6, SERCOM_RX_PAD_0, UART_TX_PAD_2); // RX=D0, TX=D6

void SERCOM3_Handler() {
  mySerial.IrqHandler();
}

// -------------------- Protocol constants --------------------
static const uint8_t  SYNC1       = 0xA5;
static const uint8_t  SYNC2       = 0x5A;
static const uint8_t  EXPECT_VER  = 0x01;
static const uint8_t  EXPECT_TYPE = 0x01;

static const size_t   HEADER_SIZE  = 6;    // SYNC1, SYNC2, VER, TYPE, LEN_L, LEN_H
static const size_t   CRC_SIZE     = 2;
static const uint16_t MAX_PAYLOAD  = 1024;


static const uint16_t PAYLOAD_LEN    = 514;
static const uint32_t STATS_INTERVAL = 10;  // print stats every N good frames

// -------------------- Parser state machine ------------------
enum ParserState {
  FIND_SYNC1,
  FIND_SYNC2,
  READ_HEADER,
  READ_PAYLOAD,
  READ_CRC
};

static ParserState state = FIND_SYNC1;

// -------------------- Buffers & indices ---------------------
static uint8_t  header[HEADER_SIZE];
static size_t   hidx = 0;

static uint16_t payLen = 0;
static uint8_t  payload[MAX_PAYLOAD];
static size_t   pidx = 0;

static uint8_t  crcBytes[CRC_SIZE];
static size_t   cidx = 0;

// -------------------- Telemetry counters --------------------
static uint32_t totalFrames = 0;
static uint32_t goodFrames  = 0;
static uint32_t crcFailures = 0;
static uint32_t lenErrors   = 0;
static uint32_t syncResets  = 0;

// -------------------- CRC-16/CCITT-FALSE --------------------
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

// -------------------- Utility helpers -----------------------
void resetParser() {
  state = FIND_SYNC1;
  hidx  = 0;
  pidx  = 0;
  cidx  = 0;
  syncResets++;
}

void printStats() {
  Serial.println(F("==== Parser Stats ===="));
  Serial.print(F("Total frames : ")); Serial.println(totalFrames);
  Serial.print(F("Good frames  : ")); Serial.println(goodFrames);
  Serial.print(F("CRC failures : ")); Serial.println(crcFailures);
  Serial.print(F("LEN errors   : ")); Serial.println(lenErrors);
  Serial.print(F("Sync resets  : ")); Serial.println(syncResets);
  Serial.println(F("======================"));
}

// Decode the 514-byte payload (little-endian) and print (status ignored)
void decodeAndPrintPayload(const uint8_t* pl, uint16_t len) {
  if (len != PAYLOAD_LEN) {
    Serial.print(F("WARN: unexpected payload length "));
    Serial.print(len);
    Serial.print(F(", expected "));
    Serial.println(PAYLOAD_LEN);
    return;
  }

  // Little-endian decode:
  uint16_t seq =
    (uint16_t)pl[0]        |
    ((uint16_t)pl[1] << 8);

  uint32_t ticks =
    (uint32_t)pl[2]        |
    ((uint32_t)pl[3] << 8) |
    ((uint32_t)pl[4] << 16)|
    ((uint32_t)pl[5] << 24);

  int16_t magX =
    (int16_t)((uint16_t)pl[6] | ((uint16_t)pl[7] << 8));
  int16_t magY =
    (int16_t)((uint16_t)pl[8] | ((uint16_t)pl[9] << 8));
  int16_t magZ =
    (int16_t)((uint16_t)pl[10] | ((uint16_t)pl[11] << 8));

  Serial.println(F("---- Mock Telemetry ----"));
  Serial.print(F("SEQ   : ")); Serial.println(seq);
  Serial.print(F("TICKS : ")); Serial.println(ticks);

  Serial.print(F("MAG   : X="));
  Serial.print(magX);
  Serial.print(F("  Y="));
  Serial.print(magY);
  Serial.print(F("  Z="));
  Serial.println(magZ);

  Serial.println(F("------------------------"));
}

// -------------------- Arduino setup -------------------------
void setup() {
  Serial.begin(115200);      // USB to PC

  // Extra UART on SERCOM3 (D0/D6)
  mySerial.begin(115200);    // Match your simulator baud rate
  pinPeripheral(0, PIO_SERCOM);      // D0 as RX (SERCOM3)
  pinPeripheral(6, PIO_SERCOM_ALT);  // D6 as TX (SERCOM3)

  // Optional small wait for Serial on native USB boards
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 2000) {
    ; // wait up to 2 seconds
  }

  Serial.println(F("ZES Mock Telemetry Parser (SERCOM3 D0/D6) @115200"));
  Serial.println(F("Expecting frames: A5 5A 01 01 0D 00 [payload(514)] CRC16"));
  resetParser();
}

// -------------------- Arduino loop --------------------------
void loop() {
  // Process all available bytes from mySerial (extra UART)
  while (mySerial.available() > 0) {
    uint8_t b = (uint8_t)mySerial.read();

    switch (state) {
      case FIND_SYNC1:
        if (b == SYNC1) {
          header[0] = b;
          hidx = 1;
          state = FIND_SYNC2;
        }
        break;

      case FIND_SYNC2:
        if (b == SYNC2) {
          header[1] = b;
          hidx = 2;
          state = READ_HEADER;
        } else {
          // false alarm, restart sync search
          state = FIND_SYNC1;
        }
        break;

      case READ_HEADER:
        header[hidx++] = b;
        if (hidx == HEADER_SIZE) {
          uint8_t ver   = header[2];
          uint8_t type_ = header[3];

          // LEN is little-endian (LSB first), payload bytes only
          payLen = (uint16_t)(header[4] | (header[5] << 8));

          if (payLen > MAX_PAYLOAD) {
            Serial.print(F("ERR: payload too large: "));
            Serial.println(payLen);
            lenErrors++;
            resetParser();
            break;
          }

          if (payLen != PAYLOAD_LEN) {
            Serial.print(F("WARN: unexpected payload length "));
            Serial.print(payLen);
            Serial.print(F(", expected "));
            Serial.println(PAYLOAD_LEN);
          }

          if (ver != EXPECT_VER || type_ != EXPECT_TYPE) {
            Serial.print(F("WARN: unexpected VER/TYPE: VER="));
            Serial.print(ver);
            Serial.print(F(" TYPE="));
            Serial.println(type_);
          }

          pidx  = 0;
          state = READ_PAYLOAD;
        }
        break;

      case READ_PAYLOAD:
        if (pidx < MAX_PAYLOAD) {
          payload[pidx++] = b;
        }
        if (pidx >= payLen) {
          cidx  = 0;
          state = READ_CRC;
        }
        break;

      case READ_CRC:
        crcBytes[cidx++] = b;
        if (cidx == CRC_SIZE) {
          // We now have: header[0..5], payload[0..payLen-1], crcBytes[0..1]
          totalFrames++;

          // Build header+payload buffer for CRC calculation
          const size_t noCrcLen = HEADER_SIZE + payLen;
          static uint8_t frameBuf[HEADER_SIZE + MAX_PAYLOAD];
          memcpy(frameBuf, header, HEADER_SIZE);
          memcpy(frameBuf + HEADER_SIZE, payload, payLen);

          uint16_t calcCrc = crc16_ccitt(frameBuf, noCrcLen);
          uint16_t recvCrc = (uint16_t)(crcBytes[0] | (crcBytes[1] << 8)); // little-endian

          if (calcCrc == recvCrc) {
            goodFrames++;

            Serial.print(F("FRAME #"));
            Serial.print(totalFrames - 1);
            Serial.print(F(" OK  len="));
            Serial.print(payLen);
            Serial.print(F("  CRC=0x"));
            Serial.println(recvCrc, HEX);

            decodeAndPrintPayload(payload, payLen);

            if ((goodFrames % STATS_INTERVAL) == 0) {
              printStats();
            }

          } else {
            crcFailures++;
            Serial.print(F("ERR: CRC mismatch (calc=0x"));
            Serial.print(calcCrc, HEX);
            Serial.print(F(", recv=0x"));
            Serial.print(recvCrc, HEX);
            Serial.println(F(")"));
          }

          resetParser();
        }
        break;
    }
  }

  // Optional small delay
  // delay(1);
}