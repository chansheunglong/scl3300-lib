/**************************************************************************/
/*
  This is a library for the muRata SCL3300 High Performance 3-axis Inclinometer
  ----> https://www.murata.com/en-global/products/sensor/inclinometer/scl3300

  Version: 1.0.0
  (c) 2020 Chan Sheung Long

  Changelog:
  2020/02/07  Initial Release
*/
/**************************************************************************/
#include <SPI.h>

// Standard SPI requests
#define REQ_READ_ACC_X 0x040000F7
#define REQ_READ_ACC_Y 0x080000FD
#define REQ_READ_ACC_Z 0x0C0000FB
#define REQ_READ_STO 0x100000E9
#define REQ_READ_TEMP 0x140000EF
#define REQ_READ_STATUS 0x180000E5
#define REQ_WRITE_SW_RESET 0xB4002098
#define REQ_WRITE_MODE1 0xB400001F
#define REQ_WRITE_MODE2 0xB4000102
#define REQ_WRITE_MODE3 0xB4000225
#define REQ_WRITE_MODE4 0xB4000338
#define REQ_READ_WHOAMI 0x40000091
#define REQ_ENABLE_ANG 0xB0001F6F
#define REQ_READ_ANG_X 0x240000C7
#define REQ_READ_ANG_Y 0x280000CD
#define REQ_READ_ANG_Z 0x2C0000CB
#define SET_POWER_DOWN_MODE 0xB400046B
#define WAKE_FROM_POWER_DOWN 0xB400001F

// SPI frame field masks
#define OPCODE_FIELD_MASK 0xFC000000
#define RS_FIELD_MASK 0x03000000
#define DATA_FIELD_MASK 0x00FFFF00
#define CRC_FIELD_MASK 0x000000FF

/*
  For debug purposes
*/
//#define SPI_SEND_DEBUG
//#define WHOAMI_DEBUG
//#define ANGLE_DEBUG
//#define DEBUG_SETZERO

class Murata_SCL3300 {
  private:
    uint16_t SPI_SEND(uint32_t REQUEST);
    void ENABLE_ANG_OUTPUT();
    bool SELF_TEST();
    bool WHOAMI();
    uint16_t READ_STATUS_SUMMARY();
    bool SELECT(bool CS_ENABLE);
    int SCL3300_CSB;
    float ANG_X_ZERO, ANG_Y_ZERO, ANG_Z_ZERO;
    //float ARC_X, ARC_Y, ARC_Z;

  public:
    void setCSPin(int pinNum);
    void startup();
    void sleep();
    void wake();
    void mode(int mode);
    bool angle(float &ANG_X, float &ANG_Y, float &ANG_Z);
    bool arc_angle(int32_t &ARC_X, int32_t &ARC_Y, int32_t &ARC_Z);
    float temperature();
    void setZero(float zeroX = 0, float zeroY = 0, float zeroZ = 0);
    void reset();
    float arcsecond_to_degree(int32_t arc_angle) {
      return (arc_angle / 9 * 0.0025);
    }

};

/*
  Set the Chip Select (CS) pin
*/
void Murata_SCL3300::setCSPin(int pinNum) {
  pinMode(pinNum, OUTPUT);
  SCL3300_CSB = pinNum;
  digitalWrite(SCL3300_CSB, HIGH);
}

/*
  SELECT() set the Chip Select (CS) pin of the sensor HIGH or LOW
*/
bool Murata_SCL3300::SELECT(bool CS_ENABLE) {
  if (CS_ENABLE) {
    digitalWrite(SCL3300_CSB, LOW);
    delay(1);
    return true;
  } else {
    digitalWrite(SCL3300_CSB, HIGH);
    delay(1);
    return false;
  }
}

/*
  Sensor first boot routine, to be called when sensor first boot or
  exit from sleep mode.
*/
void Murata_SCL3300::startup() {
  // Power-on

  // Check Connection
  while (!WHOAMI()) {
    delay(2);
  };

  // Inclinometer Mode, Low Noise Mode, 10Hz Filter Rate
  mode(4);

  delay(15);

  // Clear Status Register
  READ_STATUS_SUMMARY();

  // Read Status Register
  READ_STATUS_SUMMARY();

  // Enagle Angle Output
  ENABLE_ANG_OUTPUT();

  // After startup() return user must wait for a certain time to
  // let the internal filter sattle. It takes more than 1 seconds.
}

/*
  Set the sensor to sleep mode for minimum power consumption
*/
void Murata_SCL3300::sleep() {
  SPI_SEND(SET_POWER_DOWN_MODE);
  SPI.end();
}

/*
  Wake up the sensor after sleep mode or perform first boot
*/
void Murata_SCL3300::wake() {
  // SCL3300 uses SPI Mode 0 (CPOL = 0 and CPHA = 0) (P.17 of Datasheet)
  SPI.begin();

  // Power Up (if the sensor is previously in sleep mode)
  delay(15);

  // Do nothing when the sensor first power up
  SPI_SEND(WAKE_FROM_POWER_DOWN);

  // Normal Startup procedure
  startup();
}

/*
  Send and receive data through SPI
*/
uint16_t Murata_SCL3300::SPI_SEND(uint32_t REQUEST) {

  SELECT(true);

  delayMicroseconds(40);

  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

  delayMicroseconds(20);

  uint8_t buf[4];

  buf[0] = SPI.transfer((uint8_t)((REQUEST & 0xFF000000) >> 24));
  buf[1] = SPI.transfer((uint8_t)((REQUEST & 0x00FF0000) >> 16));
  buf[2] = SPI.transfer((uint8_t)((REQUEST & 0x0000FF00) >> 8));
  buf[3] = SPI.transfer((uint8_t)((REQUEST & 0x000000FF)));

#ifdef SPI_SEND_DEBUG
  Serial.print("RW= B");
  Serial.println(buf[0] & B10000000, BIN);

  Serial.print("ADR= B");
  Serial.println(buf[0] & B01111100, BIN);

  Serial.print("RS= B");
  Serial.print(buf[0] & B0000011, BIN);

  switch (buf[0] & B0000011) {
    case B00:
      Serial.println(F(", Startup in progress"));
      break;
    case B01:
      Serial.println(F(", Normal operation"));
      break;
    case B10:
      Serial.println(F(", Self-test running"));
      break;
    case B11:
      Serial.println(F(", Error, Check error flag or CRC of MOSI"));
      break;
    default:
      break;
  }

  Serial.print(F("D = 0x"));
  Serial.print(buf[1], HEX);
  Serial.println(buf[2], HEX);

  Serial.print(F("CSC= 0x"));
  Serial.println(buf[3], HEX);
  Serial.println(F("---------------------"));

  Serial.flush();
#endif

  uint16_t return_data = ((uint16_t)(buf[1] << 8)) + ((uint16_t)(buf[2] << 0));

  delayMicroseconds(20);

  SELECT(false);

  delayMicroseconds(20);

  SPI.endTransaction();

  return return_data;
}

/*
  Set the operating mode of the sensor
*/
void Murata_SCL3300::mode(int tmode) {
  switch (tmode) {
    case 0x01: // MODE 1
      {
        SPI_SEND(REQ_WRITE_MODE1);
        break;
      }
    case 0x02: // MODE 2
      {
        SPI_SEND(REQ_WRITE_MODE2);
        break;
      }
    case 0x03: // MODE 3
      {
        SPI_SEND(REQ_WRITE_MODE3);
        break;
      }
    case 0x04: // MODE 4
      {
        SPI_SEND(REQ_WRITE_MODE4);
        break;
      }
    default:
      break;
  }
}

/*
  Enable digital angle output of the sensor
*/
void Murata_SCL3300::ENABLE_ANG_OUTPUT() {
  SPI_SEND(REQ_ENABLE_ANG);
}

/*
  Perform a self test to validate the result of the sensor
  (Currently not implemented by the sensor)
*/
bool Murata_SCL3300::SELF_TEST() {
  SPI_SEND(REQ_READ_STO);
  // The self test result is currently useless
  // as Murata did not release any
  // information about the return value
  // leave it for future use...
}

/*
  Return status summary
*/
uint16_t Murata_SCL3300::READ_STATUS_SUMMARY() {
  return SPI_SEND(REQ_READ_STATUS);
}

/*
  Read WHOAMI register, used to check connection
*/
bool Murata_SCL3300::WHOAMI() {
  uint16_t data = SPI_SEND(REQ_READ_WHOAMI);
  if (data == 0x00C1) {
    return true;
  } else {
#ifdef WHOAMI_DEBUG
    Serial.print("WHOAMI not match, expected 0xC1, received ");
    Serial.println(data, HEX);
#endif
    return false;
  }
}

/*
  Return the digital angle result
*/
bool Murata_SCL3300::angle(float &ANG_X, float &ANG_Y, float &ANG_Z) {
  SPI_SEND(REQ_READ_ANG_X);
  ANG_X = (float)(((float)((int16_t)SPI_SEND(REQ_READ_ANG_X))) / 16384 *
                  90.0);
  SPI_SEND(REQ_READ_ANG_Y);
  ANG_Y = (float)(((float)((int16_t)SPI_SEND(REQ_READ_ANG_Y))) / 16384 *
                  90.0);
  SPI_SEND(REQ_READ_ANG_Z);
  ANG_Z = (float)(((float)((int16_t)SPI_SEND(REQ_READ_ANG_Z))) / 16384 *
                  90.0);

#ifdef ANGLE_DEBUG
  Serial.print(ANG_X);
  Serial.write(0x09);
  Serial.print(ANG_Y);
  Serial.write(0x09);
  Serial.println(ANG_Z);
#endif

  return true;
}

/*
  Return the digital angle result in arc seconds
  (Advantage is basically because it is stored as interger)

  arc seconds = sensor value * 10125/512
*/
bool Murata_SCL3300::arc_angle(int32_t &ARC_X, int32_t &ARC_Y, int32_t &ARC_Z) {

  SPI_SEND(REQ_READ_ANG_X);
  ARC_X = ((int32_t)(int16_t)SPI_SEND(REQ_READ_ANG_X)) * (int32_t)10125 / (int32_t)512;
  SPI_SEND(REQ_READ_ANG_Y);
  ARC_Y = ((int32_t)(int16_t)SPI_SEND(REQ_READ_ANG_Y)) * (int32_t)10125 / (int32_t)512;
  SPI_SEND(REQ_READ_ANG_Z);
  ARC_Z = ((int32_t)(int16_t)SPI_SEND(REQ_READ_ANG_Z)) * (int32_t)10125 / (int32_t)512;

#ifdef ANGLE_DEBUG
  Serial.print(ARC_X);
  Serial.write(0x09);
  Serial.print(ARC_Y);
  Serial.write(0x09);
  Serial.println(ARC_Z);
#endif

  return true;
}

/*
  Return the sensor temperature
*/
float Murata_SCL3300::temperature() {
  SPI_SEND(REQ_READ_TEMP);
  return (-273.0 + (((float)((int16_t)SPI_SEND(REQ_READ_TEMP))) / 18.9));
}

/*
  As the sensor is internally calibrated, this function should only be used
  if the current surface is to be set as the refrence, for global absolute
  readings, uses the result from angle() directly without calling setZero()
*/
#define SETZERO_AVERAGEING_COUNT 10
#define SETZERO_AVERAGEING_INTERVAL 100
void Murata_SCL3300::setZero(float zeroX = 0, float zeroY = 0, float zeroZ = 0) {
  if ((zeroX != 0) || (zeroY != 0) || (zeroZ != 0)) {
    // User provide set zero result
    ANG_X_ZERO = zeroX;
    ANG_Y_ZERO = zeroY;
    ANG_Z_ZERO = zeroZ;
  } else {
    // use the internal set zero method
    // by taking average
    float x, y, z;
    float x_sum = 0, y_sum = 0, z_sum = 0;
    for (int i = 0; i < SETZERO_AVERAGEING_COUNT; i++) {
      angle(x, y, z);
      x_sum += x;
      y_sum += y;
      z_sum += z;
      delay(SETZERO_AVERAGEING_INTERVAL);
    }
    ANG_X_ZERO = x_sum / SETZERO_AVERAGEING_COUNT;
    ANG_Y_ZERO = y_sum / SETZERO_AVERAGEING_COUNT;
    ANG_Z_ZERO = z_sum / SETZERO_AVERAGEING_COUNT;
  }
}

/*
  reset() removes the effect of setZero or setZeroInclination()
  and revert the angle() output to the factory calibrated data.
  And trigger a software reset of the sensor.
*/
void Murata_SCL3300::reset() {
  ANG_X_ZERO = 0;
  ANG_Y_ZERO = 0;
  ANG_Z_ZERO = 0;

  // Trigger software reset of the sensor
  SPI_SEND(REQ_WRITE_SW_RESET);

  // Sensor first boot routine
  wake();
}