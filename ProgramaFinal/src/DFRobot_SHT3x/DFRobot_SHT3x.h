// Copyright (c) 2010 DFRobot Co.Ltd. Licencia MIT incluida en LICENSE.

#ifndef DFROBOT_SHT3X_H
#define DFROBOT_SHT3X_H
#include "Arduino.h"
#include <Wire.h>

#ifdef ENABLE_DBG
#define DBG(...)                                                                                               \
  {                                                                                                            \
    Serial.print("[");                                                                                         \
    Serial.print(__FUNCTION__);                                                                                \
    Serial.print("(): ");                                                                                      \
    Serial.print(__LINE__);                                                                                    \
    Serial.print(" ] ");                                                                                       \
    Serial.println(__VA_ARGS__);                                                                               \
  }
#else
#define DBG(...)
#endif

#define SHT3X_CMD_GETDATA_POLLING_H (0x2400)
#define SHT3X_CMD_GETDATA_POLLING_M (0x240B)
#define SHT3X_CMD_GETDATA_POLLING_L (0x2416)

#define SHT3X_CMD_READ_SERIAL_NUMBER (0x3780)
#define SHT3X_CMD_GETDATA_H_CLOCKENBLED (0x2C06)
#define SHT3X_CMD_GETDATA_M_CLOCKENBLED (0x2C0D)
#define SHT3X_CMD_GETDATA_L_CLOCKENBLED (0x2C10)
#define SHT3X_CMD_SETMODE_H_FREQUENCY_HALF_HZ (0x2032)
#define SHT3X_CMD_SETMODE_M_FREQUENCY_HALF_HZ (0x2024)
#define SHT3X_CMD_SETMODE_L_FREQUENCY_HALF_HZ (0x202F)
#define SHT3X_CMD_SETMODE_H_FREQUENCY_1_HZ (0x2130)
#define SHT3X_CMD_SETMODE_M_FREQUENCY_1_HZ (0x2126)
#define SHT3X_CMD_SETMODE_L_FREQUENCY_1_HZ (0x212D)
#define SHT3X_CMD_SETMODE_H_FREQUENCY_2_HZ (0x2236)
#define SHT3X_CMD_SETMODE_M_FREQUENCY_2_HZ (0x2220)
#define SHT3X_CMD_SETMODE_L_FREQUENCY_2_HZ (0x222B)
#define SHT3X_CMD_SETMODE_H_FREQUENCY_4_HZ (0x2334)
#define SHT3X_CMD_SETMODE_M_FREQUENCY_4_HZ (0x2322)
#define SHT3X_CMD_SETMODE_L_FREQUENCY_4_HZ (0x2329)
#define SHT3X_CMD_SETMODE_H_FREQUENCY_10_HZ (0x2737)
#define SHT3X_CMD_SETMODE_M_FREQUENCY_10_HZ (0x2721)
#define SHT3X_CMD_SETMODE_L_FREQUENCY_10_HZ (0x272A)
#define SHT3X_CMD_GETDATA (0xE000)

#define SHT3X_CMD_STOP_PERIODIC_ACQUISITION_MODE (0x3093)
#define SHT3X_CMD_SOFT_RESET (0x30A2)
#define SHT3X_CMD_HEATER_ENABLE (0x306D)
#define SHT3X_CMD_HEATER_DISABLE (0x3066)
#define SHT3X_CMD_READ_STATUS_REG (0xF32D)
#define SHT3X_CMD_CLEAR_STATUS_REG (0x3041)

#define SHT3X_CMD_READ_HIGH_ALERT_LIMIT_SET (0xE11F)
#define SHT3X_CMD_READ_HIGH_ALERT_LIMIT_CLEAR (0xE114)
#define SHT3X_CMD_READ_LOW_ALERT_LIMIT_CLEAR (0xE109)
#define SHT3X_CMD_READ_LOW_ALERT_LIMIT_SET (0xE102)
#define SHT3X_CMD_WRITE_HIGH_ALERT_LIMIT_SET (0x611D)
#define SHT3X_CMD_WRITE_HIGH_ALERT_LIMIT_CLEAR (0x6116)
#define SHT3X_CMD_WRITE_LOW_ALERT_LIMIT_CLEAR (0x610B)
#define SHT3X_CMD_WRITE_LOW_ALERT_LIMIT_SET (0x6100)
class DFRobot_SHT3x {
public:
#define ERR_OK 0
#define ERR_DATA_BUS -1
#define ERR_IC_VERSION -2

  typedef struct {
    uint16_t writeDataChecksumStatus : 1;
    uint16_t commandStatus : 1;
    uint16_t reserved0 : 2;
    uint16_t systemResetDeteced : 1;
    uint16_t reserved1 : 5;
    uint16_t temperatureAlert : 1;
    uint16_t humidityAlert : 1;
    uint16_t reserved2 : 1;
    uint16_t heaterStaus : 1;
    uint16_t reserved3 : 1;
    uint16_t alertPendingStatus : 1;
  } __attribute__((packed)) sStatusRegister_t;

  typedef enum {
    ePeriodic,
    eOneShot,
  } eMode_t;

  typedef enum {
    eRepeatability_High = 0,
    eRepeatability_Medium = 1,
    eRepeatability_Low = 2,
  } eRepeatability_t;

  typedef enum {
    eMeasureFreq_Hz5 = 0,
    eMeasureFreq_1Hz = 1,
    eMeasureFreq_2Hz = 2,
    eMeasureFreq_4Hz = 3,
    eMeasureFreq_10Hz = 4,
  } eMeasureFrequency_t;

  typedef struct {
    float TemperatureC;
    float Humidity;
    float TemperatureF;
    int ERR;
  } sRHAndTemp_t;

  typedef struct {
    float highSet;
    float highClear;
    float lowSet;
    float lowClear;
  } sLimitData_t;

public:
  DFRobot_SHT3x(TwoWire *pWire = &Wire, uint8_t address = 0x45, uint8_t RST = 4);

  uint32_t readSerialNumber();

  int begin();

  bool softReset();

  bool pinReset();

  sRHAndTemp_t readTemperatureAndHumidity(eRepeatability_t repeatability);

  float getTemperatureC();

  float getTemperatureF();

  float getHumidityRH();

  bool startPeriodicMode(eMeasureFrequency_t measureFreq, eRepeatability_t repeatability = eRepeatability_High);

  sRHAndTemp_t readTemperatureAndHumidity();

  bool stopPeriodicMode();

  bool heaterEnable();

  bool heaterDisable();

  void clearStatusRegister();

  bool readAlertState();

  uint8_t environmentState();

  uint8_t setTemperatureLimitC(float highset, float highclear, float lowset, float lowclear);

  uint8_t setTemperatureLimitF(float highset, float highclear, float lowset, float lowclear);

  uint8_t setHumidityLimitRH(float highset, float highclear, float lowset, float lowclear);

  bool measureTemperatureLimitC();

  float getTemperatureHighSetC();

  float getTemperatureHighClearC();

  float getTemperatureLowClearC();

  float getTemperatureLowSetC();

  bool measureTemperatureLimitF();

  float getTemperatureHighSetF();

  float getTemperatureHighClearF();

  float getTemperatureLowClearF();

  float getTemperatureLowSetF();

  bool measureHumidityLimitRH();

  float getHumidityHighSetRH();

  float getHumidityHighClearRH();

  float getHumidityLowClearRH();

  float getHumidityLowSetRH();

private:
  void writeCommand(uint16_t cmd, size_t size);

  sStatusRegister_t readStatusRegister();

  void writeLimitData(uint16_t cmd, uint16_t limitData);

  uint8_t readLimitData(uint16_t cmd, uint16_t *pBuf);

  uint8_t readData(void *pBuf, size_t size);

  uint8_t checkCrc(uint8_t data[]);

  float convertTemperature(uint8_t rawTemperature[]);

  float convertHumidity(uint8_t rawHumidity[]);

  uint16_t convertRawTemperature(float value);

  uint16_t convertRawHumidity(float value);

  float convertTempLimitData(uint16_t limit[]);

  float convertHumidityLimitData(uint16_t limit[]);

  void write(const void *pBuf, size_t size);

private:
  sLimitData_t limitData;
  sRHAndTemp_t tempRH;
  TwoWire *_pWire;
  eMode_t measurementMode;
  uint8_t _address;
  uint8_t _RST;
  float tempHighSet;
  float tempLowSet;
};
#endif
