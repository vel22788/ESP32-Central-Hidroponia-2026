// Copyright (c) 2010 DFRobot Co.Ltd. Licencia MIT incluida en LICENSE.

#include "DFRobot_SHT3x.h"
#include "../Configuracion.h"
#include "math.h"
// Asocia el sensor con el bus I²C y su dirección y prepara el pin de reinicio si se indicó uno.
DFRobot_SHT3x::DFRobot_SHT3x(TwoWire *pWire, uint8_t address, uint8_t RST) {
  _pWire = pWire;
  _address = address;
  _RST = RST;
  measurementMode = eOneShot;
  if (_RST != 0xFF) {
    pinMode(_RST, OUTPUT);
    digitalWrite(_RST, HIGH);
  }
}

// Comprueba que el sensor responda con un número de serie válido.
int DFRobot_SHT3x::begin() {
  if (readSerialNumber() == 0) {
    DBG("bus data access error");
    return ERR_DATA_BUS;
  }
  return ERR_OK;
}

// Lee el número de serie de 32 bits y verifica el CRC de cada mitad antes de combinarlo.
uint32_t DFRobot_SHT3x::readSerialNumber() {
  uint32_t result = 0;
  uint8_t serialNumber1[3];
  uint8_t serialNumber2[3];
  uint8_t rawData[6];
  writeCommand(SHT3X_CMD_READ_SERIAL_NUMBER, 2);
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  if (readData(rawData, 6) != 6)
    return 0;
  memcpy(serialNumber1, rawData, 3);
  memcpy(serialNumber2, rawData + 3, 3);
  if ((checkCrc(serialNumber1) == serialNumber1[2]) && (checkCrc(serialNumber2) == serialNumber2[2])) {
    result = serialNumber1[0];
    result = (result << 8) | serialNumber1[1];
    result = (result << 8) | serialNumber2[0];
    result = (result << 8) | serialNumber2[1];
  }
  return result;
}

// Solicita un reinicio por software y comprueba el estado del comando en el sensor.
bool DFRobot_SHT3x::softReset() {
  sStatusRegister_t registerRaw;
  writeCommand(SHT3X_CMD_SOFT_RESET, 2);
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  registerRaw = readStatusRegister();
  if (registerRaw.commandStatus == 0)
    return true;
  else
    return false;
}

// Reinicia el sensor por su pin físico, si está conectado, y comprueba la bandera de reinicio.
bool DFRobot_SHT3x::pinReset() {
  if (_RST == 0xFF)
    return false;
  sStatusRegister_t registerRaw;
  clearStatusRegister();
  digitalWrite(_RST, LOW);
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  digitalWrite(_RST, HIGH);

  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  registerRaw = readStatusRegister();
  if (registerRaw.systemResetDeteced == 1)
    return true;
  else
    return false;
}

// Termina la adquisición periódica y verifica que el sensor aceptó la orden.
bool DFRobot_SHT3x::stopPeriodicMode() {
  sStatusRegister_t registerRaw;
  measurementMode = eOneShot;
  writeCommand(SHT3X_CMD_STOP_PERIODIC_ACQUISITION_MODE, 2);
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  registerRaw = readStatusRegister();
  if (registerRaw.commandStatus == 0)
    return true;
  else
    return false;
}

// Enciende el calentador interno y comprueba su estado.
bool DFRobot_SHT3x::heaterEnable() {
  sStatusRegister_t registerRaw;
  writeCommand(SHT3X_CMD_HEATER_ENABLE, 2);
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  registerRaw = readStatusRegister();
  if (registerRaw.heaterStaus == 1)
    return true;
  else
    return false;
}

// Apaga el calentador interno y comprueba su estado.
bool DFRobot_SHT3x::heaterDisable() {
  sStatusRegister_t registerRaw;
  writeCommand(SHT3X_CMD_HEATER_DISABLE, 2);
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  registerRaw = readStatusRegister();
  if (registerRaw.heaterStaus == 0)
    return true;
  else
    return false;
}

// Envía al sensor la orden de limpiar sus banderas de estado.
void DFRobot_SHT3x::clearStatusRegister() {
  writeCommand(SHT3X_CMD_CLEAR_STATUS_REG, 2);
  delay(Configuracion::SHT_RESET_MS);
}

// Consulta si el sensor tiene activa una alerta de temperatura o de humedad.
bool DFRobot_SHT3x::readAlertState() {
  sStatusRegister_t registerRaw;
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  registerRaw = readStatusRegister();
  if (registerRaw.humidityAlert == 1 || registerRaw.temperatureAlert == 1) {
    return true;
  } else {
    return false;
  }
}

// Toma una muestra con la repetibilidad solicitada y entrega temperatura y humedad tras validar ambas palabras por CRC.
DFRobot_SHT3x::sRHAndTemp_t DFRobot_SHT3x::readTemperatureAndHumidity(eRepeatability_t repeatability) {
  uint8_t rawData[6];
  uint8_t rawTemperature[3];
  uint8_t rawHumidity[3];
  tempRH.ERR = 0;
  switch (repeatability) {
  case eRepeatability_High:
    writeCommand(SHT3X_CMD_GETDATA_POLLING_H, 2);
    break;
  case eRepeatability_Medium:
    writeCommand(SHT3X_CMD_GETDATA_POLLING_M, 2);
    break;
  case eRepeatability_Low:
    writeCommand(SHT3X_CMD_GETDATA_POLLING_L, 2);
    break;
  }
  delay(Configuracion::SHT_CONVERSION_MS);
  if (readData(rawData, 6) != 6) {
    tempRH.ERR = -1;
    return tempRH;
  }
  memcpy(rawTemperature, rawData, 3);
  memcpy(rawHumidity, rawData + 3, 3);
  if ((checkCrc(rawTemperature) != rawTemperature[2]) || (checkCrc(rawHumidity) != rawHumidity[2])) {
    tempRH.ERR = -1;
    return tempRH;
  }
  tempRH.TemperatureC = convertTemperature(rawTemperature);
  tempRH.TemperatureF = 1.8 * tempRH.TemperatureC + 32;
  tempRH.Humidity = convertHumidity(rawHumidity);
  return tempRH;
}

// Obtiene la temperatura del aire en grados Celsius usando el modo de medición seleccionado.
float DFRobot_SHT3x::getTemperatureC() {
  if (measurementMode == eOneShot) {
    readTemperatureAndHumidity(eRepeatability_High);
  } else {
    readTemperatureAndHumidity();
  }
  return tempRH.TemperatureC;
}

// Obtiene la temperatura del aire en grados Fahrenheit usando el modo de medición seleccionado.
float DFRobot_SHT3x::getTemperatureF() {
  if (measurementMode == eOneShot) {
    readTemperatureAndHumidity(eRepeatability_High);
  } else {
    readTemperatureAndHumidity();
  }
  return tempRH.TemperatureF;
}

// Obtiene la humedad relativa usando el modo de medición seleccionado.
float DFRobot_SHT3x::getHumidityRH() {
  if (measurementMode == eOneShot) {
    readTemperatureAndHumidity(eRepeatability_High);
  } else {
    readTemperatureAndHumidity();
  }
  return tempRH.Humidity;
}

// Inicia el muestreo periódico con la frecuencia y repetibilidad pedidas y comprueba la aceptación de la orden.
bool DFRobot_SHT3x::startPeriodicMode(eMeasureFrequency_t measureFreq, eRepeatability_t repeatability) {
  const uint16_t cmd[5][3] = {{SHT3X_CMD_SETMODE_H_FREQUENCY_HALF_HZ, SHT3X_CMD_SETMODE_M_FREQUENCY_HALF_HZ,
                               SHT3X_CMD_SETMODE_L_FREQUENCY_HALF_HZ},
                              {SHT3X_CMD_SETMODE_H_FREQUENCY_1_HZ, SHT3X_CMD_SETMODE_M_FREQUENCY_1_HZ,
                               SHT3X_CMD_SETMODE_L_FREQUENCY_1_HZ},
                              {SHT3X_CMD_SETMODE_H_FREQUENCY_2_HZ, SHT3X_CMD_SETMODE_M_FREQUENCY_2_HZ,
                               SHT3X_CMD_SETMODE_L_FREQUENCY_2_HZ},
                              {SHT3X_CMD_SETMODE_H_FREQUENCY_4_HZ, SHT3X_CMD_SETMODE_M_FREQUENCY_4_HZ,
                               SHT3X_CMD_SETMODE_L_FREQUENCY_4_HZ},
                              {SHT3X_CMD_SETMODE_H_FREQUENCY_10_HZ, SHT3X_CMD_SETMODE_M_FREQUENCY_10_HZ,
                               SHT3X_CMD_SETMODE_L_FREQUENCY_10_HZ}};
  sStatusRegister_t registerRaw;
  measurementMode = ePeriodic;
  writeCommand(cmd[measureFreq][repeatability], 2);
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  registerRaw = readStatusRegister();
  if (registerRaw.commandStatus == 0)
    return true;
  else
    return false;
}

// Lee la palabra de estado del sensor y reintenta hasta diez veces al comprobar su CRC.
DFRobot_SHT3x::sStatusRegister_t DFRobot_SHT3x::readStatusRegister() {
  uint8_t register1[3];
  uint16_t data;
  sStatusRegister_t registerRaw;
  uint8_t retry = 10;
  while (retry--) {
    writeCommand(SHT3X_CMD_READ_STATUS_REG, 2);
    delay(Configuracion::SHT_PAUSA_COMANDO_MS);
    readData(register1, 3);
    if (checkCrc(register1) == register1[2]) {
      break;
    }
  }
  data = (register1[0] << 8) | register1[1];
  memcpy(&registerRaw, &data, 2);
  return registerRaw;
}

// Clasifica la condición ambiental según las alertas de temperatura y humedad y los límites del sensor.
uint8_t DFRobot_SHT3x::environmentState() {
  sStatusRegister_t registerRaw;
  float rhHighSet;
  float rhLowSet;
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  registerRaw = readStatusRegister();
  sRHAndTemp_t data = readTemperatureAndHumidity();
  if (measureTemperatureLimitC()) {
    tempHighSet = getTemperatureHighSetC();
    tempLowSet = getTemperatureLowSetC();
  }
  if (measureHumidityLimitRH()) {
    rhHighSet = getHumidityHighSetRH();
    rhLowSet = getHumidityLowSetRH();
  }
  if (registerRaw.humidityAlert == 1 && registerRaw.temperatureAlert == 0) {
    if (data.Humidity > rhHighSet)
      return 2;
    else if (data.Humidity < rhLowSet)
      return 1;
  } else if (registerRaw.humidityAlert == 0 && registerRaw.temperatureAlert == 1) {
    if (data.TemperatureC > tempHighSet)
      return 20;
    else if (data.TemperatureC < tempLowSet)
      return 10;
  } else if (registerRaw.humidityAlert == 1 && registerRaw.temperatureAlert == 1) {
    if (data.TemperatureC < tempLowSet && data.Humidity < rhLowSet)
      return 11;
    else if (data.TemperatureC > tempHighSet && data.Humidity > rhHighSet)
      return 22;
    else if (data.TemperatureC > tempHighSet && data.Humidity < rhLowSet)
      return 21;
    else if (data.TemperatureC < tempLowSet && data.Humidity > rhHighSet)
      return 12;
  }

  return 0;
}

// Recoge una muestra del modo periódico, comprueba sus dos CRC y convierte los datos a unidades físicas.
DFRobot_SHT3x::sRHAndTemp_t DFRobot_SHT3x::readTemperatureAndHumidity() {
  uint8_t rawData[6];
  uint8_t rawTemperature[3];
  uint8_t rawHumidity[3];
  tempRH.ERR = 0;
  writeCommand(SHT3X_CMD_GETDATA, 2);
  readData(rawData, 6);
  memcpy(rawTemperature, rawData, 3);
  memcpy(rawHumidity, rawData + 3, 3);
  if ((checkCrc(rawTemperature) != rawTemperature[2]) || (checkCrc(rawHumidity) != rawHumidity[2])) {
    tempRH.ERR = -1;
    return tempRH;
  }
  tempRH.TemperatureC = convertTemperature(rawTemperature);
  tempRH.Humidity = convertHumidity(rawHumidity);
  tempRH.TemperatureF = 1.8 * tempRH.TemperatureC + 32;
  return tempRH;
}

// Programa los cuatro umbrales de temperatura en Celsius y conserva los bits de humedad de cada registro.
uint8_t DFRobot_SHT3x::setTemperatureLimitC(float highset, float highclear, float lowset, float lowclear) {
  uint16_t _highset, _highclear, _lowclear, _lowset, limit[1];

  if (highset > highclear && highclear > lowclear && lowclear > lowset) {
    _highset = convertRawTemperature(highset);
    if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_SET, limit) != 0) {
      return 1;
    }
    _highset = (_highset >> 7) | (limit[0] & 0xfe00);
    writeLimitData(SHT3X_CMD_WRITE_HIGH_ALERT_LIMIT_SET, _highset);
    _highclear = convertRawTemperature(highclear);
    if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_CLEAR, limit) != 0) {
      return 1;
    }
    _highclear = (_highclear >> 7) | (limit[0] & 0xfe00);
    writeLimitData(SHT3X_CMD_WRITE_HIGH_ALERT_LIMIT_CLEAR, _highclear);
    _lowclear = convertRawTemperature(lowclear);
    if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_CLEAR, limit) != 0) {
      return 1;
    }
    _lowclear = (_lowclear >> 7) | (limit[0] & 0xfe00);
    writeLimitData(SHT3X_CMD_WRITE_LOW_ALERT_LIMIT_CLEAR, _lowclear);
    _lowset = convertRawTemperature(lowset);
    if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_SET, limit) != 0) {
      return 1;
    }
    _lowset = (_lowset >> 7) | (limit[0] & 0xfe00);
    writeLimitData(SHT3X_CMD_WRITE_LOW_ALERT_LIMIT_SET, _lowset);
  } else {
    return 1;
  }

  return 0;
}

// Convierte a Celsius los umbrales recibidos en Fahrenheit y delega su programación.
uint8_t DFRobot_SHT3x::setTemperatureLimitF(float highset, float highclear, float lowset, float lowclear) {
  float _highset, _highclear, _lowclear, _lowset;
  _highset = (highset - 32.0) * 5.0 / 9.0;
  _highclear = (highclear - 32.0) * 5.0 / 9.0;
  _lowclear = (lowclear - 32.0) * 5.0 / 9.0;
  _lowset = (lowset - 32.0) * 5.0 / 9.0;
  if (setTemperatureLimitC(_highset, _highclear, _lowset, _lowclear) == 0) {
    return 0;
  }
  return 1;
}

// Programa los cuatro umbrales de humedad relativa y conserva los bits de temperatura de cada registro.
uint8_t DFRobot_SHT3x::setHumidityLimitRH(float highset, float highclear, float lowset, float lowclear) {
  uint16_t _highset, _highclear, _lowclear, _lowset, limit[1];
  if (highset > highclear && highclear > lowclear && lowclear > lowset) {
    _highset = convertRawHumidity(highset);
    if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_SET, limit) != 0) {
      return 1;
    }
    _highset = (_highset & 0xFE00) | (limit[0] & 0x1FF);
    writeLimitData(SHT3X_CMD_WRITE_HIGH_ALERT_LIMIT_SET, _highset);

    _highclear = convertRawHumidity(highclear);
    if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_CLEAR, limit) != 0) {
      return 1;
    }
    _highclear = (_highclear & 0xFE00) | (limit[0] & 0x1FF);
    writeLimitData(SHT3X_CMD_WRITE_HIGH_ALERT_LIMIT_CLEAR, _highclear);
    _lowclear = convertRawHumidity(lowclear);
    if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_CLEAR, limit) != 0) {
      return 1;
    }
    _lowclear = (_lowclear & 0xFE00) | (limit[0] & 0x1FF);
    writeLimitData(SHT3X_CMD_WRITE_LOW_ALERT_LIMIT_CLEAR, _lowclear);
    _lowset = convertRawHumidity(lowset);
    if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_SET, limit) != 0) {
      return 1;
    }
    _lowset = (_lowset & 0xFE00) | (limit[0] & 0x1FF);
    writeLimitData(SHT3X_CMD_WRITE_LOW_ALERT_LIMIT_SET, _lowset);
  } else {
    return 1;
  }
  return 0;
}

// Recupera los límites de alarma de temperatura y los guarda en la caché en grados Celsius.
bool DFRobot_SHT3x::measureTemperatureLimitC() {
  uint16_t limit[1];
  float data;
  if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_SET, limit) != 0) {
    return false;
  }
  data = convertTempLimitData(limit);
  limitData.highSet = round(data);
  if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_CLEAR, limit) != 0) {
    return false;
  }
  data = convertTempLimitData(limit);
  limitData.highClear = round(data);
  if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_CLEAR, limit) != 0) {
    return false;
  }
  data = convertTempLimitData(limit);
  limitData.lowClear = round(data);
  if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_SET, limit) != 0) {
    return false;
  }
  data = convertTempLimitData(limit);
  limitData.lowSet = round(data);
  return true;
}

// Recupera los límites de alarma de temperatura y los guarda en la caché en grados Fahrenheit.
bool DFRobot_SHT3x::measureTemperatureLimitF() {
  uint16_t limit[1];
  float data;
  if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_SET, limit) != 0) {
    return false;
  }
  data = convertTempLimitData(limit);
  limitData.highSet = round(data * 9.0 / 5.0 + 32.0);
  if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_CLEAR, limit) != 0) {
    return false;
  }
  data = convertTempLimitData(limit);
  limitData.highClear = round(data * 9.0 / 5.0 + 32.0);
  if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_CLEAR, limit) != 0) {
    return false;
  }
  data = convertTempLimitData(limit);
  limitData.lowClear = round(data * 9.0 / 5.0 + 32.0);
  if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_SET, limit) != 0) {
    return false;
  }
  data = convertTempLimitData(limit);
  limitData.lowSet = round(data * 9.0 / 5.0 + 32.0);
  return true;
}

// Devuelve el último umbral recuperado de activación de alarma por temperatura alta en °F.
float DFRobot_SHT3x::getTemperatureHighSetF() {
  return limitData.highSet;
}

// Devuelve el último umbral recuperado de desactivación de alarma por temperatura alta en °F.
float DFRobot_SHT3x::getTemperatureHighClearF() {
  return limitData.highClear;
}

// Devuelve el último umbral recuperado de desactivación de alarma por temperatura baja en °F.
float DFRobot_SHT3x::getTemperatureLowClearF() {
  return limitData.lowClear;
}

// Devuelve el último umbral recuperado de activación de alarma por temperatura baja en °F.
float DFRobot_SHT3x::getTemperatureLowSetF() {
  return limitData.lowSet;
}

// Lee una palabra de umbral del sensor y comprueba su CRC.
uint8_t DFRobot_SHT3x::readLimitData(uint16_t cmd, uint16_t *pBuf) {
  uint8_t rawData[3];
  uint8_t crc;

  writeCommand(cmd, 2);
  readData(rawData, 3);
  crc = rawData[2];
  if (checkCrc(rawData) != crc) {
    return 1;
  }
  pBuf[1] = rawData[0];
  pBuf[1] = (pBuf[1] << 8) | rawData[1];
  return 0;
}

// Devuelve el último umbral recuperado de activación de alarma por temperatura alta en °C.
float DFRobot_SHT3x::getTemperatureHighSetC() {
  return limitData.highSet;
}

// Devuelve el último umbral recuperado de desactivación de alarma por temperatura alta en °C.
float DFRobot_SHT3x::getTemperatureHighClearC() {
  return limitData.highClear;
}

// Devuelve el último umbral recuperado de desactivación de alarma por temperatura baja en °C.
float DFRobot_SHT3x::getTemperatureLowClearC() {
  return limitData.lowClear;
}

// Devuelve el último umbral recuperado de activación de alarma por temperatura baja en °C.
float DFRobot_SHT3x::getTemperatureLowSetC() {
  return limitData.lowSet;
}

// Devuelve el último umbral recuperado de activación de alarma por humedad alta.
float DFRobot_SHT3x::getHumidityHighSetRH() {
  return limitData.highSet;
}

// Devuelve el último umbral recuperado de desactivación de alarma por humedad alta.
float DFRobot_SHT3x::getHumidityHighClearRH() {
  return limitData.highClear;
}

// Devuelve el último umbral recuperado de desactivación de alarma por humedad baja.
float DFRobot_SHT3x::getHumidityLowClearRH() {
  return limitData.lowClear;
}

// Devuelve el último umbral recuperado de activación de alarma por humedad baja.
float DFRobot_SHT3x::getHumidityLowSetRH() {
  return limitData.lowSet;
}

// Recupera los cuatro umbrales de humedad relativa y los guarda en la caché.
bool DFRobot_SHT3x::measureHumidityLimitRH() {
  uint16_t limit[1];
  if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_SET, limit) != 0) {
    return false;
  }
  limitData.highSet = convertHumidityLimitData(limit);
  if (readLimitData(SHT3X_CMD_READ_HIGH_ALERT_LIMIT_CLEAR, limit) != 0) {
    return false;
  }
  limitData.highClear = convertHumidityLimitData(limit);
  if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_CLEAR, limit) != 0) {
    return false;
  }
  limitData.lowClear = convertHumidityLimitData(limit);

  if (readLimitData(SHT3X_CMD_READ_LOW_ALERT_LIMIT_SET, limit) != 0) {
    return false;
  }
  limitData.lowSet = convertHumidityLimitData(limit);
  return true;
}

// Convierte los dos bytes de temperatura del SHT3x a grados Celsius.
float DFRobot_SHT3x::convertTemperature(uint8_t rawTemperature[]) {
  uint16_t rawValue;
  rawValue = rawTemperature[0];
  rawValue = (rawValue << 8) | rawTemperature[1];
  return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

// Convierte los dos bytes de humedad del SHT3x a porcentaje de humedad relativa.
float DFRobot_SHT3x::convertHumidity(uint8_t rawHumidity[]) {
  uint16_t rawValue;
  rawValue = rawHumidity[0];
  rawValue = (rawValue << 8) | rawHumidity[1];
  return 100.0f * (float)rawValue / 65535.0f;
}

// Convierte una temperatura Celsius al código de 16 bits que utiliza el sensor.
uint16_t DFRobot_SHT3x::convertRawTemperature(float value) {
  return (value + 45.0f) / 175.0f * 65535.0f;
}

// Convierte un porcentaje de humedad al código de 16 bits del sensor.
uint16_t DFRobot_SHT3x::convertRawHumidity(float value) {
  return value / 100.0f * 65535.0f;
}

// Interpreta los bits de temperatura de una palabra de umbral y los expresa en grados Celsius.
float DFRobot_SHT3x::convertTempLimitData(uint16_t limit[]) {
  limit[1] = limit[1] << 7;
  limit[1] = limit[1] & 0xFF80;
  limit[1] = limit[1] | 0x1A;
  return 175.0f * (float)limit[1] / 65535.0f - 45.0f;
}

// Interpreta los bits de humedad de una palabra de umbral y los expresa en porcentaje.
float DFRobot_SHT3x::convertHumidityLimitData(uint16_t limit[]) {
  limit[1] = limit[1] & 0xFE00;
  limit[1] = limit[1] | 0xCD;
  return round(100.0f * (float)limit[1] / 65535.0f);
}

// Calcula el CRC de ocho bits de una palabra de dos bytes del SHT3x.
uint8_t DFRobot_SHT3x::checkCrc(uint8_t data[]) {
  uint8_t bit;
  uint8_t crc = 0xFF;

  for (uint8_t dataCounter = 0; dataCounter < 2; dataCounter++) {
    crc ^= (data[dataCounter]);
    for (bit = 8; bit > 0; --bit) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x131;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

// Envía una orden de ajuste de umbral con la palabra de datos y su CRC.
void DFRobot_SHT3x::writeLimitData(uint16_t cmd, uint16_t limitData) {
  uint8_t _pBuf[5];
  _pBuf[0] = cmd >> 8;
  _pBuf[1] = cmd & 0xff;
  _pBuf[2] = limitData >> 8;
  _pBuf[3] = limitData & 0xff;
  uint8_t crc = checkCrc(_pBuf + 2);
  _pBuf[4] = crc;
  write(_pBuf, 5);
}

// Transmite al sensor los dos bytes que forman una orden.
void DFRobot_SHT3x::writeCommand(uint16_t cmd, size_t size) {
  uint8_t _pBuf[2];
  _pBuf[0] = cmd >> 8;
  _pBuf[1] = cmd & 0xFF;
  delay(Configuracion::SHT_PAUSA_COMANDO_MS);
  write(_pBuf, 2);
}

// Envía el contenido de un búfer a la dirección I²C del sensor.
void DFRobot_SHT3x::write(const void *pBuf, size_t size) {
  if (pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t *_pBuf = (uint8_t *)pBuf;
  _pWire->beginTransmission(_address);
  for (uint8_t i = 0; i < size; i++) {
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
}

// Recibe datos del sensor por I²C y devuelve la cantidad de bytes realmente leídos.
uint8_t DFRobot_SHT3x::readData(void *pBuf, size_t size) {
  if (pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
    return 0;
  }
  uint8_t *_pBuf = (uint8_t *)pBuf;

  memset(_pBuf, 0, size);
  _pWire->requestFrom(_address, size);
  uint8_t len = 0;
  while (len < size && _pWire->available()) {
    const int value = _pWire->read();
    if (value < 0)
      break;
    _pBuf[len++] = uint8_t(value);
  }

  return len;
}
