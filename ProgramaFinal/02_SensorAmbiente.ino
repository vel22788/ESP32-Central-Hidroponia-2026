/**
 * @file 02_SensorAmbiente.ino
 * Adaptador del sensor exterior SEN0385/SHT31 y del reloj DS3231.
 * Wire se inicia una sola vez en el coordinador; aquí se reutiliza ese bus.
 * El sensor entrega grados Celsius y humedad relativa porcentual. ERR != 0
 * indica fallo de adquisición y el coordinador publica calidad de error.
 * El RTC proporciona calendario, no una temperatura sustitutiva del PT1000.
 */
/*
 * Adaptado de singleMeasurement.ino de DFRobot (MIT).
 * Solo lectura del SEN0385; el principal configura Wire y envia por UART.
 */
#include <Wire.h>
#include "src/Configuracion.h"
#include "src/DFRobot_SHT3x/DFRobot_SHT3x.h"

DFRobot_SHT3x sht3x(&Wire, /*address=*/Configuracion::SHT_DIRECCION, /*RST=*/Configuracion::SHT_RESET_GPIO);
static bool sensorAmbienteListo = false;

/**
 * Devuelve true tras detección y reset del sensor. No vuelve a iniciar Wire;
 * el coordinador ya eligió pines, frecuencia y timeout del bus compartido.
 */
bool iniciarSensorAmbiente() {
  sensorAmbienteListo = (sht3x.begin() == 0);
  if (sensorAmbienteListo) {
    sensorAmbienteListo = sht3x.softReset();
  }
  return sensorAmbienteListo;
}

/**
 * Obtiene una pareja temperatura/humedad coherente según el driver incluido.
 * Si falla, conserva ERR y obliga a reinicializar en una petición posterior.
 * La cadencia se decide en loop(); esta función conserva las esperas del driver.
 */
DFRobot_SHT3x::sRHAndTemp_t actualizarSensorAmbiente() {
  DFRobot_SHT3x::sRHAndTemp_t data = {};
  data.ERR = -1;

  if (!sensorAmbienteListo && !iniciarSensorAmbiente()) return data;

  // Variante incluida en el ejemplo: una lectura y una validacion sencilla.
  data = sht3x.readTemperatureAndHumidity(static_cast<DFRobot_SHT3x::eRepeatability_t>(Configuracion::SHT_REPETIBILIDAD));
  if (data.ERR != 0) sensorAmbienteListo = false;
  return data;
}

#if BASE_HMI_INTEGRADA
#include "src/RelojRTC.h"
/**
 * Agrupa el arranque ambiental y RTC cuando se compila con la HMI integrada.
 * Inicializar el RTC solo lee su estado; no ajusta automáticamente su calendario.
 */
void iniciarModuloAmbientalCompleto() {
  iniciarSensorAmbiente();
  RelojRTC::iniciar();
}
void actualizarRelojModuloAmbiental() { RelojRTC::actualizar(); }
#endif
