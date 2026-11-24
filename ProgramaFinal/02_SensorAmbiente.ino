// Adaptación de singleMeasurement.ino de DFRobot. Licencia MIT.

#include <Wire.h>
#include "src/Configuracion.h"
#include "src/DFRobot_SHT3x/DFRobot_SHT3x.h"

DFRobot_SHT3x sht3x(&Wire, Configuracion::SHT_DIRECCION, Configuracion::SHT_RESET_GPIO);
static bool sensorAmbienteListo = false;

// Identifica el SHT31 y solicita un reinicio por software. Devuelve si quedó listo para medir.
bool iniciarSensorAmbiente() {
  sensorAmbienteListo = (sht3x.begin() == 0);
  if (sensorAmbienteListo) {
    sensorAmbienteListo = sht3x.softReset();
  }
  return sensorAmbienteListo;
}

// Lee temperatura y humedad del aire en una sola muestra y marca el sensor para reinicializarlo si falla.
DFRobot_SHT3x::sRHAndTemp_t actualizarSensorAmbiente() {
  DFRobot_SHT3x::sRHAndTemp_t data = {};
  data.ERR = -1;

  if (!sensorAmbienteListo && !iniciarSensorAmbiente())
    return data;

  data = sht3x.readTemperatureAndHumidity(
      static_cast<DFRobot_SHT3x::eRepeatability_t>(Configuracion::SHT_REPETIBILIDAD));
  if (data.ERR != 0)
    sensorAmbienteListo = false;
  return data;
}

#if BASE_HMI_INTEGRADA
#include "src/RelojRTC.h"
// Prepara por separado el sensor ambiental y el reloj de tiempo real.
void iniciarModuloAmbientalCompleto() {
  iniciarSensorAmbiente();
  RelojRTC::iniciar();
}

// Da servicio al reloj cuando se cumple su período de consulta.
void actualizarRelojModuloAmbiental() {
  RelojRTC::actualizar();
}
#endif
