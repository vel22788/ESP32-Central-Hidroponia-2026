#include <Arduino.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <cstring>
#include <cstdlib>
#include "src/DFRobot_SHT3x/DFRobot_SHT3x.h"
#include "src/Plataforma.h"
#include "src/Configuracion.h"
#include "src/PinesSensores.h"
#include "src/ControlHMI.h"
#include "src/RelojRTC.h"
#if !defined(CONFIG_IDF_TARGET_ESP32S3) || ARDUINO_USB_CDC_ON_BOOT
#error "ESP32S3 Dev Module, USB CDC deshabilitado; GPIO20 es alimentación EC."
#endif
#define BASE_HMI_INTEGRADA 1
bool iniciarSensorAmbiente();
DFRobot_SHT3x::sRHAndTemp_t actualizarSensorAmbiente();
void iniciarModuloAmbientalCompleto();
void actualizarRelojModuloAmbiental();
void iniciarHMI();
void actualizarHMI();

static ControlHMI::Lectura lecturaIntegrada;
static bool adquisicionHabilitada = true, suspensionMovimiento = false;
static uint32_t ultimaADS = 0, ultimaAmbiente = 0, muestraReportada = 0;

// Deja sin alimentación las dos sondas. El módulo ADS controla su encendido durante cada medición.
void establecerAlimentacionPruebaPH(bool) {
  digitalWrite(PinesSensores::PH, LOW);
  digitalWrite(PinesSensores::EC, LOW);
}

// Habilita o suspende la adquisición. Al suspenderla, invalida las cinco lecturas y avisa a la regla de humedad.
void establecerAdquisicion(bool habilitar) {
  adquisicionHabilitada = habilitar;
  if (!habilitar) {
    detenerSensoresADS();
    for (unsigned i = 0; i < 5; ++i) {
      lecturaIntegrada.calidad[i] = 2;
      lecturaIntegrada.valor[i] = NAN;
      lecturaIntegrada.instante[i] = millis();
      lecturaIntegrada.fecha[i] = RelojRTC::fechaActual();
    }
    ControlHMI::publicar(lecturaIntegrada);
    Proceso::humedad(NAN, false, millis(), ControlHMI::obtenerUmbral());
  } else {
    ultimaADS = millis() - Configuracion::PERIODO_ADS_MS;
    ultimaAmbiente = millis() - Configuracion::AMBIENTE_MS;
  }
}

// Prepara la placa en reposo e inicializa los niveles, motores, bus I²C, sensores, reloj y servicio de control.
void setup() {
  Proceso::iniciar();
  prepararSalidasMotores();
  digitalWrite(PinesSensores::PH, LOW);
  pinMode(PinesSensores::PH, OUTPUT);
  digitalWrite(PinesSensores::EC, LOW);
  pinMode(PinesSensores::EC, OUTPUT);
  Serial0.begin(Configuracion::BAUDIOS_CONSOLA);
  Wire.begin(Configuracion::I2C_SDA, Configuracion::I2C_SCL, Configuracion::I2C_HZ);
  Wire.setTimeOut(Configuracion::I2C_TIMEOUT_MS);

  gpio_set_pull_mode(static_cast<gpio_num_t>(Configuracion::I2C_SDA), GPIO_FLOATING);
  gpio_set_pull_mode(static_cast<gpio_num_t>(Configuracion::I2C_SCL), GPIO_FLOATING);
  iniciarMotores(Configuracion::PASO_US, Configuracion::MOTOR_MAX_MS, Configuracion::FILTRO_FINALES_MS);
  for (unsigned n = 1; n <= 4; ++n)
    configurarMotor(n, Configuracion::FINAL_ACTIVO, Configuracion::MOTOR_DIR_SUBIR[n - 1]);
  iniciarSensoresADS();
  iniciarModuloAmbientalCompleto();
  iniciarHMI();
  ultimaADS = millis() - Configuracion::PERIODO_ADS_MS;
  ultimaAmbiente = millis() - Configuracion::AMBIENTE_MS;
}

// Atiende las protecciones y el movimiento en cada vuelta. Con los motores quietos recoge las mediciones y actualiza la humedad.
void loop() {
  Proceso::actualizar();
  actualizarHMI();
  Proceso::actualizar();
  actualizarMotores();
  Proceso::despuesMotores();
  const bool moviendo = motoresOcupados();
  if (moviendo && !suspensionMovimiento) {
    detenerSensoresADS();
    suspensionMovimiento = true;
  }
  if (!moviendo && suspensionMovimiento) {
    suspensionMovimiento = false;
    ultimaADS = millis() - Configuracion::PERIODO_ADS_MS;
  }

  if (!moviendo) {
    actualizarRelojModuloAmbiental();
    actualizarSensoresADS();
    if (adquisicionHabilitada && !sensoresADSOcupados() &&
        uint32_t(millis() - ultimaADS) >= Configuracion::PERIODO_ADS_MS) {
      ultimaADS = millis();
      solicitarLecturaSensoresADS(Configuracion::ENCENDIDO_MS);
    }
    const auto d = obtenerSensoresADS();
    if (d.muestra != muestraReportada) {
      muestraReportada = d.muestra;
      for (unsigned i : {0u, 1u, 4u}) {
        lecturaIntegrada.valor[i] = i == 0 ? d.ph : i == 1 ? d.ecUsCm / 1000.0 : d.temperaturaC;
        lecturaIntegrada.calidad[i] = i == 0   ? (d.phValido ? (d.phProvisional ? 3 : 1) : 2)
                                      : i == 1 ? (d.ecValida ? 1 : 2)
                                               : (d.ptValida ? 1 : 2);
        lecturaIntegrada.instante[i] = i == 0 ? d.momentoPH : d.momentoEC;
        lecturaIntegrada.fecha[i] = RelojRTC::fechaActual();
      }
      ControlHMI::publicar(lecturaIntegrada);
      if (Configuracion::TRAZAS_SENSORES)
        Serial0.printf(
            "SENSORES PH=%.4f PH_ESTADO=%s PH_mV=%.3f EC_SIN_COMP_uScm=%.3f EC_ESTADO=%s PT_C=%.3f PT_ESTADO=%s\r\n",
            d.ph, d.estadoPH, d.phMv, d.ecUsCm, d.estado, d.temperaturaC, d.estadoPT);
    }
    if (adquisicionHabilitada && uint32_t(millis() - ultimaAmbiente) >= Configuracion::AMBIENTE_MS) {
      ultimaAmbiente = millis();
      const auto d = actualizarSensorAmbiente();
      for (unsigned i = 2; i <= 3; ++i) {
        lecturaIntegrada.valor[i] = i == 2 ? d.TemperatureC : d.Humidity;
        lecturaIntegrada.calidad[i] = d.ERR == 0 ? 1 : 2;
        lecturaIntegrada.instante[i] = millis();
        lecturaIntegrada.fecha[i] = RelojRTC::fechaActual();
      }
      Proceso::humedad(d.Humidity, d.ERR == 0, millis(), ControlHMI::obtenerUmbral());
      ControlHMI::publicar(lecturaIntegrada);
    }
  }
  delay(Configuracion::PAUSA_LOOP_MS);
}
