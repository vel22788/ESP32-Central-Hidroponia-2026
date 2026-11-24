#pragma once
#include <stdint.h>
#include <Adafruit_ADS1X15.h>

namespace Configuracion {
constexpr const char *FIRMWARE = "CENTRAL_HIDROPONICO";

// Tiempos de riego y protecciones hidráulicas.
constexpr uint32_t LLUVIA_MS = 60000;
constexpr uint32_t NFT_MS = 60000;
constexpr uint32_t LLENADO_MAX_MS = 60000;
constexpr uint32_t DRENAJE_MAX_MS = 60000;
constexpr uint32_t DRENAJE_FINAL_MS = 10000;
constexpr uint32_t SEPARACION_RELES_MS = 100;
constexpr uint32_t DURACION_CONFIG_MAX_MS = 3600000;
constexpr uint32_t EVENTO_MAX_S = 3600;

// Tiempos de movimiento y filtrado de contactos.
constexpr uint32_t PASO_US = 5000;
constexpr uint32_t PULSO_STEP_US = 3;
constexpr uint32_t PASO_MIN_US = 10;
constexpr uint32_t MOTOR_MAX_MS = 60000;
constexpr uint32_t FILTRO_FINALES_MS = 20;
constexpr uint32_t FILTRO_NIVELES_MS = 20;

// Adquisición analógica.
constexpr uint32_t PERIODO_ADS_MS = 5000;
constexpr uint32_t ENCENDIDO_MS = 3000;
constexpr uint32_t ENCENDIDO_PH_MS = 3000;
constexpr uint16_t MUESTRAS_EC_PT = 20;
constexpr uint16_t MUESTRAS_PH = 20;
constexpr uint32_t ADC_TIMEOUT_MS = 100;
constexpr uint32_t ADC_CONSULTA_MS = 2;

constexpr auto ADS_GANANCIA = GAIN_ONE;
constexpr auto ADS_TASA_DATOS = RATE_ADS1115_128SPS;
constexpr uint16_t ADS_CANAL_PH = ADS1X15_REG_CONFIG_MUX_SINGLE_0;
constexpr uint16_t ADS_CANAL_EC = ADS1X15_REG_CONFIG_MUX_SINGLE_1;
constexpr uint16_t ADS_CANAL_PT = ADS1X15_REG_CONFIG_MUX_SINGLE_2;
constexpr float ADC_ALIMENTACION_V = 3.3f;
constexpr float EC_PT_SENAL_MAX_V = 3.0f;
constexpr float PT_MIN_C = 0.0f, PT_MAX_C = 50.0f;

// Adquisición ambiental y reloj.
constexpr uint32_t AMBIENTE_MS = 2000;
constexpr uint8_t SHT_REPETIBILIDAD = 0;

constexpr uint32_t SHT_PAUSA_COMANDO_MS = 1;
constexpr uint32_t SHT_RESET_MS = 10;
constexpr uint32_t SHT_CONVERSION_MS = 15;
constexpr uint32_t RTC_PERIODO_MS = 1000;
constexpr uint32_t RTC_CADUCIDAD_MS = 3000;

constexpr const char *ZONA_LOCAL = "-06:00";

// Regla automática de humedad. El valor −1 deshabilita el umbral inicial.
constexpr int UMBRAL_HUMEDAD_INICIAL = -1;
constexpr uint32_t HUMEDAD_CONFIRMAR_MS = 0;
constexpr uint32_t HUMEDAD_REARME_MS = 0;
constexpr float HUMEDAD_HISTERESIS = 0.0f;
constexpr uint32_t HUMEDAD_CADUCIDAD_MS = 3 * AMBIENTE_MS;

struct PasoHumedad {
  unsigned metodo, destino;
};
constexpr PasoHumedad SECUENCIA_HUMEDAD[] = {{2, 0}, {0, 1}, {0, 2}};
constexpr unsigned PASOS_HUMEDAD = sizeof(SECUENCIA_HUMEDAD) / sizeof(SECUENCIA_HUMEDAD[0]);

constexpr unsigned BAUDIOS_CONSOLA = 115200;
constexpr unsigned BAUDIOS_HMI = 115200;
constexpr uint8_t UART_HMI = 1;
constexpr uint32_t PERDIDA_ENLACE_MS = 6000;
constexpr uint32_t LINEA_INCOMPLETA_MS = 1000;
constexpr unsigned MAX_LINEA = 1024;
constexpr unsigned HMI_BUFFER_RX = 4096;
constexpr unsigned COLA_TX = 48;
constexpr unsigned TX_BYTES_POR_VUELTA = 128;
constexpr unsigned RX_BYTES_POR_VUELTA = 128;
constexpr unsigned MAX_CACHE_ORDENES = 16;
constexpr unsigned MAX_HISTORIAL = 40;
constexpr unsigned MAX_MUESTRAS_CURVAS = 40;
constexpr unsigned LIMITE_EVENTOS = 20;
constexpr uint32_t PAUSA_LOOP_MS = 1;
constexpr bool TRAZAS_SENSORES = true;
constexpr bool TRAZAS_RECEPCION = true;

// Relés en orden de solenoides M1/M2, bolas M1/M2, válvula vertical y bomba.
constexpr uint8_t RELES[6] = {35, 45, 37, 36, 48, 47};
// Entradas de nivel en orden de depósito principal, M1 y M2.
constexpr uint8_t NIVELES[3] = {40, 39, 38};
constexpr uint8_t MOTOR_DIR[4] = {4, 6, 15, 17};
constexpr uint8_t MOTOR_STEP[4] = {5, 7, 16, 18};
constexpr uint8_t MOTOR_DIR_SUBIR[4] = {1, 1, 1, 1};
constexpr uint8_t FINAL_SIN_GPIO = 0xFF;
// Cada motor usa un par de finales en orden superior e inferior.
constexpr uint8_t MOTOR_FINALES[4][2] = {{46, 3}, {9, 10}, {11, 12}, {13, 14}};
constexpr int RELE_ACTIVO = 0, NIVEL_ACTIVO = 1, FINAL_ACTIVO = 0;
constexpr uint8_t PH_GPIO = 21, EC_GPIO = 20;
constexpr uint8_t I2C_SDA = 1, I2C_SCL = 2;
constexpr uint32_t I2C_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 25;
constexpr uint8_t ADS_DIRECCION = 0x48, SHT_DIRECCION = 0x44, RTC_DIRECCION = 0x68;
constexpr uint8_t SHT_RESET_GPIO = 0xFF;
constexpr int RX_HMI = 41, TX_HMI = 42;

// Comprueba al compilar que la secuencia de humedad tenga pasos y que cada método use un destino válido.
constexpr bool secuenciaValida() {
  for (auto p : SECUENCIA_HUMEDAD)
    if (p.metodo > 2 || (p.metodo == 2 ? p.destino != 0 : p.destino < 1 || p.destino > 2))
      return false;
  return PASOS_HUMEDAD > 0;
}
static_assert(PULSO_STEP_US >= 3 && PASO_MIN_US > PULSO_STEP_US && PASO_US >= PASO_MIN_US,
              "Intervalo/pulso STEP incompatible");
static_assert(MOTOR_MAX_MS > 0 && MOTOR_MAX_MS < 0x80000000u, "Timeout de motor invalido");
static_assert(FILTRO_FINALES_MS < MOTOR_MAX_MS && FILTRO_NIVELES_MS < LLENADO_MAX_MS,
              "Filtro mayor que timeout");
static_assert(LLUVIA_MS > SEPARACION_RELES_MS && NFT_MS > 2 * uint64_t(SEPARACION_RELES_MS),
              "Riego termina antes de abrir las rutas");
static_assert(LLENADO_MAX_MS > SEPARACION_RELES_MS && DRENAJE_MAX_MS > 0,
              "Protecciones de inundacion invalidas");
static_assert(DURACION_CONFIG_MAX_MS > 0 && DURACION_CONFIG_MAX_MS < 0x80000000u && EVENTO_MAX_S > 0 &&
                  EVENTO_MAX_S <= 3600,
              "Duracion maxima invalida");
static_assert(LLUVIA_MS <= DURACION_CONFIG_MAX_MS && NFT_MS <= DURACION_CONFIG_MAX_MS &&
                  LLENADO_MAX_MS <= DURACION_CONFIG_MAX_MS && DRENAJE_MAX_MS <= DURACION_CONFIG_MAX_MS,
              "Duracion inicial supera el limite");
static_assert(DRENAJE_FINAL_MS <= DURACION_CONFIG_MAX_MS, "Margen de drenaje supera el limite");
static_assert(MUESTRAS_EC_PT > 0 && MUESTRAS_EC_PT <= 255 && MUESTRAS_PH > 0 && MUESTRAS_PH <= 255,
              "Usar 1..255 muestras por ventana");
static_assert(ADC_CONSULTA_MS > 0 && ADC_TIMEOUT_MS > ADC_CONSULTA_MS, "Tiempos ADC incompatibles");
static_assert(PERIODO_ADS_MS > 0 && AMBIENTE_MS > 0 && RTC_PERIODO_MS > 0 && RTC_CADUCIDAD_MS >= RTC_PERIODO_MS,
              "Cadencia/caducidad invalida");
static_assert(SHT_REPETIBILIDAD <= 2 && SHT_PAUSA_COMANDO_MS >= 1 && SHT_RESET_MS >= 10 &&
                  SHT_CONVERSION_MS >= 15,
              "Ajustes SHT fuera de limites");
static_assert(ADC_ALIMENTACION_V > 0 && EC_PT_SENAL_MAX_V > 0 && EC_PT_SENAL_MAX_V < ADC_ALIMENTACION_V &&
                  PT_MIN_C < PT_MAX_C,
              "Limites analogicos incompatibles");
static_assert(UMBRAL_HUMEDAD_INICIAL >= -1 && UMBRAL_HUMEDAD_INICIAL <= 100 && HUMEDAD_HISTERESIS >= 0 &&
                  HUMEDAD_HISTERESIS <= 100 &&
                  (UMBRAL_HUMEDAD_INICIAL < 0 || UMBRAL_HUMEDAD_INICIAL + HUMEDAD_HISTERESIS <= 100),
              "Umbral/histeresis invalido");
static_assert(HUMEDAD_CONFIRMAR_MS <= DURACION_CONFIG_MAX_MS && HUMEDAD_REARME_MS <= DURACION_CONFIG_MAX_MS &&
                  HUMEDAD_CADUCIDAD_MS >= AMBIENTE_MS && secuenciaValida(),
              "Regla de humedad invalida");
static_assert(BAUDIOS_CONSOLA > 0 && BAUDIOS_HMI > 0 && PERDIDA_ENLACE_MS > 0 && LINEA_INCOMPLETA_MS > 0,
              "Enlace sin tiempos/baudios validos");
static_assert(MAX_LINEA >= 256 && MAX_LINEA <= 1024 && HMI_BUFFER_RX >= MAX_LINEA + 2 && COLA_TX >= 12 &&
                  TX_BYTES_POR_VUELTA > 0 && RX_BYTES_POR_VUELTA > 0,
              "Buffers insuficientes para protocolo HMI");
static_assert(MAX_CACHE_ORDENES > 0 && MAX_HISTORIAL > 0 && MAX_MUESTRAS_CURVAS > 0 && LIMITE_EVENTOS >= 1 &&
                  LIMITE_EVENTOS <= 20,
              "Capacidad de memoria invalida");
static_assert((RELE_ACTIVO == 0 || RELE_ACTIVO == 1) && (NIVEL_ACTIVO == 0 || NIVEL_ACTIVO == 1) &&
                  (FINAL_ACTIVO == 0 || FINAL_ACTIVO == 1),
              "Polaridad debe ser 0 o 1");
static_assert(MOTOR_DIR_SUBIR[0] <= 1 && MOTOR_DIR_SUBIR[1] <= 1 && MOTOR_DIR_SUBIR[2] <= 1 &&
                  MOTOR_DIR_SUBIR[3] <= 1,
              "DIR de subida debe ser 0 o 1");
static_assert(PH_GPIO != EC_GPIO && I2C_SDA != I2C_SCL && RX_HMI != TX_HMI && I2C_HZ > 0 && I2C_TIMEOUT_MS > 0,
              "Configuracion de hardware invalida");
}
