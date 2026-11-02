/**
 * @file Configuracion.h
 * Punto único de ajustes de operación y mapa de hardware del ESP32 central R4.
 * Los valores constexpr se fijan al compilar: guardar este header no cambia
 * la placa hasta recompilar y cargar. No es el código de la pantalla.
 *
 * Guía de edición:
 *  - Riego: LLUVIA_MS/NFT_MS; máximos de llenado/drenaje son límites de protección.
 *  - Motor: PASO_US es intervalo de pulsos, PULSO_STEP_US su tiempo en alto.
 *  - Sondas: esperas de estabilización, cantidad de muestras y consultas ADC.
 *  - Ambiente/RTC/enlace: cadencia de servicio y antigüedad máxima aceptada.
 *  - Humedad: umbral, confirmación, rearme y orden de la secuencia automática.
 *
 * ms = milisegundos; us = microsegundos; S = segundos; V = voltios.
 * Los tiempos son valores de configuración, no resultados de mediciones nuevas.
 * CONFIG RIEGO/HUMEDAD y UMBRAL pueden reemplazar ajustes en RAM. Un evento de
 * agenda usa su propia DURACION_S; cambiar NFT_MS no reescribe eventos guardados.
 * Coeficientes de calibración, CRC y formatos NVS no son ajustes de este header.
 */
#pragma once
#include <stdint.h>
#include <Adafruit_ADS1X15.h>

// AJUSTES DEL ESP32 CENTRAL: editar aquí y volver a compilar/cargar.
// Compartido por el principal, los cinco .ino y sus componentes de src/.
// ms = milisegundos; us = microsegundos; 1000 ms = 1 s.
// Valores iniciales de R3 conservados; pantalla y puente no se modifican.
// CONFIG RIEGO / CONFIG HUMEDAD / UMBRAL pueden sustituir estos valores en RAM
// hasta reiniciar. Siempre se arranca en MANUAL, con las salidas apagadas.
// Las calibraciones EC/PT/pH siguen en NVS: no se sustituyen por este archivo.
namespace Configuracion {
constexpr const char *FIRMWARE = "CENTRAL_FINAL_20261101_R4";

// ===== RIEGO: tiempos desde el encendido de la bomba =====
/**
 * Duración desde bomba ON, incluyendo separación entre relés. Para agenda
 * la duración guardada del evento sustituye estos valores de NFT/lluvia.
 * LLENADO_MAX_MS/DRENAJE_MAX_MS limitan la fase aunque el flotador no cambie.
 */
constexpr uint32_t LLUVIA_MS = 5000;
constexpr uint32_t NFT_MS = 10000;
constexpr uint32_t LLENADO_MAX_MS = 20000; // Protección; el flotador termina antes.
constexpr uint32_t DRENAJE_MAX_MS = 35000; // Protección; el flotador termina antes.
constexpr uint32_t SEPARACION_RELES_MS = 100; // Entre bomba, solenoide y bola.
constexpr uint32_t DURACION_CONFIG_MAX_MS = 3600000; // Máximo CONFIG RIEGO/HUMEDAD.
constexpr uint32_t EVENTO_MAX_S = 3600; // Máximo ejecutable; HMI admite hasta 3600 s.

// ===== MOTORES: velocidad, pulso y protección =====
// Menos PASO_US = mayor velocidad (pulsos/s). No hay rampa de aceleración.
// No expresa mm/s ni cambia la corriente del driver (ajuste físico VREF).
/**
 * Intervalo menor solicita más pulsos por segundo, pero la frecuencia efectiva
 * también depende de cuánto tarda loop(). No ajusta micropasos, corriente VREF
 * ni convierte pasos en distancia. Un pulso no dura MOTOR_MAX_MS.
 */
constexpr uint32_t PASO_US = 5000;
constexpr uint32_t PULSO_STEP_US = 3; // Nivel alto STEP; mínimo usado: 3 us.
constexpr uint32_t PASO_MIN_US = 10; // Límite inferior de la API de motores.
constexpr uint32_t MOTOR_MAX_MS = 30000;
constexpr uint32_t FILTRO_FINALES_MS = 20;
constexpr uint32_t FILTRO_NIVELES_MS = 20;

// ===== ADQUISICIÓN Y SONDAS =====
/**
 * Cadencia de solicitud cuando el convertidor está libre, no promesa de una
 * muestra cada cinco segundos: estabilización, promedios y movimiento pueden
 * alargar el ciclo. El período no corta una ventana que sigue ocupada.
 */
constexpr uint32_t PERIODO_ADS_MS = 5000; // Solicitud si el ADS está libre.
constexpr uint32_t ENCENDIDO_MS = 3000; // Estabilización de EC por ventana.
constexpr uint32_t ENCENDIDO_PH_MS = 3000; // Estabilización de pH por ventana.
constexpr uint16_t MUESTRAS_EC_PT = 20; // Promedio de pares PT/EC por ventana.
constexpr uint16_t MUESTRAS_PH = 20;
constexpr uint32_t ADC_TIMEOUT_MS = 100; // Máximo para una conversión.
constexpr uint32_t ADC_CONSULTA_MS = 2; // Intervalo entre consultas de listo.
// Constantes Adafruit. Mantener coherentes PGA, señal y timeout.
// Ganancias: GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN.
// Tasas: RATE_ADS1115_8SPS, _16SPS, _32SPS, _64SPS, _128SPS, _250SPS, _475SPS, _860SPS.
/**
 * PGA y SPS son constantes de Adafruit. Aumentar escala PGA no permite
 * superar el riel del ADS; una tasa más lenta puede exigir más ADC_TIMEOUT_MS.
 */
constexpr auto ADS_GANANCIA = GAIN_ONE;
constexpr auto ADS_TASA_DATOS = RATE_ADS1115_128SPS;
constexpr uint16_t ADS_CANAL_PH = ADS1X15_REG_CONFIG_MUX_SINGLE_0;
constexpr uint16_t ADS_CANAL_EC = ADS1X15_REG_CONFIG_MUX_SINGLE_1;
constexpr uint16_t ADS_CANAL_PT = ADS1X15_REG_CONFIG_MUX_SINGLE_2;
constexpr float ADC_ALIMENTACION_V = 3.3f; // Riel real; no cambiar para ocultar errores.
constexpr float EC_PT_SENAL_MAX_V = 3.0f; // Rango aceptado del circuito documentado.
constexpr float PT_MIN_C = 0.0f, PT_MAX_C = 50.0f;

// ===== AMBIENTE Y RELOJ =====
constexpr uint32_t AMBIENTE_MS = 2000;
constexpr uint8_t SHT_REPETIBILIDAD = 0; // 0=alta, 1=media, 2=baja (DFRobot).
// Esperas del driver local. Se pueden aumentar; mínimos conservados del driver.
constexpr uint32_t SHT_PAUSA_COMANDO_MS = 1;
constexpr uint32_t SHT_RESET_MS = 10;
constexpr uint32_t SHT_CONVERSION_MS = 15;
constexpr uint32_t RTC_PERIODO_MS = 1000;
constexpr uint32_t RTC_CADUCIDAD_MS = 3000;
// El RTC guarda hora local. Esta etiqueta no desplaza la hora almacenada.
/**
 * Etiqueta de la fecha publicada; no realiza una conversión horaria.
 * El DS3231 debe contener la hora local que se quiere usar para la agenda.
 */
constexpr const char *ZONA_LOCAL = "-06:00";

// ===== AUTOMÁTICO POR HUMEDAD =====
constexpr int UMBRAL_HUMEDAD_INICIAL = -1; // -1: aplicar desde HMI; o 0..100 %HR.
constexpr uint32_t HUMEDAD_CONFIRMAR_MS = 0; // Humedad <= umbral durante este tiempo.
constexpr uint32_t HUMEDAD_REARME_MS = 0; // Humedad > umbral+histeresis durante este tiempo.
constexpr float HUMEDAD_HISTERESIS = 0.0f; // Puntos de %HR.
constexpr uint32_t HUMEDAD_CADUCIDAD_MS = 3 * AMBIENTE_MS;
// Métodos: 0=NFT, 1=inundación, 2=lluvia. Destinos: 1=M1, 2=M2, 0=vertical.
// Orden secuencial, con las duraciones de riego, sin acciones simultáneas.
struct PasoHumedad { unsigned metodo, destino; };
/**
 * Secuencia inicial: lluvia vertical, NFT M1 y NFT M2, en ese orden.
 * No mueve motores: la posición se prepara mediante órdenes manuales.
 */
constexpr PasoHumedad SECUENCIA_HUMEDAD[] = {{2,0}, {0,1}, {0,2}};
constexpr unsigned PASOS_HUMEDAD = sizeof(SECUENCIA_HUMEDAD)/sizeof(SECUENCIA_HUMEDAD[0]);

// ===== COMUNICACIÓN, COLAS Y MEMORIA RAM =====
constexpr unsigned BAUDIOS_CONSOLA = 115200;
constexpr unsigned BAUDIOS_HMI = 115200; // Debe coincidir con pantalla/puente.
constexpr uint8_t UART_HMI = 1;
constexpr uint32_t PERDIDA_ENLACE_MS = 6000; // Watchdog independiente por puerto.
constexpr uint32_t LINEA_INCOMPLETA_MS = 1000; // Espera entre bytes sin fin de línea.
constexpr unsigned MAX_LINEA = 1024; // Protocolo actual: hasta 1024 caracteres.
constexpr unsigned HMI_BUFFER_RX = 4096;
constexpr unsigned COLA_TX = 48; // Líneas pendientes hacia la pantalla.
constexpr unsigned TX_BYTES_POR_VUELTA = 128;
constexpr unsigned RX_BYTES_POR_VUELTA = 128; // Por puerto.
constexpr unsigned MAX_CACHE_ORDENES = 16; // Por puerto; IDs ya resueltos.
constexpr unsigned MAX_HISTORIAL = 40;
constexpr unsigned MAX_MUESTRAS_CURVAS = 40;
/**
 * Capacidad de altas nuevas, no cambio del formato NVS de veinte espacios.
 * Reducirla conserva registros existentes; para poder agregar debe haber cupo.
 */
constexpr unsigned LIMITE_EVENTOS = 20; // 1..20; reducir no borra lo ya guardado.
/**
 * Pausa final de cada vuelta: no sustituye temporizadores de etapas.
 * Aumentarla también retrasa atención de comunicaciones, sensores y STEP.
 */
constexpr uint32_t PAUSA_LOOP_MS = 1; // Afecta latencia y velocidad alcanzable.
constexpr bool TRAZAS_SENSORES = true;
constexpr bool TRAZAS_RECEPCION = true;

// ===== HARDWARE: editar al cambiar/verificar el cableado =====
// Relés: solenoide M1, solenoide M2, bola M1, bola M2, vertical, bomba.
constexpr uint8_t RELES[6] = {37,35,36,45,47,48};
constexpr uint8_t NIVELES[3] = {40,39,38}; // Principal, M1, M2.
constexpr uint8_t MOTOR_DIR[4] = {4,6,15,17};
constexpr uint8_t MOTOR_STEP[4] = {5,7,16,18};
constexpr uint8_t MOTOR_FINALES[4][2] = {{3,46},{9,10},{11,12},{13,14}}; // Superior/inferior.
/**
 * Polaridades lógicas del cableado documentado: no invertir para ocultar un
 * problema de conexión. Los números de pin son GPIO, no posición en el conector.
 */
constexpr int RELE_ACTIVO = 0, NIVEL_ACTIVO = 1, FINAL_ACTIVO = 0, DIR_SUBIR = 1;
constexpr uint8_t PH_GPIO = 21, EC_GPIO = 20;
constexpr uint8_t I2C_SDA = 1, I2C_SCL = 2;
constexpr uint32_t I2C_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 25;
constexpr uint8_t ADS_DIRECCION = 0x48, SHT_DIRECCION = 0x44, RTC_DIRECCION = 0x68;
constexpr uint8_t SHT_RESET_GPIO = 0xFF; // 0xFF: sin reset conectado.
constexpr int RX_HMI = 41, TX_HMI = 42;

// ===== VALIDACIONES AL COMPILAR =====
// Ecuaciones, unidades, CRC, registros de chips y formato NVS son invariantes,
// no ajustes de operación. Los cambios incompatibles se rechazan al compilar.
/**
 * Estas comprobaciones se evalúan al compilar. Rechazan combinaciones
 * incoherentes, pero no validan mecánica, alimentación o cableado físico.
 */
constexpr bool secuenciaValida() {
  for (auto p : SECUENCIA_HUMEDAD)
    if (p.metodo > 2 || (p.metodo == 2 ? p.destino != 0 : p.destino < 1 || p.destino > 2)) return false;
  return PASOS_HUMEDAD > 0;
}
static_assert(PULSO_STEP_US >= 3 && PASO_MIN_US > PULSO_STEP_US && PASO_US >= PASO_MIN_US, "Intervalo/pulso STEP incompatible");
static_assert(MOTOR_MAX_MS > 0 && MOTOR_MAX_MS < 0x80000000u, "Timeout de motor invalido");
static_assert(FILTRO_FINALES_MS < MOTOR_MAX_MS && FILTRO_NIVELES_MS < LLENADO_MAX_MS, "Filtro mayor que timeout");
static_assert(LLUVIA_MS > SEPARACION_RELES_MS && NFT_MS > 2*uint64_t(SEPARACION_RELES_MS), "Riego termina antes de abrir las rutas");
static_assert(LLENADO_MAX_MS > SEPARACION_RELES_MS && DRENAJE_MAX_MS > 0, "Protecciones de inundacion invalidas");
static_assert(DURACION_CONFIG_MAX_MS > 0 && DURACION_CONFIG_MAX_MS < 0x80000000u && EVENTO_MAX_S > 0 && EVENTO_MAX_S <= 3600, "Duracion maxima invalida");
static_assert(LLUVIA_MS <= DURACION_CONFIG_MAX_MS && NFT_MS <= DURACION_CONFIG_MAX_MS && LLENADO_MAX_MS <= DURACION_CONFIG_MAX_MS && DRENAJE_MAX_MS <= DURACION_CONFIG_MAX_MS, "Duracion inicial supera el limite");
static_assert(MUESTRAS_EC_PT > 0 && MUESTRAS_EC_PT <= 255 && MUESTRAS_PH > 0 && MUESTRAS_PH <= 255, "Usar 1..255 muestras por ventana");
static_assert(ADC_CONSULTA_MS > 0 && ADC_TIMEOUT_MS > ADC_CONSULTA_MS, "Tiempos ADC incompatibles");
static_assert(PERIODO_ADS_MS > 0 && AMBIENTE_MS > 0 && RTC_PERIODO_MS > 0 && RTC_CADUCIDAD_MS >= RTC_PERIODO_MS, "Cadencia/caducidad invalida");
static_assert(SHT_REPETIBILIDAD <= 2 && SHT_PAUSA_COMANDO_MS >= 1 && SHT_RESET_MS >= 10 && SHT_CONVERSION_MS >= 15, "Ajustes SHT fuera de limites");
static_assert(ADC_ALIMENTACION_V > 0 && EC_PT_SENAL_MAX_V > 0 && EC_PT_SENAL_MAX_V < ADC_ALIMENTACION_V && PT_MIN_C < PT_MAX_C, "Limites analogicos incompatibles");
static_assert(UMBRAL_HUMEDAD_INICIAL >= -1 && UMBRAL_HUMEDAD_INICIAL <= 100 && HUMEDAD_HISTERESIS >= 0 && HUMEDAD_HISTERESIS <= 100 && (UMBRAL_HUMEDAD_INICIAL < 0 || UMBRAL_HUMEDAD_INICIAL + HUMEDAD_HISTERESIS <= 100), "Umbral/histeresis invalido");
static_assert(HUMEDAD_CONFIRMAR_MS <= DURACION_CONFIG_MAX_MS && HUMEDAD_REARME_MS <= DURACION_CONFIG_MAX_MS && HUMEDAD_CADUCIDAD_MS >= AMBIENTE_MS && secuenciaValida(), "Regla de humedad invalida");
static_assert(BAUDIOS_CONSOLA > 0 && BAUDIOS_HMI > 0 && PERDIDA_ENLACE_MS > 0 && LINEA_INCOMPLETA_MS > 0, "Enlace sin tiempos/baudios validos");
static_assert(MAX_LINEA >= 256 && MAX_LINEA <= 1024 && HMI_BUFFER_RX >= MAX_LINEA + 2 && COLA_TX >= 12 && TX_BYTES_POR_VUELTA > 0 && RX_BYTES_POR_VUELTA > 0, "Buffers insuficientes para protocolo HMI");
static_assert(MAX_CACHE_ORDENES > 0 && MAX_HISTORIAL > 0 && MAX_MUESTRAS_CURVAS > 0 && LIMITE_EVENTOS >= 1 && LIMITE_EVENTOS <= 20, "Capacidad de memoria invalida");
static_assert((RELE_ACTIVO == 0 || RELE_ACTIVO == 1) && (NIVEL_ACTIVO == 0 || NIVEL_ACTIVO == 1) && (FINAL_ACTIVO == 0 || FINAL_ACTIVO == 1) && (DIR_SUBIR == 0 || DIR_SUBIR == 1), "Polaridad debe ser 0 o 1");
static_assert(PH_GPIO != EC_GPIO && I2C_SDA != I2C_SCL && RX_HMI != TX_HMI && I2C_HZ > 0 && I2C_TIMEOUT_MS > 0, "Configuracion de hardware invalida");
} // namespace Configuracion
