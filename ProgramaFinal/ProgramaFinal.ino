/**
 * @file ProgramaFinal.ino
 * Coordinador del ESP32 central: único setup() y loop() del sketch.
 * Los cinco .ino numerados se compilan juntos; su orden de nombre no es un
 * planificador. El orden real depende de las llamadas de loop().
 *
 * Recorrido: protecciones -> HMI/agenda -> protecciones -> motores -> sensores
 * y reloj cuando no hay movimiento -> publicación de datos.
 * La adquisición ADS usa etapas; el SHT31 conserva las esperas de su driver.
 * Por eso se suspende I2C durante el movimiento para reducir pausas entre STEP.
 *
 * Ajustes editables: src/Configuracion.h. Cambiarlos exige recompilar y cargar.
 * El calendario del RTC y millis() cumplen funciones distintas: fecha local
 * para agenda/trazas, diferencias de millis() para duraciones.
 * Revisión lógica R4; esta edición agrega documentación, sin nueva carga.
 */
/* Coordinador final. 01 ADS/pH/EC/PT; 02 ambiente/RTC; 03 motores;
 * 04 relés/flotadores/proceso; 05 protocolo HMI. Agenda en módulo 05.
 */
#include <Arduino.h>
#include <Wire.h>
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

/**
 * Copia publicada de cinco magnitudes. Una nueva lectura conserva las edades de
 * los campos que no se actualizaron; el RTC aporta una fecha cuando es válido.
 */
static ControlHMI::Lectura lecturaIntegrada;
static bool adquisicionHabilitada=true, suspensionMovimiento=false;
static uint32_t ultimaADS=0, ultimaAmbiente=0, muestraReportada=0;

/**
 * Callback heredado: ignora la petición de encendido y fuerza pH/EC a LOW.
 * Así la alimentación queda bajo la secuencia exclusiva del módulo 01.
 */
void establecerAlimentacionPruebaPH(bool) {
  // El firmware final solo alimenta sondas dentro del módulo 01.
  digitalWrite(PinesSensores::PH,LOW); digitalWrite(PinesSensores::EC,LOW);
}
/**
 * Habilita o suspende adquisición desde consola. Al suspender, apaga sondas,
 * publica NAN/calidad ERROR y notifica humedad inválida al proceso. Al reanudar,
 * adelanta las marcas para permitir una nueva solicitud sin esperar otro período.
 */
void establecerAdquisicion(bool habilitar) {
  adquisicionHabilitada=habilitar;
  if(!habilitar) {
    detenerSensoresADS();
    for(unsigned i=0;i<5;++i) {
      lecturaIntegrada.calidad[i]=2; lecturaIntegrada.valor[i]=NAN;
      lecturaIntegrada.instante[i]=millis(); lecturaIntegrada.fecha[i]=RelojRTC::fechaActual();
    }
    ControlHMI::publicar(lecturaIntegrada);
    Proceso::humedad(NAN,false,millis(),ControlHMI::obtenerUmbral());
  } else { ultimaADS=millis()-Configuracion::PERIODO_ADS_MS; ultimaAmbiente=millis()-Configuracion::AMBIENTE_MS; }
}
/**
 * Inicialización única tras reinicio. Primero fija salidas en reposo; después
 * configura consola, I2C, motores, adquisición y HMI. No restaura riegos activos.
 */
void setup() {
  // Precargar OFF antes de OUTPUT; no restaurar ninguna acción tras reinicio.
  Proceso::iniciar(); prepararSalidasMotores();
  digitalWrite(PinesSensores::PH,LOW); pinMode(PinesSensores::PH,OUTPUT);
  digitalWrite(PinesSensores::EC,LOW); pinMode(PinesSensores::EC,OUTPUT);
  Serial0.begin(Configuracion::BAUDIOS_CONSOLA);
  Wire.begin(Configuracion::I2C_SDA,Configuracion::I2C_SCL,Configuracion::I2C_HZ); Wire.setTimeOut(Configuracion::I2C_TIMEOUT_MS);
  iniciarMotores(Configuracion::PASO_US,Configuracion::MOTOR_MAX_MS,Configuracion::FILTRO_FINALES_MS);
  for(unsigned n=1;n<=4;++n) configurarMotor(n,Configuracion::FINAL_ACTIVO,Configuracion::DIR_SUBIR);
  iniciarSensoresADS(); iniciarModuloAmbientalCompleto(); iniciarHMI();
  ultimaADS=millis()-Configuracion::PERIODO_ADS_MS; ultimaAmbiente=millis()-Configuracion::AMBIENTE_MS;
}
/**
 * Servicio cooperativo: cada vuelta avanza etapas y comprueba condiciones.
 * Las restas uint32_t de millis()/micros() comparan tiempos transcurridos incluso
 * al desbordarse el contador; no son comparaciones de fechas del calendario.
 */
void loop() {
  Proceso::actualizar(); // principal y temporizadores antes del siguiente pulso.
  actualizarHMI();
  Proceso::actualizar();
  actualizarMotores(); Proceso::despuesMotores();
  /**
   * Al empezar movimiento se cancela la ventana ADS. Al terminar se solicita una
   * ventana completa nueva, evitando promedios que mezclen antes/después de la pausa.
   */
  const bool moviendo=motoresOcupados();
  if(moviendo && !suspensionMovimiento) { detenerSensoresADS(); suspensionMovimiento=true; }
  if(!moviendo && suspensionMovimiento) { suspensionMovimiento=false; ultimaADS=millis()-Configuracion::PERIODO_ADS_MS; }
  // No intercalar I2C bloqueante con los pulsos del driver sin aceleración.
  if(!moviendo) {
    actualizarRelojModuloAmbiental();
    actualizarSensoresADS();
    if(adquisicionHabilitada && !sensoresADSOcupados() && uint32_t(millis()-ultimaADS)>=Configuracion::PERIODO_ADS_MS) {
      ultimaADS=millis(); solicitarLecturaSensoresADS(Configuracion::ENCENDIDO_MS);
    }
    const auto d=obtenerSensoresADS();
    if(d.muestra!=muestraReportada) {
      muestraReportada=d.muestra;
      /**
       * Índices de la HMI: 0 pH, 1 EC, 2 T ambiente, 3 humedad, 4 T solución.
       * EC pasa de uS/cm a mS/cm; pH provisional usa calidad 3, válida formal usa 1.
       * Cada campo conserva el momento de su ventana, aunque se publique junto a otros.
       */
      for(unsigned i:{0u,1u,4u}) {
        lecturaIntegrada.valor[i]=i==0 ? d.ph : i==1 ? d.ecUsCm/1000.0 : d.temperaturaC;
        lecturaIntegrada.calidad[i]=i==0 ? (d.phValido ? (d.phProvisional ? 3 : 1) : 2) :
          i==1 ? (d.ecValida ? 1 : 2) : (d.ptValida ? 1 : 2);
        lecturaIntegrada.instante[i]=i==0 ? d.momentoPH : d.momentoEC;
        lecturaIntegrada.fecha[i]=RelojRTC::fechaActual();
      }
      ControlHMI::publicar(lecturaIntegrada);
      if(Configuracion::TRAZAS_SENSORES) Serial0.printf("SENSORES PH=%.4f PH_ESTADO=%s PH_mV=%.3f EC_SIN_COMP_uScm=%.3f EC_ESTADO=%s PT_C=%.3f PT_ESTADO=%s\r\n",
        d.ph,d.estadoPH,d.phMv,d.ecUsCm,d.estado,d.temperaturaC,d.estadoPT);
    }
    if(adquisicionHabilitada && uint32_t(millis()-ultimaAmbiente)>=Configuracion::AMBIENTE_MS) {
      ultimaAmbiente=millis(); const auto d=actualizarSensorAmbiente();
      for(unsigned i=2;i<=3;++i) {
        lecturaIntegrada.valor[i]=i==2 ? d.TemperatureC : d.Humidity;
        lecturaIntegrada.calidad[i]=d.ERR==0 ? 1 : 2;
        lecturaIntegrada.instante[i]=millis(); lecturaIntegrada.fecha[i]=RelojRTC::fechaActual();
      }
      Proceso::humedad(d.Humidity,d.ERR==0,millis(),ControlHMI::obtenerUmbral());
      ControlHMI::publicar(lecturaIntegrada);
    }
  }
  delay(Configuracion::PAUSA_LOOP_MS);
}
