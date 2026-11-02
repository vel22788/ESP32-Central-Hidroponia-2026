/**
 * @file 01_SensoresADS.ino
 * Adquisición de pH, conductividad EC y PT1000 mediante un ADS1115 compartido.
 * Etapas: REPOSO -> ESPERA_EC -> pares PT/EC -> ESPERA_PH -> PH -> REPOSO.
 * EC y pH se alimentan por turnos; PT1000 permanece alimentado externamente.
 *
 * Unidades internas: PT en V; EC/pH en mV; EC calculada en uS/cm; temperatura
 * en grados Celsius. El coordinador publica EC en mS/cm dividiendo entre 1000.
 * La EC publicada no se compensa a 25 C y no exige una PT válida.
 * Las calibraciones se leen de NVS con CRC; este módulo no las sobrescribe.
 * Un error común del ADS invalida el bloque; las señales EC y PT se evalúan
 * por separado cuando las conversiones pueden completarse.
 */
/* Módulo 01: ADS1115, EC/PT y pH alternados. NVS solo lectura. */
#include <Adafruit_ADS1X15.h>
#include "src/Configuracion.h"
#include <Preferences.h>
#include "src/Plataforma.h"
#include "src/PinesSensores.h"
#include "src/ModeloEC.h"
#include "src/ModeloPH.h"
#include "src/AjustePH1P.h"
namespace SensoresADS {
class Modulo {
  /**
   * La máquina de estados espera estabilización y conversiones sin un delay
   * de toda la ventana; actualizar() debe atenderse repetidamente desde loop().
   */
  enum class Etapa { REPOSO, ESPERA_EC, PT, EC, ESPERA_PH, PH } etapa=Etapa::REPOSO;
  Adafruit_ADS1115 adc;
  uint32_t inicio=0, consulta=0, espera=0;
  uint8_t n=0;
  double sumaPT=0, sumaEC=0, sumaPH=0;
  bool adcListo=false, phListo=false, phProvisional=false;
  bool ptSenalValida=true, ecSenalValida=true;
  AjustePH1P::Registro ajustePH;
  ModeloPH::Registro formalPH;
  DatosSensoresADS pendiente;
  bool responde() { Wire.beginTransmission(Configuracion::ADS_DIRECCION); return Wire.endTransmission()==0; }
  /**
   * Comprueba respuesta I2C y configura ganancia/tasa. Puede reintentarse tras
   * fallo; ganancia PGA y tensión de alimentación son límites diferentes.
   */
  bool iniciarADC() {
    adcListo=responde() && adc.begin(Configuracion::ADS_DIRECCION,&Wire);
    if(adcListo) { adc.setGain(Configuracion::ADS_GANANCIA); adc.setDataRate(Configuracion::ADS_TASA_DATOS); }
    return adcListo;
  }
  void apagar() { digitalWrite(PinesSensores::PH,LOW); digitalWrite(PinesSensores::EC,LOW); }
  /**
   * Cierra el ciclo, apaga ambas sondas conmutadas e incrementa el número de
   * muestra. El coordinador detecta ese número para publicar una sola vez.
   */
  void publicar() {
    apagar(); etapa=Etapa::REPOSO;
    pendiente.muestra=datos.muestra+1; pendiente.momentoMs=millis(); datos=pendiente;
  }
  /**
   * Aborta la ventana común, invalida magnitudes y permite reintentar el ADC.
   * Un fallo del convertidor compartido no debe parecer una lectura válida antigua.
   */
  void errorADC(const char *codigo) {
    pendiente.estadoPH=codigo; pendiente.estado=codigo; pendiente.estadoPT=codigo;
    pendiente.phValido=pendiente.ecValida=pendiente.ptValida=pendiente.valida=false;
    pendiente.ph=pendiente.ecUsCm=pendiente.temperaturaC=NAN;
    pendiente.momentoPH=pendiente.momentoEC=millis(); adcListo=false; publicar();
  }
  /**
   * Abre Preferences en solo lectura (true). EC/PT exige formato y CRC válidos.
   * En pH, la presencia de ajuste_1p selecciona la vía provisional; si esa clave
   * existe pero es inválida, no se sustituye silenciosamente por el registro formal.
   */
  void leerCalibraciones() {
    Preferences memoria;
    estadoNVS="SIN_CALIBRACION";
    if(memoria.begin("cal_ec_v1",true)) {
      if(memoria.isKey("registro")) {
        estadoNVS="NVS_INVALIDA"; ModeloEC::Registro r;
        if(memoria.getBytesLength("registro")==sizeof(r) && memoria.getBytes("registro",&r,sizeof(r))==sizeof(r) && ModeloEC::registroValido(r)) {
          cal=r; estadoNVS="OK";
        }
      }
      memoria.end();
    }
    phListo=phProvisional=false;
    if(!memoria.begin("cal_ph_v1",true)) return;
    if(memoria.isKey("ajuste_1p")) {
      phListo=memoria.getBytesLength("ajuste_1p")==sizeof(ajustePH) &&
        memoria.getBytes("ajuste_1p",&ajustePH,sizeof(ajustePH))==sizeof(ajustePH) && AjustePH1P::valido(ajustePH);
      phProvisional=phListo;
    } else if(memoria.isKey("registro")) {
      phListo=memoria.getBytesLength("registro")==sizeof(formalPH) &&
        memoria.getBytes("registro",&formalPH,sizeof(formalPH))==sizeof(formalPH) && ModeloPH::valido(formalPH);
    }
    memoria.end();
  }
  /**
   * Inicia una conversión single-shot en el canal correspondiente. No espera
   * aquí a que termine: actualizar() consulta conversión lista o timeout.
   */
  void convertir(Etapa siguiente) {
    if(!responde()) { errorADC("ADS_NO_RESPONDE"); return; }
    etapa=siguiente; inicio=consulta=millis();
    adc.startADCReading(siguiente==Etapa::PT ? Configuracion::ADS_CANAL_PT :
      siguiente==Etapa::EC ? Configuracion::ADS_CANAL_EC : Configuracion::ADS_CANAL_PH,false);
  }
  /**
   * Apaga EC antes de activar pH y reinicia el contador de su ventana.
   * El cambio de alimentación precede a la espera de estabilización de pH.
   */
  void empezarPH() {
    digitalWrite(PinesSensores::EC,LOW); digitalWrite(PinesSensores::PH,HIGH);
    etapa=Etapa::ESPERA_PH; inicio=millis(); n=0;
  }
  /**
   * Promedia pares PT/EC y aplica por separado las calibraciones válidas.
   * La temperatura ajustada debe estar en rango; EC sin compensación puede seguir
   * válida aunque PT falle. Después continúa con pH, no publica todavía el ciclo.
   */
  void terminarEC() {
    pendiente.ptV=ptSenalValida ? sumaPT/Configuracion::MUESTRAS_EC_PT : NAN;
    pendiente.ecMv=ecSenalValida ? sumaEC/Configuracion::MUESTRAS_EC_PT*1000 : NAN;
    pendiente.momentoEC=millis(); pendiente.ptNominalC=ModeloEC::temperaturaC(pendiente.ptV);
    const bool ecCal=!std::strcmp(estadoNVS,"OK") && (cal.flags&ModeloEC::EC_CALIBRADA);
    const bool ptCal=!std::strcmp(estadoNVS,"OK") && (cal.flags&ModeloEC::PT_CALIBRADA) && (cal.flags&ModeloEC::PT_DOS_PUNTOS);
    if(ecCal && ecSenalValida) pendiente.ecUsCm=ModeloEC::ecUsCm(pendiente.ecMv,cal.k);
    const float t=ptCal ? ModeloEC::temperaturaAjustadaC(pendiente.ptNominalC,cal) : NAN;
    pendiente.ptValida=ModeloEC::temperaturaValida(t);
    if(pendiente.ptValida) {
      pendiente.temperaturaC=t;
      pendiente.fueraIntervaloPT=t<fminf(cal.ptReferencia1C,cal.ptReferencia2C) || t>fmaxf(cal.ptReferencia1C,cal.ptReferencia2C);
    }
    // CE a la temperatura de la muestra, sin depender del PT1000.
    pendiente.ecValida=ecCal && ecSenalValida && std::isfinite(pendiente.ecUsCm) && pendiente.ecUsCm>=0;
    pendiente.valida=pendiente.ecValida;
    pendiente.estado=pendiente.ecValida ? "OK_SIN_COMPENSACION" : !ecCal ? "EC_CALIBRACION_INVALIDA" : "EC_SENAL_INVALIDA";
    pendiente.estadoPT=!ptSenalValida ? "PT_SENAL_INVALIDA" : !ptCal ? "PT_CALIBRACION_INVALIDA" :
      !pendiente.ptValida ? "PT_TEMPERATURA_INVALIDA" : "OK";
    empezarPH();
  }
 public:
  DatosSensoresADS datos;
  ModeloEC::Registro cal;
  const char *estadoNVS="SIN_CALIBRACION";
  void iniciar() { apagar(); leerCalibraciones(); iniciarADC(); }
  bool ocupado() const { return etapa!=Etapa::REPOSO; }
  /**
   * Solicita un ciclo si está libre: false significa ocupado o ADC no disponible.
   * Reinicia sumas, activa solo EC y deja la adquisición en ESPERA_EC.
   * El parámetro esperaMs gobierna EC; pH usa ENCENDIDO_PH_MS del header.
   */
  bool solicitar(uint32_t esperaMs) {
    if(ocupado()) return false;
    pendiente=DatosSensoresADS{}; sumaPT=sumaEC=sumaPH=0; n=0; espera=esperaMs;
    ptSenalValida=ecSenalValida=true;
    if((!adcListo && !iniciarADC()) || !responde()) { errorADC("ADS_NO_RESPONDE"); return false; }
    digitalWrite(PinesSensores::PH,LOW); digitalWrite(PinesSensores::EC,HIGH);
    etapa=Etapa::ESPERA_EC; inicio=millis(); return true;
  }
  void detener() { pendiente=DatosSensoresADS{}; errorADC("DETENIDO"); }
  /**
   * Avanza una etapa por servicio: espera, comprueba I2C, consulta el ADS y
   * acumula muestras válidas. No admite saturación ni tensiones fuera del riel.
   * PT y EC alternan canales; tras sus pares, pH usa su propio promedio y fecha.
   */
  void actualizar() {
    if(!ocupado()) return;
    const uint32_t ahora=millis();
    if(etapa==Etapa::ESPERA_EC || etapa==Etapa::ESPERA_PH) {
      if(uint32_t(ahora-inicio)>=(etapa==Etapa::ESPERA_PH ? Configuracion::ENCENDIDO_PH_MS : espera)) convertir(etapa==Etapa::ESPERA_EC ? Etapa::PT : Etapa::PH);
      return;
    }
    if(uint32_t(ahora-inicio)>=Configuracion::ADC_TIMEOUT_MS) { errorADC("ADC_TIMEOUT"); return; }
    if(uint32_t(ahora-consulta)<Configuracion::ADC_CONSULTA_MS) return;
    consulta=ahora;
    if(!responde()) { errorADC("ADS_NO_RESPONDE"); return; }
    if(!adc.conversionComplete()) return;
    const auto raw=adc.getLastConversionResults();
    if(!responde()) { errorADC("ADS_NO_RESPONDE"); return; }
    const float v=adc.computeVolts(raw);
    const bool senalValida=raw!=INT16_MAX && raw!=INT16_MIN && std::isfinite(v) && v>=0 && v<Configuracion::ADC_ALIMENTACION_V &&
       (etapa==Etapa::PH || ModeloEC::tensionSenalValida(v));
    if(etapa==Etapa::PT) {
      if(senalValida) sumaPT+=v; else ptSenalValida=false;
      convertir(Etapa::EC);
    }
    else if(etapa==Etapa::EC) {
      if(senalValida) sumaEC+=v; else ecSenalValida=false;
      if(++n<Configuracion::MUESTRAS_EC_PT) convertir(Etapa::PT); else terminarEC();
    } else {
      if(!senalValida) { errorADC("ADC_SENAL_INVALIDA"); return; }
      sumaPH+=v;
      if(++n<Configuracion::MUESTRAS_PH) { convertir(Etapa::PH); return; }
      pendiente.phMv=sumaPH/Configuracion::MUESTRAS_PH*1000; pendiente.momentoPH=millis();
      pendiente.ph=phListo ? phProvisional ? AjustePH1P::calcular(pendiente.phMv,ajustePH) : ModeloPH::leer(pendiente.phMv,formalPH) : NAN;
      pendiente.phValido=AjustePH1P::enRango(pendiente.ph);
      pendiente.phProvisional=phProvisional;
      pendiente.estadoPH=!phListo ? "SIN_CALIBRACION" : !pendiente.phValido ? "FUERA_RANGO" : phProvisional ? "PROVISIONAL_UN_PUNTO" : "OK";
      publicar();
    }
  }
};
static Modulo modulo;
}
/**
 * API del módulo: estas funciones delegan en una única instancia. Los getters
 * no convierten ni escriben NVS. leerEC() devuelve mS/cm, mientras que el campo
 * DatosSensoresADS::ecUsCm conserva uS/cm. Consultar también las banderas de validez.
 */
void iniciarSensoresADS() { SensoresADS::modulo.iniciar(); }
bool solicitarLecturaSensoresADS(uint32_t espera) { return SensoresADS::modulo.solicitar(espera); }
void actualizarSensoresADS() { SensoresADS::modulo.actualizar(); }
void detenerSensoresADS() { SensoresADS::modulo.detener(); }
bool sensoresADSOcupados() { return SensoresADS::modulo.ocupado(); }
DatosSensoresADS obtenerSensoresADS() { return SensoresADS::modulo.datos; }
const char *estadoCalibracionADS() { return SensoresADS::modulo.estadoNVS; }
uint32_t flagsCalibracionADS() { return SensoresADS::modulo.cal.flags; }
float factorEC() { return SensoresADS::modulo.cal.k; }
float gananciaPT() { return SensoresADS::modulo.cal.ptGanancia; }
float offsetPT() { return SensoresADS::modulo.cal.ptOffsetC; }
float leerTemperaturaSolucion() { return SensoresADS::modulo.datos.temperaturaC; }
float leerEC() { return SensoresADS::modulo.datos.ecUsCm/1000; }
float leerPH() { return SensoresADS::modulo.datos.ph; }
