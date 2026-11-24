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
  enum class Etapa { REPOSO, ESPERA_EC, PT, EC, ESPERA_PH, PH } etapa = Etapa::REPOSO;
  Adafruit_ADS1115 adc;
  uint32_t inicio = 0, consulta = 0, espera = 0;
  uint8_t n = 0;
  double sumaPT = 0, sumaEC = 0, sumaPH = 0;
  bool adcListo = false, phListo = false, phProvisional = false;
  bool ptSenalValida = true, ecSenalValida = true;
  AjustePH1P::Registro ajustePH;
  ModeloPH::Registro formalPH;
  DatosSensoresADS pendiente;
  // Comprueba si el ADS1115 responde en su dirección I²C.
  bool responde() {
    Wire.beginTransmission(Configuracion::ADS_DIRECCION);
    return Wire.endTransmission() == 0;
  }

  // Inicializa el ADC y configura la ganancia y la tasa de conversión.
  bool iniciarADC() {
    adcListo = responde() && adc.begin(Configuracion::ADS_DIRECCION, &Wire);
    if (adcListo) {
      adc.setGain(Configuracion::ADS_GANANCIA);
      adc.setDataRate(Configuracion::ADS_TASA_DATOS);
    }
    return adcListo;
  }

  // Corta la alimentación conmutada de las placas de pH y conductividad.
  void apagar() {
    digitalWrite(PinesSensores::PH, LOW);
    digitalWrite(PinesSensores::EC, LOW);
  }

  // Apaga las sondas y deja disponible la ventana terminada con un número de muestra nuevo.
  void publicar() {
    apagar();
    etapa = Etapa::REPOSO;
    pendiente.muestra = datos.muestra + 1;
    pendiente.momentoMs = millis();
    datos = pendiente;
  }

  // Termina la ventana con el diagnóstico recibido e invalida las tres mediciones.
  void errorADC(const char *codigo) {
    pendiente.estadoPH = codigo;
    pendiente.estado = codigo;
    pendiente.estadoPT = codigo;
    pendiente.phValido = pendiente.ecValida = pendiente.ptValida = pendiente.valida = false;
    pendiente.ph = pendiente.ecUsCm = pendiente.temperaturaC = NAN;
    pendiente.momentoPH = pendiente.momentoEC = millis();
    adcListo = false;
    publicar();
  }

  // Recupera de NVS el ajuste EC/PT y el de pH. Si existe la clave provisional de pH, comprueba primero ese registro.
  void leerCalibraciones() {
    Preferences memoria;
    estadoNVS = "SIN_CALIBRACION";
    if (memoria.begin("cal_ec_v1", true)) {
      if (memoria.isKey("registro")) {
        estadoNVS = "NVS_INVALIDA";
        ModeloEC::Registro r;
        if (memoria.getBytesLength("registro") == sizeof(r) &&
            memoria.getBytes("registro", &r, sizeof(r)) == sizeof(r) && ModeloEC::registroValido(r)) {
          cal = r;
          estadoNVS = "OK";
        }
      }
      memoria.end();
    }
    phListo = phProvisional = false;
    if (!memoria.begin("cal_ph_v1", true))
      return;
    if (memoria.isKey("ajuste_1p")) {
      phListo = memoria.getBytesLength("ajuste_1p") == sizeof(ajustePH) &&
                memoria.getBytes("ajuste_1p", &ajustePH, sizeof(ajustePH)) == sizeof(ajustePH) &&
                AjustePH1P::valido(ajustePH);
      phProvisional = phListo;
    } else if (memoria.isKey("registro")) {
      phListo = memoria.getBytesLength("registro") == sizeof(formalPH) &&
                memoria.getBytes("registro", &formalPH, sizeof(formalPH)) == sizeof(formalPH) &&
                ModeloPH::valido(formalPH);
    }
    memoria.end();
  }

  // Lanza una conversión individual en el canal de la etapa solicitada y reinicia su temporizador.
  void convertir(Etapa siguiente) {
    if (!responde()) {
      errorADC("ADS_NO_RESPONDE");
      return;
    }
    etapa = siguiente;
    inicio = consulta = millis();
    adc.startADCReading(siguiente == Etapa::PT   ? Configuracion::ADS_CANAL_PT
                        : siguiente == Etapa::EC ? Configuracion::ADS_CANAL_EC
                                                 : Configuracion::ADS_CANAL_PH,
                        false);
  }

  // Apaga la placa de CE y enciende la de pH para que se estabilice antes de medir.
  void empezarPH() {
    digitalWrite(PinesSensores::EC, LOW);
    digitalWrite(PinesSensores::PH, HIGH);
    etapa = Etapa::ESPERA_PH;
    inicio = millis();
    n = 0;
  }

  // Promedia PT y CE, aplica sus calibraciones por separado y pasa a la medición de pH.
  void terminarEC() {
    pendiente.ptV = ptSenalValida ? sumaPT / Configuracion::MUESTRAS_EC_PT : NAN;
    pendiente.ecMv = ecSenalValida ? sumaEC / Configuracion::MUESTRAS_EC_PT * 1000 : NAN;
    pendiente.momentoEC = millis();
    pendiente.ptNominalC = ModeloEC::temperaturaC(pendiente.ptV);
    const bool ecCal = !std::strcmp(estadoNVS, "OK") && (cal.flags & ModeloEC::EC_CALIBRADA);
    const bool ptCal = !std::strcmp(estadoNVS, "OK") && (cal.flags & ModeloEC::PT_CALIBRADA) &&
                       (cal.flags & ModeloEC::PT_DOS_PUNTOS);
    if (ecCal && ecSenalValida)
      pendiente.ecUsCm = ModeloEC::ecUsCm(pendiente.ecMv, cal.k);
    const float t = ptCal ? ModeloEC::temperaturaAjustadaC(pendiente.ptNominalC, cal) : NAN;
    pendiente.ptValida = ModeloEC::temperaturaValida(t);
    if (pendiente.ptValida) {
      pendiente.temperaturaC = t;
      pendiente.fueraIntervaloPT = t < fminf(cal.ptReferencia1C, cal.ptReferencia2C) ||
                                   t > fmaxf(cal.ptReferencia1C, cal.ptReferencia2C);
    }

    pendiente.ecValida = ecCal && ecSenalValida && std::isfinite(pendiente.ecUsCm) && pendiente.ecUsCm >= 0;
    pendiente.valida = pendiente.ecValida;
    pendiente.estado = pendiente.ecValida ? "OK_SIN_COMPENSACION"
                       : !ecCal           ? "EC_CALIBRACION_INVALIDA"
                                          : "EC_SENAL_INVALIDA";
    pendiente.estadoPT = !ptSenalValida        ? "PT_SENAL_INVALIDA"
                         : !ptCal              ? "PT_CALIBRACION_INVALIDA"
                         : !pendiente.ptValida ? "PT_TEMPERATURA_INVALIDA"
                                               : "OK";
    empezarPH();
  }

public:
  DatosSensoresADS datos;
  ModeloEC::Registro cal;
  const char *estadoNVS = "SIN_CALIBRACION";
  // Arranca con las sondas apagadas, carga las calibraciones e intenta iniciar el ADC.
  void iniciar() {
    apagar();
    leerCalibraciones();
    iniciarADC();
  }

  // Indica si hay una ventana analógica en curso.
  bool ocupado() const {
    return etapa != Etapa::REPOSO;
  }

  // Acepta una ventana cuando el ADC está libre y empieza la estabilización de la placa de CE.
  bool solicitar(uint32_t esperaMs) {
    if (ocupado())
      return false;
    pendiente = DatosSensoresADS{};
    sumaPT = sumaEC = sumaPH = 0;
    n = 0;
    espera = esperaMs;
    ptSenalValida = ecSenalValida = true;
    if ((!adcListo && !iniciarADC()) || !responde()) {
      errorADC("ADS_NO_RESPONDE");
      return false;
    }
    digitalWrite(PinesSensores::PH, LOW);
    digitalWrite(PinesSensores::EC, HIGH);
    etapa = Etapa::ESPERA_EC;
    inicio = millis();
    return true;
  }

  // Cancela la ventana activa, apaga las sondas y entrega el estado DETENIDO.
  void detener() {
    pendiente = DatosSensoresADS{};
    errorADC("DETENIDO");
  }

  // Avanza la ventana por etapas. Espera la estabilización, alterna PT y CE y termina con el promedio de pH.
  void actualizar() {
    if (!ocupado())
      return;
    const uint32_t ahora = millis();
    if (etapa == Etapa::ESPERA_EC || etapa == Etapa::ESPERA_PH) {
      if (uint32_t(ahora - inicio) >= (etapa == Etapa::ESPERA_PH ? Configuracion::ENCENDIDO_PH_MS : espera))
        convertir(etapa == Etapa::ESPERA_EC ? Etapa::PT : Etapa::PH);
      return;
    }
    if (uint32_t(ahora - inicio) >= Configuracion::ADC_TIMEOUT_MS) {
      errorADC("ADC_TIMEOUT");
      return;
    }
    if (uint32_t(ahora - consulta) < Configuracion::ADC_CONSULTA_MS)
      return;
    consulta = ahora;
    if (!responde()) {
      errorADC("ADS_NO_RESPONDE");
      return;
    }
    if (!adc.conversionComplete())
      return;
    const auto raw = adc.getLastConversionResults();
    if (!responde()) {
      errorADC("ADS_NO_RESPONDE");
      return;
    }
    const float v = adc.computeVolts(raw);
    const bool senalValida = raw != INT16_MAX && raw != INT16_MIN && std::isfinite(v) && v >= 0 &&
                             v < Configuracion::ADC_ALIMENTACION_V &&
                             (etapa == Etapa::PH || ModeloEC::tensionSenalValida(v));
    if (etapa == Etapa::PT) {
      if (senalValida)
        sumaPT += v;
      else
        ptSenalValida = false;
      convertir(Etapa::EC);
    } else if (etapa == Etapa::EC) {
      if (senalValida)
        sumaEC += v;
      else
        ecSenalValida = false;
      if (++n < Configuracion::MUESTRAS_EC_PT)
        convertir(Etapa::PT);
      else
        terminarEC();
    } else {
      if (!senalValida) {
        errorADC("ADC_SENAL_INVALIDA");
        return;
      }
      sumaPH += v;
      if (++n < Configuracion::MUESTRAS_PH) {
        convertir(Etapa::PH);
        return;
      }
      pendiente.phMv = sumaPH / Configuracion::MUESTRAS_PH * 1000;
      pendiente.momentoPH = millis();
      pendiente.ph = phListo ? phProvisional ? AjustePH1P::calcular(pendiente.phMv, ajustePH)
                                             : ModeloPH::leer(pendiente.phMv, formalPH)
                             : NAN;
      pendiente.phValido = AjustePH1P::enRango(pendiente.ph);
      pendiente.phProvisional = phProvisional;
      pendiente.estadoPH = !phListo              ? "SIN_CALIBRACION"
                           : !pendiente.phValido ? "FUERA_RANGO"
                           : phProvisional       ? "PROVISIONAL_UN_PUNTO"
                                                 : "OK";
      publicar();
    }
  }
};
static Modulo modulo;
}

// Prepara el módulo analógico al arrancar el central.
void iniciarSensoresADS() {
  SensoresADS::modulo.iniciar();
}

// Solicita una ventana con la espera de CE indicada y devuelve si se pudo iniciar.
bool solicitarLecturaSensoresADS(uint32_t espera) {
  return SensoresADS::modulo.solicitar(espera);
}

// Atiende la etapa analógica pendiente desde el bucle principal.
void actualizarSensoresADS() {
  SensoresADS::modulo.actualizar();
}

// Interrumpe la adquisición analógica y corta la alimentación de ambas sondas.
void detenerSensoresADS() {
  SensoresADS::modulo.detener();
}

// Devuelve si el módulo analógico sigue midiendo.
bool sensoresADSOcupados() {
  return SensoresADS::modulo.ocupado();
}

// Devuelve una copia de la última ventana analógica completa.
DatosSensoresADS obtenerSensoresADS() {
  return SensoresADS::modulo.datos;
}

// Devuelve el diagnóstico de la calibración EC/PT recuperada de NVS.
const char *estadoCalibracionADS() {
  return SensoresADS::modulo.estadoNVS;
}

// Entrega las marcas que identifican los ajustes presentes en el registro EC/PT.
uint32_t flagsCalibracionADS() {
  return SensoresADS::modulo.cal.flags;
}

// Devuelve el factor K empleado para calcular la conductividad.
float factorEC() {
  return SensoresADS::modulo.cal.k;
}

// Devuelve la ganancia del ajuste de temperatura del PT1000.
float gananciaPT() {
  return SensoresADS::modulo.cal.ptGanancia;
}

// Devuelve la corrección aditiva del PT1000 en grados Celsius.
float offsetPT() {
  return SensoresADS::modulo.cal.ptOffsetC;
}

// Devuelve la última temperatura de la solución en grados Celsius.
float leerTemperaturaSolucion() {
  return SensoresADS::modulo.datos.temperaturaC;
}

// Devuelve la última conductividad sin compensación térmica, expresada en mS/cm.
float leerEC() {
  return SensoresADS::modulo.datos.ecUsCm / 1000;
}

// Devuelve el último pH calculado por el módulo.
float leerPH() {
  return SensoresADS::modulo.datos.ph;
}
