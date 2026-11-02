/**
 * @file AjustePH1P.h
 * Corrección provisional de pH con ancla asignada por el usuario y pendiente
 * histórica conservada. No equivale a una calibración formal de dos patrones.
 * La ecuación usa objetivoPH + pendiente * (mV - anclaMv), sin ATC.
 * El módulo 01 comprueba el registro NVS y publica esta calidad como PROVISIONAL;
 * la HMI compatible muestra ~. esperaMs/muestras son metadatos históricos,
 * no los tiempos de adquisición actuales (estos están en Configuracion.h).
 */
#pragma once
#include <cmath>
#include "Configuracion.h"
#include <cstddef>
#include <cstdint>

// Corrección provisional de un punto asignado por el usuario. Sin ATC.
// Pendiente conservada de la hipótesis de soluciones anteriores intercambiadas.
namespace AjustePH1P {
constexpr uint32_t MARCA=0x50483150, VERSION=1, FLAGS=3;
constexpr float PENDIENTE=-0.006701475769351598f;
struct Registro {
  uint32_t marca=MARCA, version=VERSION, flags=FLAGS;
  float pendiente=PENDIENTE, anclaMv=0, objetivoPH=0;
  // Metadatos del ajuste histórico; NO gobiernan los tiempos de adquisición.
  uint32_t esperaMs=3000, muestras=20, crc=0;
};
static_assert(sizeof(Registro)==36,"Formato ajuste PH 1P incompatible");
inline uint32_t crc32(const void *datos,size_t n) {
  const auto *p=static_cast<const uint8_t *>(datos);
  uint32_t crc=0xffffffffu;
  for(size_t i=0;i<n;++i) {
    crc^=p[i];
    for(unsigned b=0;b<8;++b) crc=(crc>>1)^((crc&1) ? 0xedb88320u : 0u);
  }
  return ~crc;
}
inline bool parametrosValidos(const Registro &r) {
  return r.marca==MARCA && r.version==VERSION && r.flags==FLAGS &&
    std::isfinite(r.pendiente) && r.pendiente<0 &&
    std::isfinite(r.anclaMv) && r.anclaMv>0 && r.anclaMv<Configuracion::ADC_ALIMENTACION_V*1000 &&
    std::isfinite(r.objetivoPH) && r.objetivoPH>=0 && r.objetivoPH<=14 &&
    r.esperaMs<=30000 && r.muestras>0 && r.muestras<=100;
}
/**
 * Comprueba metadatos y CRC del ajuste provisional. Validez de memoria no
 * convierte el ancla elegida en una referencia metrológica certificada.
 */
inline bool valido(const Registro &r) {
  return parametrosValidos(r) && r.crc==crc32(&r,offsetof(Registro,crc));
}
inline bool preparar(float mv,float objetivo,Registro &r) {
  Registro candidato;
  candidato.anclaMv=mv; candidato.objetivoPH=objetivo;
  if(!parametrosValidos(candidato)) return false;
  candidato.crc=crc32(&candidato,offsetof(Registro,crc)); r=candidato; return true;
}
/**
 * Calcula pH respecto del ancla en mV. Puede dar un resultado fuera de 0..14;
 * el consumidor debe aplicar enRango() y conservar la calidad PROVISIONAL.
 */
inline double calcular(double mv,const Registro &r) {
  if(!valido(r) || !std::isfinite(mv) || mv<=0 || mv>=Configuracion::ADC_ALIMENTACION_V*1000) return NAN;
  return double(r.objetivoPH)+double(r.pendiente)*(mv-double(r.anclaMv));
}
inline bool enRango(double ph) { return std::isfinite(ph) && ph>=0 && ph<=14; }
} // namespace AjustePH1P
