#pragma once
#include <cmath>
#include "Configuracion.h"
#include <cstddef>
#include <cstdint>

namespace AjustePH1P {
constexpr uint32_t MARCA = 0x50483150, VERSION = 1, FLAGS = 3;
constexpr float PENDIENTE = -0.006701475769351598f;
struct Registro {
  uint32_t marca = MARCA, version = VERSION, flags = FLAGS;
  float pendiente = PENDIENTE, anclaMv = 0, objetivoPH = 0;

  uint32_t esperaMs = 3000, muestras = 20, crc = 0;
};
static_assert(sizeof(Registro) == 36, "Formato ajuste PH 1P incompatible");
// Calcula el CRC32 del registro provisional de pH.
inline uint32_t crc32(const void *datos, size_t n) {
  const auto *p = static_cast<const uint8_t *>(datos);
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (unsigned b = 0; b < 8; ++b)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
  }
  return ~crc;
}

// Valida el formato, la pendiente, el punto de referencia y los metadatos del ajuste.
inline bool parametrosValidos(const Registro &r) {
  return r.marca == MARCA && r.version == VERSION && r.flags == FLAGS && std::isfinite(r.pendiente) &&
         r.pendiente < 0 && std::isfinite(r.anclaMv) && r.anclaMv > 0 &&
         r.anclaMv < Configuracion::ADC_ALIMENTACION_V * 1000 && std::isfinite(r.objetivoPH) &&
         r.objetivoPH >= 0 && r.objetivoPH <= 14 && r.esperaMs <= 30000 && r.muestras > 0 && r.muestras <= 100;
}

// Comprueba que los parámetros del ajuste sean válidos y coincidan con el CRC almacenado.
inline bool valido(const Registro &r) {
  return parametrosValidos(r) && r.crc == crc32(&r, offsetof(Registro, crc));
}

// Crea y sella un ajuste de un punto con el voltaje medido y el pH asignado a esa referencia.
inline bool preparar(float mv, float objetivo, Registro &r) {
  Registro candidato;
  candidato.anclaMv = mv;
  candidato.objetivoPH = objetivo;
  if (!parametrosValidos(candidato))
    return false;
  candidato.crc = crc32(&candidato, offsetof(Registro, crc));
  r = candidato;
  return true;
}

// Calcula el pH provisional desde el punto de referencia y la pendiente guardada.
inline double calcular(double mv, const Registro &r) {
  if (!valido(r) || !std::isfinite(mv) || mv <= 0 || mv >= Configuracion::ADC_ALIMENTACION_V * 1000)
    return NAN;
  return double(r.objetivoPH) + double(r.pendiente) * (mv - double(r.anclaMv));
}

// Verifica que el pH calculado sea finito y esté entre 0 y 14.
inline bool enRango(double ph) {
  return std::isfinite(ph) && ph >= 0 && ph <= 14;
}
}
