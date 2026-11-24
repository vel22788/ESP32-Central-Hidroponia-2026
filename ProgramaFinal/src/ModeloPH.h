#pragma once
#include <cmath>
#include "Configuracion.h"
#include <cstddef>
#include <cstdint>

namespace ModeloPH {
constexpr uint32_t MAGIC = 0x50484331, VERSION = 1;
constexpr float MAX_MV = Configuracion::ADC_ALIMENTACION_V * 1000.0f;

struct Punto {
  float ph = 0, mv = 0, dispersionMv = 0;
};

struct Registro {
  uint32_t magic = MAGIC, version = VERSION;
  float ph1 = 0, mv1 = 0, ph2 = 0, mv2 = 0;
  float dispersion1Mv = 0, dispersion2Mv = 0;
  uint32_t crc = 0;
};
static_assert(sizeof(Registro) == 36, "Formato NVS pH incompatible");

// Calcula el CRC32 que protege los datos de calibración de pH.
inline uint32_t crc32(const void *datos, size_t n) {
  const auto *p = static_cast<const uint8_t *>(datos);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

// Comprueba que un patrón tenga pH, tensión y dispersión dentro de los límites admitidos.
inline bool puntoValido(const Punto &p) {
  return std::isfinite(p.ph) && p.ph >= 0 && p.ph <= 14 && std::isfinite(p.mv) && p.mv > 0 && p.mv < MAX_MV &&
         std::isfinite(p.dispersionMv) && p.dispersionMv >= 0 && p.dispersionMv < MAX_MV;
}

// Verifica que ambos patrones estén suficientemente separados y formen una recta de pendiente negativa.
inline bool parValido(const Punto &a, const Punto &b) {
  if (!puntoValido(a) || !puntoValido(b))
    return false;

  const float dp = b.ph - a.ph, dv = b.mv - a.mv;
  const float ruido = a.dispersionMv > b.dispersionMv ? a.dispersionMv : b.dispersionMv;
  return std::fabs(dp) >= 1.0f && std::fabs(dv) > 1.0f && std::fabs(dv) > 4.0f * ruido && dp * dv < 0;
}

// Construye y sella el registro formal de pH a partir de dos patrones válidos.
inline bool preparar(const Punto &a, const Punto &b, Registro &r) {
  if (!parValido(a, b))
    return false;
  r = Registro{};
  r.ph1 = a.ph;
  r.mv1 = a.mv;
  r.ph2 = b.ph;
  r.mv2 = b.mv;
  r.dispersion1Mv = a.dispersionMv;
  r.dispersion2Mv = b.dispersionMv;
  r.crc = crc32(&r, offsetof(Registro, crc));
  return true;
}

// Verifica la identidad del registro, su CRC y la validez de los dos puntos guardados.
inline bool valido(const Registro &r) {
  return r.magic == MAGIC && r.version == VERSION && r.crc == crc32(&r, offsetof(Registro, crc)) &&
         parValido({r.ph1, r.mv1, r.dispersion1Mv}, {r.ph2, r.mv2, r.dispersion2Mv});
}

// Calcula la pendiente de la recta de calibración en unidades de pH por milivoltio.
inline float pendiente(const Registro &r) {
  return (r.ph2 - r.ph1) / (r.mv2 - r.mv1);
}

// Calcula la ordenada al origen de la recta definida por los patrones.
inline float intercepto(const Registro &r) {
  return r.ph1 - pendiente(r) * r.mv1;
}

// Convierte milivoltios a pH usando los dos puntos y devuelve NaN si el registro o el resultado es inválido.
inline float leer(float mv, const Registro &r) {
  if (!valido(r) || !std::isfinite(mv) || mv <= 0 || mv >= MAX_MV)
    return NAN;
  const float ph = r.ph1 + (mv - r.mv1) * pendiente(r);
  return std::isfinite(ph) && ph >= 0 && ph <= 14 ? ph : NAN;
}

// Indica si el voltaje medido queda fuera del tramo entre los dos patrones.
inline bool extrapola(float mv, const Registro &r) {
  const float minimo = r.mv1 < r.mv2 ? r.mv1 : r.mv2;
  const float maximo = r.mv1 > r.mv2 ? r.mv1 : r.mv2;
  return mv < minimo || mv > maximo;
}
}
