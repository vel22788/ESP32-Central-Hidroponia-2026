/**
 * @file ModeloPH.h
 * Modelo lineal de pH de dos puntos para la cadena completa de medición.
 * Entradas en mV, salida de pH sin unidades; sin compensación térmica automática.
 * Exige dos puntos válidos con pendiente negativa y separación respecto al ruido.
 * Esas verificaciones son controles numéricos, no una garantía de exactitud.
 * El módulo 01 lee el registro ya guardado; este header no escribe la memoria.
 */
#pragma once
#include <cmath>
#include "Configuracion.h"
#include <cstddef>
#include <cstdint>

// Ajuste de la cadena completa en mV. Sin valores de pH predeterminados ni ATC.
namespace ModeloPH {
constexpr uint32_t MAGIC = 0x50484331, VERSION = 1;
constexpr float MAX_MV = Configuracion::ADC_ALIMENTACION_V*1000.0f;  // Riel documentado del ADS1115, no rango PGA.

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

inline bool puntoValido(const Punto &p) {
  return std::isfinite(p.ph) && p.ph >= 0 && p.ph <= 14 &&
         std::isfinite(p.mv) && p.mv > 0 && p.mv < MAX_MV &&
         std::isfinite(p.dispersionMv) && p.dispersionMv >= 0 &&
         p.dispersionMv < MAX_MV;
}

/**
 * Exige puntos separados y pendiente negativa respecto de mV. La separación
 * frente a dispersión evita construir una recta numéricamente mal condicionada.
 */
inline bool parValido(const Punto &a, const Punto &b) {
  if (!puntoValido(a) || !puntoValido(b)) return false;
  // Controles numericos del calibrador; no tolerancias de exactitud del sensor.
  const float dp = b.ph - a.ph, dv = b.mv - a.mv;
  const float ruido = a.dispersionMv > b.dispersionMv ? a.dispersionMv : b.dispersionMv;
  return std::fabs(dp) >= 1.0f && std::fabs(dv) > 1.0f &&
         std::fabs(dv) > 4.0f * ruido && dp * dv < 0;
}

inline bool preparar(const Punto &a, const Punto &b, Registro &r) {
  if (!parValido(a, b)) return false;
  r = Registro{};
  r.ph1 = a.ph; r.mv1 = a.mv; r.ph2 = b.ph; r.mv2 = b.mv;
  r.dispersion1Mv = a.dispersionMv; r.dispersion2Mv = b.dispersionMv;
  r.crc = crc32(&r, offsetof(Registro, crc));
  return true;
}

inline bool valido(const Registro &r) {
  return r.magic == MAGIC && r.version == VERSION &&
         r.crc == crc32(&r, offsetof(Registro, crc)) &&
         parValido({r.ph1, r.mv1, r.dispersion1Mv}, {r.ph2, r.mv2, r.dispersion2Mv});
}

inline float pendiente(const Registro &r) { return (r.ph2 - r.ph1) / (r.mv2 - r.mv1); }
inline float intercepto(const Registro &r) { return r.ph1 - pendiente(r) * r.mv1; }
/**
 * Interpola/extrapola la recta de calibración válida. Devuelve NAN ante
 * registro/señal inválidos o pH fuera de 0..14; no impone un pH predeterminado.
 * extrapola() informa si los mV están fuera del intervalo entre patrones.
 */
inline float leer(float mv, const Registro &r) {
  if (!valido(r) || !std::isfinite(mv) || mv <= 0 || mv >= MAX_MV) return NAN;
  const float ph = r.ph1 + (mv - r.mv1) * pendiente(r);
  return std::isfinite(ph) && ph >= 0 && ph <= 14 ? ph : NAN;
}
inline bool extrapola(float mv, const Registro &r) {
  const float minimo = r.mv1 < r.mv2 ? r.mv1 : r.mv2;
  const float maximo = r.mv1 > r.mv2 ? r.mv1 : r.mv2;
  return mv < minimo || mv > maximo;
}
}
