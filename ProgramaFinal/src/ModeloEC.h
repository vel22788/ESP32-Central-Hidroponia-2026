/**
 * @file ModeloEC.h
 * Modelos numéricos y formato de calibración EC/PT1000; sin acceso a hardware.
 * ecUsCm(): mV -> uS/cm; temperaturaC(): V -> grados Celsius nominales.
 * El ajuste PT aplica ganancia y offset; no convierte EC en una lectura a 25 C.
 * ec25UsCm() se conserva como auxiliar, pero el módulo 01 actual no lo utiliza.
 * Flags declaran qué calibraciones están disponibles. Marca, versión, tamaño y
 * CRC protegen el registro NVS; no editar su estructura como ajuste de operación.
 * Las funciones preparar/sellar/migrar solo modifican objetos en RAM; aquí no
 * se escribe NVS ni se acredita una calibración física nueva.
 */
#pragma once

#include <cmath>
#include "Configuracion.h"
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cerrno>

// Ecuaciones de DFRobot_ECPRO (ZIP original de fuentes/).
// EC recibe mV; el PT1000 recibe V. No se reutiliza su EEPROM.
namespace ModeloEC {
constexpr float K_MIN = 0.5f;
constexpr float K_MAX = 1.5f;
constexpr uint32_t EC_CALIBRADA = 1;
constexpr uint32_t TEMP_COMPARADA = 2;
constexpr uint32_t BANCO_REFERENCIA_DECLARADA = 4;
constexpr uint32_t PT_CALIBRADA = 8;
constexpr uint32_t PT_DOS_PUNTOS = 16;
constexpr uint32_t VERSION = 3;
constexpr uint32_t MARCA = 0x45434331;

/**
 * Registro persistente V3 de 64 bytes. flags indica EC y/o PT de dos puntos;
 * crc cubre los bytes previos a su campo. Cambiar orden/tipos rompe compatibilidad.
 */
struct Registro {
  uint32_t marca = MARCA;
  uint32_t version = VERSION;
  uint32_t flags = 0;
  float k = 1.0f;
  float patronUsCm = 0;
  float ecCalMv = 0;
  float temperaturaCalC = 0;
  float temperaturaReferenciaC = 0;
  float temperaturaComparadaC = 0;
  float ptGanancia = 1;
  float ptOffsetC = 0;
  float ptNominal1C = 0, ptReferencia1C = 0;
  float ptNominal2C = 0, ptReferencia2C = 0;
  uint32_t crc = 0;
};
static_assert(sizeof(Registro) == 64, "Formato NVS inesperado");

struct RegistroV2 {
  uint32_t marca = MARCA, version = 2, flags = 0;
  float k = 1, patronUsCm = 0, ecCalMv = 0, temperaturaCalC = 0;
  float temperaturaReferenciaC = 0, temperaturaComparadaC = 0;
  uint32_t crc = 0;
};
static_assert(sizeof(RegistroV2) == 40, "Formato anterior inesperado");

inline bool kValido(float k) {
  return std::isfinite(k) && k >= K_MIN && k <= K_MAX;
}
/**
 * Conversión nominal de DFRobot y factor K. mv está en milivoltios y la salida
 * en uS/cm; no normaliza temperatura. El llamador debe validar señal/calibración.
 */
inline float ecUsCm(float mv, float k = 1.0f) {
  return 100000.0f * mv / 820.0f / 200.0f * k;
}
/**
 * Transforma voltaje del circuito PT1000 en resistencia y temperatura nominal.
 * No aplicar directamente a mV: el argumento v está en voltios.
 */
inline float temperaturaC(float v) {
  const float resistencia = (v / (30.0f / 1.8f) + 0.223f) / 0.000124f / 2.0f;
  return (resistencia - 1000.0f) / 3.85f;
}
inline float temperaturaAjustadaC(float nominal, const Registro &r) {
  return r.flags & PT_CALIBRADA ? nominal * r.ptGanancia + r.ptOffsetC : nominal;
}
inline float ec25UsCm(float mv, float t, float k) {
  return ecUsCm(mv, k) / (1.0f + 0.02f * (t - 25.0f));
}
inline bool calcularK(float mv, float patron, float &k) {
  if (!std::isfinite(mv) || mv <= 0 || !std::isfinite(patron) || patron <= 0)
    return false;
  const float candidato = patron / ecUsCm(mv);
  if (!kValido(candidato)) return false;
  k = candidato;
  return true;
}
inline bool alimentacionValida(float v) {
  return std::isfinite(v) && v >= 3.3f && v <= 5.0f;
}
inline bool hardwareCompatible(float vEc, float vPt, float desfaseGnd) {
  // Solo admite la referencia comun de este programa; no compensa el colector.
  return alimentacionValida(vEc) && alimentacionValida(vPt) && std::isfinite(desfaseGnd) &&
         desfaseGnd == 0.0f;
}
inline bool tensionSenalValida(float v) {
  return std::isfinite(v) && v >= 0 && v <= Configuracion::EC_PT_SENAL_MAX_V;
}
inline bool temperaturaValida(float t) {
  return std::isfinite(t) && t >= Configuracion::PT_MIN_C && t <= Configuracion::PT_MAX_C;
}
/**
 * Obtiene ganancia y offset de dos parejas nominal/referencia en C.
 * Los parámetros de salida solo se escriben si el ajuste supera controles.
 */
inline bool parametrosPT(float n1, float r1, float n2, float r2, float &ganancia, float &offset) {
  // Separacion de 1 C: control numerico del ajuste, no criterio metrologico.
  if (!std::isfinite(n1) || !std::isfinite(n2) || !temperaturaValida(r1) ||
      !temperaturaValida(r2) || std::fabs(n2 - n1) < 1 || std::fabs(r2 - r1) < 1)
    return false;
  const float g = (r2 - r1) / (n2 - n1), o = r1 - g * n1;
  if (!std::isfinite(g) || !std::isfinite(o) || g <= 0) return false;
  ganancia = g; offset = o;
  return true;
}
inline bool proponerPT(Registro &r, float n1, float r1, float n2, float r2) {
  float ganancia, offset;
  if (!parametrosPT(n1, r1, n2, r2, ganancia, offset)) return false;
  r.ptGanancia = ganancia; r.ptOffsetC = offset;
  r.ptNominal1C = n1; r.ptReferencia1C = r1;
  r.ptNominal2C = n2; r.ptReferencia2C = r2;
  r.flags |= PT_CALIBRADA | PT_DOS_PUNTOS;
  return true;
}
inline bool numero(const char *s, float &valor) {
  if (!s || !*s) return false;
  char *fin = nullptr;
  errno = 0;
  const float v = std::strtof(s, &fin);
  if (fin == s || *fin != '\0' || errno == ERANGE || !std::isfinite(v)) return false;
  valor = v;
  return true;
}
inline uint32_t crc32(const void *datos, size_t longitud) {
  const auto *p = static_cast<const uint8_t *>(datos);
  uint32_t crc = 0xffffffff;
  for (size_t i = 0; i < longitud; ++i) {
    crc ^= p[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
inline void sellar(Registro &r) { r.crc = crc32(&r, offsetof(Registro, crc)); }
/**
 * Comprueba identidad/CRC y coherencia de banderas y valores de calibración.
 * Que el blob sea válido no garantiza que el sensor o su señal actual estén bien.
 */
inline bool registroValido(const Registro &r) {
  if (r.marca != MARCA || r.version != VERSION || (r.flags & ~31u) ||
      !kValido(r.k) || r.crc != crc32(&r, offsetof(Registro, crc))) return false;
  if ((r.flags & BANCO_REFERENCIA_DECLARADA) && !(r.flags & EC_CALIBRADA)) return false;
  if ((r.flags & PT_DOS_PUNTOS) && !(r.flags & PT_CALIBRADA)) return false;
  if (r.flags & PT_CALIBRADA) {
    float g, o;
    if (!(r.flags & PT_DOS_PUNTOS) ||
        !parametrosPT(r.ptNominal1C, r.ptReferencia1C, r.ptNominal2C, r.ptReferencia2C, g, o) ||
        !std::isfinite(r.ptGanancia) || !std::isfinite(r.ptOffsetC) ||
        std::fabs(g - r.ptGanancia) > 0.000001f || std::fabs(o - r.ptOffsetC) > 0.0001f)
      return false;
  }
  if ((r.flags & EC_CALIBRADA) &&
      (!std::isfinite(r.patronUsCm) || r.patronUsCm <= 0 ||
       !std::isfinite(r.ecCalMv) || r.ecCalMv <= 0 ||
       !temperaturaValida(r.temperaturaCalC))) return false;
  if ((r.flags & TEMP_COMPARADA) &&
      (!temperaturaValida(r.temperaturaReferenciaC) ||
       !temperaturaValida(r.temperaturaComparadaC))) return false;
  return true;
}
inline bool migrarV2(const RegistroV2 &v, Registro &r) {
  if (v.marca != MARCA || v.version != 2 || (v.flags & ~7u) ||
      v.crc != crc32(&v, offsetof(RegistroV2, crc))) return false;
  Registro candidato;
  candidato.flags = v.flags; candidato.k = v.k;
  candidato.patronUsCm = v.patronUsCm; candidato.ecCalMv = v.ecCalMv;
  candidato.temperaturaCalC = v.temperaturaCalC;
  candidato.temperaturaReferenciaC = v.temperaturaReferenciaC;
  candidato.temperaturaComparadaC = v.temperaturaComparadaC;
  sellar(candidato);
  if (!registroValido(candidato)) return false;
  r = candidato;
  return true;
}
}  // namespace ModeloEC
