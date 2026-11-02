/**
 * @file RelojRTC.cpp
 * Acceso I2C al DS3231, validación BCD/calendario y ajuste explícito.
 * Se usa UTC0 para convertir el calendario numéricamente sin restarle otras
 * seis horas: el chip ya contiene la hora local. ZONA_LOCAL es solo la etiqueta.
 * OSF (oscilador detenido) invalida la hora y requiere ajuste; no se ignora.
 * Los errores I2C y datos antiguos se publican como inválidos, nunca como ahora.
 */
#include "RelojRTC.h"
#include "Configuracion.h"
#include <Wire.h>
#include <cstring>

namespace RelojRTC {
namespace {
constexpr uint8_t DIRECCION = Configuracion::RTC_DIRECCION;
constexpr uint32_t PERIODO_MS = Configuracion::RTC_PERIODO_MS, CADUCIDAD_MS = Configuracion::RTC_CADUCIDAD_MS;
Datos datos;
uint32_t ultimaConsulta = 0;

/**
 * Lee un bloque del chip con repeated start; exige la cantidad completa.
 * Datos incompletos se descartan en vez de construir una fecha parcial.
 */
bool leer(uint8_t registro, uint8_t *salida, uint8_t cantidad) {
  Wire.beginTransmission(DIRECCION);
  Wire.write(registro);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(DIRECCION, cantidad) != cantidad) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < cantidad; ++i) {
    if (!Wire.available()) return false;
    const int v = Wire.read();
    if (v < 0) return false;
    salida[i] = uint8_t(v);
  }
  return true;
}
bool escribir(uint8_t registro, const uint8_t *bytes, uint8_t cantidad) {
  Wire.beginTransmission(DIRECCION);
  if (Wire.write(registro) != 1 || Wire.write(bytes, cantidad) != cantidad) return false;
  return Wire.endTransmission() == 0;
}
bool bcd(uint8_t entrada, unsigned maximo, unsigned &valor) {
  if ((entrada & 15) > 9 || (entrada >> 4) > 9) return false;
  valor = (entrada >> 4) * 10 + (entrada & 15);
  return valor <= maximo;
}
uint8_t aBCD(unsigned valor) { return uint8_t((valor / 10) * 16 + valor % 10); }
void fallo(const char *motivo) {
  datos.valido = false;
  datos.estado = motivo;
  datos.instante = millis();
}
/**
 * Lee calendario y estado; OSF invalida hora. Verifica BCD, bits reservados,
 * modo 12/24 h y fecha real antes de publicar una nueva muestra del RTC.
 */
void consultar() {
  ultimaConsulta = millis();
  uint8_t r[7], estado;
  if (!leer(0, r, 7) || !leer(0x0F, &estado, 1)) { fallo("RTC_NO_RESPONDE"); return; }
  if (estado & 0x80) { fallo("RTC_REQUIERE_AJUSTE"); return; } // OSF
  // Siglo 2100 y bits reservados no se aceptan como calendario 2000-2099.
  if ((r[0] & 0x80) || (r[1] & 0x80) || (r[2] & 0x80) ||
      (r[3] & 0xF8) || r[3] < 1 || r[3] > 7 || (r[4] & 0xC0) || (r[5] & 0xE0)) {
    fallo("RTC_CALENDARIO_INVALIDO"); return;
  }
  unsigned s, m, h, d, mes, y;
  bool ok = bcd(r[0], 59, s) && bcd(r[1], 59, m) && bcd(r[4], 31, d) &&
    bcd(r[5], 12, mes) && bcd(r[6], 99, y);
  if (r[2] & 0x40) {
    ok = ok && bcd(r[2] & 0x1F, 12, h) && h >= 1;
    if (ok) h = h % 12 + ((r[2] & 0x20) ? 12 : 0);
  } else ok = ok && bcd(r[2] & 0x3F, 23, h);
  if (!ok) { fallo("RTC_CALENDARIO_INVALIDO"); return; }
  char fecha[11], hora[9];
  snprintf(fecha, sizeof(fecha), "%04u-%02u-%02u", 2000 + y, mes, d);
  snprintf(hora, sizeof(hora), "%02u:%02u:%02u", h, m, s);
  time_t valor;
  if (!calendarioValido(fecha, hora, valor)) { fallo("RTC_CALENDARIO_INVALIDO"); return; }
  datos.valido = true; datos.calendario = valor;
  datos.instante = millis(); datos.estado = "OK";
}
}

/**
 * Valida AAAA-MM-DD y HH:MM:SS entre 2000 y 2099, incluido febrero bisiesto.
 * Devuelve el contador del calendario local mediante el parámetro valor;
 * no está convirtiendo esa hora local a un instante UTC de internet.
 */
bool calendarioValido(const String &fecha, const String &hora, time_t &valor) {
  if (fecha.length() != 10 || hora.length() != 8 || fecha[4] != '-' || fecha[7] != '-' ||
      hora[2] != ':' || hora[5] != ':') return false;
  for (unsigned i = 0; i < 10; ++i)
    if (i != 4 && i != 7 && (fecha[i] < '0' || fecha[i] > '9')) return false;
  for (unsigned i = 0; i < 8; ++i)
    if (i != 2 && i != 5 && (hora[i] < '0' || hora[i] > '9')) return false;
  int y, mes, d, h, m, s;
  if (sscanf(fecha.c_str(), "%d-%d-%d", &y, &mes, &d) != 3 ||
      sscanf(hora.c_str(), "%d:%d:%d", &h, &m, &s) != 3 || y < 2000 || y > 2099 ||
      mes < 1 || mes > 12 || d < 1 || h > 23 || m > 59 || s > 59) return false;
  const int dias[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (d > dias[mes - 1] + (mes == 2 && y % 4 == 0)) return false;
  struct tm t = {};
  t.tm_year = y - 1900; t.tm_mon = mes - 1; t.tm_mday = d;
  t.tm_hour = h; t.tm_min = m; t.tm_sec = s; t.tm_isdst = 0;
  valor = mktime(&t); // TZ=UTC0: contador del calendario local, sin aplicar otro -6.
  return valor != time_t(-1);
}

void iniciar() {
  setenv("TZ", "UTC0", 1); tzset();
  datos = Datos{};
  consultar();
}
void actualizar() {
  if (uint32_t(millis() - ultimaConsulta) >= PERIODO_MS) consultar();
}
/**
 * Devuelve copia; si supera CADUCIDAD_MS la marca antigua/inválida.
 * No realiza una transacción I2C adicional.
 */
Datos obtener() {
  Datos copia = datos;
  if (uint32_t(millis() - copia.instante) > CADUCIDAD_MS) {
    copia.valido = false; copia.estado = "RTC_ANTIGUO";
  }
  return copia;
}
/**
 * Extrapola segundos desde la última lectura válida y añade ZONA_LOCAL.
 * Si no hay reloj utilizable devuelve NA, sin tomar la hora de compilación.
 */
String fechaActual() {
  const Datos copia = obtener();
  if (!copia.valido) return "NA";
  const time_t local = copia.calendario + uint32_t(millis() - copia.instante) / 1000;
  struct tm t;
  if (!gmtime_r(&local, &t)) return "NA";
  char fecha[26];
  strftime(fecha, sizeof(fecha), "%Y-%m-%dT%H:%M:%S", &t);
  return String(fecha) + Configuracion::ZONA_LOCAL;
}
/**
 * Ajuste explícito: valida fecha/hora, escribe registros, habilita oscilador
 * y limpia OSF después de escribir. Relee y confirma el calendario con margen
 * de un segundo. El protocolo exige reposo/MANUAL antes de invocarlo.
 */
bool ajustar(const String &fecha, const String &hora) {
  time_t valor;
  if (!calendarioValido(fecha, hora, valor)) return false;
  uint8_t control, estado;
  if (!leer(0x0E, &control, 1) || !leer(0x0F, &estado, 1)) { fallo("RTC_NO_RESPONDE"); return false; }
  struct tm t;
  if (!gmtime_r(&valor, &t)) return false;
  const uint8_t r[] = {aBCD(t.tm_sec), aBCD(t.tm_min), aBCD(t.tm_hour),
    uint8_t(t.tm_wday + 1), aBCD(t.tm_mday), aBCD(t.tm_mon + 1), aBCD(t.tm_year - 100)};
  control &= 0x7F; // EOSC=0: conservar oscilador durante respaldo.
  if (!escribir(0x0E, &control, 1) || !escribir(0, r, 7)) { fallo("RTC_ERROR_AJUSTE"); return false; }
  estado &= 0x7F; // Limpiar OSF solo tras un ajuste explicito completo.
  if (!escribir(0x0F, &estado, 1)) { fallo("RTC_ERROR_AJUSTE"); return false; }
  consultar();
  return datos.valido && datos.calendario >= valor && datos.calendario <= valor + 1;
}
}
