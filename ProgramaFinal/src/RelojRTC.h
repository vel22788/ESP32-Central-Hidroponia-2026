/**
 * @file RelojRTC.h
 * Contrato del DS3231. Datos::calendario representa el calendario local guardado;
 * instante es millis() de su lectura. valido/estado indican si puede utilizarse.
 * obtener() devuelve una copia con control de antigüedad. Ajustar fecha requiere
 * una petición explícita; compilar o reiniciar no establece una hora inventada.
 */
#pragma once
#include <Arduino.h>
#include <time.h>

// El calendario del DS3231 se guarda en hora local de Guatemala, UTC-06:00.
// No se ajusta automaticamente al compilar ni se usa una hora ficticia.
namespace RelojRTC {
struct Datos {
  bool valido = false;
  time_t calendario = 0;
  uint32_t instante = 0;
  const char *estado = "SIN_DATOS";
};
void iniciar();
void actualizar();
Datos obtener();
String fechaActual();
/**
 * Entradas: fecha AAAA-MM-DD y hora HH:MM:SS locales. true significa escritura
 * y comprobación correctas, no una sincronización automática por red.
 */
bool ajustar(const String &fecha, const String &hora);
bool calendarioValido(const String &fecha, const String &hora, time_t &valor);
}
