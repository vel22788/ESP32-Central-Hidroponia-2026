#pragma once
#include <Arduino.h>
#include <time.h>

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
bool ajustar(const String &fecha, const String &hora);
bool calendarioValido(const String &fecha, const String &hora, time_t &valor);
}
