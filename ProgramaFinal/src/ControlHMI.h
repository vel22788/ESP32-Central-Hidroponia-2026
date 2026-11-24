#pragma once
#include <Arduino.h>

namespace ControlHMI {
struct Lectura {
  double valor[5] = {};
  uint8_t calidad[5] = {};
  uint32_t instante[5] = {};
  String fecha[5] = {"NA", "NA", "NA", "NA", "NA"};
};
void iniciar(void (*adquisicion)(bool), void (*alimentacionPH)(bool) = nullptr);
void actualizar();
int obtenerUmbral();
void publicar(const Lectura &lectura);
void procesarLinea(String linea, bool deHmi);
}
