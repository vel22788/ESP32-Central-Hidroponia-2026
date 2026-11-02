/**
 * @file ControlHMI.h
 * Interfaz pública de comunicación ASCII V2 entre central, pantalla y puente PC.
 * La lectura ordena PH, CE, TAMB, HAMB y TSOL; cada campo tiene calidad y edad.
 * iniciar() recibe callbacks del coordinador; actualizar() atiende enlaces y
 * agenda; publicar() recibe mediciones reales, no órdenes de actuación.
 * La lógica de interbloqueo pertenece a Proceso, no al dibujo de la pantalla.
 */
#pragma once
#include <Arduino.h>

namespace ControlHMI {
// PH, CE sin compensar[mS/cm], TAMB[C], HAMB[%], TSOL[C]. Calidad: 0 sin datos, 1 OK, 2 error, 3 provisional.
// Cada campo conserva su propio instante: otra lectura no rejuvenece ese dato.
struct Lectura {
  double valor[5] = {};
  uint8_t calidad[5] = {};
  uint32_t instante[5] = {};
  String fecha[5] = {"NA", "NA", "NA", "NA", "NA"};
};
/**
 * Inicializa UART, callbacks y agenda; llamar una vez después del hardware.
 * El callback opcional de alimentación no concede acceso directo a las sondas.
 */
void iniciar(void (*adquisicion)(bool), void (*alimentacionPH)(bool) = nullptr);
void actualizar();
int obtenerUmbral();
/**
 * Copia valores/calidades y añade una muestra al histórico RAM. No cambia
 * el instante de campos antiguos: quien adquiere entrega la edad correcta.
 */
void publicar(const Lectura &lectura);
void procesarLinea(String linea, bool deHmi);
}
