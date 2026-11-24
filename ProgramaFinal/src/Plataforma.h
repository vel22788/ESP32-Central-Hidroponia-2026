#pragma once
#include <Arduino.h>
#include <cmath>
#include <climits>

enum class AccionMovimiento : uint8_t { NINGUNA, SUBIR, BAJAR, REFERENCIAR, NIVELAR, INCLINAR };
struct DatosMotor {
  bool configurado = false, arriba = false, abajo = false, moviendo = false, subiendo = false,
       referenciado = false;
  int rawArriba = 0, rawAbajo = 0, activo = 0, dirSubir = 0;
  int32_t posicion = 0;
  uint32_t pasos = 0;
};
struct DatosMovimiento {
  bool ocupado = false, posicionesConfiguradas = false;
  const char *estado = "DETENIDO", *motivo = "ARRANQUE", *fase = "REPOSO";
  AccionMovimiento accion = AccionMovimiento::NINGUNA;
  int32_t bajada = 0, inclinacion = 0;
};
void prepararSalidasMotores();
void iniciarMotores(uint32_t, uint32_t, uint32_t);
void actualizarMotores();
const char *configurarMotor(uint8_t, int, int);
const char *configurarPosicionesModulo(uint8_t, int32_t, int32_t);
const char *solicitarMovimiento(uint8_t, AccionMovimiento);
const char *moverMotor(uint8_t, bool, uint32_t);
void detenerMotor(uint8_t);
void detenerModulo(uint8_t);
void detenerTodosLosMotores();
bool motoresOcupados();
DatosMotor obtenerMotor(uint8_t);
DatosMovimiento obtenerMovimiento(uint8_t);
uint32_t revisionMotores();

struct DatosSensoresADS {
  uint32_t muestra = 0, momentoMs = 0, momentoEC = 0, momentoPH = 0;
  const char *estado = "SIN_DATOS";
  bool valida = false, fueraIntervaloPT = false;
  float temperaturaC = NAN, ptNominalC = NAN, ecUsCm = NAN, ecMv = NAN, ptV = NAN;
  float ph = NAN, phMv = NAN;
  bool phValido = false, phProvisional = false, ptValida = false, ecValida = false;
  const char *estadoPH = "SIN_DATOS";
  const char *estadoPT = "SIN_DATOS";
};
void iniciarSensoresADS();
bool solicitarLecturaSensoresADS(uint32_t);
void actualizarSensoresADS();
void detenerSensoresADS();
bool sensoresADSOcupados();
DatosSensoresADS obtenerSensoresADS();
const char *estadoCalibracionADS();
uint32_t flagsCalibracionADS();
float factorEC();
float gananciaPT();
float offsetPT();
float leerTemperaturaSolucion();
float leerEC();
float leerPH();
void establecerAdquisicion(bool);
void establecerAlimentacionPruebaPH(bool);

namespace Proceso {
void iniciar();
void actualizar();
void despuesMotores();
const char *regar(unsigned modulo, unsigned metodo, bool desdeHmi, bool automatico = false);
const char *mover(unsigned modulo, AccionMovimiento, bool desdeHmi);
void parar(const char *motivo = "PARADA", bool salirAuto = true);
void pararModulo(unsigned);
void perderEnlace(bool desdeHmi);
bool ocupado();
bool riegoActivo();
int nivel(unsigned);
const char *movimiento(unsigned);
const char *riego(unsigned);
const char *alarma();
const char *regla();
const char *fase();
const char *reconocer();
const char *modo(bool automatico, bool agendaDisponible = false);
void reservarAgenda(bool reservar);
const char *evento(unsigned modulo, unsigned metodo, uint32_t duracionMs);
void bloquear(const char *motivo);
bool automatico();
const char *configurarRiego(uint32_t lluvia, uint32_t nft, uint32_t llenado, uint32_t drenaje);
const char *configurarHumedad(unsigned metodo, unsigned destino, uint32_t confirmar, uint32_t rearme,
                              float histeresis);
void humedad(float valor, bool valida, uint32_t instante, int umbral);
}
