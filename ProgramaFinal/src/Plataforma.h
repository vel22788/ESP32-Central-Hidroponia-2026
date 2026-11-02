/**
 * @file Plataforma.h
 * Contratos compartidos por los seis .ino y el protocolo: tipos y declaraciones.
 * No controla hardware por sí mismo; las implementaciones están en los módulos.
 * API de movimiento: motor 1..4, módulo 1..2. Las funciones de Proceso que usan
 * índices para presentación se documentan donde se implementan.
 * Las estructuras se devuelven por valor: consultarlas no inicia movimientos.
 * NAN y los indicadores de validez permiten distinguir fallo de un cero real.
 */
#pragma once
#include <Arduino.h>
#include <cmath>
#include <climits>

/**
 * Acciones solicitadas al controlador. REFERENCIAR/NIVELAR conservan cero
 * al confirmar superiores; INCLINAR utiliza finales y no un ángulo calculado.
 */
enum class AccionMovimiento : uint8_t { NINGUNA, SUBIR, BAJAR, REFERENCIAR, NIVELAR, INCLINAR };
/**
 * posicion/pasos son contadores de pulsos, no realimentación de posición.
 * rawArriba/rawAbajo conservan niveles eléctricos; arriba/abajo aplican polaridad.
 */
struct DatosMotor {
  bool configurado=false, arriba=false, abajo=false, moviendo=false, subiendo=false, referenciado=false;
  int rawArriba=0, rawAbajo=0, activo=0, dirSubir=0;
  int32_t posicion=0;
  uint32_t pasos=0;
};
struct DatosMovimiento {
  bool ocupado=false, posicionesConfiguradas=false;
  const char *estado="DETENIDO", *motivo="ARRANQUE", *fase="REPOSO";
  AccionMovimiento accion=AccionMovimiento::NINGUNA;
  int32_t bajada=0, inclinacion=0;
};
/**
 * API de movimiento asíncrono. Las solicitudes devuelven códigos de texto
 * (ACEPTADO, OCUPADO, error...). Hay que seguir llamando actualizarMotores()
 * y consultar obtenerMovimiento() para conocer el final y su motivo.
 */
void prepararSalidasMotores();
void iniciarMotores(uint32_t,uint32_t,uint32_t);
void actualizarMotores();
const char *configurarMotor(uint8_t,int,int);
const char *configurarPosicionesModulo(uint8_t,int32_t,int32_t);
const char *solicitarMovimiento(uint8_t,AccionMovimiento);
const char *moverMotor(uint8_t,bool,uint32_t);
void detenerMotor(uint8_t);
void detenerModulo(uint8_t);
void detenerTodosLosMotores();
bool motoresOcupados();
DatosMotor obtenerMotor(uint8_t);
DatosMovimiento obtenerMovimiento(uint8_t);
uint32_t revisionMotores();

/**
 * Unidades: temperaturaC/ptNominalC en C; ecUsCm en uS/cm; ecMv/phMv en mV;
 * ptV en V. muestra identifica publicación y momentoEC/momentoPH sus ventanas.
 * valida se conserva como validez EC; consultar ecValida, ptValida y phValido
 * por separado para no atribuir a todas las magnitudes la misma calidad.
 */
struct DatosSensoresADS {
  uint32_t muestra=0, momentoMs=0, momentoEC=0, momentoPH=0;
  const char *estado="SIN_DATOS";
  bool valida=false, fueraIntervaloPT=false;
  float temperaturaC=NAN, ptNominalC=NAN, ecUsCm=NAN, ecMv=NAN, ptV=NAN;
  float ph=NAN, phMv=NAN;
  bool phValido=false, phProvisional=false, ptValida=false, ecValida=false;
  const char *estadoPH="SIN_DATOS";
  const char *estadoPT="SIN_DATOS";
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

/**
 * Interfaz del proceso. Regar/mover/evento solicitan operaciones; actualizar
 * las ejecuta por etapas. nivel() usa índices 0 principal, 1 M1 y 2 M2.
 * El booleano desdeHmi identifica el dueño de la orden para supervisar enlace.
 */
namespace Proceso {
void iniciar();
void actualizar();
void despuesMotores();
const char *regar(unsigned modulo,unsigned metodo,bool desdeHmi,bool automatico=false);
const char *mover(unsigned modulo,AccionMovimiento,bool desdeHmi);
void parar(const char *motivo="PARADA",bool salirAuto=true);
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
const char *modo(bool automatico,bool agendaDisponible=false);
void reservarAgenda(bool reservar);
const char *evento(unsigned modulo,unsigned metodo,uint32_t duracionMs);
void bloquear(const char *motivo);
bool automatico();
const char *configurarRiego(uint32_t lluvia,uint32_t nft,uint32_t llenado,uint32_t drenaje);
const char *configurarHumedad(unsigned metodo,unsigned destino,uint32_t confirmar,uint32_t rearme,float histeresis);
void humedad(float valor,bool valida,uint32_t instante,int umbral);
}
