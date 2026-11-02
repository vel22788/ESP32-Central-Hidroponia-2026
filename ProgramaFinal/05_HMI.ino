/**
 * @file 05_HMI.ino
 * Adaptador pequeño entre el coordinador y ControlHMI.cpp.
 * Registra callbacks de adquisición/alimentación y atiende el protocolo desde
 * loop(). Las funciones de los sensores y relés conservan la propiedad del
 * hardware; la pantalla solicita operaciones mediante mensajes ASCII V2.
 */
/* Adaptador del protocolo ASCII V2. El principal conserva adquisicion y tiempos. */
#include "src/ControlHMI.h"
#include "src/Configuracion.h"
/**
 * Registra funciones del coordinador como callbacks. El callback de pH
 * solo garantiza reposo; una orden externa no puede saltarse el módulo 01.
 */
void iniciarHMI() { ControlHMI::iniciar(establecerAdquisicion, establecerAlimentacionPruebaPH); }
/**
 * Se llama en cada vuelta: atiende latidos, recepción, agenda y salida.
 * No contiene un segundo loop ni un programa separado para otra placa.
 */
void actualizarHMI() { ControlHMI::actualizar(); }
