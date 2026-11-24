#include "src/ControlHMI.h"
#include "src/Configuracion.h"
// Conecta el servicio de control con las funciones del coordinador que habilitan la adquisición y apagan las sondas.
void iniciarHMI() {
  ControlHMI::iniciar(establecerAdquisicion, establecerAlimentacionPruebaPH);
}

// Atiende las órdenes pendientes y el programador horario del central.
void actualizarHMI() {
  ControlHMI::actualizar();
}
