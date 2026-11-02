/**
 * @file PinesSensores.h
 * Alias de los mandos de alimentación de pH y EC tomados de Configuracion.h.
 * No son entradas analógicas: las señales de medida entran en el ADS1115.
 * PT1000 es permanente; EN de motores queda separado de estos mandos.
 */
#pragma once
#include <stdint.h>
#include "Configuracion.h"

// PCB actual: correccion de cableado declarada por el usuario el 2026-10-22.
// EN de motores permanece a GND, separado de GPIO21.
namespace PinesSensores {
constexpr uint8_t PH = Configuracion::PH_GPIO;
constexpr uint8_t EC = Configuracion::EC_GPIO;
}
