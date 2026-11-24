#include <Arduino.h>
#include "src/Configuracion.h"
#include <driver/gpio.h>
#include <climits>

namespace Motores {
class Control {
  enum class Fase : uint8_t { REPOSO, MANUAL, REFERENCIA, POSICION, FINALES };
  struct Eje {
    bool configurado = false, moviendo = false, subir = false, referenciado = false;
    bool esperandoFinal = false;
    int activo = Configuracion::FINAL_ACTIVO, dirSubir = LOW;
    int raw[2] = {0, 0};
    uint32_t cambio[2] = {0, 0}, pasos = 0, ultimoPaso = 0;
    int32_t posicion = 0, destino = 0;
  };
  struct Pareja {
    Fase fase = Fase::REPOSO;
    AccionMovimiento accion = AccionMovimiento::NINGUNA;
    const char *estado = "DETENIDO", *motivo = "ARRANQUE";
    bool posiciones = false, prueba = false;
    uint8_t ejePrueba = 0;
    uint32_t inicio = 0, restantes = 0, ultimoPaso = 0;
    int32_t bajada = 0, inclinacion = 0;
  };
  const uint8_t (&dir)[4] = Configuracion::MOTOR_DIR;
  const uint8_t (&step)[4] = Configuracion::MOTOR_STEP;
  const uint8_t (&finales)[4][2] = Configuracion::MOTOR_FINALES;
  Eje e[4];
  Pareja p[2];
  uint32_t intervalo = Configuracion::PASO_US, timeout = Configuracion::MOTOR_MAX_MS,
           estabilidad = Configuracion::FILTRO_FINALES_MS, revision = 0;
  bool listo = false;

  // Comprueba que el eje tenga asignados los dos finales de carrera.
  bool tieneFinales(uint8_t i) const {
    return finales[i][0] != Configuracion::FINAL_SIN_GPIO && finales[i][1] != Configuracion::FINAL_SIN_GPIO;
  }

  // Interpreta el nivel eléctrico de un final según la polaridad configurada para ese eje.
  bool activo(uint8_t i, uint8_t lado) const {
    return e[i].configurado && e[i].raw[lado] == e[i].activo;
  }

  // Comprueba que ambos finales llevan el tiempo de filtrado sin cambiar.
  bool estable(uint8_t i) const {
    return uint32_t(millis() - e[i].cambio[0]) >= estabilidad &&
           uint32_t(millis() - e[i].cambio[1]) >= estabilidad;
  }

  // Detecta si los dos finales opuestos de un eje se declaran activos a la vez.
  bool contradictorio(uint8_t i) const {
    return activo(i, 0) && activo(i, 1);
  }

  // Indica si ambos ejes del módulo tienen una referencia superior confirmada.
  bool referencia(uint8_t m) const {
    return e[m * 2].referenciado && e[m * 2 + 1].referenciado;
  }

  // Lee los finales de los cuatro ejes y registra cuándo cambia cada contacto.
  void leer() {
    for (uint8_t i = 0; i < 4; ++i) {
      if (!tieneFinales(i))
        continue;
      for (uint8_t lado = 0; lado < 2; ++lado) {
        const int valor = digitalRead(finales[i][lado]);
        if (valor != e[i].raw[lado]) {
          e[i].raw[lado] = valor;
          e[i].cambio[lado] = millis();
        }
      }
    }
  }

  // Devuelve el motivo que impide arrancar un eje o un puntero nulo si está listo.
  const char *comprobar(uint8_t i) const {
    if (!listo)
      return "MOTORES_NO_INICIADOS";
    if (!tieneFinales(i))
      return "MOTORES_DESHABILITADOS";
    if (!e[i].configurado)
      return "CONFIG_MOTOR_PENDIENTE";
    if (contradictorio(i))
      return "FINALES_CONTRADICTORIOS";
    if (!estable(i))
      return "FINALES_INESTABLES";
    return nullptr;
  }

  // Detiene los pulsos del eje indicado y deja STEP en bajo.
  void pararEje(uint8_t i) {
    e[i].moviendo = false;
    digitalWrite(step[i], LOW);
  }

  // Detiene la pareja, guarda la causa y descarta la referencia si hubo fallo o se interrumpió su búsqueda.
  void finalizar(uint8_t m, const char *motivo, bool fallo = false) {
    const bool referenciando = p[m].fase == Fase::REFERENCIA;
    for (uint8_t i = m * 2; i < m * 2 + 2; ++i) {
      pararEje(i);
      e[i].esperandoFinal = false;
      if (fallo || referenciando)
        e[i].referenciado = false;
    }
    p[m].fase = Fase::REPOSO;
    p[m].prueba = false;
    p[m].estado = fallo ? "FALLO" : "DETENIDO";
    p[m].motivo = motivo;
    ++revision;
  }

  // Fija DIR para subir o bajar y prepara el eje para su próximo pulso.
  void orientar(uint8_t i, bool subir) {
    e[i].subir = subir;
    e[i].moviendo = true;
    digitalWrite(dir[i], subir ? e[i].dirSubir : !e[i].dirSubir);
    e[i].ultimoPaso = micros();
  }

  // Orienta cada eje hacia su final superior para establecer el cero del módulo.
  void prepararReferencia(uint8_t m) {
    p[m].fase = Fase::REFERENCIA;
    for (uint8_t i = m * 2; i < m * 2 + 2; ++i) {
      e[i].referenciado = false;
      e[i].esperandoFinal = activo(i, 0);
      if (e[i].esperandoFinal)
        pararEje(i);
      else
        orientar(i, true);
    }
    p[m].ultimoPaso = micros();
  }

  // Calcula los destinos en pasos y orienta los ejes que aún tienen recorrido pendiente.
  bool prepararPosicion(uint8_t m) {
    p[m].fase = Fase::POSICION;
    bool alguno = false;
    for (uint8_t i = m * 2; i < m * 2 + 2; ++i) {
      e[i].destino =
          p[m].bajada + ((i % 2 && p[m].accion == AccionMovimiento::INCLINAR) ? p[m].inclinacion : 0);
      if (e[i].posicion == e[i].destino) {
        pararEje(i);
        continue;
      }
      const bool subir = e[i].posicion > e[i].destino;
      if (activo(i, subir ? 0 : 1)) {
        finalizar(m, "LIMITE_ANTES_DEL_DESTINO", true);
        return false;
      }
      orientar(i, subir);
      alguno = true;
    }
    p[m].ultimoPaso = micros();
    if (!alguno)
      finalizar(m, "POSICION_ALCANZADA");
    return true;
  }

  // Revisa límites, tiempos y destinos de la pareja y decide qué ejes deben detenerse.
  void comprobarPareja(uint8_t m) {
    Pareja &q = p[m];
    if (q.fase == Fase::REPOSO)
      return;
    for (uint8_t i = m * 2; i < m * 2 + 2; ++i)
      if (contradictorio(i)) {
        finalizar(m, "FINALES_CONTRADICTORIOS", true);
        return;
      }
    if (uint32_t(millis() - q.inicio) >= timeout) {
      finalizar(m, "TIMEOUT", true);
      return;
    }
    if (q.fase == Fase::REFERENCIA) {
      bool completos = true;
      for (uint8_t i = m * 2; i < m * 2 + 2; ++i) {
        if (activo(i, 0)) {
          pararEje(i);
          e[i].esperandoFinal = true;
          if (uint32_t(millis() - e[i].cambio[0]) >= estabilidad) {
            e[i].posicion = 0;
            e[i].referenciado = true;
          } else
            completos = false;
        } else {
          if (e[i].esperandoFinal) {
            finalizar(m, "FINAL_INESTABLE", true);
            return;
          }
          completos = false;
        }
      }
      if (completos) {
        if (q.accion == AccionMovimiento::REFERENCIAR || q.accion == AccionMovimiento::NIVELAR) {
          q.fase = Fase::POSICION;
          finalizar(m, "REFERENCIA_SUPERIOR");
        } else
          prepararPosicion(m);
        ++revision;
      }
      return;
    }
    if (q.fase == Fase::FINALES) {
      bool completos = true;
      for (uint8_t i = m * 2; i < m * 2 + 2; ++i) {
        const uint8_t lado = e[i].subir ? 0 : 1;
        if (activo(i, lado)) {
          pararEje(i);
          e[i].esperandoFinal = true;
          if (uint32_t(millis() - e[i].cambio[lado]) < estabilidad)
            completos = false;
        } else {
          if (e[i].esperandoFinal) {
            finalizar(m, "FINAL_INESTABLE", true);
            return;
          }
          completos = false;
        }
      }
      if (completos)
        finalizar(m, "INCLINACION_ALCANZADA");
      return;
    }
    if (q.prueba) {
      const uint8_t i = q.ejePrueba;
      if (activo(i, e[i].subir ? 0 : 1)) {
        finalizar(m, "LIMITE_ALCANZADO");
        return;
      }
      if (!q.restantes)
        finalizar(m, "PASOS_COMPLETADOS");
      return;
    }
    if (q.fase == Fase::MANUAL) {
      const uint8_t lado = q.accion == AccionMovimiento::SUBIR ? 0 : 1;
      if (activo(m * 2, lado) || activo(m * 2 + 1, lado)) {
        finalizar(m, activo(m * 2, lado) && activo(m * 2 + 1, lado) ? "LIMITES_ALCANZADOS" : "LIMITE_UN_LADO");
      }
      return;
    }
    bool quedan = false;
    for (uint8_t i = m * 2; i < m * 2 + 2; ++i) {
      if (!e[i].moviendo)
        continue;
      if (e[i].posicion == e[i].destino) {
        pararEje(i);
        continue;
      }
      if (activo(i, e[i].subir ? 0 : 1)) {
        finalizar(m, "LIMITE_ANTES_DEL_DESTINO", true);
        return;
      }
      quedan = true;
    }
    if (!quedan)
      finalizar(m, "POSICION_ALCANZADA");
  }

public:
  // Deja en bajo las salidas STEP y DIR y reinicia los estados de los cuatro ejes.
  void reposo() {
    for (uint8_t i = 0; i < 4; ++i) {
      gpio_set_level(static_cast<gpio_num_t>(step[i]), LOW);
      pinMode(step[i], OUTPUT);
      gpio_set_level(static_cast<gpio_num_t>(dir[i]), LOW);
      pinMode(dir[i], OUTPUT);
      e[i] = Eje{};
    }
    for (auto &q : p)
      q = Pareja{};
    listo = false;
    ++revision;
  }

  // Configura el intervalo de paso, el tiempo máximo y el filtro e inicializa las entradas de los finales.
  void iniciar(uint32_t intervaloUs, uint32_t timeoutMs, uint32_t estabilidadMs) {
    intervalo = intervaloUs >= Configuracion::PASO_MIN_US ? intervaloUs : Configuracion::PASO_MIN_US;
    timeout = timeoutMs ? timeoutMs : Configuracion::MOTOR_MAX_MS;
    estabilidad = estabilidadMs;
    const uint8_t modoFinal = Configuracion::FINAL_ACTIVO == LOW ? INPUT_PULLUP : INPUT_PULLDOWN;
    for (uint8_t i = 0; i < 4; ++i) {
      if (!tieneFinales(i))
        continue;
      for (uint8_t lado = 0; lado < 2; ++lado) {
        pinMode(finales[i][lado], modoFinal);
        e[i].raw[lado] = digitalRead(finales[i][lado]);
        e[i].cambio[lado] = millis();
      }
    }
    listo = true;
    ++revision;
  }

  // Asigna al motor la polaridad de sus finales y el nivel de DIR que corresponde a subir.
  const char *configurar(uint8_t numero, int nivelActivo, int nivelSubir) {
    if (!numero || numero > 4 || (nivelActivo != 0 && nivelActivo != 1) || (nivelSubir != 0 && nivelSubir != 1))
      return "PARAMETRO_INVALIDO";
    if (!listo)
      return "MOTORES_NO_INICIADOS";
    const uint8_t i = numero - 1, m = i / 2;
    if (!tieneFinales(i))
      return "MOTORES_DESHABILITADOS";
    if (p[m].fase != Fase::REPOSO)
      return "OCUPADO";
    e[i].activo = nivelActivo;
    e[i].dirSubir = nivelSubir;
    e[i].configurado = true;
    e[m * 2].referenciado = e[m * 2 + 1].referenciado = false;
    leer();
    ++revision;
    return "OK";
  }

  // Guarda las distancias de bajada e inclinación expresadas en pasos para el módulo.
  const char *posiciones(uint8_t modulo, int32_t bajada, int32_t inclinacion) {
    if (!modulo || modulo > 2 || bajada <= 0 || inclinacion < 0 || bajada > INT32_MAX - inclinacion)
      return "PARAMETRO_INVALIDO";
    if (!listo)
      return "MOTORES_NO_INICIADOS";
    Pareja &q = p[modulo - 1];
    if (q.fase != Fase::REPOSO)
      return "OCUPADO";
    q.bajada = bajada;
    q.inclinacion = inclinacion;
    q.posiciones = true;
    ++revision;
    return "OK";
  }

  // Valida la orden e inicia la maniobra de la pareja, ya sea manual, de referencia o de inclinación.
  const char *solicitar(uint8_t modulo, AccionMovimiento accion) {
    if (!modulo || modulo > 2 || accion == AccionMovimiento::NINGUNA ||
        static_cast<uint8_t>(accion) > static_cast<uint8_t>(AccionMovimiento::INCLINAR))
      return "PARAMETRO_INVALIDO";
    const uint8_t m = modulo - 1;
    Pareja &q = p[m];
    leer();
    if (q.fase != Fase::REPOSO)
      return !q.prueba && q.accion == accion ? "YA_EN_CURSO" : "OCUPADO";
    for (uint8_t i = m * 2; i < m * 2 + 2; ++i) {
      const char *error = comprobar(i);
      if (error)
        return error;
    }
    const bool posicion = false;
    if (posicion && !q.posiciones)
      return "POSICIONES_PENDIENTES";

    if (accion == AccionMovimiento::SUBIR || accion == AccionMovimiento::BAJAR) {
      const uint8_t lado = accion == AccionMovimiento::SUBIR ? 0 : 1;
      if (activo(m * 2, lado) || activo(m * 2 + 1, lado))
        return "LIMITE_ACTIVO";
    }
    q.accion = accion;
    q.prueba = false;
    q.inicio = millis();
    q.motivo = "ORDEN";
    q.estado = accion == AccionMovimiento::SUBIR      ? "SUBIENDO"
               : accion == AccionMovimiento::BAJAR    ? "BAJANDO"
               : accion == AccionMovimiento::NIVELAR  ? "NIVELANDO"
               : accion == AccionMovimiento::INCLINAR ? "INCLINANDO"
                                                      : "REFERENCIANDO";
    for (uint8_t i = m * 2; i < m * 2 + 2; ++i)
      e[i].pasos = 0;
    if (accion == AccionMovimiento::INCLINAR) {
      q.fase = Fase::FINALES;
      for (uint8_t i = m * 2; i < m * 2 + 2; ++i) {
        e[i].referenciado = false;
        const bool subir = (i % 2) == 1;
        e[i].subir = subir;
        e[i].esperandoFinal = activo(i, subir ? 0 : 1);
        if (e[i].esperandoFinal)
          pararEje(i);
        else
          orientar(i, subir);
      }
      q.ultimoPaso = micros();
    } else if (accion == AccionMovimiento::SUBIR || accion == AccionMovimiento::BAJAR) {
      q.fase = Fase::MANUAL;
      orientar(m * 2, accion == AccionMovimiento::SUBIR);
      orientar(m * 2 + 1, accion == AccionMovimiento::SUBIR);
      q.ultimoPaso = micros();
    } else if (accion == AccionMovimiento::NIVELAR || accion == AccionMovimiento::REFERENCIAR ||
               !referencia(m)) {
      prepararReferencia(m);
    } else
      prepararPosicion(m);
    ++revision;
    return "ACEPTADO";
  }

  // Inicia una prueba individual de un número limitado de pasos y exige una nueva referencia de la pareja.
  const char *probar(uint8_t numero, bool subir, uint32_t pasos) {
    if (!numero || numero > 4 || !pasos || pasos > INT32_MAX)
      return "PARAMETRO_INVALIDO";
    const uint8_t i = numero - 1, m = i / 2;
    leer();
    if (p[m].fase != Fase::REPOSO)
      return "OCUPADO";
    const char *error = comprobar(i);
    if (error)
      return error;
    if (activo(i, subir ? 0 : 1))
      return "LIMITE_ACTIVO";

    e[m * 2].referenciado = e[m * 2 + 1].referenciado = false;
    Pareja &q = p[m];
    q.fase = Fase::MANUAL;
    q.prueba = true;
    q.ejePrueba = i;
    q.accion = AccionMovimiento::NINGUNA;
    q.estado = "PROBANDO";
    q.motivo = "ORDEN_PRUEBA";
    q.inicio = millis();
    q.restantes = pasos;
    q.ultimoPaso = micros();
    e[i].pasos = 0;
    orientar(i, subir);
    ++revision;
    return "ACEPTADO";
  }

  // Revisa las protecciones y emite los pulsos STEP que corresponden a los ejes activos.
  void actualizar() {
    if (!listo)
      return;
    leer();
    for (uint8_t m = 0; m < 2; ++m)
      comprobarPareja(m);
    uint8_t mascara = 0;
    const uint32_t ahora = micros();
    for (uint8_t m = 0; m < 2; ++m) {
      if (p[m].fase == Fase::REPOSO || uint32_t(ahora - p[m].ultimoPaso) < intervalo)
        continue;

      for (uint8_t i = m * 2; i < m * 2 + 2; ++i)
        if (e[i].moviendo) {
          if (uint32_t(ahora - e[i].ultimoPaso) < intervalo)
            continue;
          if (e[i].referenciado && (e[i].subir ? e[i].posicion <= 0 : e[i].posicion == INT32_MAX)) {
            finalizar(m, "POSICION_FUERA_DE_RANGO", true);
            break;
          }
          mascara |= uint8_t(1u << i);
        }
      if (p[m].fase == Fase::REPOSO)
        mascara &= uint8_t(~(3u << (m * 2)));
    }
    if (!mascara)
      return;
    for (uint8_t i = 0; i < 4; ++i)
      if (mascara & (1u << i))
        digitalWrite(step[i], HIGH);
    delayMicroseconds(Configuracion::PULSO_STEP_US);
    for (uint8_t i = 0; i < 4; ++i)
      if (mascara & (1u << i)) {
        digitalWrite(step[i], LOW);
        e[i].ultimoPaso = ahora;
        ++e[i].pasos;
        if (e[i].referenciado)
          e[i].posicion += e[i].subir ? -1 : 1;
      }
    for (uint8_t m = 0; m < 2; ++m)
      if (mascara & (3u << (m * 2))) {
        p[m].ultimoPaso = ahora;
        if (p[m].prueba && p[m].restantes)
          --p[m].restantes;
        comprobarPareja(m);
      }
  }

  // Detiene por petición los dos ejes del módulo indicado.
  void parar(uint8_t modulo) {
    if (modulo && modulo <= 2)
      finalizar(modulo - 1, "PARADA_SOLICITADA");
  }

  // Detiene las dos parejas de motores.
  void pararTodos() {
    parar(1);
    parar(2);
  }

  // Indica si al menos una pareja está ejecutando una maniobra.
  bool ocupado() const {
    return p[0].fase != Fase::REPOSO || p[1].fase != Fase::REPOSO;
  }

  // Devuelve el contador de revisiones del estado de movimiento.
  uint32_t version() const {
    return revision;
  }

  // Devuelve el estado de un motor, incluidos sus finales, sentido y pasos emitidos.
  DatosMotor datos(uint8_t numero) const {
    DatosMotor d;
    if (!numero || numero > 4)
      return d;
    const uint8_t i = numero - 1;
    d.configurado = e[i].configurado;
    d.arriba = activo(i, 0);
    d.abajo = activo(i, 1);
    d.moviendo = e[i].moviendo;
    d.subiendo = e[i].subir;
    d.referenciado = e[i].referenciado;
    d.rawArriba = e[i].raw[0];
    d.rawAbajo = e[i].raw[1];
    d.activo = e[i].configurado ? e[i].activo : -1;
    d.dirSubir = e[i].configurado ? e[i].dirSubir : -1;
    d.posicion = e[i].posicion;
    d.pasos = e[i].pasos;
    return d;
  }

  // Devuelve la fase, acción y causa de parada del módulo solicitado.
  DatosMovimiento movimiento(uint8_t modulo) const {
    DatosMovimiento d;
    if (!modulo || modulo > 2)
      return d;
    const Pareja &q = p[modulo - 1];
    d.ocupado = q.fase != Fase::REPOSO;
    d.estado = q.estado;
    d.motivo = q.motivo;
    d.accion = q.accion;
    d.fase = q.fase == Fase::REFERENCIA ? "REFERENCIA_SUPERIOR"
             : q.fase == Fase::POSICION ? "POSICION"
             : q.fase == Fase::MANUAL   ? "MANUAL"
             : q.fase == Fase::FINALES  ? "FINALES_INCLINACION"
                                        : "REPOSO";
    d.posicionesConfiguradas = q.posiciones;
    d.bajada = q.bajada;
    d.inclinacion = q.inclinacion;
    return d;
  }
};
static Control control;
}

// Prepara las salidas de los motores en reposo durante el arranque.
void prepararSalidasMotores() {
  Motores::control.reposo();
}

// Inicializa los motores con el intervalo de paso y los tiempos de protección recibidos.
void iniciarMotores(uint32_t intervaloUs, uint32_t timeoutMs, uint32_t estabilidadMs) {
  Motores::control.iniciar(intervaloUs, timeoutMs, estabilidadMs);
}

// Atiende los finales y genera los pulsos pendientes desde el bucle principal.
void actualizarMotores() {
  Motores::control.actualizar();
}

// Configura los niveles activos del final y de la dirección de subida para un motor.
const char *configurarMotor(uint8_t numero, int activo, int dirSubir) {
  return Motores::control.configurar(numero, activo, dirSubir);
}

// Configura las distancias en pasos asociadas a un módulo.
const char *configurarPosicionesModulo(uint8_t modulo, int32_t bajada, int32_t inclinacion) {
  return Motores::control.posiciones(modulo, bajada, inclinacion);
}

// Pide una maniobra para una pareja y devuelve su resultado de admisión.
const char *solicitarMovimiento(uint8_t modulo, AccionMovimiento accion) {
  return Motores::control.solicitar(modulo, accion);
}

// Solicita una prueba de un solo motor con sentido y número de pasos definidos.
const char *moverMotor(uint8_t numero, bool subir, uint32_t pasos) {
  return Motores::control.probar(numero, subir, pasos);
}

// Detiene la pareja a la que pertenece el número de motor recibido.
void detenerMotor(uint8_t numero) {
  if (numero && numero <= 4)
    Motores::control.parar((numero - 1) / 2 + 1);
}

// Detiene los dos motores de un módulo.
void detenerModulo(uint8_t modulo) {
  Motores::control.parar(modulo);
}

// Detiene los cuatro motores de la plataforma.
void detenerTodosLosMotores() {
  Motores::control.pararTodos();
}

// Indica al coordinador si algún módulo sigue en movimiento.
bool motoresOcupados() {
  return Motores::control.ocupado();
}

// Devuelve una instantánea del estado del eje solicitado.
DatosMotor obtenerMotor(uint8_t numero) {
  return Motores::control.datos(numero);
}

// Devuelve una instantánea del movimiento de la pareja solicitada.
DatosMovimiento obtenerMovimiento(uint8_t modulo) {
  return Motores::control.movimiento(modulo);
}

// Entrega la revisión de movimiento para detectar cambios desde la consulta anterior.
uint32_t revisionMotores() {
  return Motores::control.version();
}
