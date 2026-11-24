#include "src/Plataforma.h"
#include "src/Configuracion.h"
#include <driver/gpio.h>
#include <cstring>

namespace Proceso {
struct Nivel {
  int raw = -1;
  uint32_t cambio = 0;
} entradas[3];
enum class Fase { REPOSO, BOMBA, SOLENOIDE, RIEGO, LLENADO, DRENAJE };
Fase etapa = Fase::REPOSO;
bool reles[6] = {}, origenHmi = false, origenAuto = false, enAuto = false;
bool reservaAgenda = false;
uint32_t limiteEventoMs = 0;
bool origenMotor[2] = {};
unsigned modulo = 0, metodo = 0;
uint32_t inicio = 0, cambio = 0;
const char *fallo = "NA";
uint32_t lluviaMs = Configuracion::LLUVIA_MS, nftMs = Configuracion::NFT_MS;
uint32_t llenadoMs = Configuracion::LLENADO_MAX_MS, drenajeMs = Configuracion::DRENAJE_MAX_MS;
struct Regla {
  bool lista = true, armada = true, baja = false, alta = false, lote = false;
  Configuracion::PasoHumedad pasos[Configuracion::PASOS_HUMEDAD] = {};
  // Carga en la regla de humedad la secuencia inicial de métodos y destinos configurada para el central.
  Regla() {
    for (unsigned i = 0; i < Configuracion::PASOS_HUMEDAD; ++i)
      pasos[i] = Configuracion::SECUENCIA_HUMEDAD[i];
  }
  unsigned cantidad = Configuracion::PASOS_HUMEDAD, siguiente = 0;
  unsigned metodo = 0, destino = 1;
  uint32_t confirmar = Configuracion::HUMEDAD_CONFIRMAR_MS, rearme = Configuracion::HUMEDAD_REARME_MS,
           desdeBaja = 0, desdeAlta = 0, instante = 0;
  float histeresis = Configuracion::HUMEDAD_HISTERESIS, valor = NAN;
  int umbral = Configuracion::UMBRAL_HUMEDAD_INICIAL;
  bool valida = false;
} automaticoHumedad;

// Cambia un relé solo cuando varía la orden y traduce el estado lógico a su nivel eléctrico.
void salida(unsigned i, bool on) {
  if (reles[i] == on)
    return;
  digitalWrite(Configuracion::RELES[i], on ? Configuracion::RELE_ACTIVO : !Configuracion::RELE_ACTIVO);
  reles[i] = on;
}

// Desactiva primero la bomba, apaga las válvulas y devuelve el riego al reposo.
void apagar() {
  salida(5, false);
  for (unsigned i = 0; i < 6; ++i)
    salida(i, false);
  etapa = Fase::REPOSO;
  limiteEventoMs = 0;
}

// Detiene riego y motores, cancela el lote pendiente y, cuando se solicita, sale de automático.
void parar(const char *motivo, bool salirAuto) {
  apagar();
  detenerTodosLosMotores();
  if (salirAuto)
    enAuto = false;
  reservaAgenda = false;
  automaticoHumedad.baja = automaticoHumedad.alta = automaticoHumedad.lote = false;
  automaticoHumedad.siguiente = 0;
  Serial0.printf("PROCESO FIN MOTIVO=%s\r\n", motivo);
}

// Detiene el proceso, enclava el motivo de alarma y desarma la regla de humedad.
void bloquear(const char *motivo) {
  parar(motivo, true);
  fallo = motivo;
  automaticoHumedad.armada = false;
}

// Lee los tres flotadores y guarda el instante del último cambio de cada entrada.
void leerNiveles() {
  for (unsigned i = 0; i < 3; ++i) {
    const int raw = digitalRead(Configuracion::NIVELES[i]);
    if (raw != entradas[i].raw) {
      entradas[i].raw = raw;
      entradas[i].cambio = millis();
    }
  }
}

// Devuelve el nivel filtrado del flotador o −1 mientras la lectura sea desconocida o inestable.
int nivel(unsigned i) {
  if (i > 2 || entradas[i].raw < 0 ||
      uint32_t(millis() - entradas[i].cambio) < Configuracion::FILTRO_NIVELES_MS)
    return -1;
  return entradas[i].raw == Configuracion::NIVEL_ACTIVO ? 1 : 0;
}

// Indica si existe un ciclo de riego en curso.
bool riegoActivo() {
  return etapa != Fase::REPOSO;
}

// Indica si el riego, el movimiento, un lote de humedad o un evento mantienen ocupado el sistema.
bool ocupado() {
  return riegoActivo() || motoresOcupados() || automaticoHumedad.lote || reservaAgenda;
}

// Devuelve la causa de la alarma enclavada.
const char *alarma() {
  return fallo;
}

// Indica si el central está en modo automático.
bool automatico() {
  return enAuto;
}

// Devuelve el nombre de la fase hidráulica actual.
const char *fase() {
  switch (etapa) {
  case Fase::BOMBA:
    return "BOMBA";
  case Fase::SOLENOIDE:
    return "SOLENOIDE";
  case Fase::RIEGO:
    return metodo == 2 ? "LLUVIA" : "NFT";
  case Fase::LLENADO:
    return "LLENADO";
  case Fase::DRENAJE:
    return "DRENAJE";
  default:
    return "REPOSO";
  }
}

// Cambia de fase hidráulica y registra el instante de entrada para medir su duración.
void cambiar(Fase f) {
  etapa = f;
  cambio = millis();
}

// Valida niveles, modo y disponibilidad. Si admite el riego, arranca la bomba e inicia la secuencia solicitada.
const char *regar(unsigned m, unsigned tipo, bool deHmi, bool porHumedad) {
  leerNiveles();
  if (tipo > 2 || (tipo == 2 ? m != 0 : (m < 1 || m > 2)))
    return "PARAMETRO_INVALIDO";
  if (!porHumedad && enAuto)
    return "REQUIERE_MODO_MANUAL";
  if (std::strcmp(fallo, "NA"))
    return "ALARMA_ACTIVA";
  if (nivel(0) != 1)
    return "NIVEL_PRINCIPAL_NO_SEGURO";
  if (riegoActivo() || motoresOcupados() || ((automaticoHumedad.lote || reservaAgenda) && !porHumedad))
    return "OCUPADO";
  if (tipo != 2 && nivel(m) != 0)
    return "MODULO_REQUIERE_FLOTADOR_INACTIVO";
  modulo = m;
  metodo = tipo;
  origenHmi = deHmi;
  origenAuto = porHumedad;
  limiteEventoMs = 0;
  inicio = millis();
  salida(5, true);
  cambiar(Fase::BOMBA);
  return "ACEPTADO";
}

// Reserva o libera el proceso hidráulico para que un evento complete sus etapas.
void reservarAgenda(bool reservar) {
  reservaAgenda = reservar;
}

// Inicia en automático un riego programado y le aplica el límite temporal del evento.
const char *evento(unsigned m, unsigned tipo, uint32_t duracionMs) {
  if (!enAuto)
    return "REQUIERE_MODO_AUTO";
  if (!duracionMs || duracionMs > Configuracion::EVENTO_MAX_S * 1000U)
    return "PARAMETRO_INVALIDO";
  if (automaticoHumedad.lote)
    return "OCUPADO";
  const char *r = regar(m, tipo, false, true);
  if (!std::strcmp(r, "ACEPTADO"))
    limiteEventoMs = duracionMs;
  return r;
}

// Verifica modo manual, nivel y ausencia de riego antes de pedir movimiento de la pareja.
const char *mover(unsigned m, AccionMovimiento accion, bool deHmi) {
  leerNiveles();
  if (m < 1 || m > 2)
    return "PARAMETRO_INVALIDO";
  if (enAuto)
    return "REQUIERE_MODO_MANUAL";
  if (std::strcmp(fallo, "NA"))
    return "ALARMA_ACTIVA";
  if (nivel(0) != 1)
    return "NIVEL_PRINCIPAL_NO_SEGURO";
  if (riegoActivo())
    return "RIEGO_EN_CURSO";
  const char *resultado = solicitarMovimiento(m, accion);
  if (!std::strcmp(resultado, "ACEPTADO"))
    origenMotor[m - 1] = deHmi;
  return resultado;
}

// Detiene el módulo indicado. En automático cancela la secuencia completa.
void pararModulo(unsigned m) {
  if (enAuto) {
    parar("PARADA_AUTOMATICO");
    return;
  }
  if (m == 0) {
    parar("PARADA_GENERAL");
    return;
  }
  if (m < 3) {
    detenerModulo(m);
    if (riegoActivo() && modulo == m)
      apagar();
  }
}

// Detiene las acciones manuales que dependían del enlace perdido y registra el fallo si había alguna activa.
void perderEnlace(bool deHmi) {
  bool afectado = false;
  if (riegoActivo() && !origenAuto && origenHmi == deHmi) {
    apagar();
    afectado = true;
  }
  for (unsigned m = 0; m < 2; ++m)
    if (obtenerMovimiento(m + 1).ocupado && origenMotor[m] == deHmi) {
      detenerModulo(m + 1);
      afectado = true;
    }
  if (afectado) {
    fallo = "ENLACE_PERDIDO";
    Serial0.println("PROCESO FIN MOTIVO=ENLACE_PERDIDO");
  }
}

// Devuelve el estado de movimiento de la pareja seleccionada por índice.
const char *movimiento(unsigned m) {
  return obtenerMovimiento(m + 1).estado;
}

// Informa si está activa la ruta de riego consultada.
const char *riego(unsigned i) {
  if (!riegoActivo())
    return "INACTIVO";
  const unsigned actual = metodo == 2 ? 4 : metodo == 0 ? modulo - 1 : modulo + 1;
  return i == actual ? "EN_MARCHA" : "INACTIVO";
}

// Borra la alarma cuando el sistema está libre, el nivel principal es seguro y los finales son coherentes.
const char *reconocer() {
  leerNiveles();
  if (ocupado() || nivel(0) != 1)
    return "CONDICION_NO_SEGURA";
  for (unsigned n = 1; n <= 4; ++n) {
    const auto d = obtenerMotor(n);
    if (d.arriba && d.abajo)
      return "FINALES_CONTRADICTORIOS";
  }
  fallo = "NA";
  return "OK";
}

// Valida y aplica en RAM las duraciones de lluvia y NFT y los máximos de llenado y drenaje.
const char *configurarRiego(uint32_t lluvia, uint32_t nft, uint32_t llenado, uint32_t drenaje) {
  if (ocupado() || enAuto)
    return "OCUPADO";
  for (auto t : {lluvia, nft, llenado, drenaje})
    if (!t || t > Configuracion::DURACION_CONFIG_MAX_MS)
      return "PARAMETRO_INVALIDO";
  lluviaMs = lluvia;
  nftMs = nft;
  llenadoMs = llenado;
  drenajeMs = drenaje;
  return "OK";
}

// Configura una regla de humedad de un paso con su destino, confirmación, rearme e histéresis.
const char *configurarHumedad(unsigned tipo, unsigned destino, uint32_t confirmar, uint32_t rearme,
                              float histeresis) {
  if (ocupado() || enAuto)
    return "OCUPADO";
  if (tipo > 2 || (tipo == 2 ? destino != 0 : (destino < 1 || destino > 2)) || !confirmar || !rearme ||
      confirmar > Configuracion::DURACION_CONFIG_MAX_MS || rearme > Configuracion::DURACION_CONFIG_MAX_MS ||
      !std::isfinite(histeresis) || histeresis <= 0 || histeresis > 100)
    return "PARAMETRO_INVALIDO";
  auto &r = automaticoHumedad;
  r.lista = true;
  r.metodo = tipo;
  r.destino = destino;
  r.confirmar = confirmar;
  r.rearme = rearme;
  r.histeresis = histeresis;
  r.cantidad = 1;
  r.pasos[0] = {tipo, destino};
  r.siguiente = 0;
  r.lote = false;
  r.armada = true;
  r.baja = r.alta = false;
  return "OK";
}

// Recibe la humedad ambiental, valida su rango y conserva el instante y el umbral de evaluación.
void humedad(float v, bool valida, uint32_t instante, int umbral) {
  auto &r = automaticoHumedad;
  r.valor = v;
  r.valida = valida && std::isfinite(v) && v >= 0 && v <= 100;
  r.instante = instante;
  r.umbral = umbral;
}

// Cambia entre manual y automático después de comprobar que haya reglas y condiciones válidas para el modo solicitado.
const char *modo(bool habilitar, bool agendaDisponible) {
  if (habilitar == enAuto)
    return "OK";
  const bool usarHumedad = automaticoHumedad.umbral >= 0;
  if (habilitar && !usarHumedad && !agendaDisponible)
    return "REGLA_PENDIENTE";
  if (habilitar && usarHumedad &&
      (!automaticoHumedad.lista || automaticoHumedad.umbral + automaticoHumedad.histeresis > 100))
    return "REGLA_PENDIENTE";
  if (habilitar && (nivel(0) != 1 || std::strcmp(fallo, "NA")))
    return "CONDICION_NO_SEGURA";
  if (habilitar && usarHumedad &&
      (!automaticoHumedad.valida ||
       uint32_t(millis() - automaticoHumedad.instante) > Configuracion::HUMEDAD_CADUCIDAD_MS))
    return "HUMEDAD_INVALIDA";
  parar("CAMBIO_MODO", false);
  enAuto = habilitar;
  automaticoHumedad.armada = true;
  return "OK";
}

// Informa si la regla de humedad está lista, ejecutándose, incompleta o bloqueada.
const char *regla() {
  if (!automaticoHumedad.lista || automaticoHumedad.umbral < 0)
    return "INCOMPLETA";
  if (std::strcmp(fallo, "NA"))
    return "BLOQUEADA";
  return automaticoHumedad.lote ? "EN_EJECUCION" : automaticoHumedad.armada ? "LISTA" : "BLOQUEADA";
}

// Inicializa los relés apagados y configura la polarización y lectura inicial de los flotadores.
void iniciar() {
  for (unsigned i = 0; i < 6; ++i) {
    gpio_set_level(static_cast<gpio_num_t>(Configuracion::RELES[i]), !Configuracion::RELE_ACTIVO);
    pinMode(Configuracion::RELES[i], OUTPUT);
    reles[i] = false;
  }
  for (auto pin : Configuracion::NIVELES)
    pinMode(pin, Configuracion::NIVEL_ACTIVO == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  leerNiveles();
}

// Propaga al proceso el primer fallo detectado en una pareja de motores.
void despuesMotores() {
  for (unsigned m = 1; m <= 2; ++m)
    if (!std::strcmp(obtenerMovimiento(m).estado, "FALLO")) {
      if (!std::strcmp(fallo, "NA"))
        bloquear(obtenerMovimiento(m).motivo);
      return;
    }
}

// Vigila el nivel principal, avanza las fases de riego y gobierna el disparo, la secuencia y el rearme por humedad.
void actualizar() {
  leerNiveles();

  if ((ocupado() || enAuto) && entradas[0].raw != Configuracion::NIVEL_ACTIVO) {
    bloquear("NIVEL_PRINCIPAL_BAJO");
    return;
  }
  const uint32_t ahora = millis();
  if (riegoActivo()) {
    if (limiteEventoMs && uint32_t(ahora - inicio) >= limiteEventoMs && metodo == 1) {
      bloquear("TIMEOUT_EVENTO");
    } else if (etapa == Fase::DRENAJE) {
      const auto &flotador = entradas[modulo];

      if (flotador.raw != Configuracion::NIVEL_ACTIVO &&
          uint32_t(ahora - flotador.cambio) >=
              Configuracion::FILTRO_NIVELES_MS + Configuracion::DRENAJE_FINAL_MS)
        apagar();
      else if (uint32_t(ahora - cambio) >= drenajeMs)
        bloquear("TIMEOUT_DRENAJE");
    } else if (metodo != 2 && nivel(modulo) == 1) {
      if (metodo == 0)
        bloquear("NIVEL_MODULO_ACTIVO_EN_NFT");
      else {
        salida(5, false);
        salida(modulo - 1, false);
        salida(modulo + 1, true);
        cambiar(Fase::DRENAJE);
      }
    } else if (uint32_t(ahora - inicio) >= (metodo == 1      ? llenadoMs
                                            : limiteEventoMs ? limiteEventoMs
                                            : metodo == 2    ? lluviaMs
                                                             : nftMs)) {
      if (metodo == 1)
        bloquear("TIMEOUT_LLENADO");
      else
        apagar();
    } else if (etapa == Fase::BOMBA && uint32_t(ahora - cambio) >= Configuracion::SEPARACION_RELES_MS) {
      salida(metodo == 2 ? 4 : modulo - 1, true);
      cambiar(metodo == 1 ? Fase::LLENADO : metodo == 0 ? Fase::SOLENOIDE : Fase::RIEGO);
    } else if (etapa == Fase::SOLENOIDE && uint32_t(ahora - cambio) >= Configuracion::SEPARACION_RELES_MS) {
      salida(modulo + 1, true);
      cambiar(Fase::RIEGO);
    }
  }
  auto &r = automaticoHumedad;
  if (!enAuto)
    return;

  if (r.umbral < 0)
    return;
  if (!r.valida || uint32_t(ahora - r.instante) > Configuracion::HUMEDAD_CADUCIDAD_MS) {
    bloquear("HUMEDAD_INVALIDA");
    return;
  }
  if (reservaAgenda) {
    r.baja = r.alta = false;
    return;
  }
  if (r.lote) {
    if (!riegoActivo()) {
      if (r.siguiente < r.cantidad) {
        const auto paso = r.pasos[r.siguiente];
        const char *resultado = regar(paso.destino, paso.metodo, false, true);
        if (!std::strcmp(resultado, "ACEPTADO"))
          ++r.siguiente;
        else
          bloquear(resultado);
      } else {
        r.lote = false;
        r.siguiente = 0;
        r.baja = r.alta = false;
      }
    }
    return;
  }
  if (ocupado()) {
    r.baja = r.alta = false;
    return;
  }
  if (!r.armada) {
    if (r.valor > r.umbral + r.histeresis) {
      if (!r.alta) {
        r.alta = true;
        r.desdeAlta = ahora;
      }
      if (uint32_t(ahora - r.desdeAlta) >= r.rearme) {
        r.armada = true;
        r.alta = false;
      }
    } else
      r.alta = false;
  } else if (r.valor <= r.umbral) {
    if (!r.baja) {
      r.baja = true;
      r.desdeBaja = ahora;
    }
    if (uint32_t(ahora - r.desdeBaja) >= r.confirmar) {
      const auto paso = r.pasos[0];
      const char *resultado = regar(paso.destino, paso.metodo, false, true);
      r.baja = false;
      if (!std::strcmp(resultado, "ACEPTADO")) {
        r.armada = false;
        r.lote = true;
        r.siguiente = 1;
      }
    }
  } else
    r.baja = false;
}
}
