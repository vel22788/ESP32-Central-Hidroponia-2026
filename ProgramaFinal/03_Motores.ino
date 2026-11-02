/**
 * @file 03_Motores.ino
 * Control STEP/DIR de cuatro ejes agrupados en M1=(1,2) y M2=(3,4).
 * La API pública numera motores 1..4 y módulos 1..2; internamente usa 0..3/0..1.
 * Cada eje tiene final superior (lado 0) e inferior (lado 1).
 *
 * SUBIR/BAJAR paran la pareja al llegar el primer final del sentido pedido.
 * NIVELAR/REFERENCIAR llevan cada eje a su superior y confirman la referencia.
 * INCLINAR baja el primer eje y sube el segundo hasta sus propios finales.
 * Se conservan rutas antiguas por posición para compatibilidad; las órdenes
 * actuales NIVELAR/INCLINAR no requieren distancias ficticias en pasos.
 *
 * Sin aceleración ni conversión a mm: intervalo STEP en us, timeout/filtro en ms.
 * Parar STEP no corta la alimentación: EN permanece conectado a GND.
 * La autorización de proceso y el nivel principal se comprueban en módulo 04;
 * este módulo comprueba sus finales, rangos y tiempo de movimiento.
 */
/* Modulo 03: motores/finales. Sin UART, Wire ni NVS.
 * M1 = motores 1/2 (izquierda/derecha); M2 = 3/4.
 * EN permanente a GND. Este modulo no usa GPIO21, reservado a alimentacion pH.
 */
#include <Arduino.h>
#include "src/Configuracion.h"
#include <driver/gpio.h>
#include <climits>

namespace Motores {
class Control {
  enum class Fase : uint8_t { REPOSO, MANUAL, REFERENCIA, POSICION, FINALES };
  /**
   * Estado por eje. posicion es un contador relativo de pasos tras referencia,
   * no un encoder ni una medida física de altura; raw[] guarda niveles eléctricos.
   */
  struct Eje {
    bool configurado = false, moviendo = false, subir = false, referenciado = false;
    bool esperandoFinal = false;
    int activo = Configuracion::FINAL_ACTIVO, dirSubir = Configuracion::DIR_SUBIR;
    int raw[2] = {0, 0};
    uint32_t cambio[2] = {0, 0}, pasos = 0, ultimoPaso = 0;
    int32_t posicion = 0, destino = 0;
  };
  /**
   * Estado de una maniobra común a dos ejes. inicio limita toda la maniobra;
   * ultimoPaso coordina sus pulsos. Los campos por posición conservan compatibilidad.
   */
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
  Eje e[4]; Pareja p[2];
  uint32_t intervalo = Configuracion::PASO_US, timeout = Configuracion::MOTOR_MAX_MS, estabilidad = Configuracion::FILTRO_FINALES_MS, revision = 0;
  bool listo = false;

  bool activo(uint8_t i, uint8_t lado) const { return e[i].configurado && e[i].raw[lado] == e[i].activo; }
  /**
   * Acepta los finales cuando ambos niveles llevan el tiempo de filtro sin cambio.
   * Durante una maniobra, detectar un final detiene el eje antes de confirmar estabilidad.
   */
  bool estable(uint8_t i) const {
    return uint32_t(millis() - e[i].cambio[0]) >= estabilidad &&
      uint32_t(millis() - e[i].cambio[1]) >= estabilidad;
  }
  bool contradictorio(uint8_t i) const { return activo(i, 0) && activo(i, 1); }
  bool referencia(uint8_t m) const { return e[m*2].referenciado && e[m*2+1].referenciado; }
  void leer() {
    for (uint8_t i = 0; i < 4; ++i) for (uint8_t lado = 0; lado < 2; ++lado) {
      const int valor = digitalRead(finales[i][lado]);
      if (valor != e[i].raw[lado]) { e[i].raw[lado] = valor; e[i].cambio[lado] = millis(); }
    }
  }
  const char *comprobar(uint8_t i) const {
    if (!listo) return "MOTORES_NO_INICIADOS";
    if (!e[i].configurado) return "CONFIG_MOTOR_PENDIENTE";
    if (contradictorio(i)) return "FINALES_CONTRADICTORIOS";
    if (!estable(i)) return "FINALES_INESTABLES";
    return nullptr;
  }
  void pararEje(uint8_t i) { e[i].moviendo = false; digitalWrite(step[i], LOW); }
  /**
   * Detiene STEP de ambos ejes y registra causa. Un fallo o una referencia
   * interrumpida invalida el cero; no deshabilita eléctricamente los DRV8825.
   */
  void finalizar(uint8_t m, const char *motivo, bool fallo = false) {
    const bool referenciando = p[m].fase == Fase::REFERENCIA;
    for (uint8_t i = m*2; i < m*2+2; ++i) {
      pararEje(i); e[i].esperandoFinal = false;
      if (fallo || referenciando) e[i].referenciado = false;
    }
    p[m].fase = Fase::REPOSO; p[m].prueba = false;
    p[m].estado = fallo ? "FALLO" : "DETENIDO"; p[m].motivo = motivo;
    ++revision;
  }
  void orientar(uint8_t i, bool subir) {
    e[i].subir = subir; e[i].moviendo = true;
    digitalWrite(dir[i], subir ? e[i].dirSubir : !e[i].dirSubir);
    e[i].ultimoPaso = micros();
  }
  /**
   * Cada eje sube hasta su final superior. Si ya está activo no emite más pasos;
   * comprobarPareja() confirma estabilidad antes de declarar el cero.
   */
  void prepararReferencia(uint8_t m) {
    p[m].fase = Fase::REFERENCIA;
    for (uint8_t i = m*2; i < m*2+2; ++i) {
      e[i].referenciado = false;
      e[i].esperandoFinal = activo(i, 0);
      if (e[i].esperandoFinal) pararEje(i); else orientar(i, true);
    }
    p[m].ultimoPaso = micros();
  }
  /**
   * Ruta conservada por compatibilidad: calcula destinos en pasos a partir del
   * cero y verifica límites. No es el criterio de las maniobras actuales por finales.
   */
  bool prepararPosicion(uint8_t m) {
    p[m].fase = Fase::POSICION;
    bool alguno = false;
    for (uint8_t i = m*2; i < m*2+2; ++i) {
      e[i].destino = p[m].bajada + ((i % 2 && p[m].accion == AccionMovimiento::INCLINAR) ? p[m].inclinacion : 0);
      if (e[i].posicion == e[i].destino) { pararEje(i); continue; }
      const bool subir = e[i].posicion > e[i].destino;
      if (activo(i, subir ? 0 : 1)) { finalizar(m, "LIMITE_ANTES_DEL_DESTINO", true); return false; }
      orientar(i, subir); alguno = true;
    }
    p[m].ultimoPaso = micros();
    if (!alguno) finalizar(m, "POSICION_ALCANZADA");
    return true;
  }
  /**
   * Prioridad de servicio: finales contradictorios y timeout, luego reglas de
   * la fase. REFERENCIA y FINALES detienen cada eje por separado; MANUAL detiene
   * la pareja al primer límite. El fin de una orden se conoce por su estado.
   */
  void comprobarPareja(uint8_t m) {
    Pareja &q = p[m];
    if (q.fase == Fase::REPOSO) return;
    for (uint8_t i = m*2; i < m*2+2; ++i)
      if (contradictorio(i)) { finalizar(m, "FINALES_CONTRADICTORIOS", true); return; }
    if (uint32_t(millis() - q.inicio) >= timeout) { finalizar(m, "TIMEOUT", true); return; }
    if (q.fase == Fase::REFERENCIA) {
      bool completos = true;
      for (uint8_t i = m*2; i < m*2+2; ++i) {
        if (activo(i, 0)) {
          pararEje(i); e[i].esperandoFinal = true;
          if (uint32_t(millis() - e[i].cambio[0]) >= estabilidad) {
            e[i].posicion = 0; e[i].referenciado = true;
          } else completos = false;
        } else {
          if (e[i].esperandoFinal) { finalizar(m, "FINAL_INESTABLE", true); return; }
          completos = false;
        }
      }
      if (completos) {
        if (q.accion == AccionMovimiento::REFERENCIAR || q.accion == AccionMovimiento::NIVELAR) {
          // Cambiar de fase antes de terminar para conservar los ceros comprobados.
          q.fase = Fase::POSICION; finalizar(m, "REFERENCIA_SUPERIOR");
        } else prepararPosicion(m);
        ++revision;
      }
      return;
    }
    if (q.fase == Fase::FINALES) {
      bool completos=true;
      for(uint8_t i=m*2;i<m*2+2;++i) {
        const uint8_t lado=e[i].subir ? 0 : 1;
        if(activo(i,lado)) {
          pararEje(i); e[i].esperandoFinal=true;
          if(uint32_t(millis()-e[i].cambio[lado])<estabilidad) completos=false;
        } else {
          if(e[i].esperandoFinal) { finalizar(m,"FINAL_INESTABLE",true); return; }
          completos=false;
        }
      }
      if(completos) finalizar(m,"INCLINACION_ALCANZADA");
      return;
    }
    if (q.prueba) {
      const uint8_t i = q.ejePrueba;
      if (activo(i, e[i].subir ? 0 : 1)) { finalizar(m, "LIMITE_ALCANZADO"); return; }
      if (!q.restantes) finalizar(m, "PASOS_COMPLETADOS");
      return;
    }
    if (q.fase == Fase::MANUAL) {
      const uint8_t lado = q.accion == AccionMovimiento::SUBIR ? 0 : 1;
      if (activo(m*2, lado) || activo(m*2+1, lado)) {
        // En elevacion normal se detienen ambos al llegar el primer lado.
        finalizar(m, activo(m*2, lado) && activo(m*2+1, lado) ? "LIMITES_ALCANZADOS" : "LIMITE_UN_LADO");
      }
      return;
    }
    bool quedan = false;
    for (uint8_t i = m*2; i < m*2+2; ++i) {
      if (!e[i].moviendo) continue;
      if (e[i].posicion == e[i].destino) { pararEje(i); continue; }
      if (activo(i, e[i].subir ? 0 : 1)) { finalizar(m, "LIMITE_ANTES_DEL_DESTINO", true); return; }
      quedan = true;
    }
    if (!quedan) finalizar(m, "POSICION_ALCANZADA");
  }

 public:
  void reposo() {
    for (uint8_t i = 0; i < 4; ++i) {
      gpio_set_level(static_cast<gpio_num_t>(step[i]), LOW); pinMode(step[i], OUTPUT);
      gpio_set_level(static_cast<gpio_num_t>(dir[i]), LOW); pinMode(dir[i], OUTPUT);
      e[i] = Eje{};
    }
    for (auto &q : p) q = Pareja{};
    listo = false; ++revision;
  }
  /**
   * Configura intervalo STEP (us), timeout y filtro (ms). Lee ocho entradas sin
   * pull interno; la polaridad se declara con configurar(), no se deduce del color.
   */
  void iniciar(uint32_t intervaloUs, uint32_t timeoutMs, uint32_t estabilidadMs) {
    intervalo = intervaloUs >= Configuracion::PASO_MIN_US ? intervaloUs : Configuracion::PASO_MIN_US;
    timeout = timeoutMs ? timeoutMs : Configuracion::MOTOR_MAX_MS; estabilidad = estabilidadMs;
    for (uint8_t i = 0; i < 4; ++i) for (uint8_t lado = 0; lado < 2; ++lado) {
      // Placas con VCC/GND/senal: entradas sin pull interno; polaridad por comprobar.
      pinMode(finales[i][lado], INPUT);
      e[i].raw[lado] = digitalRead(finales[i][lado]); e[i].cambio[lado] = millis();
    }
    listo = true; ++revision;
  }
  /**
   * Declara polaridad del final y nivel DIR para subir, solo en reposo.
   * Reconfigurar invalida la referencia de ambos ejes de la pareja.
   */
  const char *configurar(uint8_t numero, int nivelActivo, int nivelSubir) {
    if (!numero || numero > 4 || (nivelActivo != 0 && nivelActivo != 1) || (nivelSubir != 0 && nivelSubir != 1)) return "PARAMETRO_INVALIDO";
    if (!listo) return "MOTORES_NO_INICIADOS";
    const uint8_t i = numero-1, m = i/2;
    if (p[m].fase != Fase::REPOSO) return "OCUPADO";
    e[i].activo = nivelActivo; e[i].dirSubir = nivelSubir; e[i].configurado = true;
    e[m*2].referenciado = e[m*2+1].referenciado = false;
    leer(); ++revision; return "OK";
  }
  const char *posiciones(uint8_t modulo, int32_t bajada, int32_t inclinacion) {
    if (!modulo || modulo > 2 || bajada <= 0 || inclinacion < 0 || bajada > INT32_MAX-inclinacion) return "PARAMETRO_INVALIDO";
    if (!listo) return "MOTORES_NO_INICIADOS";
    Pareja &q = p[modulo-1];
    if (q.fase != Fase::REPOSO) return "OCUPADO";
    q.bajada = bajada; q.inclinacion = inclinacion; q.posiciones = true;
    ++revision; return "OK";
  }
  /**
   * Valida módulo/acción/finales y prepara una maniobra asíncrona. ACEPTADO
   * no significa destino alcanzado; YA_EN_CURSO evita reiniciar la misma maniobra.
   * Las rutas actuales NIVELAR e INCLINAR terminan mediante finales físicos.
   */
  const char *solicitar(uint8_t modulo, AccionMovimiento accion) {
    if (!modulo || modulo > 2 || accion == AccionMovimiento::NINGUNA ||
        static_cast<uint8_t>(accion) > static_cast<uint8_t>(AccionMovimiento::INCLINAR)) return "PARAMETRO_INVALIDO";
    const uint8_t m = modulo-1; Pareja &q = p[m];
    leer();
    if (q.fase != Fase::REPOSO) return !q.prueba && q.accion == accion ? "YA_EN_CURSO" : "OCUPADO";
    for (uint8_t i = m*2; i < m*2+2; ++i) { const char *error = comprobar(i); if (error) return error; }
    const bool posicion = false; // NIVELAR/INCLINAR ahora usan finales físicos.
    if (posicion && !q.posiciones) return "POSICIONES_PENDIENTES";

    if (accion == AccionMovimiento::SUBIR || accion == AccionMovimiento::BAJAR) {
      const uint8_t lado = accion == AccionMovimiento::SUBIR ? 0 : 1;
      if (activo(m*2, lado) || activo(m*2+1, lado)) return "LIMITE_ACTIVO";
    }
    q.accion = accion; q.prueba = false; q.inicio = millis(); q.motivo = "ORDEN";
    q.estado = accion == AccionMovimiento::SUBIR ? "SUBIENDO" : accion == AccionMovimiento::BAJAR ? "BAJANDO" :
      accion == AccionMovimiento::NIVELAR ? "NIVELANDO" : accion == AccionMovimiento::INCLINAR ? "INCLINANDO" : "REFERENCIANDO";
    for (uint8_t i = m*2; i < m*2+2; ++i) e[i].pasos = 0;
    if (accion == AccionMovimiento::INCLINAR) {
      q.fase=Fase::FINALES;
      for(uint8_t i=m*2;i<m*2+2;++i) {
        e[i].referenciado=false;
        const bool subir=(i%2)==1;
        e[i].subir=subir; e[i].esperandoFinal=activo(i,subir ? 0 : 1);
        if(e[i].esperandoFinal) pararEje(i); else orientar(i,subir);
      }
      q.ultimoPaso=micros();
    } else if (accion == AccionMovimiento::SUBIR || accion == AccionMovimiento::BAJAR) {
      q.fase = Fase::MANUAL;
      orientar(m*2, accion == AccionMovimiento::SUBIR); orientar(m*2+1, accion == AccionMovimiento::SUBIR);
      q.ultimoPaso = micros();
    } else if (accion == AccionMovimiento::NIVELAR || accion == AccionMovimiento::REFERENCIAR || !referencia(m)) {
      prepararReferencia(m);
    } else prepararPosicion(m);
    ++revision; return "ACEPTADO";
  }
  /**
   * Ruta individual conservada dentro del controlador: pasos solicitados son
   * un máximo, pues un final o timeout puede detener antes. Altera la alineación
   * de la pareja y por eso invalida su referencia; no es un archivo de prueba aparte.
   */
  const char *probar(uint8_t numero, bool subir, uint32_t pasos) {
    if (!numero || numero > 4 || !pasos || pasos > INT32_MAX) return "PARAMETRO_INVALIDO";
    const uint8_t i = numero-1, m = i/2;
    leer();
    if (p[m].fase != Fase::REPOSO) return "OCUPADO";
    const char *error = comprobar(i); if (error) return error;
    if (activo(i, subir ? 0 : 1)) return "LIMITE_ACTIVO";
    // Una prueba individual desalinearia el modulo: exigir nueva referencia.
    e[m*2].referenciado = e[m*2+1].referenciado = false;
    Pareja &q = p[m]; q.fase = Fase::MANUAL; q.prueba = true; q.ejePrueba = i;
    q.accion = AccionMovimiento::NINGUNA; q.estado = "PROBANDO"; q.motivo = "ORDEN_PRUEBA";
    q.inicio = millis(); q.restantes = pasos; q.ultimoPaso = micros(); e[i].pasos = 0;
    orientar(i, subir); ++revision; return "ACEPTADO";
  }
  /**
   * Lee finales, comprueba maniobras y emite a lo sumo un pulso por eje listo
   * en esta llamada. No recupera pulsos atrasados en ráfaga. La máscara agrupa
   * ejes para elevar STEP juntos; PULSO_STEP_US fija solo el ancho en alto.
   */
  void actualizar() {
    if (!listo) return;
    leer();
    for (uint8_t m = 0; m < 2; ++m) comprobarPareja(m);
    uint8_t mascara = 0;
    const uint32_t ahora = micros();
    for (uint8_t m = 0; m < 2; ++m) {
      if (p[m].fase == Fase::REPOSO || uint32_t(ahora-p[m].ultimoPaso) < intervalo) continue;
      // Todos los ejes activos de una pareja comparten el mismo instante de paso.
      for (uint8_t i = m*2; i < m*2+2; ++i) if (e[i].moviendo) {
        if (uint32_t(ahora-e[i].ultimoPaso) < intervalo) continue;
        if (e[i].referenciado && (e[i].subir ? e[i].posicion <= 0 : e[i].posicion == INT32_MAX)) {
          finalizar(m, "POSICION_FUERA_DE_RANGO", true); break;
        }
        mascara |= uint8_t(1u << i);
      }
      if (p[m].fase == Fase::REPOSO) mascara &= uint8_t(~(3u << (m*2)));
    }
    if (!mascara) return;
    for (uint8_t i = 0; i < 4; ++i) if (mascara & (1u << i)) digitalWrite(step[i], HIGH);
    delayMicroseconds(Configuracion::PULSO_STEP_US);
    for (uint8_t i = 0; i < 4; ++i) if (mascara & (1u << i)) {
      digitalWrite(step[i], LOW); e[i].ultimoPaso = ahora; ++e[i].pasos;
      if (e[i].referenciado) e[i].posicion += e[i].subir ? -1 : 1;
    }
    for (uint8_t m = 0; m < 2; ++m) if (mascara & (3u << (m*2))) {
      p[m].ultimoPaso = ahora;
      if (p[m].prueba && p[m].restantes) --p[m].restantes;
      comprobarPareja(m);
    }
  }
  void parar(uint8_t modulo) { if (modulo && modulo <= 2) finalizar(modulo-1, "PARADA_SOLICITADA"); }
  void pararTodos() { parar(1); parar(2); }
  bool ocupado() const { return p[0].fase != Fase::REPOSO || p[1].fase != Fase::REPOSO; }
  uint32_t version() const { return revision; }
  DatosMotor datos(uint8_t numero) const {
    DatosMotor d; if (!numero || numero > 4) return d;
    const uint8_t i = numero-1;
    d.configurado = e[i].configurado; d.arriba = activo(i, 0); d.abajo = activo(i, 1);
    d.moviendo = e[i].moviendo; d.subiendo = e[i].subir; d.referenciado = e[i].referenciado;
    d.rawArriba = e[i].raw[0]; d.rawAbajo = e[i].raw[1]; d.activo = e[i].configurado ? e[i].activo : -1;
    d.dirSubir = e[i].configurado ? e[i].dirSubir : -1; d.posicion = e[i].posicion; d.pasos = e[i].pasos;
    return d;
  }
  DatosMovimiento movimiento(uint8_t modulo) const {
    DatosMovimiento d; if (!modulo || modulo > 2) return d;
    const Pareja &q = p[modulo-1]; d.ocupado = q.fase != Fase::REPOSO;
    d.estado = q.estado; d.motivo = q.motivo; d.accion = q.accion;
    d.fase = q.fase == Fase::REFERENCIA ? "REFERENCIA_SUPERIOR" : q.fase == Fase::POSICION ? "POSICION" :
      q.fase == Fase::MANUAL ? "MANUAL" : q.fase == Fase::FINALES ? "FINALES_INCLINACION" : "REPOSO";
    d.posicionesConfiguradas = q.posiciones; d.bajada = q.bajada; d.inclinacion = q.inclinacion;
    return d;
  }
};
static Control control;
} // namespace Motores

/**
 * Fachada pública: traduce la API del sketch a la instancia interna.
 * detenerMotor() detiene su pareja completa; no hay parada eléctrica por EN.
 * DatosMotor/DatosMovimiento son copias de consulta, sin actuaciones nuevas.
 */
void prepararSalidasMotores() { Motores::control.reposo(); }
void iniciarMotores(uint32_t intervaloUs, uint32_t timeoutMs, uint32_t estabilidadMs) { Motores::control.iniciar(intervaloUs, timeoutMs, estabilidadMs); }
void actualizarMotores() { Motores::control.actualizar(); }
const char *configurarMotor(uint8_t numero, int activo, int dirSubir) { return Motores::control.configurar(numero, activo, dirSubir); }
const char *configurarPosicionesModulo(uint8_t modulo, int32_t bajada, int32_t inclinacion) { return Motores::control.posiciones(modulo, bajada, inclinacion); }
const char *solicitarMovimiento(uint8_t modulo, AccionMovimiento accion) { return Motores::control.solicitar(modulo, accion); }
const char *moverMotor(uint8_t numero, bool subir, uint32_t pasos) { return Motores::control.probar(numero, subir, pasos); }
void detenerMotor(uint8_t numero) { if (numero && numero <= 4) Motores::control.parar((numero-1)/2+1); }
void detenerModulo(uint8_t modulo) { Motores::control.parar(modulo); }
void detenerTodosLosMotores() { Motores::control.pararTodos(); }
bool motoresOcupados() { return Motores::control.ocupado(); }
DatosMotor obtenerMotor(uint8_t numero) { return Motores::control.datos(numero); }
DatosMovimiento obtenerMovimiento(uint8_t modulo) { return Motores::control.movimiento(modulo); }
uint32_t revisionMotores() { return Motores::control.version(); }
