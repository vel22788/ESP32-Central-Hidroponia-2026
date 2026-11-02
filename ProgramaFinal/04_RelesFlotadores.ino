/**
 * @file 04_RelesFlotadores.ino
 * Proceso hidráulico y autorización de movimientos. Métodos: 0 NFT,
 * 1 inundación/drenaje, 2 lluvia. Destinos: 1 M1, 2 M2, 0 vertical.
 * Un recurso hidráulico compartido impide riego y movimiento simultáneos.
 *
 * NFT: bomba -> solenoide -> bola -> apagar al terminar o detener por protección.
 * Lluvia: bomba -> solenoide vertical -> apagar al terminar.
 * Inundación: llenar hasta flotador activo; quitar bomba/solenoide, energizar
 * bola para drenar; flotador inactivo termina. Los máximos son protecciones,
 * no tiempos medidos de llenado/vaciado ni una medida de volumen.
 *
 * Relés activos según Configuracion::RELE_ACTIVO. Bola energizada abierta y
 * desenergizada cerrada según el montaje documentado. Arranque en MANUAL.
 * Alarma enclavada: reconocerla permite nuevas órdenes, no reanuda una anterior.
 * La agenda reside en ControlHMI.cpp; aquí solo se ejecutan sus ciclos.
 */
/* Módulo 04: niveles, seis relés y secuencias; sin UART, ADC ni agenda. */
#include "src/Plataforma.h"
#include "src/Configuracion.h"
#include <driver/gpio.h>
#include <cstring>

namespace Proceso {
/**
 * Cada entrada conserva valor eléctrico y último cambio. nivel() convierte
 * a 1 activo, 0 inactivo o -1 desconocido/inestable tras el filtro.
 */
struct Nivel { int raw=-1; uint32_t cambio=0; } entradas[3];
enum class Fase { REPOSO, BOMBA, SOLENOIDE, RIEGO, LLENADO, DRENAJE };
Fase etapa=Fase::REPOSO;
bool reles[6]={}, origenHmi=false, origenAuto=false, enAuto=false;
bool reservaAgenda=false;
uint32_t limiteEventoMs=0;
bool origenMotor[2]={};
unsigned modulo=0, metodo=0; // NFT=0, INUNDACION=1, LLUVIA=2; módulo 1/2 o 0 vertical.
uint32_t inicio=0, cambio=0;
const char *fallo="NA";
uint32_t lluviaMs=Configuracion::LLUVIA_MS, nftMs=Configuracion::NFT_MS;
uint32_t llenadoMs=Configuracion::LLENADO_MAX_MS, drenajeMs=Configuracion::DRENAJE_MAX_MS;
/**
 * La regla de humedad arma un lote secuencial una sola vez por descenso.
 * Para repetir debe superar umbral+histeresis y cumplir rearme. La configuración
 * inicial copia la secuencia del header; CONFIG HUMEDAD puede elegir un solo paso.
 */
struct Regla {
  bool lista=true, armada=true, baja=false, alta=false, lote=false;
  Configuracion::PasoHumedad pasos[Configuracion::PASOS_HUMEDAD]={};
  Regla() { for(unsigned i=0;i<Configuracion::PASOS_HUMEDAD;++i) pasos[i]=Configuracion::SECUENCIA_HUMEDAD[i]; }
  unsigned cantidad=Configuracion::PASOS_HUMEDAD, siguiente=0;
  unsigned metodo=0, destino=1;
  uint32_t confirmar=Configuracion::HUMEDAD_CONFIRMAR_MS, rearme=Configuracion::HUMEDAD_REARME_MS, desdeBaja=0, desdeAlta=0, instante=0;
  float histeresis=Configuracion::HUMEDAD_HISTERESIS, valor=NAN;
  int umbral=Configuracion::UMBRAL_HUMEDAD_INICIAL;
  bool valida=false;
} automaticoHumedad;

/**
 * Traduce ON lógico a la polaridad eléctrica configurada y evita escrituras
 * repetidas. Índices: 0/1 solenoides M1/M2, 2/3 bolas, 4 vertical, 5 bomba.
 */
void salida(unsigned i,bool on) {
  if(reles[i]==on) return;
  digitalWrite(Configuracion::RELES[i],on ? Configuracion::RELE_ACTIVO : !Configuracion::RELE_ACTIVO);
  reles[i]=on;
}
/**
 * Retira primero impulsión y luego todas las rutas; termina solo el ciclo
 * hidráulico. No borra la agenda ni obliga por sí misma a salir de AUTO.
 */
void apagar() {
  salida(5,false); // retirar impulsión antes de cerrar rutas.
  for(unsigned i=0;i<6;++i) salida(i,false);
  etapa=Fase::REPOSO;
  limiteEventoMs=0;
}
/**
 * Parada global: hidráulica y motores, cancelación de reservas/lote.
 * salirAuto decide si vuelve a MANUAL; no reanuda una acción después.
 */
void parar(const char *motivo,bool salirAuto) {
  apagar(); detenerTodosLosMotores();
  if(salirAuto) enAuto=false;
  reservaAgenda=false;
  automaticoHumedad.baja=automaticoHumedad.alta=automaticoHumedad.lote=false;
  automaticoHumedad.siguiente=0;
  Serial0.printf("PROCESO FIN MOTIVO=%s\r\n",motivo);
}
/**
 * Parada con alarma enclavada y regla desarmada. La condición debe resolverse
 * y reconocerse explícitamente para permitir nuevas órdenes.
 */
void bloquear(const char *motivo) {
  parar(motivo,true); fallo=motivo; automaticoHumedad.armada=false;
}
void leerNiveles() {
  for(unsigned i=0;i<3;++i) {
    const int raw=digitalRead(Configuracion::NIVELES[i]);
    if(raw!=entradas[i].raw) { entradas[i].raw=raw; entradas[i].cambio=millis(); }
  }
}
int nivel(unsigned i) {
  if(i>2 || entradas[i].raw<0 || uint32_t(millis()-entradas[i].cambio)<Configuracion::FILTRO_NIVELES_MS) return -1;
  return entradas[i].raw==Configuracion::NIVEL_ACTIVO ? 1 : 0;
}
bool riegoActivo() { return etapa!=Fase::REPOSO; }
bool ocupado() { return riegoActivo() || motoresOcupados() || automaticoHumedad.lote || reservaAgenda; }
const char *alarma() { return fallo; }
bool automatico() { return enAuto; }
const char *fase() {
  switch(etapa) {
    case Fase::BOMBA:return "BOMBA";
    case Fase::SOLENOIDE:return "SOLENOIDE";
    case Fase::RIEGO:return metodo==2 ? "LLUVIA" : "NFT";
    case Fase::LLENADO:return "LLENADO";
    case Fase::DRENAJE:return "DRENAJE";
    default:return "REPOSO";
  }
}
void cambiar(Fase f) { etapa=f; cambio=millis(); }
/**
 * Valida método/destino, modo, alarma, nivel principal y exclusión de recursos.
 * Un módulo horizontal exige flotador inactivo al inicio. ACEPTADO enciende
 * bomba e inicia etapas; porHumedad también identifica ejecución automática.
 */
const char *regar(unsigned m,unsigned tipo,bool deHmi,bool porHumedad) {
  leerNiveles();
  if(tipo>2 || (tipo==2 ? m!=0 : (m<1 || m>2))) return "PARAMETRO_INVALIDO";
  if(!porHumedad && enAuto) return "REQUIERE_MODO_MANUAL";
  if(std::strcmp(fallo,"NA")) return "ALARMA_ACTIVA";
  if(nivel(0)!=1) return "NIVEL_PRINCIPAL_NO_SEGURO";
  if(riegoActivo() || motoresOcupados() || ((automaticoHumedad.lote || reservaAgenda) && !porHumedad)) return "OCUPADO";
  if(tipo!=2 && nivel(m)!=0) return "MODULO_REQUIERE_FLOTADOR_INACTIVO";
  modulo=m; metodo=tipo; origenHmi=deHmi; origenAuto=porHumedad;
  limiteEventoMs=0; inicio=millis(); salida(5,true); cambiar(Fase::BOMBA);
  return "ACEPTADO";
}
void reservarAgenda(bool reservar) { reservaAgenda=reservar; }
/**
 * Entrada usada por agenda: exige AUTO y duración en ms válida. NFT/lluvia
 * usan esa duración; inundación mantiene flotadores y añade límite total del evento.
 */
const char *evento(unsigned m,unsigned tipo,uint32_t duracionMs) {
  if(!enAuto) return "REQUIERE_MODO_AUTO";
  if(!duracionMs || duracionMs>Configuracion::EVENTO_MAX_S*1000U) return "PARAMETRO_INVALIDO";
  if(automaticoHumedad.lote) return "OCUPADO";
  const char *r=regar(m,tipo,false,true);
  if(!std::strcmp(r,"ACEPTADO")) limiteEventoMs=duracionMs;
  return r;
}
/**
 * Autoriza movimiento solo en MANUAL, sin riego, con nivel principal seguro
 * y sin alarma. Registra el puerto de origen para supervisar pérdida de enlace.
 */
const char *mover(unsigned m,AccionMovimiento accion,bool deHmi) {
  leerNiveles();
  if(m<1 || m>2) return "PARAMETRO_INVALIDO";
  if(enAuto) return "REQUIERE_MODO_MANUAL";
  if(std::strcmp(fallo,"NA")) return "ALARMA_ACTIVA";
  if(nivel(0)!=1) return "NIVEL_PRINCIPAL_NO_SEGURO";
  if(riegoActivo()) return "RIEGO_EN_CURSO";
  const char *resultado=solicitarMovimiento(m,accion);
  if(!std::strcmp(resultado,"ACEPTADO")) origenMotor[m-1]=deHmi;
  return resultado;
}
void pararModulo(unsigned m) {
  if(enAuto) { parar("PARADA_AUTOMATICO"); return; }
  if(m==0) { parar("PARADA_GENERAL"); return; }
  if(m<3) { detenerModulo(m); if(riegoActivo() && modulo==m) apagar(); }
}
/**
 * Cancela actuaciones manuales pertenecientes al puerto perdido.
 * El automático no depende del latido de esa pantalla, pero conserva sus
 * protecciones de niveles, alarmas y, cuando aplica, humedad/RTC.
 */
void perderEnlace(bool deHmi) {
  bool afectado=false;
  if(riegoActivo() && !origenAuto && origenHmi==deHmi) { apagar(); afectado=true; }
  for(unsigned m=0;m<2;++m) if(obtenerMovimiento(m+1).ocupado && origenMotor[m]==deHmi) {
    detenerModulo(m+1); afectado=true;
  }
  if(afectado) { fallo="ENLACE_PERDIDO"; Serial0.println("PROCESO FIN MOTIVO=ENLACE_PERDIDO"); }
}
/**
 * Consultas para presentación: movimiento recibe índice 0/1 y lo convierte
 * a módulo 1/2. riego recibe 0 NFT M1, 1 NFT M2, 2 inundación M1,
 * 3 inundación M2 o 4 lluvia; no solicita operaciones.
 */
const char *movimiento(unsigned m) { return obtenerMovimiento(m+1).estado; }
const char *riego(unsigned i) {
  if(!riegoActivo()) return "INACTIVO";
  const unsigned actual=metodo==2 ? 4 : metodo==0 ? modulo-1 : modulo+1;
  return i==actual ? "EN_MARCHA" : "INACTIVO";
}
/**
 * Limpia alarma solo en reposo, con principal seguro y sin finales opuestos
 * activos. Reconocer no arranca bombas, mueve ejes ni cambia a AUTO.
 */
const char *reconocer() {
  leerNiveles();
  if(ocupado() || nivel(0)!=1) return "CONDICION_NO_SEGURA";
  for(unsigned n=1;n<=4;++n) { const auto d=obtenerMotor(n); if(d.arriba && d.abajo) return "FINALES_CONTRADICTORIOS"; }
  fallo="NA"; return "OK";
}
/**
 * Sustituye duraciones en RAM únicamente en reposo y MANUAL. Son parámetros
 * volátiles: reiniciar restaura Configuracion.h; no reescribe eventos guardados.
 */
const char *configurarRiego(uint32_t lluvia,uint32_t nft,uint32_t llenado,uint32_t drenaje) {
  if(ocupado() || enAuto) return "OCUPADO";
  for(auto t:{lluvia,nft,llenado,drenaje}) if(!t || t>Configuracion::DURACION_CONFIG_MAX_MS) return "PARAMETRO_INVALIDO";
  lluviaMs=lluvia; nftMs=nft; llenadoMs=llenado; drenajeMs=drenaje; return "OK";
}
/**
 * Configura una regla de un paso en RAM con confirmación/rearme e histéresis
 * positivos. Esta API tiene restricciones distintas de los valores iniciales
 * del header, donde confirmación/rearme/histéresis pueden empezar en cero.
 */
const char *configurarHumedad(unsigned tipo,unsigned destino,uint32_t confirmar,uint32_t rearme,float histeresis) {
  if(ocupado() || enAuto) return "OCUPADO";
  if(tipo>2 || (tipo==2 ? destino!=0 : (destino<1 || destino>2)) || !confirmar || !rearme ||
     confirmar>Configuracion::DURACION_CONFIG_MAX_MS || rearme>Configuracion::DURACION_CONFIG_MAX_MS || !std::isfinite(histeresis) || histeresis<=0 || histeresis>100) return "PARAMETRO_INVALIDO";
  auto &r=automaticoHumedad;
  r.lista=true; r.metodo=tipo; r.destino=destino; r.confirmar=confirmar; r.rearme=rearme; r.histeresis=histeresis;
  r.cantidad=1; r.pasos[0]={tipo,destino}; r.siguiente=0; r.lote=false;
  r.armada=true; r.baja=r.alta=false; return "OK";
}
void humedad(float v,bool valida,uint32_t instante,int umbral) {
  auto &r=automaticoHumedad; r.valor=v; r.valida=valida && std::isfinite(v) && v>=0 && v<=100;
  r.instante=instante; r.umbral=umbral;
}
/**
 * Valida la entrada a AUTO según regla o agenda habilitada. Si hay regla de
 * humedad, exige un dato válido y reciente. El cambio cancela acciones previas;
 * no interpreta entrar a AUTO como recuperar un riego interrumpido.
 */
const char *modo(bool habilitar,bool agendaDisponible) {
  if(habilitar==enAuto) return "OK";
  const bool usarHumedad=automaticoHumedad.umbral>=0;
  if(habilitar && !usarHumedad && !agendaDisponible) return "REGLA_PENDIENTE";
  if(habilitar && usarHumedad && (!automaticoHumedad.lista ||
      automaticoHumedad.umbral+automaticoHumedad.histeresis>100)) return "REGLA_PENDIENTE";
  if(habilitar && (nivel(0)!=1 || std::strcmp(fallo,"NA"))) return "CONDICION_NO_SEGURA";
  if(habilitar && usarHumedad && (!automaticoHumedad.valida || uint32_t(millis()-automaticoHumedad.instante)>Configuracion::HUMEDAD_CADUCIDAD_MS)) return "HUMEDAD_INVALIDA";
  parar("CAMBIO_MODO",false); enAuto=habilitar;
  automaticoHumedad.armada=true; return "OK";
}
const char *regla() {
  if(!automaticoHumedad.lista || automaticoHumedad.umbral<0) return "INCOMPLETA";
  if(std::strcmp(fallo,"NA")) return "BLOQUEADA";
  return automaticoHumedad.lote ? "EN_EJECUCION" : automaticoHumedad.armada ? "LISTA" : "BLOQUEADA";
}
void iniciar() {
  for(unsigned i=0;i<6;++i) {
    gpio_set_level(static_cast<gpio_num_t>(Configuracion::RELES[i]),!Configuracion::RELE_ACTIVO);
    pinMode(Configuracion::RELES[i],OUTPUT); reles[i]=false;
  }
  for(auto pin:Configuracion::NIVELES) pinMode(pin,INPUT);
  leerNiveles();
}
void despuesMotores() {
  for(unsigned m=1;m<=2;++m) if(!std::strcmp(obtenerMovimiento(m).estado,"FALLO")) {
    if(!std::strcmp(fallo,"NA")) bloquear(obtenerMovimiento(m).motivo);
    return;
  }
}
/**
 * Servicio frecuente: primero nivel principal crudo (sin esperar filtro para
 * parar), luego temporizadores/flotadores del ciclo y finalmente regla de humedad.
 * En inundación el flotador decide transiciones; sus máximos limitan espera.
 * La reserva de agenda evita intercalar un lote de humedad entre M1 y M2.
 */
void actualizar() {
  leerNiveles();
  // La caída cruda del principal detiene antes de emitir el siguiente STEP.
  if((ocupado() || enAuto) && entradas[0].raw!=Configuracion::NIVEL_ACTIVO) { bloquear("NIVEL_PRINCIPAL_BAJO"); return; }
  const uint32_t ahora=millis();
  if(riegoActivo()) {
    if(limiteEventoMs && uint32_t(ahora-inicio)>=limiteEventoMs && metodo==1) {
      bloquear("TIMEOUT_EVENTO");
    } else if(etapa==Fase::DRENAJE) {
      if(nivel(modulo)==0) apagar();
      else if(uint32_t(ahora-cambio)>=drenajeMs) bloquear("TIMEOUT_DRENAJE");
    } else if(metodo!=2 && nivel(modulo)==1) {
      if(metodo==0) bloquear("NIVEL_MODULO_ACTIVO_EN_NFT");
      else { salida(5,false); salida(modulo-1,false); salida(modulo+1,true); cambiar(Fase::DRENAJE); }
    } else if(uint32_t(ahora-inicio)>=(metodo==1 ? llenadoMs : limiteEventoMs ? limiteEventoMs : metodo==2 ? lluviaMs : nftMs)) {
      if(metodo==1) bloquear("TIMEOUT_LLENADO"); else apagar();
    } else if(etapa==Fase::BOMBA && uint32_t(ahora-cambio)>=Configuracion::SEPARACION_RELES_MS) {
      salida(metodo==2 ? 4 : modulo-1,true);
      cambiar(metodo==1 ? Fase::LLENADO : metodo==0 ? Fase::SOLENOIDE : Fase::RIEGO);
    } else if(etapa==Fase::SOLENOIDE && uint32_t(ahora-cambio)>=Configuracion::SEPARACION_RELES_MS) {
      salida(modulo+1,true); cambiar(Fase::RIEGO);
    }
  }
  auto &r=automaticoHumedad;
  if(!enAuto) return;
  // Agenda sola no exige un umbral ni inventa humedad. Con regla configurada,
  // su fallo sigue deteniendo todo el automático, incluso un evento activo.
  if(r.umbral<0) return;
  if(!r.valida || uint32_t(ahora-r.instante)>Configuracion::HUMEDAD_CADUCIDAD_MS) {
    bloquear("HUMEDAD_INVALIDA"); return;
  }
  if(reservaAgenda) { r.baja=r.alta=false; return; }
  if(r.lote) {
    if(!riegoActivo()) {
      if(r.siguiente<r.cantidad) {
        const auto paso=r.pasos[r.siguiente];
        const char *resultado=regar(paso.destino,paso.metodo,false,true);
        if(!std::strcmp(resultado,"ACEPTADO")) ++r.siguiente;
        else bloquear(resultado);
      } else { r.lote=false; r.siguiente=0; r.baja=r.alta=false; }
    }
    return;
  }
  if(ocupado()) { r.baja=r.alta=false; return; }
  if(!r.armada) {
    if(r.valor>r.umbral+r.histeresis) {
      if(!r.alta) { r.alta=true; r.desdeAlta=ahora; }
      if(uint32_t(ahora-r.desdeAlta)>=r.rearme) { r.armada=true; r.alta=false; }
    } else r.alta=false;
  } else if(r.valor<=r.umbral) {
    if(!r.baja) { r.baja=true; r.desdeBaja=ahora; }
    if(uint32_t(ahora-r.desdeBaja)>=r.confirmar) {
      const auto paso=r.pasos[0];
      const char *resultado=regar(paso.destino,paso.metodo,false,true);
      r.baja=false;
      if(!std::strcmp(resultado,"ACEPTADO")) { r.armada=false; r.lote=true; r.siguiente=1; }
    }
  } else r.baja=false;
}
} // namespace Proceso
