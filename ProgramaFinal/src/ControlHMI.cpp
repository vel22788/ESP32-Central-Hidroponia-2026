/**
 * @file ControlHMI.cpp
 * Protocolo ASCII V2, estado presentado a la HMI, historial/curvas RAM y agenda NVS.
 * Dos entradas independientes: UART HMI y consola central para el puente PC.
 * Cada puerto conserva su sesión, latido, IDs y resultados de órdenes.
 *
 * Una orden ACEPTADA todavía puede estar en ejecución. ESTADO VER permite
 * consultarla. Repetir un ID resuelto devuelve su respuesta sin repetir la
 * actuación; si el resultado ya no está en caché se indica desconocido.
 * Las consultas se regeneran para que las edades de datos sigan aumentando.
 *
 * Agenda: hasta veinte registros físicos; un límite de altas menor no borra los
 * existentes. Persistencia con CRC y relectura. Historial, curvas y umbral son RAM.
 * Evaluación por minuto del RTC local, sin recuperar minutos atrasados ni repetir
 * un minuto al atrasar el reloj. AMBOS reserva el proceso y ejecuta M1, luego M2.
 * Los GPIO se accionan mediante Proceso; este archivo interpreta y coordina.
 */
#include <esp_system.h>
#include <math.h>
#include <time.h>
#include <limits.h>
#include <cstring>
// Protocolo ASCII V2 derivado del banco probado; datos y RTC fisicos.
// Sin simulación: las órdenes se enlazan con los módulos físicos 01-04.
#include "ControlHMI.h"
#include "Plataforma.h"
#include "Configuracion.h"
#include "RelojRTC.h"
#include "ModeloEC.h"
#include "PinesSensores.h"

#include <Wire.h>
#include <Preferences.h>
extern const char *estadoCalibracionADS();
extern float factorEC();
extern float gananciaPT();
extern float offsetPT();
namespace ControlHMI {






HardwareSerial HMI(Configuracion::UART_HMI);
#define Consola Serial0

const int RX_HMI = Configuracion::RX_HMI;
const int TX_HMI = Configuracion::TX_HMI;
const unsigned BAUDIOS = Configuracion::BAUDIOS_HMI;
// Historial, muestras y umbral en RAM. Agenda propia en NVS con CRC.
const uint32_t PERDIDA_HMI_MS = Configuracion::PERDIDA_ENLACE_MS;
const unsigned MAX_LINEA = Configuracion::MAX_LINEA;

// Capacidad fija del formato NVS v1. LIMITE_EVENTOS regula nuevas altas.
const unsigned MAX_EVENTOS = 20;
const unsigned MAX_HISTORIAL = Configuracion::MAX_HISTORIAL;
const unsigned MAX_MUESTRAS = Configuracion::MAX_MUESTRAS_CURVAS;
const unsigned MAX_CACHE = Configuracion::MAX_CACHE_ORDENES;  // por puerto
const unsigned MAX_VISTA = MAX_HISTORIAL > MAX_MUESTRAS ?
  (MAX_HISTORIAL > MAX_EVENTOS ? MAX_HISTORIAL : MAX_EVENTOS) :
  (MAX_MUESTRAS > MAX_EVENTOS ? MAX_MUESTRAS : MAX_EVENTOS);

/**
 * Representación acotada de una línea: palabras de comando y campos CLAVE=VALOR.
 * ID identifica una petición dentro de su puerto; no identifica un evento de agenda.
 */
struct Comando {
  String palabra[4];
  String clave[24];
  String valor[24];
  unsigned palabras = 0;
  unsigned campos = 0;
  uint32_t id = 0;
  bool deHmi = false;
};
struct Entrada {
  String linea;
  uint32_t ultimoByte = 0;
  bool descartar = false;
  bool cr = false;
};
struct OrdenGuardada {
  uint32_t id = 0;
  String comando;
  String respuesta;
  String familia;
};
struct Evento {
  uint32_t id = 0;
  String hora, metodo, destino, dias;
  uint32_t duracion = 0;
  bool habilitado = false;
};
struct Medicion {
  uint32_t numero = 0;
  uint32_t instante = 0;
  uint32_t instanteCampo[5] = {};
  String fechaCampo[5] = {"NA", "NA", "NA", "NA", "NA"};
  String fecha = "NA";
  double valor[5] = {};
  uint8_t calidad[5] = {};  // 0 SIN_DATOS; 1 OK; 2 ERROR
};
/**
 * Snapshot paginado de listas. Un cursor pertenece a esta vista y a su puerto;
 * no es un índice permanente de agenda/historial/curvas.
 */
struct Vista {
  String fila[MAX_VISTA];
  unsigned cantidad = 0;
  uint32_t serial = 0;
  uint32_t revision = 0;
};
Entrada entradaHmi, entradaPc;
OrdenGuardada cache[2][MAX_CACHE];
unsigned siguienteCache[2] = {};
uint32_t ultimoId[2] = {};


Evento agenda[MAX_EVENTOS], respaldoAgenda[MAX_EVENTOS];
unsigned eventos=0;
uint32_t proximoEvento=1;
bool agendaPersistente=false;
String estadoAgenda="VACIA";
String historial[MAX_HISTORIAL];
unsigned registros = 0;
Medicion muestras[MAX_MUESTRAS], actual;
unsigned mediciones = 0;
Vista vistas[2][3];  // cada puerto: agenda, historial, curvas
String boot;
const char *sensores[] = {"PH", "CE", "TAMB", "HAMB", "TSOL"};
bool enlaceHmi = false, enlacePc=false;
uint32_t ultimoPingPc=0;
uint32_t ultimoPingHmi = 0;
uint32_t revision = 0, revisionConfig = 0, revisionAgenda = 0;
uint32_t proximaAccion = 1, proximoRegistro = 1, proximaVista = 1;
int umbral = Configuracion::UMBRAL_HUMEDAD_INICIAL;
int obtenerUmbral() { return umbral; }
bool configPersistente = false;
String estadoConfig = "SIN_CONFIGURACION";
void (*controlAdquisicion)(bool) = nullptr;
void (*controlAlimentacionPH)(bool) = nullptr;
bool adquisicionActiva = true;
void cargarConfiguracion();
bool guardarConfiguracion();
void cambioModoAgenda();
String colaTx[Configuracion::COLA_TX];
unsigned cabezaTx = 0, cantidadTx = 0, posicionTx = 0;
uint32_t descartadasTx = 0, recibidasHmi = 0, enviadasHmi = 0;


// ---------- Filtrado basico y campos NOMBRE=VALOR ----------
/**
 * Analiza entero decimal sin signo y rechaza vacíos, signos, letras y overflow.
 * No usa conversión parcial: toda la cadena debe representar el número.
 */
bool entero(const String &texto, uint32_t &numero) {
  if(texto.length() == 0) return false;
  numero = 0;
  for(unsigned i = 0; i < texto.length(); i++) {
    char c = texto[i];
    if(c < '0' || c > '9') return false;
    unsigned digito = c - '0';
    if(numero > (UINT32_MAX - digito) / 10U) return false;
    numero = numero * 10U + digito;
  }
  return true;
}

String campo(const Comando &c, const String &nombre) {
  for(unsigned i = 0; i < c.campos; i++) {
    if(c.clave[i] == nombre) return c.valor[i];
  }
  return "";
}

bool opcion(const String &valor, const String &lista) {
  if(valor.length() == 0) return false;
  return (String("|") + lista + "|").indexOf(String("|") + valor + "|") >= 0;
}

bool camposValidos(const Comando &c, const String &lista) {
  for(unsigned i = 0; i < c.campos; i++) {
    if(c.clave[i] != "ID" && !opcion(c.clave[i], lista)) return false;
  }
  return true;
}

/**
 * Separa palabras y pares CLAVE=VALOR; rechaza duplicados, exceso de campos
 * y sintaxis inválida. La semántica específica se comprueba al interpretar.
 */
bool separar(const String &linea, Comando &c) {
  unsigned pos = 0;
  while(pos < linea.length()) {
    while(pos < linea.length() && linea[pos] == ' ') pos++;
    if(pos == linea.length()) break;
    unsigned fin = pos;
    while(fin < linea.length() && linea[fin] != ' ') fin++;
    String parte = linea.substring(pos, fin);
    pos = fin;
    int igual = parte.indexOf('=');
    if(igual < 0) {
      if(c.palabras == 4) return false;
      c.palabra[c.palabras++] = parte;
      continue;
    }
    if(igual == 0 || igual == int(parte.length()) - 1 || c.campos == 24) return false;
    String clave = parte.substring(0, igual);
    String valor = parte.substring(igual + 1);
    if(valor.indexOf('=') >= 0 || campo(c, clave).length()) return false;
    for(unsigned i = 0; i < clave.length(); i++) {
      char ch = clave[i];
      if(ch != '_' && !(ch >= 'A' && ch <= 'Z') && !(ch >= '0' && ch <= '9')) return false;
    }
    c.clave[c.campos] = clave;
    c.valor[c.campos++] = valor;
  }
  String id = campo(c, "ID");
  if(id.length() && (!entero(id, c.id) || c.id == 0)) return false;
  return c.palabras > 0;
}

// ---------- Respuestas y seguimiento por puerto ----------
/**
 * HMI: encola líneas para no bloquear loop al transmitir. PC: imprime con
 * prefijo TX_PC> que el puente filtra. Una cola llena incrementa descartadasTx;
 * no se presenta la encolación como confirmación de recepción.
 */
void enviar(bool aHmi, const String &linea) {
  if(aHmi) {
    if(cantidadTx == Configuracion::COLA_TX) { ++descartadasTx; return; }
    colaTx[(cabezaTx + cantidadTx) % Configuracion::COLA_TX] = linea + "\r\n";
    ++cantidadTx;
  } else Consola.println(String("TX_PC> ") + linea);
}
/**
 * Vacía como máximo el presupuesto de bytes y lo que acepta la UART.
 * Conserva posición dentro de la línea para continuar en la siguiente vuelta.
 */
void vaciarTx() {
  unsigned presupuesto = Configuracion::TX_BYTES_POR_VUELTA;
  while(cantidadTx && presupuesto) {
    const unsigned libre = HMI.availableForWrite();
    if(!libre) return;
    const String &linea = colaTx[cabezaTx];
    unsigned n = linea.length() - posicionTx;
    if(n > libre) n = libre;
    if(n > presupuesto) n = presupuesto;
    const size_t escritos = HMI.write(reinterpret_cast<const uint8_t *>(linea.c_str() + posicionTx), n);
    posicionTx += escritos; presupuesto -= escritos;
    if(posicionTx == linea.length()) {
      colaTx[cabezaTx] = ""; cabezaTx = (cabezaTx + 1) % Configuracion::COLA_TX;
      --cantidadTx; posicionTx = 0; ++enviadasHmi;
    }
    if(!escritos) return;
  }
}

OrdenGuardada *buscarOrden(bool deHmi, uint32_t id) {
  unsigned puerto = deHmi ? 0 : 1;
  for(unsigned i = 0; i < MAX_CACHE; i++) {
    if(cache[puerto][i].id == id) return &cache[puerto][i];
  }
  return nullptr;
}

/**
 * Asocia respuesta al ID y puerto; guarda resultados de cambios/aceptaciones
 * para deduplicar. Las respuestas de consulta no se congelan en esa caché.
 */
void responder(const Comando &c, const String &tipo, const String &datos = "") {
  String respuesta = tipo + " ID=" + String(c.id);
  if(datos.length()) respuesta += " " + datos;
  enviar(c.deHmi, respuesta);
  if(c.id && (tipo.startsWith("OK ") || tipo.startsWith("ACEPTADO ") || tipo == "ERROR")) {
    OrdenGuardada *orden = buscarOrden(c.deHmi, c.id);
    if(orden) orden->respuesta = respuesta;
  }
}

void error(const Comando &c, const String &codigo) {
  responder(c, "ERROR", "CODIGO=" + codigo);
}

// ---------- Calendario fisico, adquirido por el modulo ambiental ----------
String fechaActual() { return RelojRTC::fechaActual(); }

/**
 * Añade un registro RAM con BOOT y contador para identificarlo. Cuando se
 * llena, retira el más antiguo. Reiniciar pierde historial y cambia BOOT.
 */
void registrar(const String &codigo, const String &detalle) {
  if(registros == MAX_HISTORIAL) {
    for(unsigned i = 1; i < registros; i++) historial[i - 1] = historial[i];
    registros--;
  }
  historial[registros++] = "REGISTRO REGISTRO_ID=" + boot + ":" + String(proximoRegistro++) +
    " FECHA=" + fechaActual() + " CODIGO=" + codigo + (detalle.length() ? " " + detalle : "");
}

/**
 * Crea una entrada del histórico RAM a partir de la última copia agregada.
 * Un número nuevo puede actualizar solo algunos campos: sus fechas siguen
 * siendo las de adquisición, no la fecha global de publicación.
 */
void publicarMuestra() {
  if(actual.numero == UINT32_MAX) return;
  actual.numero++;
  actual.instante = millis();
  actual.fecha = fechaActual();
  if(mediciones == MAX_MUESTRAS) {
    for(unsigned i = 1; i < mediciones; i++) muestras[i - 1] = muestras[i];
    mediciones--;
  }
  muestras[mediciones++] = actual;
}

/**
 * Serializa valor, calidad, fecha y opcionalmente edad por campo. PROVISIONAL
 * conserva valor; ERROR/SIN_DATOS usan NA. No inventa un cero ante fallo.
 */
String datosMedicion(const Medicion &m, bool conEdad) {
  String texto;
  for(unsigned i = 0; i < 5; i++) {
    String nombre = sensores[i];
    String calidad = m.calidad[i] == 3 ? "PROVISIONAL" : m.calidad[i] == 1 ? "OK" : m.calidad[i] == 2 ? "ERROR" : "SIN_DATOS";
    texto += " " + nombre + "=" + ((m.calidad[i] == 1 || m.calidad[i] == 3) ? String(m.valor[i], 4) : String("NA"));
    texto += " " + nombre + "_Q=" + calidad;
    texto += " " + nombre + "_TS=" + (m.calidad[i] ? m.fechaCampo[i] : String("NA"));
    if(conEdad) texto += " " + nombre + "_MS=" + (m.calidad[i] ? String(uint32_t(millis() - m.instanteCampo[i])) : String("NA"));
  }
  return texto;
}

String firmaEstado() {
  String f=" MODO="+String(Proceso::automatico() ? "AUTO" : "MANUAL");
  const int p=Proceso::nivel(0), a=Proceso::nivel(1), b=Proceso::nivel(2);
  f+=" NIVEL_P="+String(p<0 ? "DESCONOCIDO" : p ? "SEGURO" : "BAJO");
  f+=" NIVEL_M1="+String(a<0 ? "DESCONOCIDO" : a ? "SUFICIENTE" : "INSUFICIENTE");
  f+=" NIVEL_M2="+String(b<0 ? "DESCONOCIDO" : b ? "SUFICIENTE" : "INSUFICIENTE");
  f+=" MOV_M1="+String(Proceso::movimiento(0))+" MOV_M2="+String(Proceso::movimiento(1));
  const char *n[]={"NFT_M1","NFT_M2","INUND_M1","INUND_M2","LLUVIA"};
  for(unsigned i=0;i<5;++i) f+=" "+String(n[i])+"="+Proceso::riego(i);
  return f+" REGLA="+Proceso::regla()+" ALARMA="+Proceso::alarma();
}
String estadoActual() {
  static String anterior;
  const String firma=firmaEstado();
  if(firma!=anterior) { anterior=firma; if(revision!=UINT32_MAX) ++revision; }
  return "BOOT="+boot+" REV="+String(revision)+firma;
}
bool horaValida(const String &hora) {
  if(hora.length() != 5 || hora[2] != ':') return false;
  uint32_t h, m;
  return entero(hora.substring(0, 2), h) && entero(hora.substring(3), m) && h < 24 && m < 60;
}

bool diasValidos(const String &texto) {
  const char *dias[] = {"LUN", "MAR", "MIE", "JUE", "VIE", "SAB", "DOM"};
  if(!texto.length()) return false;
  unsigned pos = 0;
  int anterior = -1;
  while(pos < texto.length()) {
    int fin = texto.indexOf(',', pos);
    if(fin < 0) fin = texto.length();
    String dia = texto.substring(pos, fin);
    int encontrado = -1;
    for(int i = 0; i < 7; i++) if(dia == dias[i]) encontrado = i;
    if(encontrado <= anterior) return false;
    anterior = encontrado;
    pos = fin + 1;
    if(pos == texto.length()) return false; // coma final
  }
  return true;
}



// Registro independiente: no recuperar ni sobrescribir agendas del banco anterior.
// Un solo blob NVS con CRC; confirmación tras releer. Error incierto inhibe AUTO.
struct EventoGuardado {
  uint32_t id, duracion;
  char hora[6], metodo[12], destino[10], dias[28];
  uint8_t habilitado;
};
struct AgendaGuardada {
  uint32_t version, revision, siguiente, cantidad;
  EventoGuardado evento[MAX_EVENTOS];
  uint32_t crc;
};
AgendaGuardada agendaEscritura, agendaVerificacion;
bool textoTerminado(const char *s,size_t n) { return std::memchr(s,0,n)!=nullptr; }
/**
 * Verifica versión, CRC, campos terminados en cero, IDs únicos, calendario
 * semanal y opciones permitidas. MAX_EVENTOS es capacidad del formato persistente;
 * LIMITE_EVENTOS se aplica a altas, no invalida registros ya guardados.
 */
bool agendaValida(const AgendaGuardada &r) {
  if(r.version!=1 || r.cantidad>MAX_EVENTOS || !r.siguiente ||
      r.crc!=ModeloEC::crc32(&r,offsetof(AgendaGuardada,crc))) return false;
  for(unsigned i=0;i<r.cantidad;++i) {
    const auto &e=r.evento[i];
    if(!textoTerminado(e.hora,sizeof(e.hora)) || !textoTerminado(e.metodo,sizeof(e.metodo)) ||
       !textoTerminado(e.destino,sizeof(e.destino)) || !textoTerminado(e.dias,sizeof(e.dias)) ||
       !e.id || e.id>=r.siguiente || !e.duracion || e.duracion>3600 || e.habilitado>1 ||
       !horaValida(e.hora) || !diasValidos(e.dias) || !opcion(e.metodo,"NFT|INUNDACION|LLUVIA") ||
       !opcion(e.destino,String(e.metodo)=="LLUVIA" ? "VERTICAL" : "M1|M2|AMBOS")) return false;
    for(unsigned j=0;j<i;++j) if(e.id==r.evento[j].id) return false;
  }
  return true;
}
/**
 * Lee solo el espacio agenda_final/registro. Una agenda inválida no se
 * convierte silenciosamente en una agenda utilizable; no importa la del banco.
 */
void cargarAgenda() {
  eventos=0; proximoEvento=1; revisionAgenda=0; agendaPersistente=false; estadoAgenda="VACIA";
  Preferences nvs;
  if(!nvs.begin("agenda_final",true)) { estadoAgenda="NO_DISPONIBLE"; return; }
  if(!nvs.isKey("registro")) { nvs.end(); return; }
  const bool ok=nvs.getBytesLength("registro")==sizeof(agendaVerificacion) &&
    nvs.getBytes("registro",&agendaVerificacion,sizeof(agendaVerificacion))==sizeof(agendaVerificacion) &&
    agendaValida(agendaVerificacion);
  nvs.end();
  if(!ok) { estadoAgenda="INVALIDA"; return; }
  const auto &r=agendaVerificacion;
  eventos=r.cantidad; proximoEvento=r.siguiente; revisionAgenda=r.revision;
  for(unsigned i=0;i<eventos;++i) {
    const auto &e=r.evento[i];
    agenda[i].id=e.id; agenda[i].duracion=e.duracion; agenda[i].hora=e.hora;
    agenda[i].metodo=e.metodo; agenda[i].destino=e.destino; agenda[i].dias=e.dias;
    agenda[i].habilitado=e.habilitado;
  }
  agendaPersistente=true; estadoAgenda="RECUPERADA";
}
/**
 * Construye un blob con CRC, lo escribe y relee para comparar byte a byte.
 * Si no puede confirmar, marca INCIERTA e impide AUTO; no afirma persistencia.
 * Este espacio es distinto de las calibraciones EC/PT y pH.
 */
bool guardarAgenda() {
  if(estadoAgenda=="INVALIDA" || estadoAgenda=="INCIERTA") return false;
  agendaEscritura={}; auto &r=agendaEscritura;
  r.version=1; r.revision=revisionAgenda; r.siguiente=proximoEvento; r.cantidad=eventos;
  for(unsigned i=0;i<eventos;++i) {
    const auto &e=agenda[i]; auto &g=r.evento[i];
    g.id=e.id; g.duracion=e.duracion; g.habilitado=e.habilitado;
    snprintf(g.hora,sizeof(g.hora),"%s",e.hora.c_str());
    snprintf(g.metodo,sizeof(g.metodo),"%s",e.metodo.c_str());
    snprintf(g.destino,sizeof(g.destino),"%s",e.destino.c_str());
    snprintf(g.dias,sizeof(g.dias),"%s",e.dias.c_str());
  }
  r.crc=ModeloEC::crc32(&r,offsetof(AgendaGuardada,crc));
  if(!agendaValida(r)) return false;
  Preferences nvs;
  if(!nvs.begin("agenda_final",false)) return false;
  const bool ok=nvs.putBytes("registro",&r,sizeof(r))==sizeof(r) &&
    nvs.getBytes("registro",&agendaVerificacion,sizeof(r))==sizeof(r) &&
    std::memcmp(&r,&agendaVerificacion,sizeof(r))==0;
  nvs.end();
  if(!ok) { estadoAgenda="INCIERTA"; agendaPersistente=false; return false; }
  estadoAgenda="GUARDADA"; agendaPersistente=true; return true;
}
bool agendaHabilitada() {
  if(estadoAgenda=="INVALIDA" || estadoAgenda=="INCIERTA") return false;
  for(unsigned i=0;i<eventos;++i) if(agenda[i].habilitado) return true;
  return false;
}
// Umbral y perfiles siguen volátiles; guardar agenda no persiste esos parámetros.
void cargarConfiguracion() { estadoConfig="UMBRAL_VOLATIL_AGENDA_NVS"; cargarAgenda(); }
bool guardarConfiguracion() { configPersistente=false; return true; }
int indiceEvento(uint32_t id) {
  for(unsigned i = 0; i < eventos; i++) if(agenda[i].id == id) return i;
  return -1;
}

String filaEvento(const Evento &e) {
  return "EVENTO EVENTO_ID=" + String(e.id) + " HORA=" + e.hora + " METODO=" + e.metodo +
    " DESTINO=" + e.destino + " DURACION_S=" + String(e.duracion) + " DIAS=" + e.dias +
    " HABILITADO=" + String(e.habilitado ? 1 : 0);
}



/**
 * Construye snapshot al recibir CURSOR=INICIO y devuelve hasta diez filas
 * por página. Los siguientes cursores consultan la misma copia, aunque cambie
 * la lista real; un cursor que no pertenece a esa vista se rechaza.
 */
void consultarLista(const Comando &c, unsigned tipo) {
  uint32_t limite;
  String cursor = campo(c, "CURSOR");
  if(c.palabras != 2 || !camposValidos(c, "CURSOR|LIMITE") ||
     !entero(campo(c, "LIMITE"), limite) || limite < 1 || limite > 10) {
    error(c, "PARAMETRO_INVALIDO"); return;
  }
  Vista &v = vistas[c.deHmi ? 0 : 1][tipo];
  unsigned inicio = 0;
  if(cursor == "INICIO") {
    v.serial = proximaVista++;
    v.revision = revisionAgenda;
    v.cantidad = tipo == 0 ? eventos : tipo == 1 ? registros : mediciones;
    for(unsigned i = 0; i < v.cantidad; i++) {
      if(tipo == 0) v.fila[i] = filaEvento(agenda[i]);
      if(tipo == 1) v.fila[i] = historial[registros - 1 - i];
      if(tipo == 2) v.fila[i] = "MUESTRA BOOT=" + boot + " NUM=" + String(muestras[i].numero) + datosMedicion(muestras[i], false);
    }
  } else {
    int separador = cursor.indexOf(':');
    uint32_t serial, posicion;
    if(separador < 0 || !entero(cursor.substring(0, separador), serial) ||
       !entero(cursor.substring(separador + 1), posicion) || serial != v.serial ||
       !v.serial || posicion >= v.cantidad) {
      error(c, "SNAPSHOT_VENCIDO"); return;
    }
    inicio = posicion;
  }
  unsigned cantidad = v.cantidad - inicio;
  if(cantidad > limite) cantidad = limite;
  String familia = tipo == 0 ? "AGENDA" : tipo == 1 ? "HISTORIAL" : "CURVAS";
  String campos = "SNAP=" + String(v.serial) + " TOTAL=" + String(cantidad);
  if(tipo == 0) {
    campos += " REV_AGENDA=" + String(v.revision);
  }
  responder(c, familia + " INICIO", campos);
  for(unsigned i = inicio; i < inicio + cantidad; i++) responder(c, v.fila[i]);
  unsigned siguiente = inicio + cantidad;
  String sig = siguiente == v.cantidad ? String("FIN") : String(v.serial) + ":" + String(siguiente);
  responder(c, familia + " FIN", "TOTAL=" + String(cantidad) + " SIG=" + sig);
}

/**
 * Alta/edición/habilitación/eliminación solo en MANUAL y reposo. Conserva
 * respaldo RAM para revertir si falla la escritura. Confirma OK después de
 * guardar; un fallo incierto puede exigir revisión del estado persistente.
 */
void cambiarAgenda(const Comando &c) {
  if(Proceso::automatico()) { error(c,"REQUIERE_MODO_MANUAL"); return; }
  if(Proceso::ocupado()) { error(c,"OCUPADO"); return; }
  if(estadoAgenda=="INVALIDA" || estadoAgenda=="INCIERTA") { error(c,"ERROR_NVS"); return; }
  const unsigned eventosAntes = eventos;
  const uint32_t proximoAntes = proximoEvento;
  for(unsigned i = 0; i < eventos; ++i) respaldoAgenda[i] = agenda[i];
  String accion = c.palabra[1];
  bool agregar = accion == "AGREGAR";
  bool editar = accion == "EDITAR";
  if(c.palabras != 2 || !opcion(accion, "AGREGAR|EDITAR|ACTIVAR|DESACTIVAR|ELIMINAR")) {
    error(c, "PARAMETRO_INVALIDO"); return;
  }
  uint32_t id = 0;
  int indice = -1;
  if(!agregar) {
    if(!entero(campo(c, "EVENTO_ID"), id) || !id) { error(c, "PARAMETRO_INVALIDO"); return; }
    indice = indiceEvento(id);
    if(indice < 0) { error(c, "EVENTO_NO_EXISTE"); return; }
  }
  if(agregar || editar) {
    String permitidos = "HORA|METODO|DESTINO|DURACION_S|DIAS|HABILITADO";
    if(editar) permitidos += "|EVENTO_ID";
    Evento nuevo;
    uint32_t habilitado;
    nuevo.hora = campo(c, "HORA"); nuevo.metodo = campo(c, "METODO");
    nuevo.destino = campo(c, "DESTINO"); nuevo.dias = campo(c, "DIAS");
    if(!camposValidos(c, permitidos) || !horaValida(nuevo.hora) || !diasValidos(nuevo.dias) ||
       !opcion(nuevo.metodo, "NFT|INUNDACION|LLUVIA") ||
       !opcion(nuevo.destino, nuevo.metodo == "LLUVIA" ? "VERTICAL" : "M1|M2|AMBOS") ||
       !entero(campo(c, "DURACION_S"), nuevo.duracion) || !nuevo.duracion || nuevo.duracion>Configuracion::EVENTO_MAX_S ||
       !entero(campo(c, "HABILITADO"), habilitado) || habilitado > 1) {
      error(c, "PARAMETRO_INVALIDO"); return;
    }
    if(agregar && eventos >= Configuracion::LIMITE_EVENTOS) { error(c, "AGENDA_LLENA"); return; }
    if(agregar && proximoEvento == UINT32_MAX) { error(c, "AGENDA_LLENA"); return; }
    nuevo.id = agregar ? proximoEvento++ : id;
    nuevo.habilitado = habilitado;
    if(agregar) indice = eventos++;
    agenda[indice] = nuevo;
    id = nuevo.id;
  } else {
    if(!camposValidos(c, "EVENTO_ID")) { error(c, "PARAMETRO_INVALIDO"); return; }
    if(accion == "ELIMINAR") {
      for(unsigned i = indice + 1; i < eventos; i++) agenda[i - 1] = agenda[i];
      eventos--;
    } else agenda[indice].habilitado = accion == "ACTIVAR";
  }
  if(revisionAgenda == UINT32_MAX) {
    eventos = eventosAntes; proximoEvento = proximoAntes;
    for(unsigned i = 0; i < eventos; ++i) agenda[i] = respaldoAgenda[i];
    error(c, "CONFIG_AGOTADA"); return;
  }
  revisionAgenda++;
  if(!guardarAgenda()) {
    eventos = eventosAntes; proximoEvento = proximoAntes; --revisionAgenda;
    for(unsigned i = 0; i < eventos; ++i) agenda[i] = respaldoAgenda[i];
    error(c, "ERROR_NVS"); return;
  }
  responder(c, "OK AGENDA", "EVENTO_ID=" + String(id) + " REV_AGENDA=" + String(revisionAgenda) + " PERSISTENTE=" + String(agendaPersistente ? 1 : 0));
  registrar("AGENDA_" + accion, "EVENTO_ID=" + String(id) + " ORIGEN=" + (c.deHmi ? "HMI" : "PC"));
  enviar(true, "AVISO AGENDA REV_AGENDA=" + String(revisionAgenda));
}

// Órdenes reales; el mismo protocolo sirve por UART directo o por el puente USB.
bool enlaceVivo(bool hmi) {
  return (hmi ? enlaceHmi : enlacePc) && uint32_t(millis()-(hmi ? ultimoPingHmi : ultimoPingPc))<=PERDIDA_HMI_MS;
}
/**
 * Traduce métodos/destinos a Proceso y exige enlace vivo para iniciar.
 * Las órdenes de parada no exigen ese latido: no se bloquea un intento de parar.
 * ACEPTADO identifica admisión; los estados posteriores describen la ejecución.
 */
void controlManual(const Comando &c) {
  const String nombre=c.palabra[0], destino=c.palabra[1];
  if(nombre=="RIEGO") {
    const String metodo=c.palabra[2];
    if(c.palabras!=4 || !camposValidos(c,"") || !opcion(c.palabra[3],"ON|OFF") ||
       (!(destino=="VERTICAL" && metodo=="LLUVIA") &&
        !(opcion(destino,"M1|M2") && opcion(metodo,"NFT|INUNDACION")))) { error(c,"PARAMETRO_INVALIDO"); return; }
    const unsigned m=destino=="M1" ? 1 : destino=="M2" ? 2 : 0;
    const unsigned tipo=metodo=="NFT" ? 0 : metodo=="INUNDACION" ? 1 : 2;
    if(c.palabra[3]=="OFF") {
      const unsigned i=tipo==2 ? 4 : tipo==0 ? m-1 : m+1;
      if(std::strcmp(Proceso::riego(i),"INACTIVO")) Proceso::parar("PARADA_RIEGO");
      responder(c,"OK RIEGO","DESTINO="+destino+" METODO="+metodo+" ESTADO=INACTIVO"); return;
    }
    if(!enlaceVivo(c.deHmi)) { error(c,"ENLACE_NO_CONFIRMADO"); return; }
    const char *r=Proceso::regar(m,tipo,c.deHmi);
    if(std::strcmp(r,"ACEPTADO")) { error(c,r); return; }
    responder(c,"ACEPTADO RIEGO","DESTINO="+destino+" METODO="+metodo+" ACCION="+String(proximaAccion++));
    registrar("RIEGO_INICIO","DESTINO="+destino+" METODO="+metodo); return;
  }
  const bool parar=nombre=="PARAR";
  if(c.palabras!=(parar ? 2u : 3u) || !camposValidos(c,"") ||
     (!opcion(destino,"M1|M2") && !(parar && destino=="MOVIMIENTOS")) ||
     (!parar && !opcion(c.palabra[2],"SUBIR|BAJAR|NIVELAR|INCLINAR"))) { error(c,"PARAMETRO_INVALIDO"); return; }
  const unsigned m=destino=="M1" ? 1 : destino=="M2" ? 2 : 0;
  if(parar) {
    Proceso::pararModulo(m);
    responder(c,"OK PARAR","DESTINO="+(m ? destino : String("AMBOS"))+" ESTADO=DETENIDO"); return;
  }
  if(!enlaceVivo(c.deHmi)) { error(c,"ENLACE_NO_CONFIRMADO"); return; }
  const auto accion=c.palabra[2]=="SUBIR" ? AccionMovimiento::SUBIR : c.palabra[2]=="BAJAR" ? AccionMovimiento::BAJAR :
    c.palabra[2]=="NIVELAR" ? AccionMovimiento::NIVELAR : AccionMovimiento::INCLINAR;
  const char *r=Proceso::mover(m,accion,c.deHmi);
  if(!std::strcmp(r,"EN_DESTINO")) { responder(c,"OK MOVER","DESTINO="+destino+" ESTADO=DETENIDO"); return; }
  if(std::strcmp(r,"ACEPTADO") && std::strcmp(r,"YA_EN_CURSO")) { error(c,r); return; }
  responder(c,"ACEPTADO MOVER","DESTINO="+destino+" ACCION="+String(proximaAccion++));
  registrar("MOVIMIENTO_INICIO","DESTINO="+destino+" TIPO="+c.palabra[2]);
}

// ---------- Identificacion del comando completo ----------
/**
 * Despachador de comandos válidos. Separa consulta, configuración, actuación
 * y ajuste de reloj. SIM está deshabilitado; PH no permite alimentación directa.
 * CONFIG es de consola PC y sus cambios son RAM, no coeficientes de calibración.
 */
void interpretar(const Comando &c) {
  String nombre = c.palabra[0];
  if(nombre == "PING" && c.palabras == 1 && camposValidos(c, "")) {
    if(c.deHmi) { ultimoPingHmi=millis(); enlaceHmi=true; }
    else { ultimoPingPc=millis(); enlacePc=true; }
    responder(c, "PONG"); return;
  }
  if(nombre == "DATOS" && c.palabras == 2 && c.palabra[1] == "VER" && camposValidos(c, "")) {
    responder(c, "DATOS", "BOOT=" + boot + " MUESTRA=" + String(actual.numero) + datosMedicion(actual, true)); return;
  }
  if(nombre == "ESTADO" && c.palabras == 2 && c.palabra[1] == "VER" && camposValidos(c, "")) {
    responder(c, "ESTADO", estadoActual()); return;
  }
  if(nombre=="MODO" && c.palabras==2 && camposValidos(c,"")) {
    if(c.palabra[1]=="VER") { responder(c,"MODO","VALOR="+String(Proceso::automatico() ? "AUTO" : "MANUAL")+" REV="+String(revision)); return; }
    if(!opcion(c.palabra[1],"AUTO|MANUAL")) { error(c,"PARAMETRO_INVALIDO"); return; }
    const bool usarAgenda=agendaHabilitada();
    if(c.palabra[1]=="AUTO" && (estadoAgenda=="INVALIDA" || estadoAgenda=="INCIERTA")) { error(c,"ERROR_NVS"); return; }
    if(c.palabra[1]=="AUTO" && usarAgenda && !RelojRTC::obtener().valido) { error(c,"RTC_INVALIDO"); return; }
    const bool antes=Proceso::automatico();
    const char *r=Proceso::modo(c.palabra[1]=="AUTO",usarAgenda);
    if(std::strcmp(r,"OK")) { error(c,r); return; }
    if(antes!=Proceso::automatico()) cambioModoAgenda();
    estadoActual(); responder(c,"OK MODO","VALOR="+c.palabra[1]+" REV="+String(revision)); return;
  }
  if(nombre == "UMBRAL" && c.palabras == 2 && camposValidos(c, "")) {
    if(c.palabra[1] == "VER") {
      responder(c, "UMBRAL", "VALOR=" + String(umbral<0 ? 0 : umbral) + " UNIDAD=PCT REV_CONFIG=" + String(revisionConfig) + " PERSISTENTE=" + String(configPersistente ? 1 : 0)); return;
    }
    uint32_t valor;
    if(!entero(c.palabra[1], valor) || valor > 100) { error(c, "PARAMETRO_INVALIDO"); return; }
    if(revisionConfig == UINT32_MAX) { error(c, "CONFIG_AGOTADA"); return; }
    const int anterior = umbral;
    if(Proceso::automatico()) { error(c,"REQUIERE_MODO_MANUAL"); return; }
    umbral = valor; revisionConfig++;
    const auto ambiente=actual;
    Proceso::humedad(ambiente.valor[3],ambiente.calidad[3]==1,ambiente.instanteCampo[3],umbral);
    if(!guardarConfiguracion()) { umbral = anterior; --revisionConfig; error(c, "ERROR_NVS"); return; }
    responder(c, "OK UMBRAL", "VALOR=" + String(umbral<0 ? 0 : umbral) + " UNIDAD=PCT REV_CONFIG=" + String(revisionConfig) + " PERSISTENTE=" + String(configPersistente ? 1 : 0));
    registrar("UMBRAL_CAMBIADO", "VALOR=" + String(umbral<0 ? 0 : umbral) + " UNIDAD=PCT");
    enviar(true, "AVISO CONFIG REV_CONFIG=" + String(revisionConfig)); return;
  }
  if(nombre == "AGENDA") {
    if(c.palabra[1] == "VER") consultarLista(c, 0);
    else cambiarAgenda(c);
    return;
  }
  if((nombre == "HISTORIAL" || nombre == "CURVAS") && c.palabra[1] == "VER") {
    consultarLista(c, nombre == "HISTORIAL" ? 1 : 2); return;
  }
  if(nombre == "RIEGO" || nombre == "MOVER" || nombre == "PARAR") { controlManual(c); return; }
  if(nombre == "SIM") { error(c, "NO_DISPONIBLE"); return; }
  if(nombre=="PH") { error(c,"ALIMENTACION_CONTROLADA_POR_MODULO_01"); return; }
  if(nombre=="ALARMAS" && c.palabras==2 && c.palabra[1]=="RECONOCER" && camposValidos(c,"")) {
    const char *r=Proceso::reconocer();
    if(std::strcmp(r,"OK")) error(c,r); else responder(c,"OK ALARMAS","RECONOCIDA=1"); return;
  }
  if(nombre=="CONFIG" && !c.deHmi) {
    if(Proceso::ocupado()) { error(c,"OCUPADO"); return; }
    uint32_t a,b,d,e;
    if(c.palabras==2 && c.palabra[1]=="RIEGO" && camposValidos(c,"LLUVIA_MS|NFT_MS|LLENADO_MS|DRENAJE_MS") &&
       entero(campo(c,"LLUVIA_MS"),a) && entero(campo(c,"NFT_MS"),b) && entero(campo(c,"LLENADO_MS"),d) && entero(campo(c,"DRENAJE_MS"),e)) {
      const char *r=Proceso::configurarRiego(a,b,d,e);
      if(std::strcmp(r,"OK")) error(c,r); else responder(c,"OK CONFIG","PERSISTENTE=0"); return;
    }
    if(c.palabras==2 && c.palabra[1]=="HUMEDAD" && camposValidos(c,"METODO|DESTINO|CONFIRMAR_MS|REARME_MS|HISTERESIS") &&
       entero(campo(c,"CONFIRMAR_MS"),a) && entero(campo(c,"REARME_MS"),b) && entero(campo(c,"HISTERESIS"),d) &&
       opcion(campo(c,"METODO"),"NFT|INUNDACION|LLUVIA") && opcion(campo(c,"DESTINO"),"M1|M2|VERTICAL")) {
      const unsigned tipo=campo(c,"METODO")=="NFT" ? 0 : campo(c,"METODO")=="INUNDACION" ? 1 : 2;
      const unsigned destino=campo(c,"DESTINO")=="M1" ? 1 : campo(c,"DESTINO")=="M2" ? 2 : 0;
      const char *r=Proceso::configurarHumedad(tipo,destino,a,b,d);
      if(std::strcmp(r,"OK")) error(c,r); else responder(c,"OK CONFIG","PERSISTENTE=0"); return;
    }
    error(c,"PARAMETRO_INVALIDO"); return;
  }
  if(nombre == "ORDEN" && c.palabras == 2 && c.palabra[1] == "VER" && camposValidos(c, "NUM")) {
    uint32_t numero;
    if(!entero(campo(c, "NUM"), numero) || !numero) { error(c, "PARAMETRO_INVALIDO"); return; }
    OrdenGuardada *guardada = buscarOrden(c.deHmi, numero);
    if(!guardada || !guardada->respuesta.length()) { error(c, "RESULTADO_DESCONOCIDO"); return; }
    Comando original;
    if(!separar(guardada->respuesta, original)) { error(c, "INTERNO"); return; }
    String datos = "NUM=" + String(numero) + " ESTADO=RESUELTA RESPUESTA=" + original.palabra[0] + " FAMILIA=" + guardada->familia;
    for(unsigned i = 0; i < original.campos; i++) {
      if(original.clave[i] != "ID" && original.clave[i] != "ESTADO") datos += " " + original.clave[i] + "=" + original.valor[i];
    }
    responder(c, "ORDEN", datos); return;
  }
  if(nombre == "RELOJ" && c.palabras == 2 && c.palabra[1] == "VER" && camposValidos(c, "")) {
    String fecha = fechaActual();
    responder(c, "RELOJ", RelojRTC::obtener().valido ? "VALIDO=1 FECHA=" + fecha.substring(0,10) + " HORA=" + fecha.substring(11,19) + " ZONA=" + String(Configuracion::ZONA_LOCAL) : "VALIDO=0 FECHA=NA HORA=NA ZONA=" + String(Configuracion::ZONA_LOCAL)); return;
  }
  if(nombre == "RELOJ" && c.palabras == 2 && c.palabra[1] == "AJUSTAR" && camposValidos(c, "FECHA|HORA")) {
    if(Proceso::ocupado() || Proceso::automatico()) { error(c,"OCUPADO"); return; }
    time_t valor;
    if(!RelojRTC::calendarioValido(campo(c, "FECHA"), campo(c, "HORA"), valor)) { error(c, "PARAMETRO_INVALIDO"); return; }
    if(!RelojRTC::ajustar(campo(c, "FECHA"), campo(c, "HORA"))) { error(c, "RTC_INVALIDO"); return; }
    registrar("RELOJ_AJUSTADO", "ORIGEN=" + String(c.deHmi ? "HMI" : "PC"));
    responder(c, "OK RELOJ", "VALIDO=1 FECHA=" + campo(c, "FECHA") + " HORA=" + campo(c, "HORA") + " ZONA=" + String(Configuracion::ZONA_LOCAL)); return;
  }
  if(nombre == "DIAG" && c.palabras == 2 && c.palabra[1] == "VER" && camposValidos(c, "")) {
    responder(c, "DIAG", String("FIRMWARE=") + Configuracion::FIRMWARE + " EVENTOS=AGENDA_NVS CE=SIN_COMPENSACION PH=AJUSTE_1P MOTORES=REALES RELES=REALES NIVELES=REALES FINALES=REALES RTC=" +
      String(RelojRTC::obtener().estado) + " CAL_EC_PT=" + String(estadoCalibracionADS()) +
      " K=" + String(factorEC(), 6) + " PT_G=" + String(gananciaPT(), 8) + " PT_O=" + String(offsetPT(), 6) +
      " AGENDA=" + estadoAgenda + " CONFIG=" + estadoConfig + " ADQUISICION=" + String(adquisicionActiva ? 1 : 0) +
      " PH_MANDO=" + String(digitalRead(PinesSensores::PH) ? "ON" : "OFF") +
      " PH_GPIO=" + String(PinesSensores::PH) + " EC_GPIO=" + String(PinesSensores::EC) +
      " RX_HMI=" + String(recibidasHmi) + " TX_HMI=" + String(enviadasHmi) + " TX_DESCARTADAS=" + String(descartadasTx)); return;
  }
  if(nombre == "I2C" && c.palabras == 2 && c.palabra[1] == "VER" && camposValidos(c, "")) {
    if(Proceso::ocupado()) { error(c,"OCUPADO"); return; }
    String informe, errores;
    const uint8_t direcciones[] = {Configuracion::ADS_DIRECCION, Configuracion::SHT_DIRECCION, uint8_t(Configuracion::SHT_DIRECCION==0x44 ? 0x45 : 0x44), Configuracion::RTC_DIRECCION};
    char nombres[4][8];
    for(unsigned i=0;i<4;++i) snprintf(nombres[i],sizeof(nombres[i]),"%s_%02X",i==0 ? "ADS" : i==3 ? "RTC" : "SHT",direcciones[i]);
    for(unsigned i = 0; i < 4; ++i) {
      Wire.beginTransmission(direcciones[i]);
      const uint8_t resultado = Wire.endTransmission();
      if(i) informe += " ";
      informe += String(nombres[i]) + "=" + String(resultado == 0 ? 1 : 0);
      errores += " " + String(nombres[i]) + "_ERR=" + String(resultado);
    }
    informe += errores + " SDA_PIN=" + String(Configuracion::I2C_SDA) + " SCL_PIN=" + String(Configuracion::I2C_SCL) + " SDA=" + String(digitalRead(Configuracion::I2C_SDA)) + " SCL=" + String(digitalRead(Configuracion::I2C_SCL));
    responder(c, "I2C", informe); return;
  }
  if(opcion(nombre, "CONFIG|REGLA|ALARMAS|DIAG|RELOJ")) { error(c, "NO_DISPONIBLE"); return; }
  if(opcion(nombre, "PING|DATOS|ESTADO|MODO|UMBRAL|HISTORIAL|CURVAS")) {
    error(c, "PARAMETRO_INVALIDO"); return;
  }
  error(c, "COMANDO_DESCONOCIDO");
}

/**
 * Negocia HOLA VERSION=2, atiende consola y controla IDs por puerto.
 * Mismo ID+mismo texto resuelto devuelve respuesta; texto distinto da conflicto.
 * Un ID antiguo fuera de caché no provoca repetir una actuación desconocida.
 * La consola admite consultas sin ID; la HMI exige ID salvo negociación HOLA.
 */
void procesarLinea(String linea, bool deHmi) {
  linea.trim();
  if(!linea.length()) return;
  if(Configuracion::TRAZAS_RECEPCION) Consola.println(String(deHmi ? "RX_HMI> " : "RX_PC> ") + linea);
  if(deHmi) ++recibidasHmi;
  if(!deHmi && (linea == "AYUDA" || linea == "h")) {
    Consola.println(String(Configuracion::FIRMWARE) + ": modulos 01-05; agenda NVS; PH provisional.");
    Consola.println("DIAG VER | I2C VER | DATOS VER | ESTADO VER | RELOJ VER");
    Consola.println("RELOJ AJUSTAR FECHA=AAAA-MM-DD HORA=HH:MM:SS (hora local Guatemala)");
    Consola.println("UMBRAL VER | AGENDA/HISTORIAL/CURVAS VER CURSOR=INICIO LIMITE=10");
    Consola.println("0 detener adquisicion | 1 reanudar. SIM no esta disponible.");
    Consola.println("HOLA VERSION=2 y PING mantienen cada enlace; RIEGO M1 NFT ON/OFF; MOVER M1 NIVELAR/INCLINAR/SUBIR/BAJAR");
    Consola.println("ALARMAS RECONOCER; CONFIG RIEGO LLUVIA_MS=" + String(Configuracion::LLUVIA_MS) + " NFT_MS=" + String(Configuracion::NFT_MS) + " LLENADO_MS=" + String(Configuracion::LLENADO_MAX_MS) + " DRENAJE_MS=" + String(Configuracion::DRENAJE_MAX_MS));
    Consola.println("CONFIG HUMEDAD METODO=... DESTINO=... CONFIRMAR_MS=... REARME_MS=... HISTERESIS=...");
    return;
  }
  if(!deHmi && (linea == "0" || linea == "1")) {
    if(linea=="0") Proceso::parar("PARADA_GENERAL");
    adquisicionActiva = linea == "1";
    if(controlAdquisicion) controlAdquisicion(adquisicionActiva);
    Consola.println(adquisicionActiva ? "ADQUISICION_REANUDADA" : "ADQUISICION_DETENIDA; pH/EC LOW; PT1000 permanente.");
    return;
  }
  Comando c;
  c.deHmi = deHmi;
  if(linea.length() > MAX_LINEA || !separar(linea, c)) { error(c, "PARAMETRO_INVALIDO"); return; }
  unsigned puerto = deHmi ? 0 : 1;
  if(c.palabra[0] == "HOLA") {
    if(c.palabras != 1 || c.campos != 1 || campo(c, "VERSION") != "2") {
      error(c, "VERSION_INCOMPATIBLE"); return;
    }
    if(deHmi) { ultimoPingHmi=millis(); enlaceHmi=true; }
    else { ultimoPingPc=millis(); enlacePc=true; }
    uint32_t siguiente = ultimoId[puerto] == UINT32_MAX ? UINT32_MAX : ultimoId[puerto] + 1;
    enviar(deHmi, "HOLA VERSION=2 BOOT=" + boot + " PROX_ID=" + String(siguiente)); return;
  }
  if(deHmi && !c.id) { error(c, "PARAMETRO_INVALIDO"); return; }
  if(c.id) {
    OrdenGuardada *anterior = buscarOrden(deHmi, c.id);
    if(anterior) {
      if(anterior->comando != linea) { enviar(deHmi, "ERROR ID=" + String(c.id) + " CODIGO=ID_CONFLICTO"); return; }
      if(anterior->respuesta.length()) { enviar(deHmi, anterior->respuesta); return; }
      // Las consultas se regeneran: no congelar PH_MS/CE_MS/... en un cache.
    } else {
      if(c.id <= ultimoId[puerto]) { enviar(deHmi, "ERROR ID=" + String(c.id) + " CODIGO=RESULTADO_DESCONOCIDO"); return; }
      ultimoId[puerto] = c.id;
      OrdenGuardada &nueva = cache[puerto][siguienteCache[puerto]];
      siguienteCache[puerto] = (siguienteCache[puerto] + 1) % MAX_CACHE;
      nueva.id = c.id; nueva.comando = linea; nueva.respuesta = ""; nueva.familia = c.palabra[0];
    }
  }
  interpretar(c);
}

// ---------- Recepcion por linea; no se usa readStringUntil ni delay ----------
/**
 * Acumula caracteres hasta LF (admite CRLF) con presupuesto de bytes.
 * Caracteres inválidos, línea larga o incompleta activan descarte hasta LF;
 * no se ejecuta como orden un fragmento recibido a medias.
 */
void atenderPuerto(HardwareSerial &puerto, Entrada &e, bool deHmi) {
  unsigned presupuesto = Configuracion::RX_BYTES_POR_VUELTA;
  while(puerto.available() && presupuesto--) {
    int dato = puerto.read();
    e.ultimoByte = millis();
    if(dato == '\n') {
      if(e.descartar) enviar(deHmi, "ERROR ID=0 CODIGO=PARAMETRO_INVALIDO");
      else procesarLinea(e.linea, deHmi);
      e.linea = ""; e.descartar = false; e.cr = false;
      continue;
    }
    if(e.descartar) continue;
    if(dato == '\r' && !e.cr) { e.cr = true; continue; }
    unsigned maximo = deHmi ? MAX_LINEA : MAX_LINEA + 11;
    if(e.cr || dato < 32 || dato > 126 || e.linea.length() >= maximo) {
      e.linea = ""; e.descartar = true;
    } else e.linea += char(dato);
  }
  if((e.linea.length() || e.cr) && uint32_t(millis() - e.ultimoByte) > Configuracion::LINEA_INCOMPLETA_MS) {
    e.linea = ""; e.cr = false; e.descartar = true;
  }
}


// Sin recuperación de minutos perdidos ni repetición tras un retroceso de reloj.
// Arranque, entrada en AUTO y recuperación de RTC esperan el próximo minuto.
int64_t minutoEvaluado=-1;
bool agendaAutoAnterior=false, relojAnterior=false, agendaEnCurso=false, segundoPendiente=false;
Evento eventoEnCurso;
void cambioModoAgenda() { agendaAutoAnterior=false; }
unsigned metodoEvento(const Evento &e) { return e.metodo=="NFT" ? 0 : e.metodo=="INUNDACION" ? 1 : 2; }
String detalleEvento(const Evento &e) {
  return "EVENTO_ID="+String(e.id)+" DESTINO="+e.destino+" METODO="+e.metodo+" ORIGEN=AGENDA";
}
void terminarEvento(const char *motivo) {
  registrar("EVENTO_FIN",detalleEvento(eventoEnCurso)+" MOTIVO="+motivo);
  agendaEnCurso=segundoPendiente=false; Proceso::reservarAgenda(false);
}
/**
 * Supervisa evento en curso, RTC y próximo minuto elegible. No recupera
 * minutos perdidos. Para AMBOS mantiene reserva entre M1 y M2; un recurso
 * ocupado omite el evento y deja evidencia. El calendario decide cuándo;
 * Proceso ejecuta etapas/flotadores y aplica la duración en ms.
 */
void evaluarAgenda() {
  const bool automatico=Proceso::automatico();
  const auto reloj=RelojRTC::obtener();
  if(agendaEnCurso) {
    if(!automatico) terminarEvento(std::strcmp(Proceso::alarma(),"NA") ? Proceso::alarma() : "CANCELADO");
    else if(!reloj.valido) { Proceso::bloquear("RTC_INVALIDO"); terminarEvento("RTC_INVALIDO"); }
    else if(!Proceso::riegoActivo()) {
      if(segundoPendiente) {
        segundoPendiente=false;
        const char *r=Proceso::evento(2,metodoEvento(eventoEnCurso),eventoEnCurso.duracion*1000U);
        if(std::strcmp(r,"ACEPTADO")) terminarEvento(r);
      } else terminarEvento("COMPLETADO");
    }
  }
  if(!reloj.valido) { relojAnterior=false; agendaAutoAnterior=Proceso::automatico(); return; }
  const time_t local=reloj.calendario+uint32_t(millis()-reloj.instante)/1000;
  const int64_t minuto=int64_t(local)/60;
  const bool omitirActual=!relojAnterior || !Proceso::automatico() || !agendaAutoAnterior;
  relojAnterior=true; agendaAutoAnterior=Proceso::automatico();
  if(omitirActual) { if(minuto>minutoEvaluado) minutoEvaluado=minuto; return; }
  if(minuto<=minutoEvaluado) return;
  minutoEvaluado=minuto;
  if(!agendaHabilitada()) return;
  struct tm t; if(!gmtime_r(&local,&t)) return;
  char hora[6]; strftime(hora,sizeof(hora),"%H:%M",&t);
  const char *dias[]={"DOM","LUN","MAR","MIE","JUE","VIE","SAB"};
  for(unsigned i=0;i<eventos;++i) {
    const Evento &e=agenda[i];
    if(!e.habilitado || e.hora!=hora || (String(",")+e.dias+",").indexOf(String(",")+dias[t.tm_wday]+",")<0) continue;
    if(Proceso::ocupado()) { registrar("EVENTO_OMITIDO",detalleEvento(e)+" MOTIVO=OCUPADO"); continue; }
    Proceso::reservarAgenda(true);
    const unsigned m=e.destino=="VERTICAL" ? 0 : e.destino=="M2" ? 2 : 1;
    const char *r=Proceso::evento(m,metodoEvento(e),e.duracion*1000U);
    if(std::strcmp(r,"ACEPTADO")) {
      Proceso::reservarAgenda(false); registrar("EVENTO_OMITIDO",detalleEvento(e)+" MOTIVO="+r);
    } else {
      eventoEnCurso=e; agendaEnCurso=true; segundoPendiente=e.destino=="AMBOS";
      registrar("EVENTO_INICIO",detalleEvento(e)+" DURACION_S="+String(e.duracion));
    }
  }
}

/**
 * Primero vence los watchdogs por puerto, luego recibe, evalúa agenda,
 * registra cambios de estado y transmite. Un latido PC no mantiene una orden
 * manual perteneciente a la HMI, ni viceversa.
 */
void actualizar() {
  // Vencer el enlace antes de aceptar nuevas órdenes o generar otro STEP.
  if(enlaceHmi && uint32_t(millis()-ultimoPingHmi)>PERDIDA_HMI_MS) { enlaceHmi=false; Proceso::perderEnlace(true); }
  if(enlacePc && uint32_t(millis()-ultimoPingPc)>PERDIDA_HMI_MS) { enlacePc=false; Proceso::perderEnlace(false); }
  atenderPuerto(HMI,entradaHmi,true); atenderPuerto(Consola,entradaPc,false);
  evaluarAgenda();
  static String estadoAnterior;
  const String estado=firmaEstado()+" FASE="+Proceso::fase();
  if(estado!=estadoAnterior) {
    estadoAnterior=estado;
    registrar("ESTADO_CAMBIO","FASE="+String(Proceso::fase())+" ALARMA="+Proceso::alarma());
  }
  vaciarTx();
}
/**
 * Recibe solo mediciones. Rechaza como ERROR cualquier valor no finito
 * marcado OK/PROVISIONAL, y conserva tiempos individuales para la HMI.
 */
void publicar(const Lectura &lectura) {
  for(unsigned i = 0; i < 5; ++i) {
    actual.valor[i] = lectura.valor[i]; actual.calidad[i] = lectura.calidad[i];
    actual.instanteCampo[i] = lectura.instante[i]; actual.fechaCampo[i] = lectura.fecha[i];
    if((actual.calidad[i] == 1 || actual.calidad[i] == 3) && !isfinite(actual.valor[i])) actual.calidad[i] = 2;
  }
  publicarMuestra();
}
/**
 * Guarda callbacks, inicia UART y genera BOOT aleatorio para esta sesión.
 * Carga agenda pero no retoma actuaciones ni escribe las calibraciones.
 */
void iniciar(void (*adquisicion)(bool), void (*alimentacionPH)(bool)) {
  controlAdquisicion = adquisicion;
  controlAlimentacionPH = alimentacionPH;
  if(controlAlimentacionPH) controlAlimentacionPH(false);
  HMI.setRxBufferSize(Configuracion::HMI_BUFFER_RX);
  HMI.begin(BAUDIOS, SERIAL_8N1, RX_HMI, TX_HMI);
  entradaHmi.linea.reserve(MAX_LINEA + 1);
  entradaPc.linea.reserve(MAX_LINEA + 1);
  char identificador[17];
  snprintf(identificador, sizeof(identificador), "%08lX%08lX", (unsigned long)esp_random(), (unsigned long)esp_random());
  boot = identificador;
  cargarConfiguracion();
  registrar("ARRANQUE", String("FIRMWARE=") + Configuracion::FIRMWARE + " PH=PROVISIONAL EVENTOS=AGENDA_NVS");
  Consola.println(String(Configuracion::FIRMWARE) + " lista. Sensores reales; RTC; HMI RX" + String(RX_HMI) + "/TX" + String(TX_HMI) + ". AYUDA.");
}
} // namespace ControlHMI
