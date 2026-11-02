# Plataforma hidropónica automatizada — ESP32 central

**Año:** 2026 · **Autor:** Ángel Velásquez · **Universidad del Valle de Guatemala**  
**Curso:** Diseño e Innovación en Ingeniería  
**Firmware:** `CENTRAL_FINAL_20261101_R4`

Este repositorio contiene el programa del controlador central ESP32-S3 de una
plataforma hidropónica automatizada. Integra adquisición de pH, conductividad,
PT1000 y ambiente, reloj DS3231, motores con finales de carrera, relés,
flotadores, riego y comunicación con la pantalla HMI.
## Estructura

```text
ProgramaFinal/
  ProgramaFinal.ino          Coordinador: setup y loop
  01_SensoresADS.ino         pH, EC y PT1000 por ADS1115
  02_SensorAmbiente.ino      SHT31 y servicio del RTC
  03_Motores.ino             Cuatro ejes y ocho finales de carrera
  04_RelesFlotadores.ino      Riego, niveles e interbloqueos
  05_HMI.ino                 Adaptador de comunicación
  src/
    Configuracion.h         Ajustes de operación y mapa de GPIO
    Plataforma.h            Contratos y datos compartidos
    ControlHMI.h/.cpp        Protocolo ASCII V2 y agenda
    RelojRTC.h/.cpp          Acceso y validación del DS3231
    ModeloEC.h              Conversión y registro de calibración EC/PT
    ModeloPH.h              Modelo de pH de dos puntos
    AjustePH1P.h             Ajuste provisional de pH
    PinesSensores.h         Alias de mandos de alimentación
    DFRobot_SHT3x/           Driver ambiental local y licencia
librerias/
  Adafruit_ADS1X15/          Dependencia del ADC
  Adafruit_BusIO/            Dependencia de ADS1X15
```

Los seis `.ino` forman **un solo sketch**. Conservar el nombre `ProgramaFinal`
de la carpeta, su `.ino` principal y todos los archivos de `src/`. El firmware
de pantalla y el puente USB de PC se entregan por separado.

## Requisitos

- Arduino IDE o Arduino CLI.
- Plataforma **ESP32 de Espressif 3.3.10**, utilizada en la verificación.
- Placa **ESP32S3 Dev Module**.
- **USB CDC On Boot deshabilitado**: GPIO20 se utiliza para el mando EC.
- Adafruit ADS1X15 **2.6.2** y Adafruit BusIO **1.17.4**, incluidas en `librerias/`.
- Arduino, Wire, SPI y Preferences los aporta la plataforma ESP32. El driver
  DFRobot SHT3x adaptado ya está dentro de `ProgramaFinal/src/`.

## Abrir y compilar

### Arduino IDE

1. Instalar la plataforma ESP32 de Espressif indicada.
2. Copiar `Adafruit_ADS1X15` y `Adafruit_BusIO` de `librerias/` a la carpeta
   `libraries` del sketchbook de Arduino, conservando sus nombres y contenido.
3. Abrir `ProgramaFinal/ProgramaFinal.ino` y seleccionar ESP32S3 Dev Module,
   con USB CDC On Boot deshabilitado.
4. Compilar el sketch. Arduino incorpora los otros cinco `.ino` automáticamente.

### Arduino CLI

Desde la raíz de este repositorio, con Arduino CLI instalado:

```sh
arduino-cli core install esp32:esp32@3.3.10 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=default --libraries ./librerias --output-dir ./build ./ProgramaFinal
```

El primer comando requiere internet. La compilación usa los fuentes y las dos
bibliotecas incluidas; no carga la placa.

## Configurar tiempos y hardware

Editar [`ProgramaFinal/src/Configuracion.h`](ProgramaFinal/src/Configuracion.h).
Allí se explican unidades y límites de cada grupo:

- Duraciones de lluvia/NFT, límites de llenado/drenaje y separación entre relés.
- Intervalo/ancho del pulso STEP, timeout y filtros de finales/niveles.
- Estabilización, muestras y cadencia ADS; ambiente y RTC.
- Regla por humedad, comunicaciones, buffers y mapa de GPIO/polaridades.

Después de editar, **recompilar y cargar** para aplicar cambios. `ms` son
milisegundos y `us` microsegundos. La duración de un evento se guarda en segundos
y no se reescribe al cambiar `NFT_MS`. `CONFIG RIEGO`, `CONFIG HUMEDAD` y `UMBRAL`
pueden sustituir ajustes en RAM hasta reiniciar. Las calibraciones EC/PT/pH
se recuperan de NVS; no se editan como tiempos en este header.

## Carga y consulta

Conectar la placa, comprobar el puerto real y cerrar cualquier monitor o puente
que lo esté ocupando. Retirar potencia de cargas antes del reinicio de carga.
Desde Arduino IDE, compilar/subir con la configuración indicada. Con CLI:

```sh
arduino-cli board list
arduino-cli upload --port COM_PUERTO_REAL --fqbn esp32:esp32:esp32s3:CDCOnBoot=default --input-dir ./build ./ProgramaFinal
```

Sustituir `COM_PUERTO_REAL` por el puerto comprobado. No borrar toda la flash:
calibraciones y agenda residen en NVS. Tras arrancar, comprobar MANUAL y salidas
en reposo. Consola del central a **115200 baudios**: `AYUDA`, `DIAG VER`,
`DATOS VER` y `ESTADO VER` permiten consultar sin iniciar una actuación.

UART directo a la HMI: central **RX41/TX42**, cruzado con HMI **TX43/RX44** y
GND común. La HMI compatible y el puente USB no forman parte de este repositorio.

## Comportamiento y límites conocidos

- Arranque en MANUAL; no se reanudan acciones interrumpidas.
- EC en mS/cm **sin compensación térmica**, independiente de la validez del PT1000.
- El pH de un punto conserva calidad PROVISIONAL; no acredita calibración formal.
- Se mantienen pendientes físicos documentados de PT1000 y ensayos completos de
  actuaciones/eventos de esta revisión. Un flotador no mide volumen de agua.
- Los finales y la lógica de proceso autorizan/paran maniobras; detener STEP no
  corta alimentación del driver, cuyo EN permanece conectado a GND.
- Historial/curvas/umbral son RAM; la agenda tiene su espacio NVS con CRC.
- La edición documentada compila con ESP32 3.3.10: **462 720 bytes de programa**
  y **45 136 bytes globales**. 

## Procedencia y licencias

Código de aplicación del proyecto y adaptación local de DFRobot SHT3x. Las
bibliotecas de terceros mantienen sus avisos y licencias en sus carpetas:

- [`DFRobot SHT3x`](ProgramaFinal/src/DFRobot_SHT3x/LICENSE).
- [`Adafruit ADS1X15`](librerias/Adafruit_ADS1X15/license.txt).
- [`Adafruit BusIO`](librerias/Adafruit_BusIO/LICENSE).


