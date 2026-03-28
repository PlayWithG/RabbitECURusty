# DSG DQ250 Standalone — rusEFI Lua Script

Script Lua para control universal de la transmisión **DSG DQ250** (VW/Audi 6 velocidades, doble embrague) usando **rusEFI** como ECU completa.

---

## Propósito

A diferencia de `dsg_dq250_control.lua` (que requiere conservar la red CAN VW original del vehículo donante), este script está diseñado para **instalaciones universales**:

| Componente | control.lua | standalone.lua |
|---|---|---|
| Red CAN VW | Requerida (ABS, BCM, etc.) | **No requerida** |
| Fuente de RPM/MAP/CLT | Sensores VW originales | Sensores físicos de rusEFI |
| Fuente de VSS | Mensaje CAN 0x5A0 del ABS | `getSensor("VehicleSpeed")` |
| Fuente de freno | Mensaje CAN 0x1AC | `getDigital(2)` |
| Módulos VW conservados | Todos los originales | Solo DSG DQ250 + selector |

**rusEFI simula en CAN todos los módulos VW** que la TCU DSG espera recibir.

---

## Requisitos de Hardware

- **rusEFI** en cualquier plataforma compatible (Proteus, Hellen, Frankenso, etc.)
- **CAN bus a 500 kbit/s** — configurar en TunerStudio
- Señales de motor conectadas a rusEFI:
  - RPM (sensor de cigüeñal/árbol de levas)
  - MAP (sensor de presión en colector)
  - CLT (temperatura de refrigerante)
  - TPS (posición del acelerador)
  - PPS (posición del pedal, si existe)
  - VSS (velocidad del vehículo — rueda o caja de cambios)
  - Switch de freno → **entrada digital configurada como freno** en rusEFI
- **DSG DQ250** con su selector/palancas conectados directamente a la TCU DSG

---

## Arquitectura CAN

### Recepción (CAN RX)

| ID | Fuente | Datos usados |
|---|---|---|
| `0x440` | DSG DQ250 TCU | Torque limit, marcha engranada, flags de cambio |

### Transmisión (CAN TX simulada)

| ID | Periodo | Módulo simulado | Contenido |
|---|---|---|---|
| `0x280` | 10 ms | ECU principal | RPM × 4, torque modificado, torque estimado (MAP), pedal |
| `0x380` | 10 ms | ECU principal | Posición de pedal (espejo) |
| `0x488` | 10 ms | ECU principal | Presión MAP/boost codificada + contador + checksum XOR |
| `0x288` | 20 ms | ECU principal | Temperatura CLT, VSS, freno, target ISC, escala de torque |
| `0x480` | 20 ms | ECU principal | Combustible consumido + checksum XOR |
| `0x362` | 20 ms | ECU principal | Contador auxiliar + temperatura |
| `0x284` | 20 ms | ECU principal | Contador doble (DLC=6) |
| `0x48A` | 20 ms | ECU principal | Nibble de contador |
| `0x588` | 20 ms | ECU principal | Handshake fijo |
| `0x5A0` | 100 ms | Módulo ABS/ESP | VSS simulado, contador de distancia 11 bits, marcha |
| `0x580` | 1000 ms | ECU principal | Contador de arranque |
| `0x050` | 1000 ms | BCM/Gateway | Modo de encendido activo (fijo) |

---

## Calibración

Todos los parámetros ajustables están en la sección `CALIBRACIÓN` al inicio del script. **No es necesario recompilar**.

### Relación VSS/RPM por marcha

```lua
local VSSPerRPM = { 22, 43, 67, 93, 120, 150 }  -- km/h por 1000 rpm, marchas 1-6
```

Calcular para tu vehículo:

```
VSS_por_1000rpm[n] = (circunferencia_rueda_m × 1000) / relacion_total_marcha_n
```

Ejemplo rueda 225/40R18 (circ. ≈ 1.936 m), marcha 1 relación 3.308 × diferencial 3.47:
`1936 / (3.308 × 3.47) ≈ 169 mm/rev → 22 km/h por 1000 rpm`

### Estimación de torque desde MAP

```lua
local MAP_MIN = 28.8    -- kPa: MAP en ralentí
local MAP_MAX = 174.5   -- kPa: MAP a plena carga
```

| Tipo de motor | MAP_MIN | MAP_MAX |
|---|---|---|
| Naturalmente aspirado | 25 | 98 |
| Turbo moderado (≤0.6 bar) | 25 | 160 |
| Turbo estándar (≤1.0 bar) | 28 | 200 |
| Turbo alto boost (>1.0 bar) | 28 | 220 |

El torque estimado se mapea linealmente: `MAP_MIN → 0`, `MAP_MAX → 255`.

### Duración de reducción de torque en cambios

```lua
local SHIFT_UP_COUNT   = 15   -- 75 ms (ticks × 5 ms)
local SHIFT_DOWN_COUNT = 20   -- 100 ms
local SHIFT_DOWN_BLIP  =  8   -- 40 ms de blip rev-match
```

### Bus CAN

```lua
local CAN_BUS = 1   -- 1 = CAN1, 2 = CAN2 (Proteus tiene dos buses)
```

---

## Lógica de Control de Torque

Idéntica a `dsg_dq250_control.lua`, derivada de `TORQUE.c` en RabbitECURusty:

### Detección de cambio (0x440 byte3)

| Bit | Evento | Acción |
|---|---|---|
| `0x04` | Upshift en progreso | Iniciar contador, optional quick cut |
| `0x04 + 0x80` | Embrague engranando post-subida | Activar reducción EST |
| `0x01` | Downshift en progreso | Activar reducción + rev-match |
| `0x10` | Cambio completado (normal) | Restaurar timing/fuel/ETB |

### Modificador de torque EST (escala 0–256)

```
256 = sin reducción (100%)
110 = reducción automático (43% — fase temprana)
 10 = reducción manual paletas (4% — fase temprana)
```

**Fase tardía** (guiada por slip del embrague):
- RPM slip < 150 → subir modifier gradualmente (embrague engranando)
- RPM slip > 250 → bajar modifier (embrague deslizando)
- RPM slip 150-250 → mantener modifier

### Rev-Match en bajadas (3 fases ETB)

1. **Blip** (`SHIFT_DOWN_BLIP` ticks): ETB add = `COLD_BLIP_PCT` o `HOT_BLIP_PCT` interpolado por CLT
2. **Ramp down** (ticks 8-38 post-blip): reducir ETB add a 0
3. **Zero** (ticks 38-58): ETB add = 0
4. **Ramp up** (hasta fin): ETB add sigue al pedal del conductor

---

## Entradas rusEFI utilizadas

| Función rusEFI | Valor retornado | Uso en el script |
|---|---|---|
| `getSensor("Rpm")` | RPM del motor | TX 0x280 bytes 2-3, cálculo slip |
| `getSensor("Map")` | kPa (presión MAP) | TX 0x488, estimación de torque |
| `getSensor("Clt")` | °C (temperatura refrigerante) | TX 0x288 byte1, calibración blip |
| `getSensor("Tps1")` | 0-100% (posición válvula) | Flag TPS cerrado en 0x280 |
| `getSensor("AcceleratorPedal")` | 0-100% (pedal conductor) | TX 0x280/0x380, rev-match |
| `getSensor("VehicleSpeed")` | km/h | TX 0x288 byte3, 0x5A0, flags VSS |
| `getDigital(2)` | bool (switch de freno) | TX 0x288 byte2 |
| `getConsumedGrams()` | gramos de combustible | TX 0x480 bytes 2-3 |

> **Nota sobre el freno:** `getDigital(2)` retorna `brakePedalState`, que es el switch de freno configurado en TunerStudio (Inputs → Switch Inputs → Brake switch). No requiere pin configurable en el script.

---

## Checksums XOR (mensajes 0x488 y 0x480)

Implementados según `USER_DIAG_APPEND_XOR()` de `DIAG.h`:

```
byte7 = byte0 XOR byte1 XOR byte2 XOR byte3 XOR byte4 XOR byte5 XOR byte6
```

Función en el script:
```lua
local function xorFill(t)
    local x = 0
    for i = 1, #t - 1 do x = x ~ t[i] end
    t[#t] = x & 0xFF
    return t
end
```

---

## Gauges en TunerStudio

Configurar **Gauges → Lua Outputs** con los siguientes índices:

| LuaGauge | Descripción | Rango |
|---|---|---|
| 1 | Marcha engranada (del 0x440) | 0-6 |
| 2 | Modificador EST de torque | 0-256 |
| 3 | RPM slip del embrague | 0-9999 |
| 4 | Contador de cambio (ticks) | 0-20 |
| 5 | VSS rusEFI (km/h) | 0-300 |
| 6 | Posición rev-match ETB (%) | 0-10 |
| 7 | Freno pisado | 0/1 |
| 8 | Modo manual paletas | 0/1 |

---

## Puesta en Marcha

1. Configurar CAN bus en TunerStudio a **500 kbit/s**
2. Verificar que el switch de freno está configurado en **Inputs → Switch Inputs → Brake switch**
3. Verificar que VSS está calibrado y reporta km/h correctos
4. Ajustar `VSSPerRPM[]` para la relación de transmisión del vehículo
5. Ajustar `MAP_MIN` / `MAP_MAX` para el tipo de motor
6. Cargar el script en **Lua Scripting** en TunerStudio
7. Con analizador CAN confirmar:
   - `0x280` cada 10 ms
   - `0x288` cada 20 ms
   - `0x5A0` cada 100 ms
8. Verificar que la TCU DSG responde con `0x440` (atxGear > 0 indica enganche)
9. Monitorear **LuaGauge 2** (modifier = 256 en reposo, baja durante cambio)

---

## Diferencias vs dsg_dq250_control.lua

| | control.lua | standalone.lua |
|---|---|---|
| **Uso** | Swap en vehículo VW original | Instalación universal en cualquier auto |
| **CAN RX** | 0x050, 0x5A0, 0x440, 0x1AC | Solo 0x440 |
| **CAN TX** | Solo 0x280 | 12 mensajes (simula ECU + ABS + BCM) |
| **VSS** | Calculado de pulsos CAN ABS | `getSensor("VehicleSpeed")` nativo |
| **Freno** | Mensaje CAN 0x1AC | `getDigital(2)` nativo |
| **Torque** | Hardcoded para Golf Mk6 GTI | Configurable MAP_MIN/MAP_MAX |
| **Timers** | Contador de ticks | `Timer.new()` preciso |

---

## Archivos de Referencia

| Archivo origen | Contenido relevante |
|---|---|
| `source/Client/DIAG.c` | Todos los mensajes CAN TX, estructuras de bytes, contadores |
| `source/Client/TORQUE.c` | Lógica de modificación de torque, rev-match |
| `source/Client/SENSORS.c` | Parsing de 0x440, cálculo VSS, RPM slip |
| `source/Client/EST.c` | Interpolación de mapas de encendido |
| `source/Client/FUEL.c` | Modificador de combustible, quick cut |
| `source/Client/USERCAL.h` | Constantes de calibración originales |
| `source/lua/dsg_dq250_control.lua` | Versión anterior (requiere red CAN VW) |
