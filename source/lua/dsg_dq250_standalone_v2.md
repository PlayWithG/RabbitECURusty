# DSG DQ250 Standalone v2 — Documentación

**Versión:** v2 mejorada con estimador de torque multi-método  
**Archivo:** `dsg_dq250_standalone_v2.lua`  
**Estado:** Producción  

## Cambios principales vs. v1

### 1. Estimador de torque 4 métodos + auto-selección

El estimador v1 usaba un mapeo lineal simple `MAP_MIN → 0, MAP_MAX → 255`. La v2 implementa:

| Método | Fuente | Precisión | Requisitos | Auto |
|--------|--------|-----------|-----------|------|
| **1: Lineal MAP** | MAP (kPa) | ⭐ Baja | Ninguno | Fallback final |
| **2: MAP×RPM** | MAP + corrección RPM | ⭐⭐ Media | RPM | 3er intento |
| **3: Tabla 3D TS** | tabla3d(1, RPM, MAP) | ⭐⭐⭐ Alta | TunerStudio | 2do intento |
| **4: MAF directo** | Sensor MAF (g/s) | ⭐⭐⭐⭐ Máxima | MAF calibrado | 1er intento |

**Selección automática (TORQUE_METHOD=0):**
```
MAF disponible? → Usar MAF
   NO ↓
Tabla3d configurada y válida? → Usar tabla3d
   NO ↓
MAP y RPM disponibles? → Usar MAP×RPM
   NO ↓
Usar lineal MAP (fallback universal)
```

**DSG siempre recibe un valor válido 0-255** — nunca falla.

---

### 2. Corrección por temperatura IAT (aire)

Nueva función `iatFactor(iat)` aplica **ley termodinámica** de densidad del aire:

```lua
factor = (273 + IAT_REF) / (273 + iat)
```

**Efecto:**
- **IAT = 25°C** (referencia): factor = 1.0 (sin cambio)
- **IAT = 0°C** (aire frío): factor ≈ 1.095 (torque +10%)
- **IAT = 50°C** (aire caliente): factor ≈ 0.915 (torque -8.5%)

Habilitación en calibración:
```lua
local IAT_CORRECTION = true    -- true = aplicar
local IAT_REF        = 25.0    -- °C de referencia
```

---

### 3. Nuevas funciones HELPERS

#### `clampedInterp(x1, y1, x2, y2, x)`
Interpolación que **NO extrapola** (patrón de rusReference `launch_control.lua`):
- Si `x ≤ x1`: devuelve `y1`
- Si `x ≥ x2`: devuelve `y2`
- En medio: interpola lineal con `interpolate()`

Previene valores fuera de rango (e.g., RPM correction factor > 1.0).

#### `rpmTqFactor(rpm)` 
Calcula factor de corrección RPM (0.0–1.0) según curva de motor:
```
RPM_IDLE (800)      → RPM_FACTOR_IDLE (0.65)
         ↓ interpolado
RPM_PEAK_TQ (3500)  → 1.0 (máximo torque)
         ↓ interpolado
RPM_REDLINE (7000)  → RPM_FACTOR_TOP (0.85)
```

Calibración:
```lua
local RPM_IDLE      = 800    -- RPM ralentí
local RPM_PEAK_TQ   = 3500   -- RPM torque máximo
local RPM_REDLINE   = 7000   -- RPM corte
local RPM_FACTOR_IDLE = 0.65 -- factor en ralentí
local RPM_FACTOR_TOP  = 0.85 -- factor en redline
```

#### `iatFactor(iat)`
Devuelve factor multiplicativo por temperatura del aire:
```
factor = (273 + 25) / (273 + iat)
```

#### `calcTorqueLinear(map)` (MÉTODO 1)
Mapeo simple lineal original:
```lua
tq = (map - MAP_MIN) / (MAP_MAX - MAP_MIN) × 255
```

#### `calcTorqueMapRpm(rpm, map)` (MÉTODO 2)
Base lineal MAP × factor RPM:
```lua
base = calcTorqueLinear(map)
tq = base × rpmTqFactor(rpm)
```

#### `calcTorqueTable(rpm, map)` (MÉTODO 3)
Lee tabla 3D configurada en TunerStudio:
```lua
tq = table3d(1, rpm, map)  -- tabla Lua #1
```

**Configuración en TunerStudio:**
- Crear "Lua Script Table" #1
- Eje X: RPM (0–7000 recomendado)
- Eje Y: MAP (0–250 kPa recomendado)
- Valores: 0–255 (escala torque estimado)

#### `calcTorqueMAF(rpm, maf)` (MÉTODO 4)
Directo desde flujo másico de aire:
```lua
massPerCycle = maf_g_s × 120 / rpm
ratio = massPerCycle / mafMassRef
tq = ratio × 255
```

**Inicialización `initMAFRef()`** (llamada antes de `setTickRate()`):
```lua
mafMassRef = displacement_cc × 0.00051
```

Fórmula derivada de VE≈85% a plena carga. Ajustable editando constante multiplicativa.

#### `calcTorqueEst(rpm, map, maf, iat)` (PRINCIPAL)
Orquesta los 4 métodos según `TORQUE_METHOD`:
- `0` = auto (cascada MAF→tabla→MAP×RPM→lineal)
- `1` = fuerza lineal MAP
- `2` = fuerza MAP×RPM
- `3` = fuerza tabla3d
- `4` = fuerza MAF

Aplica corrección IAT al resultado final y asegura rango 0–255.

Expone método activo en `lastTqMethod` (gauge 8).

---

## Nuevas calibraciones

### Estimador torque
```lua
local TORQUE_METHOD    = 0      -- método (0=auto, 1-4=forzado)
local RPM_IDLE         = 800
local RPM_PEAK_TQ      = 3500
local RPM_REDLINE      = 7000
local RPM_FACTOR_IDLE  = 0.65   -- factor torque en ralentí (vs. pico)
local RPM_FACTOR_TOP   = 0.85   -- factor torque en redline
```

### Corrección térmica
```lua
local IAT_CORRECTION   = true   -- habilitar/deshabilitar
local IAT_REF          = 25.0   -- °C referencia densidad nominal
```

---

## Cambios en `onTick()`

### Nuevas lecturas de sensores
```lua
local maf = getSensor("Maf")           -- g/s (nil si no disponible)
local iat = getSensor("Iat") or IAT_REF -- °C, con fallback
```

### Llamada a `calcTorqueEst()`
**v1:**
```lua
local tqEst = calcTorqueEst(map)
```

**v2:**
```lua
local tqEst = calcTorqueEst(rpm, map, maf, iat)
```

### LuaGauges actualizado
Gauge 8 cambió de `manualMode` a `lastTqMethod`:
```lua
setLuaGauge(8, lastTqMethod)   -- Método torque activo (1-4)
```

Útil para verificar qué método se está usando en tiempo real.

---

## Configuración recomendada en TunerStudio

### Opción A: Auto-selección (recomendado)
```
TORQUE_METHOD = 0   (auto)
```
Requiere:
- RPM válido ✓
- MAP válido ✓
- Sensores opcionalmente: MAF, tabla3d

El script intenta MAF → tabla → MAP×RPM → lineal automáticamente.

### Opción B: Tabla 3D configurada
```
TORQUE_METHOD = 3
```
Crear tabla "Lua Script Table #1":
- Eje X: RPM (0, 1000, 2000, 3000, 4000, 5000, 6000, 7000)
- Eje Y: MAP (0, 50, 100, 150, 200, 250)
- Valores: calibrar según dynamómetro / datos reales motor

**Ventaja:** Máxima precisión si motor caracterizado.

### Opción C: MAF directo
```
TORQUE_METHOD = 4
```
Requiere:
- Sensor MAF funcional y calibrado
- `getSensor("Maf")` devuelve g/s válido

**Ventaja:** Más preciso que MAP.

### Opción D: Fallback MAP×RPM (sin sensor extra)
```
TORQUE_METHOD = 2
```
Solo requiere RPM + MAP. Buen balance precisión/simplicidad.

---

## Instalación

1. **Copia v2 en rusEFI ECU:**
   ```bash
   cp source/lua/dsg_dq250_standalone_v2.lua /path/to/rusefi/firmware/lua_script.lua
   ```

2. **En TunerStudio:**
   - *Lua → Upload* → seleccionar `dsg_dq250_standalone_v2.lua`
   - *Calibration → Lua Script Tables* → crear tabla #1 si usas método 3
   - *Gauges → Lua* → verificar gauge 8 muestre método activo

3. **Monitoreo:**
   - Gauge 1: Marcha DSG actual
   - Gauge 8: Método torque activo (debug)
   - Terminal rusEFI: `get_sensor` verificar MAF, IAT

---

## Verificación funcional

### Log esperado al arranque
```
onTick() iniciando...
initMAFRef() → mafMassRef = 1.02 g/ciclo  (ej: 1998cc × 0.00051)
```

### Monitoreo en TunerStudio
1. Verificar gauge 8 = método usado (1-4)
2. Con TORQUE_METHOD=0 (auto):
   - Motor parado: método 1 (lineal)
   - Motor en ralentí, MAF disponible: método 4
   - MAP alto (carga): método 4 o 3 si tabla configurada
3. Confirmar torque estimado 0–255 válido

### Logs diagnóstico
```lua
-- Agregar temporalmente en onTick() para debug:
if rpm > 500 then
  print(string.format("rpm=%d map=%.1f maf=%s iat=%.1f method=%d tqEst=%d", 
        rpm, map, maf and string.format("%.2f", maf) or "nil", iat, lastTqMethod, tqEst))
end
```

---

## Diferencias técnicas clave vs. v1

| Aspecto | v1 | v2 |
|--------|----|----|
| Métodos torque | 1 (lineal) | 4 + auto |
| Corrección RPM | No | Sí (curva personalizable) |
| Corrección IAT | No | Sí (ley física) |
| Soporte MAF | No | Sí (sensor directo) |
| Tabla 3D | No | Sí (configurable) |
| Garantía fallback | No (lineal) | Sí (siempre 0-255) |
| LuaGauges debug | 8 (manualMode) | 8 (método activo) |

---

## Calibración recomendada por tipo de motor

### NA (aspiración natural)
```lua
TORQUE_METHOD    = 2       -- MAP×RPM generalmente suficiente
MAP_MIN          = 25      -- vacío
MAP_MAX          = 98      -- máximo típico NA
RPM_PEAK_TQ      = 4500    -- típico NA
IAT_CORRECTION   = true
```

### Turbo moderado (~0.6 bar)
```lua
TORQUE_METHOD    = 0       -- auto (MAF si disponible)
MAP_MIN          = 25
MAP_MAX          = 160     -- máximo turbo 0.6 bar
RPM_PEAK_TQ      = 3500    -- típico turbo
IAT_CORRECTION   = true    -- crucial en turbo
```

### Turbo alto (~1+ bar) — Golf Mk6 GTI 2.0T
```lua
TORQUE_METHOD    = 3       -- tabla 3D recomendado
MAP_MIN          = 28.8    -- (original)
MAP_MAX          = 174.5   -- (original, ajustable)
RPM_PEAK_TQ      = 3000    -- pico típico turbo
IAT_CORRECTION   = true    -- siempre
```

---

## Notas de desarrollo

### Extrapolación de `interpolate()`
La función `interpolate()` de rusEFI **NO clampa**. Extrapola fuera del rango x1-x2.  
Solución: `clampedInterp()` wrapper clampa automáticamente.

### Unidades MAF
`getSensor("Maf")` devuelve **g/s** (confirmado de USERCAL.h).

### Índices tabla3d
`table3d(idx, x, y)` — idx es 1-based (Lua), internamente convierte a 0-based.  
Válidos: 1, 2, 3, 4 (hasta 4 tablas simultaneas).

### Densidad aire IAT
Fórmula simplificada (termodinámica):
```
ρ(T) / ρ_ref = T_ref / T
```
Con escala absoluta (Kelvin): `(273.15 + T_ref) / (273.15 + T_celsius)`

---

## FAQ

**P: ¿Necesito sensor MAF?**  
R: No. Auto-modo intenta MAF pero fallback a MAP×RPM→lineal. MAF es opcional para máxima precisión.

**P: ¿Cómo calibro la tabla3d?**  
R: Con dynamómetro (chasis o motor) mide torque real vs. RPM/carga, completaría matriz en TunerStudio.

**P: ¿Afecta la v2 a cambios DSG?**  
R: No. Solo mejora estimación torque. Lógica cambios (`onCanDSG`, rev-match, etc.) sin cambios.

**P: ¿Rollback a v1?**  
R: v1 guardado como `dsg_dq250_standalone.lua`. Solo reemplazar `Lua Script` en TS.

**P: ¿Gauge 8 siempre muestra método correcto?**  
R: Sí. Se actualiza en cada tick según TORQUE_METHOD y sensores disponibles.

---

## Versión

- **v2.0** — 2026-03-31 inicial
- **Autor:** Claude, basado en RabbitECURusty GPL v2+ (DIAG.c, TORQUE.c)
- **Licencia:** GPL v2+
