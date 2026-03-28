-- ============================================================
-- DSG DQ250 Standalone — rusEFI Lua Script (Universal)
-- ============================================================
-- Instalación universal: rusEFI ES la ECU completa.
-- Del donante VW solo se conservan: DSG DQ250 + selector/palancas.
-- Este script SIMULA en CAN todos los módulos VW que la TCU DSG
-- espera (ECU principal, ABS, BCM/Gateway). Todos los datos del
-- motor provienen de los sensores físicos conectados a rusEFI.
--
-- Traducido/derivado de RabbitECURusty (DIAG.c, TORQUE.c,
-- SENSORS.c, EST.c, FUEL.c) — GPL v2+
--
-- CAN RX: 0x440 (DSG DQ250 TCU → rusEFI)
-- CAN TX: 0x280, 0x380, 0x488          @10ms
--         0x288, 0x480, 0x362, 0x284,
--         0x48A, 0x588                  @20ms
--         0x5A0                         @100ms
--         0x580, 0x050                  @1000ms
--
-- NOTA: No modifica dsg_dq250_control.lua
-- ============================================================


-- ============================================================
-- CALIBRACIÓN — ajustar según motor y vehículo, sin recompilar
-- ============================================================

-- VSS por 1000 RPM por marcha (km/h × 1000 rpm⁻¹)
-- Ejemplo: Golf Mk6 GTI 2.0T, neumáticos 225/40R18
-- Ajustar para tu relación de transmisión y tamaño de rueda
local VSSPerRPM = { 22, 43, 67, 93, 120, 150 }   -- marchas 1-6

-- Estimación de torque a partir de MAP (UNIVERSAL, sin hardcode)
-- Mapeo lineal MAP → 0-255 (0 = sin torque, 255 = máximo)
--   Motor NA (hasta ~100 kPa):         MAP_MIN=25, MAP_MAX=98
--   Turbo moderado (~0.6 bar):         MAP_MIN=25, MAP_MAX=160
--   Turbo alto boost (~1+ bar):        MAP_MIN=25, MAP_MAX=210
local MAP_MIN = 28.8    -- kPa: MAP de ralentí (vacío)
local MAP_MAX = 174.5   -- kPa: MAP a plena carga

-- Duración del torque reduction (ticks × 5 ms a 200 Hz)
local SHIFT_UP_COUNT   = 15   -- 75 ms
local SHIFT_DOWN_COUNT = 20   -- 100 ms
local SHIFT_DOWN_BLIP  =  8   -- 40 ms (blip rev-match)

-- Umbrales VSS para activar/desactivar control de torque
local ATX_ON_VSS  = 15   -- kph: activa vehicleMovingDS
local ATX_OFF_VSS =  5   -- kph: desactiva vehicleMovingDS

-- Retardo máximo de encendido durante cambio (grados, negativo)
local MAX_TIMING_RETARD = -25.0

-- Blip del acelerador para rev-match en bajada de marcha
local COLD_BLIP_PCT = 8.0   -- % ETB add en frío  (< 0 °C)
local HOT_BLIP_PCT  = 4.0   -- % ETB add en caliente (> 50 °C)

-- Bus CAN físico (1 = CAN1, 2 = CAN2 — ver hardware rusEFI)
local CAN_BUS = 1


-- ============================================================
-- TABLAS DE CÓDIGOS ROTATIVOS (de DIAG.c — no modificar)
-- ============================================================
local Codes648  = {54,54,78,78,78,78,139,139,139,139,232,232,232,232,54,54}
local Codes1152 = {28,28,81,81,81,81,132,132,132,132,193,193,193,193,28,28}


-- ============================================================
-- CONTADORES CAN TX
-- ============================================================
local cnt10   = 0    -- grupo 10 ms  (0x280, 0x380, 0x488)  0-15
local cnt20   = 0    -- grupo 20 ms  (0x288, 0x480, etc.)    0-15
local cnt1000 = 0    -- grupo 1000ms (0x580)                  0-15

-- Contador de distancia 11 bits para 0x5A0 (simulación ABS)
local distCnt    = 0
local distToggle = 0   -- alterna bit7 del byte0 de 0x5A0


-- ============================================================
-- ESTADO DSG — actualizado por CAN RX 0x440 y tick()
-- ============================================================
local atxGear         = 0
local manualMode      = false
local downShift       = false
local postShift       = false
local estTorqueModify = false
local atxTorqueLimit  = 255
local oldATXStat      = 0xFF

-- Modificadores de torque (escala 0-256; 256 = sin reducción)
local estTorqueModifier  = 256
local fuelTorqueModifier = 256

local gearShiftCount    = 0
local pressureCtrlCount = 0

local vehicleMovingUS = false
local vehicleMovingDS = false

local dsgRPMSlip     = 0
local dsgRPMSlipNext = 0

local revMatchPos    = 0.0
local quickCutActive = false


-- ============================================================
-- HELPERS
-- ============================================================

-- Limitar valor a rango 0-255 y convertir a entero
local function clampByte(v)
    return math.max(0, math.min(255, math.floor(v or 0)))
end

-- XOR de bytes 0..N-2, resultado escrito en posición N (último)
-- Equivalente a USER_DIAG_APPEND_XOR() en DIAG.h
local function xorFill(t)
    local x = 0
    for i = 1, #t - 1 do
        x = x ~ t[i]
    end
    t[#t] = x & 0xFF
    return t
end

-- Torque estimado (0-255) desde MAP en kPa
-- Mapeo lineal configurable: MAP_MIN→0, MAP_MAX→255
local function calcTorqueEst(map)
    if map == nil or map <= MAP_MIN then return 0 end
    if map >= MAP_MAX then return 255 end
    return math.floor((map - MAP_MIN) / (MAP_MAX - MAP_MIN) * 255)
end


-- ============================================================
-- CALLBACK CAN RX 0x440 — DSG DQ250 TCU
-- Equivalente a SENSORS_vGetCANSensorData() en SENSORS.c:686-763
--
-- Estructura 0x440 (8 bytes):
--   data[1] = byte0: ATX Torque Limit (0-255)
--   data[3] = byte2: bit7=ManualMode, bits3:0=GearEngaged
--   data[4] = byte3: 0x04=upshift, 0x01=downshift,
--                    0x80=post-shift, 0x10=normal
-- ============================================================
local function onCanDSG(bus, id, dlc, data)
    atxTorqueLimit = data[1]

    local byte2 = data[3]
    local byte3 = data[4]

    manualMode = (byte2 & 0x80) ~= 0
    atxGear    =  byte2 & 0x0F

    if byte3 == nil or byte3 == 0xFF then return end

    if byte3 ~= oldATXStat then
        oldATXStat = byte3

        if (byte3 & 0x04) ~= 0 then
            -- ---- SUBIDA DE MARCHA (SENSORS.c:700-727) ----
            downShift = false
            if (byte3 & 0x01) ~= 0 and vehicleMovingUS then
                if not postShift then
                    gearShiftCount    = SHIFT_UP_COUNT
                    pressureCtrlCount = SHIFT_UP_COUNT
                    postShift         = true
                    if manualMode then
                        setFuelMult(0.0)
                        quickCutActive = true
                    end
                end
            end
            if postShift and not estTorqueModify and (byte3 & 0x80) ~= 0 then
                estTorqueModify   = true
                estTorqueModifier = manualMode and 10 or 110
                if quickCutActive then
                    setFuelMult(1.0)
                    quickCutActive = false
                end
            end

        elseif (byte3 & 0x05) == 0x01 then
            -- ---- BAJADA DE MARCHA (SENSORS.c:729-737) ----
            estTorqueModify   = true
            downShift         = true
            postShift         = true
            gearShiftCount    = SHIFT_DOWN_COUNT
            pressureCtrlCount = SHIFT_DOWN_COUNT
            estTorqueModifier = manualMode and 10 or 110

        elseif (byte3 & 0x10) ~= 0 then
            -- ---- DSG NORMAL — cambio completado (SENSORS.c:738-748) ----
            gearShiftCount    = 0
            pressureCtrlCount = 0
            downShift         = false
            postShift         = false
            estTorqueModify   = false
            quickCutActive    = false
            setTimingAdd(0)
            setFuelMult(1.0)
            setEtbAdd(0)
        end
    end
end

canRxAdd(CAN_BUS, 0x440, onCanDSG)
enableCanTx(true)


-- ============================================================
-- CÁLCULO RPM SLIP
-- Equivalente a SENSORS.c:856-883
-- ============================================================
local function calcRPMSlip(rpm, vss)
    if rpm == nil or rpm < 100 or vss == nil or vss < 1 then
        dsgRPMSlip     = 9999
        dsgRPMSlipNext = 9999
        return
    end
    local gear = atxGear
    if gear < 1 then gear = 1 end
    if gear > 6 then gear = 6 end
    dsgRPMSlip = math.abs((vss * 1000) / VSSPerRPM[gear] - rpm)
    if gear < 6 then
        dsgRPMSlipNext = math.abs((vss * 1000) / VSSPerRPM[gear + 1] - rpm)
    else
        dsgRPMSlipNext = dsgRPMSlip
    end
end


-- ============================================================
-- REV-MATCH (blip del acelerador en bajadas)
-- Equivalente a TORQUE_vRun() sección rev-match (TORQUE.c:255-350)
-- ============================================================
local function calcRevMatch(clt, pps)
    if gearShiftCount == 0 or not downShift or not vehicleMovingDS then
        revMatchPos = 0.0
        setEtbAdd(0)
        return
    end
    if atxGear < 1 or atxGear >= 6 then
        revMatchPos = 0.0
        setEtbAdd(0)
        return
    end
    -- Posición de blip según temperatura (TORQUE_u16GetAutoRevMatch)
    local autoBlip
    if clt == nil or clt < 0 then
        autoBlip = COLD_BLIP_PCT
    elseif clt > 50 then
        autoBlip = HOT_BLIP_PCT
    else
        autoBlip = HOT_BLIP_PCT + (COLD_BLIP_PCT - HOT_BLIP_PCT) * (50 - clt) / 50
    end

    local elapsed = SHIFT_DOWN_COUNT - gearShiftCount

    if elapsed <= SHIFT_DOWN_BLIP then
        revMatchPos = autoBlip                           -- fase blip
    else
        local pe = elapsed - SHIFT_DOWN_BLIP
        local pd = math.max(60, SHIFT_DOWN_COUNT - SHIFT_DOWN_BLIP)
        if pe < 30 then
            revMatchPos = autoBlip * (30 - pe) / 30     -- ramp down
        elseif pe < 50 then
            revMatchPos = 0.0                            -- zero
        else
            local add = math.max(autoBlip * 0.5, (pps or 0) * 0.08)
            revMatchPos = add * (pe - 50) / math.max(1, pd - 50)  -- ramp up
        end
    end
    setEtbAdd(revMatchPos)
end


-- ============================================================
-- CAN TX — GRUPO 10 ms
-- Equivalente al bloque "2 == (tick % 10)" en DIAG.c:383-475
-- ============================================================
local function sendGroup10ms(rpm, map, tqEst, tqMod, pedalRep, tpsClosed)
    local rpm4 = math.floor((rpm or 0) * 4)

    -- 0x280: ECU principal → DSG (RPM + torque)
    -- Byte0: 0x01 si TPS cerrado (o RPM < 400), 0x00 si TPS abierto
    local b0
    if rpm < 400 then
        b0 = 0x01
        txCan(CAN_BUS, 0x280, 0, {0x01, 0, 0, 0, 0, 0, 0, 0})
    else
        b0 = tpsClosed and 0x01 or 0x00
        txCan(CAN_BUS, 0x280, 0, {
            b0,
            clampByte(tqMod),
            rpm4 & 0xFF,
            (rpm4 >> 8) & 0xFF,
            clampByte(tqEst),
            clampByte(pedalRep),
            20,                       -- IdleStabilisationTorque (fijo)
            clampByte(tqEst)
        })
    end

    -- 0x380: ECU → DSG (espejo de pedal)
    -- DWH = 0x2083{PPS}00 / DWL = 0x0000FE00 (DIAG.c:441-445)
    txCan(CAN_BUS, 0x380, 0, {
        0x20, 0x83, clampByte(pedalRep), 0x00,
        0x00, 0x00, 0xFE, 0x00
    })

    -- 0x488: ECU → DSG (presión MAP/boost + contador + XOR)
    -- val = floor((MAP_kPa × 1000 − 28800) / 573) clamped 0-254
    -- (DIAG.c:451-473; 28800 = 28.8 kPa en unidades internas)
    local mapVal = 0
    if map ~= nil and map > 28.8 then
        mapVal = math.min(254, math.floor((map * 1000 - 28800) / 573))
    end
    txCan(CAN_BUS, 0x488, 0, xorFill({
        cnt10 * 16,              -- nibble contador en bits 7:4
        mapVal, mapVal,          -- MAP codificado, repetido
        0x7E,                    -- 126 fijo
        0xFE, 0xFF, 0xFF, 0      -- byte7 ← rellena xorFill
    }))

    cnt10 = (cnt10 + 1) % 16
end


-- ============================================================
-- CAN TX — GRUPO 20 ms
-- Equivalente al bloque "6 == (tick % 20)" en DIAG.c:478-549
-- ============================================================
local function sendGroup20ms(clt, vss, brake, isc, tqEst, fuel16)
    local ci = cnt20 + 1   -- índice Lua (1-based)

    -- Codificación CLT: CTS_tTempCFiltered / 500 en DIAG.c
    -- CTS en milli-°C → /500 = celsius × 2
    local cltByte = 0
    if clt ~= nil and clt >= 0 then
        cltByte = clampByte(clt * 2)
    end

    -- 0x288: ECU → DSG (CLT / VSS / freno / ISC / escala de torque)
    txCan(CAN_BUS, 0x288, 0, {
        Codes648[ci],
        cltByte,
        brake and 0x03 or 0x00,        -- bit0-1: freno
        math.min(255, math.floor((vss or 0) / 8)) | 0x06,  -- VSS/8 | 0x06
        0x00,
        clampByte(isc),
        (tqEst >> 8) & 0xFF,           -- TorqueScale hi
        tqEst & 0xFF                   -- TorqueScale lo
    })

    -- 0x480: ECU → DSG (combustible consumido + XOR)
    -- DIAG.c:529-537: byte2 = fuelLo, byte3 = fuelHi (posición invertida)
    txCan(CAN_BUS, 0x480, 0, xorFill({
        Codes1152[ci],
        0x00,
        fuel16 & 0xFF,                 -- fuel lo en byte2
        (fuel16 >> 8) & 0xFF,          -- fuel hi en byte3
        24, 0x00, 4, 0                 -- byte7 ← rellena xorFill
    }))

    -- 0x362: ECU → DSG (contador + temperatura auxiliar)
    -- DIAG.c:481-496
    txCan(CAN_BUS, 0x362, 0, {
        cnt20, 0x00, 0x00, 0xA6,
        0xFE, 0x00, 0x00,
        cnt20 >= 8 and (cnt20 + 72) or (cnt20 + 88)
    })

    -- 0x284: ECU → DSG (contador doble, DLC=6)
    -- DIAG.c:498-502
    txCan(CAN_BUS, 0x284, 0, {cnt20, cnt20, 0, 0, 0, 0})

    -- 0x48A: ECU → DSG (nibble contador)
    -- DIAG.c:539-541
    txCan(CAN_BUS, 0x48A, 0, {
        cnt20 * 16 + 2, 0x00, 0x00, 0x02,
        0x00, 0x00, 0x00, cnt20 * 16
    })

    -- 0x588: Handshake fijo (DIAG.c:544-546)
    txCan(CAN_BUS, 0x588, 0, {232, 60, 127, 135, 52, 0, 0, 153})

    cnt20 = (cnt20 + 1) % 16
end


-- ============================================================
-- CAN TX — GRUPO 100 ms: 0x5A0 (simula módulo ABS para la DSG)
-- La DSG usa este mensaje para confirmar velocidad del vehículo
-- y calcular el slip del embrague
-- ============================================================
local function sendGroup100ms(vss)
    -- Acumular contador de distancia 11 bits (rollover en 2048)
    -- Incremento ≈ VSS_kph / 3.6 cada 100 ms (≈ metros × 0.1)
    distCnt    = (distCnt + math.floor((vss or 0) / 3.6)) % 2048
    distToggle = distToggle ~ 0x80   -- alterna bit7

    -- Nibble de marcha actual (del feedback 0x440)
    local gNib = (atxGear >= 1 and atxGear <= 6) and atxGear or 0

    txCan(CAN_BUS, 0x5A0, 0, {
        distToggle,              -- byte0: toggle bit7
        0x00, 0x00,
        gNib * 16,               -- byte3: nibble de marcha en bits 7:4
        0x00,
        (distCnt >> 8) & 0x07,  -- byte5: bits 10:8 del contador
        distCnt & 0xFF,          -- byte6: bits 7:0 del contador
        0x00
    })
end


-- ============================================================
-- CAN TX — GRUPO 1000 ms
-- ============================================================
local function sendGroup1000ms()
    -- 0x580: contador de arranque ECU (DIAG.c:555-560)
    txCan(CAN_BUS, 0x580, 0, {cnt1000, 0, 0, 0, 0, 0, 0, 0})
    -- 0x050: Gateway/BCM — modo de encendido activo (fijo)
    txCan(CAN_BUS, 0x050, 0, {0x45, 0, 0, 0, 0, 0, 0, 0})
    cnt1000 = (cnt1000 + 1) % 16
end


-- ============================================================
-- TIMERS para cadencia de transmisión (patrón de nissan-tcu.lua)
-- ============================================================
local t10ms   = Timer.new(); t10ms:reset()
local t20ms   = Timer.new(); t20ms:reset()
local t100ms  = Timer.new(); t100ms:reset()
local t1000ms = Timer.new(); t1000ms:reset()


-- ============================================================
-- TICK PRINCIPAL @200 Hz (5 ms)
-- Equivalente a TORQUE_vRun() en TORQUE.c (tarea de 5 ms)
-- ============================================================
setTickRate(200)

function onTick()

    -- ---- Leer sensores desde rusEFI ----
    local rpm   = getSensor("Rpm")              or 0   -- RPM
    local map   = getSensor("Map")              or 0   -- kPa
    local clt   = getSensor("Clt")                     -- °C (puede ser nil)
    local pps   = getSensor("AcceleratorPedal") or 0   -- 0-100 %
    local tps   = getSensor("Tps1")             or 0   -- 0-100 %
    local vss   = getSensor("VehicleSpeed")     or 0   -- km/h
    local brake = getDigital(2)                        -- pedalState (bool)

    -- ---- Flags de velocidad (TORQUE.c:71-74) ----
    if vss > ATX_ON_VSS  then vehicleMovingDS = true  end
    if vss < ATX_OFF_VSS then vehicleMovingDS = false end
    if vss > 15          then vehicleMovingUS = true  end
    if vss <  5          then vehicleMovingUS = false end

    -- ---- RPM slip del embrague DSG (SENSORS.c:856-883) ----
    calcRPMSlip(rpm, vss)

    -- ---- Decrementar contador de cambio (SENSORS.c:754-758) ----
    if postShift and estTorqueModify and gearShiftCount > 0 then
        gearShiftCount = gearShiftCount - 1
        if pressureCtrlCount > 0 then pressureCtrlCount = pressureCtrlCount - 1 end
        if gearShiftCount == 0 then
            estTorqueModify = false
            postShift       = false
            downShift       = false
        end
    end

    -- ---- Lógica principal de modificación de torque (TORQUE.c:155-236) ----
    if estTorqueModify and vehicleMovingUS then
        local threshold = downShift
            and math.floor(SHIFT_DOWN_COUNT / 2)
            or  (SHIFT_UP_COUNT - 5)

        if gearShiftCount > threshold then
            -- Fase temprana: reducción máxima
            estTorqueModifier = manualMode and 10 or 110
        else
            -- Fase tardía: ramp guiado por slip del embrague
            if dsgRPMSlip < 150 then
                -- Embrague engranando → subir modifier (volver a normal)
                estTorqueModifier = math.min(estTorqueModifier + (manualMode and 5 or 2), 256)
            elseif dsgRPMSlip > 250 then
                -- Embrague deslizando → mantener reducción
                estTorqueModifier = math.max(estTorqueModifier - 5, manualMode and 20 or 110)
            end
        end

        fuelTorqueModifier = manualMode and 220 or 240
        setTimingAdd(MAX_TIMING_RETARD * (1.0 - estTorqueModifier / 256.0))
        setFuelMult(fuelTorqueModifier / 256.0)

    elseif not estTorqueModify then
        -- Sin reducción activa: restaurar a 100%
        if estTorqueModifier ~= 256 then
            estTorqueModifier  = 256
            fuelTorqueModifier = 256
            setTimingAdd(0)
            setFuelMult(1.0)
        end
    end

    -- ---- Rev-match ETB (TORQUE.c:255-350) ----
    calcRevMatch(clt, pps)

    -- ---- Preparar valores para TX ----
    local tqEst    = calcTorqueEst(map)
    local tqMod    = math.floor(tqEst * estTorqueModifier / 256)
    local pedalRep = math.floor(pps / 100 * 255)
    local tpsClosed = (tps < 1.0)
    -- ISC target: aproximación basada en RPM (IAC_u16ISCTargetRamp / 10)
    local isc      = clampByte(rpm < 1200 and rpm / 10 or 90)
    -- Combustible consumido: acumulador de rusEFI, 16 bits
    local fuel16   = math.floor(getConsumedGrams()) % 65536

    -- ---- Transmisión periódica ----
    if t10ms:getElapsedSeconds() >= 0.010 then
        t10ms:reset()
        sendGroup10ms(rpm, map, tqEst, tqMod, pedalRep, tpsClosed)
    end

    if t20ms:getElapsedSeconds() >= 0.020 then
        t20ms:reset()
        sendGroup20ms(clt, vss, brake, isc, tqEst, fuel16)
    end

    if t100ms:getElapsedSeconds() >= 0.100 then
        t100ms:reset()
        sendGroup100ms(vss)
    end

    if t1000ms:getElapsedSeconds() >= 1.000 then
        t1000ms:reset()
        sendGroup1000ms()
    end

    -- ---- LuaGauges: monitoreo en TunerStudio (Gauges → Lua) ----
    setLuaGauge(1, atxGear)           -- Marcha engranada (DSG)
    setLuaGauge(2, estTorqueModifier) -- Modificador EST (0-256)
    setLuaGauge(3, dsgRPMSlip)        -- RPM slip embrague
    setLuaGauge(4, gearShiftCount)    -- Contador cambio (ticks)
    setLuaGauge(5, vss)               -- VSS rusEFI (km/h)
    setLuaGauge(6, revMatchPos)       -- Posición rev-match ETB (%)
    setLuaGauge(7, brake and 1 or 0)  -- Freno pisado (0/1)
    setLuaGauge(8, manualMode and 1 or 0)  -- Modo manual paletas (0/1)
end
