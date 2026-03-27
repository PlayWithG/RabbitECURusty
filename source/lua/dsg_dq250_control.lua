-- ============================================================
-- DSG DQ250 Torque Control Script for rusEFI
-- Traducido de RabbitECURusty (NXP MK64F12, GPL v2+)
-- Archivos origen: source/Client/TORQUE.c, SENSORS.c, EST.c,
--                  FUEL.c, DIAG.c, USERCAL.h
--
-- Comunicacion CAN:
--   RX  0x440 (DSG DQ250 TCU → ECU): torque limit, marcha, shift flags
--   RX  0x5A0 (ABS/ESP → ECU):       VSS distancia, selector marchas
--   RX  0x1AC (Frenos → ECU):         pedal de freno
--   TX  0x280 (ECU → DSG DQ250):      RPM×4, OutputTorqueModified
-- ============================================================

-- ============================================================
-- CALIBRACION  (ajustar segun vehiculo, sin recompilar)
-- ============================================================

-- VSS por 1000 RPM para marchas 1-6 del Golf Mk6 GTI con DQ250
-- Equivalente a USERCAL_stRAMCAL.u16VSSPerRPM[]
-- Unidades: km/h por cada 1000 rpm
local VSSPerRPM = { 22, 43, 67, 93, 120, 150 }

-- Duracion del torque reduction en ticks (1 tick = 5ms @ 200Hz)
-- Equivalente a u16ShiftUpCountLimit / u16ShiftDownCountLimit
local SHIFT_UP_COUNT   = 15   -- 75ms
local SHIFT_DOWN_COUNT = 20   -- 100ms
local SHIFT_DOWN_BLIP  = 8    -- 40ms  (duracion del blip)

-- Velocidades para activar/desactivar control de torque
-- Equivalente a u16ATXTorqueOnVSS / u16ATXTorqueOffVSS
local ATX_ON_VSS  = 15  -- kph: activa vehicleMovingDS
local ATX_OFF_VSS = 5   -- kph: desactiva vehicleMovingDS

-- Retardo maximo de encendido durante cambio (grados, negativo = retraso)
-- Equivalente a la diferencia entre MapaNormal y aUserTimingMapStage1
local MAX_TIMING_RETARD = -25.0

-- Posicion de blip del acelerador para rev-match segun temperatura
-- Equivalente a u16ColdOffThrottleBlip / u16HotOffThrottleBlip
local COLD_BLIP_PCT = 8.0   -- % ETB add en frio  (<0 degC)
local HOT_BLIP_PCT  = 4.0   -- % ETB add en caliente (>50 degC)

-- Bus CAN fisico (1 o 2 segun hardware rusEFI)
local CAN_BUS = 1

-- Factor de calibracion VSS CAN (ajustar si la velocidad no coincide)
-- Equivalente a USERCAL_stRAMCAL.u16VSSCANCal
local VSS_CAN_CAL = 100

-- ============================================================
-- ESTADO DSG  (variables globales del script)
-- ============================================================

-- Desde CAN 0x440 (actualizado en callback onCanDSG):
local atxGear         = 0      -- Marcha engranada reportada por DSG (1-6)
local manualMode      = false  -- true = paletas manuales activas
local downShift       = false  -- Bajada de marcha en progreso
local postShift       = false  -- Post-cambio activo
local estTorqueModify = false  -- Modificacion de torque habilitada
local atxTorqueLimit  = 255    -- Limite de torque de la DSG (raw 0-255)
local oldATXStat      = 0xFF   -- Ultimo byte de estado 0x440[4]

-- Calculado en tick():
-- 256 = sin reduccion (100%), 10 = reduccion maxima (~4%)
-- Equivalente a TORQUE_u32ESTTorqueModifier en escala 0-256
local estTorqueModifier  = 256
-- 220=manual (85.9%), 240=auto (93.8%), 256=sin cambio
-- Equivalente a TORQUE_u32FuelTorqueModifier
local fuelTorqueModifier = 256
-- Contador regresivo del cambio (ticks a 5ms)
-- Equivalente a TORQUE_u16GearShiftCount
local gearShiftCount     = 0
local pressureCtrlCount  = 0

-- Desde CAN 0x5A0 (actualizado en callback onCanABS):
-- Equivalente a SENSORS_u16CANVSS
local canVSS        = 0
local oldDistCount  = 0
local oldDistance   = 0
local vssTimeout    = 0

-- Flags de velocidad (actualizados en tick()):
-- Equivalentes a TORQUE_boVehicleMovingUS / TORQUE_boVehicleMovingDS
local vehicleMovingUS = false
local vehicleMovingDS = false

-- RPM slip para realimentacion del embrague DSG
-- Equivalentes a SENSORS_u16VSSDSGGearRPMSlip / SlipNext
local dsgRPMSlip     = 0
local dsgRPMSlipNext = 0

-- Rev-match:
-- Equivalente a TORQUE_u16RevMatchPosition (en % para setEtbAdd)
local revMatchPos = 0.0

-- Misc:
local brakePedalPressed = false

-- Contador para CAN TX cada 10ms (cada 2 ticks de 5ms)
-- Equivalente a la cadencia del grupo 10ms en DIAG.c
local txCounter = 0

-- Quick cut activo (subida manual)
local quickCutActive = false

-- ============================================================
-- CALLBACK CAN RX: CAN 0x440 — DSG DQ250 TCU
-- Equivalente a la seccion "Check CAN torque reduction requests"
-- en SENSORS_vGetCANSensorData() (SENSORS.c:686-763)
--
-- Estructura del mensaje 0x440 (8 bytes):
--   data[1] = byte 0 del mensaje = ATX Torque Limit
--   data[2] = byte 1 (reservado)
--   data[3] = byte 2 = [bit7]=ManualMode, [bits3:0]=SelectedGear
--   data[4] = byte 3 = shift status:
--               0x04 = upshift en progreso
--               0x01 = downshift en progreso
--               0x80 = post-shift (embrague engranando)
--               0x10 = DSG normal (sin cambio)
-- ============================================================
local function onCanDSG(bus, id, dlc, data)
    -- data[1..8] son los bytes del frame CAN (1-indexed en rusEFI Lua)

    -- buf[16] → Torque Limit de la DSG
    atxTorqueLimit = data[1]

    -- buf[18] → marcha y modo
    local byte2 = data[3]
    -- buf[19] → flags de cambio
    local byte3 = data[4]

    -- Decodificar marcha y modo manual
    -- SENSORS.c:691-692
    manualMode = (byte2 & 0x80) ~= 0
    atxGear    = byte2 & 0x0F

    if byte3 == nil or byte3 == 0xFF then
        return
    end

    -- Detectar CAMBIO DE ESTADO en buf[19]
    -- SENSORS.c:696-758
    if byte3 ~= oldATXStat then
        oldATXStat = byte3

        if (byte3 & 0x04) ~= 0 then
            -- ------------------------------------------------
            -- SUBIDA DE MARCHA (upshift)
            -- SENSORS.c:700-727
            -- ------------------------------------------------
            downShift = false

            -- bit 0x01 junto con 0x04 = trigger real del cambio
            if (byte3 & 0x01) ~= 0 and vehicleMovingUS then
                if not postShift then
                    gearShiftCount     = SHIFT_UP_COUNT
                    pressureCtrlCount  = SHIFT_UP_COUNT
                    postShift          = true

                    -- Modo manual: quick fuel cut inmediato en subida
                    -- Equivalente a FUEL_vQuickCut(percent, duration)
                    if manualMode then
                        setFuelMult(0.0)
                        quickCutActive = true
                    end
                end
            end

            -- bit 0x80 = embrague engranando → pedir reduccion de torque
            -- SENSORS.c:719-727
            if postShift and not estTorqueModify and (byte3 & 0x80) ~= 0 then
                estTorqueModify  = true
                estTorqueModifier = manualMode and 10 or 110
                -- Terminar quick cut si habia uno activo
                if quickCutActive then
                    setFuelMult(1.0)
                    quickCutActive = false
                end
            end

        elseif (byte3 & 0x05) == 0x01 then
            -- ------------------------------------------------
            -- BAJADA DE MARCHA (downshift)
            -- SENSORS.c:729-737
            -- ------------------------------------------------
            estTorqueModify   = true
            downShift         = true
            postShift         = true
            gearShiftCount    = SHIFT_DOWN_COUNT
            pressureCtrlCount = SHIFT_DOWN_COUNT
            estTorqueModifier = manualMode and 10 or 110

        elseif (byte3 & 0x10) ~= 0 then
            -- ------------------------------------------------
            -- DSG NORMAL — cambio completado
            -- SENSORS.c:738-748
            -- ------------------------------------------------
            gearShiftCount    = 0
            pressureCtrlCount = 0
            downShift         = false
            postShift         = false
            estTorqueModify   = false
            quickCutActive    = false
            -- Restaurar timing y combustible
            setTimingAdd(0)
            setFuelMult(1.0)
            setEtbAdd(0)
        end

    else
        -- Sin cambio de estado: decrementar contadores
        -- (en el original esto ocurre en SENSORS_vGetCANSensorData,
        --  pero en Lua lo manejamos en tick() para ritmo fijo de 5ms)
        -- No hacemos nada aqui; tick() se encarga del decremento
    end
end

-- ============================================================
-- CALLBACK CAN RX: CAN 0x5A0 — ABS/ESP (VSS + Selector)
-- Equivalente al calculo de SENSORS_u16CANVSS en SENSORS.c:785-818
--
-- Estructura del mensaje 0x5A0 (8 bytes):
--   data[1] = byte 0 = bit7: toggle pulso distancia
--   data[4] = byte 3 = bits7:4: posicion selector DSG
--   data[6] = byte 5 = distancia hi byte
--   data[7] = byte 6 = distancia lo byte
-- ============================================================
local function onCanABS(bus, id, dlc, data)
    local byte0  = data[1]   -- toggle distancia
    local distHi = data[6]   -- distancia hi
    local distLo = data[7]   -- distancia lo

    local newDistCount = byte0 & 0x80

    if newDistCount ~= oldDistCount then
        oldDistCount = newDistCount

        -- Distancia acumulada de 11 bits con rollover en 2048
        -- SENSORS.c:790-808
        local dist  = distHi * 256 + distLo
        local delta

        if dist >= oldDistance then
            delta = dist - oldDistance
        else
            -- Rollover
            delta = dist - oldDistance + 2048
        end

        -- Aplicar factor de calibracion
        delta = delta * VSS_CAN_CAL / 100
        if delta > 0xFFFF then delta = 0xFFFF end

        -- Filtro paso bajo (50/50) igual que en el original
        canVSS      = math.floor(delta / 2) + math.floor(canVSS / 2)
        oldDistance = dist
        vssTimeout  = 0
    else
        -- Timeout: si no llegan pulsos → velocidad = 0
        if vssTimeout > 125 then
            canVSS = 0
        else
            vssTimeout = vssTimeout + 1
        end
    end
end

-- ============================================================
-- CALLBACK CAN RX: CAN 0x1AC — Sistema de Frenos
-- Equivalente a SENSORS.c:670-676
--
-- Estructura del mensaje 0x1AC (8 bytes):
--   data[7] = byte 6 = bit6: pedal de freno presionado
-- ============================================================
local function onCanBrake(bus, id, dlc, data)
    brakePedalPressed = (data[7] & 0x40) ~= 0
end

-- ============================================================
-- REGISTRO DE CALLBACKS CAN RX
-- Equivalente a la configuracion de mailboxes en CANHA.c
-- ============================================================
canRxAdd(CAN_BUS, 0x050, function(bus, id, dlc, data) end)  -- Power mode (monitoreo)
canRxAdd(CAN_BUS, 0x5A0, onCanABS)
canRxAdd(CAN_BUS, 0x440, onCanDSG)
canRxAdd(CAN_BUS, 0x1AC, onCanBrake)
enableCanTx(true)

-- ============================================================
-- HELPER: Calcular RPM slip de la DSG
-- Equivalente a SENSORS.c:856-883
-- Mide cuanto difiere el RPM actual del RPM esperado para la
-- marcha reportada por la DSG. Guia el ramp-up del modifier.
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

    -- Slip marcha actual: |RPM_esperado - RPM_actual|
    -- SENSORS.c:863-867
    local expectedRPM = (vss * 1000) / VSSPerRPM[gear]
    dsgRPMSlip = math.abs(expectedRPM - rpm)

    -- Slip marcha siguiente: indica si el embrague de destino esta cerca
    -- SENSORS.c:869-882
    if gear < 6 then
        local nextExpected = (vss * 1000) / VSSPerRPM[gear + 1]
        dsgRPMSlipNext = math.abs(nextExpected - rpm)
    else
        dsgRPMSlipNext = dsgRPMSlip
    end
end

-- ============================================================
-- HELPER: Calcular posicion ETB para rev-match en bajada
-- Equivalente a TORQUE_vRun() rev-match section (TORQUE.c:255-350)
-- y TORQUE_u16GetAutoRevMatch() (TORQUE.c:399-420)
--
-- Genera un blip del acelerador para igualar RPM antes del engrane
-- del embrague en una bajada de marcha.
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

    -- Posicion de blip basada en temperatura del refrigerante
    -- TORQUE_u16GetAutoRevMatch(): frio=COLD_BLIP, caliente=HOT_BLIP
    local autoBlip
    if clt == nil or clt < 0 then
        autoBlip = COLD_BLIP_PCT
    elseif clt > 50 then
        autoBlip = HOT_BLIP_PCT
    else
        -- Interpolacion lineal entre frio y caliente
        autoBlip = HOT_BLIP_PCT + (COLD_BLIP_PCT - HOT_BLIP_PCT) * (50 - clt) / 50
    end

    -- Tiempo transcurrido desde inicio del cambio
    local elapsed = SHIFT_DOWN_COUNT - gearShiftCount

    if elapsed <= SHIFT_DOWN_BLIP then
        -- ---- FASE BLIP: maximo add al acelerador ----
        -- TORQUE.c:280-282 (primeras iteraciones del shift)
        revMatchPos = autoBlip

    else
        local postElapsed  = elapsed - SHIFT_DOWN_BLIP
        local postDuration = SHIFT_DOWN_COUNT - SHIFT_DOWN_BLIP
        if postDuration < 60 then postDuration = 60 end

        if postElapsed < 30 then
            -- ---- POST-BLIP FASE 1: ramp down ----
            -- TORQUE.c:287-291
            revMatchPos = autoBlip * (30 - postElapsed) / 30

        elseif postElapsed < 50 then
            -- ---- POST-BLIP FASE 2: acelerador a cero ----
            -- TORQUE.c:292-295
            revMatchPos = 0.0

        else
            -- ---- POST-BLIP FASE 3: ramp up siguiendo pedal ----
            -- TORQUE.c:297-317
            local pedalPct = pps or 0
            -- Mapeo aproximado del pedal a posicion ETB add
            local pedalAdd = pedalPct * 0.08  -- escalar 0-100% pedal a 0-8% add
            if pedalAdd < autoBlip * 0.5 then pedalAdd = autoBlip * 0.5 end

            local t = postElapsed - 50
            local d = postDuration - 50
            if d < 1 then d = 1 end
            revMatchPos = pedalAdd * t / d
        end
    end

    setEtbAdd(revMatchPos)
end

-- ============================================================
-- TICK PRINCIPAL  @200Hz (5ms)
-- Equivalente a TORQUE_vRun() en TORQUE.c ejecutado cada 5ms
-- ============================================================
setTickRate(200)

function tick()
    -- ---- Leer sensores ----
    local rpm = getSensor("RPM")       or 0
    local map = getSensor("Map")       or 0   -- kPa
    local clt = getSensor("Clt")              -- degC (puede ser nil)
    local pps = getSensor("AcceleratorPedal") or 0  -- 0-100%

    -- Usar VSS nativo rusEFI; si no disponible, usar VSS calculado de CAN
    local vss = getSensor("VehicleSpeed")
    if vss == nil or vss == 0 then
        vss = canVSS
    end

    -- ---- Actualizar flags de velocidad ----
    -- Equivalente a TORQUE.c:71-74
    if vss > ATX_ON_VSS  then vehicleMovingDS = true  end
    if vss < ATX_OFF_VSS then vehicleMovingDS = false end
    if vss > 15          then vehicleMovingUS = true  end
    if vss < 5           then vehicleMovingUS = false end

    -- ---- Calcular RPM slip DSG ----
    calcRPMSlip(rpm, vss)

    -- ---- Decrementar contador de cambio ----
    -- Equivalente a SENSORS.c:754-758 (rama "sin cambio de estado")
    if postShift and estTorqueModify and gearShiftCount > 0 then
        gearShiftCount = gearShiftCount - 1
        if pressureCtrlCount > 0 then
            pressureCtrlCount = pressureCtrlCount - 1
        end
        -- Al llegar a cero: fin del torque reduction
        if gearShiftCount == 0 then
            estTorqueModify  = false
            postShift        = false
            downShift        = false
        end
    end

    -- ---- LOGICA PRINCIPAL DE MODIFICACION DE TORQUE ----
    -- Equivalente a TORQUE_vRun() TORQUE.c:155-236
    if estTorqueModify and vehicleMovingUS then

        -- Determinar umbral de fase temprana/tardia
        -- TORQUE.c:160-167
        local threshold
        if not downShift then
            threshold = SHIFT_UP_COUNT - 5
        else
            threshold = math.floor(SHIFT_DOWN_COUNT / 2)
        end

        if gearShiftCount > threshold then
            -- ---- FASE TEMPRANA: maxima reduccion de torque ----
            -- TORQUE.c:170-178
            if manualMode then
                estTorqueModifier = 10    -- ~4% torque
            else
                estTorqueModifier = 110   -- ~43% torque
            end

        else
            -- ---- FASE TARDIA: ramp guiado por slip RPM ----
            -- TORQUE.c:182-211

            if dsgRPMSlip < 150 then
                -- Embrague engranando: subir modifier gradualmente
                -- (torque vuelve a la normalidad)
                if manualMode then
                    estTorqueModifier = math.min(estTorqueModifier + 5, 256)
                else
                    estTorqueModifier = math.min(estTorqueModifier + 2, 256)
                end

            elseif dsgRPMSlip > 250 then
                -- Embrague aun deslizando: mantener reduccion
                if manualMode then
                    estTorqueModifier = math.max(estTorqueModifier - 5, 20)
                else
                    estTorqueModifier = math.max(estTorqueModifier - 5, 110)
                end
                -- else: slip entre 150-250, mantener modifier sin cambio
            end
        end

        -- ---- Modificador de combustible ----
        -- TORQUE.c:214-221 + FUEL.c:1051-1058
        if manualMode then
            fuelTorqueModifier = 220  -- 85.9%
        else
            fuelTorqueModifier = 240  -- 93.8%
        end

        -- ---- Aplicar retardo de encendido ----
        -- Equivalente a interpolacion MapaNormal <-> MapaStage1 en EST.c:572-606
        -- estTorqueModifier=10  → retard maximo (MAX_TIMING_RETARD)
        -- estTorqueModifier=256 → sin retardo (0 deg)
        local retardFraction = 1.0 - (estTorqueModifier / 256.0)
        setTimingAdd(MAX_TIMING_RETARD * retardFraction)

        -- ---- Aplicar reduccion de combustible ----
        -- Equivalente a FUEL.c:1051-1058
        if not quickCutActive then
            setFuelMult(fuelTorqueModifier / 256.0)
        end

        -- ---- Rev-match en bajada ----
        calcRevMatch(clt, pps)

    else
        -- ---- SIN CAMBIO ACTIVO: torque completo ----
        -- TORQUE.c:229-236
        estTorqueModifier  = 256
        fuelTorqueModifier = 256
        if not quickCutActive then
            setTimingAdd(0)
            setFuelMult(1.0)
        end
        revMatchPos = 0.0
        setEtbAdd(0)
    end

    -- ---- Calculo de torque estimado (para CAN TX) ----
    -- Equivalente a TORQUE.c:116-124 (estimacion via MAP)
    local mapTorque = 8
    if map > 30 then
        mapTorque = math.min(255, math.floor(8 + (map - 30) / 0.66))
    end

    -- OutputTorqueModified escalado a 0-255
    -- Equivalente a TORQUE_u32OutputTorqueModified enviado en CAN 0x280
    local torqueOut = math.floor(mapTorque * estTorqueModifier / 256)

    -- ---- CAN TX: 0x280 → DSG DQ250 (cada 10ms = cada 2 ticks) ----
    -- Equivalente a DIAG.c:387-438 (grupo 10ms, ID=640=0x280)
    txCounter = txCounter + 1
    if txCounter >= 2 then
        txCounter = 0

        local rpmRaw  = math.floor(rpm * 4)
        local rpmLo   = rpmRaw % 256
        local rpmHi   = math.floor(rpmRaw / 256)
        local pedalB  = math.floor(pps)

        -- Estructura segun DIAG.c:411-429:
        -- DWH[3]=0x01, DWH[2]=TorqueModified, DWH[1]=RPM_lo, DWH[0]=RPM_hi
        -- DWL[3]=TorqueEstimate, DWL[2]=PedalPos, DWL[1]=IdleTorque, DWL[0]=TorqueEstimate
        txCan(CAN_BUS, 0x280, 0, {
            0x01,         -- byte 0: flag motor en marcha
            torqueOut,    -- byte 1: OutputTorqueModified (0-255)
            rpmLo,        -- byte 2: RPM×4 byte bajo
            rpmHi,        -- byte 3: RPM×4 byte alto
            mapTorque,    -- byte 4: TorqueEstimate (de MAP)
            pedalB,       -- byte 5: PedalPositionReport
            20,           -- byte 6: IdleStabilisationTorque
            mapTorque     -- byte 7: TorqueEstimate (repetido)
        })
    end

    -- ---- Gauges de debug en TunerStudio ----
    -- Configurar como "LuaGauge1..8" en TunerStudio → Gauge
    setLuaGauge(1, atxGear)                            -- Marcha DSG (1-6)
    setLuaGauge(2, estTorqueModifier)                  -- Modifier 10-256
    setLuaGauge(3, dsgRPMSlip)                         -- Slip RPM embrague
    setLuaGauge(4, gearShiftCount)                     -- Ticks restantes
    setLuaGauge(5, vss)                                -- VSS kph
    setLuaGauge(6, revMatchPos)                        -- Rev-match ETB %
    setLuaGauge(7, brakePedalPressed and 1 or 0)       -- Pedal freno
    setLuaGauge(8, manualMode and 1 or 0)              -- Modo manual
end
