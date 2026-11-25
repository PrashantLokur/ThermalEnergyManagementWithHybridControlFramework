function [controlOut, estimatedParams] = thermalLogicAndEstimation(state, Input, heatMeasured)
% thermalLogicAndEstimation - Determines control commands and estimates system parameters.
%
% All temperatures are handled in KELVIN.
%
% Inputs:
%   state:        State vector with standardized variables.
%   Input:        Struct with sensor inputs (standardized names, in K where applicable).
%                 Expected temperature fields in Kelvin: envT, clntInvTout, clntBatTin
%   heatMeasured: Struct with measured heat loads (standardized names).
%
% Outputs:
%   controlOut:      Struct with actuator commands and mode info.
%   estimatedParams: Struct with physical properties for the model.

    % === Input Extraction (Kelvin) ===
    envT_K        = Input.envT;         % K
    clntInvTout_K = Input.clntInvTout_K;  % K
    clntBatTin_K  = Input.clntBatTin_K;   % K
    compN         = Input.compN;
    pumpN         = Input.pumpN;
    prevMode      = Input.prevMode;    % Received from the main loop

    % === Persistent Variables for State-Holding Logic ===
    persistent clntBatTinFiltered cmdExvChillerSet initialized cmdHeater ...
               radRelayOn psRelayOn chillerRelayOn chillerTfState ...
               plantRefrigBypassHold plantRadBypassHold
    if isempty(initialized)
        clntBatTinFiltered = clntBatTin_K;  % K
        cmdExvChillerSet   = 0;
        cmdHeater          = 0;
        % Plant-side (rule-identical) logic states; Simulink Relay blocks start "off",
        % Memory blocks start at 0, Transfer Fcn starts at 0.
        radRelayOn            = false;
        psRelayOn             = false;
        chillerRelayOn        = false;
        chillerTfState        = 0;
        plantRefrigBypassHold = 0;
        plantRadBypassHold    = 0;
        initialized        = true;
    end

    % Optional: simple low-pass on clntBatTin (comment out if not desired)
    alphaFilt = 0.2; % 0..1
    clntBatTinFiltered = (1-alphaFilt)*clntBatTinFiltered + alphaFilt*clntBatTin_K;

    % === Determine Requested Mode (same as Controls/Thermal Mode/Mode Selection) ===
    % T_env >= 21 °C -> cooling (1);  19 < T_env < 21 °C -> off (0);  T_env <= 19 °C -> heat pump (2)
    envT_C = envT_K - 273.15;
    if envT_C >= 21
        setpoint = 1;  % Cooling Mode
    elseif envT_C > 19
        setpoint = 0;  % Off
    else
        setpoint = 2;  % Heat Pump Mode
    end

    % === Determine Actual Thermal Mode (same as Controls/Thermal Mode/Mode Transition) ===
    % Operating: any change of the requested mode goes through off first.
    % Off: the rule-based model enters the requested mode once the compressor is
    % stopped (<= 10 rpm). Under NMPC the compressor command never drops below
    % 10 rpm and the plant compressor is at rest when a mode starts, so the
    % requested mode is entered directly.
    if prevMode > 0
        if prevMode ~= setpoint
            thermalMode = 0;
        else
            thermalMode = setpoint;
        end
    else
        thermalMode = setpoint;
    end

    % === Generate Actuator Commands based on Thermal Mode ===
    % Refrigerant Bypass and Air Blend (for heat pump mode)
    cmdRefrigBypass = (thermalMode == 2);
    cmdBlendAir     = (thermalMode == 2);

    % Radiator Bypass in heat pump mode when coolant (inverter outlet) <= 25°C (298.15 K)
    if thermalMode == 2
        cmdRadBypass = (clntInvTout_K <= (273.15 + 25));
    else
        cmdRadBypass = 0;
    end

    % Chiller EXV hysteresis for battery cooling: 35°C / 25°C  -> 308.15 K / 298.15 K
    if clntBatTinFiltered >= (273.15 + 35)
        cmdExvChillerSet = 1;
    elseif clntBatTinFiltered <= (273.15 + 25)
        cmdExvChillerSet = 0;
    end

    if thermalMode == 1
        cmdExvChiller = cmdExvChillerSet * 0.024; % Proportional command
    else
        cmdExvChiller = 0;
    end



    % Evaporator EXV (cooling mode)
    cmdExvEvap = (thermalMode== 1);
    % Heat Pump EXV (heat pump mode)
    cmdExvHeatpump = (thermalMode == 2);

    % Parallel/Serial mixing (compare Kelvin temps)
    % Original compare used max(envT, clntInvTout-273.15) with °C numbers 35/30
    % Convert to K: 35°C=308.15 K, 30°C=303.15 K
    cmdParallelSerial = 1; % default serial
    if thermalMode >1

        cmdParallelSerial = 1;
    else
        T_compare_K = max(envT_K, clntInvTout_K);
        if T_compare_K >= (273.15 + 35)
            cmdParallelSerial = 0; % parallel if heat source is hot
        elseif T_compare_K <= (273.15 + 30)
            cmdParallelSerial = 1; % back to serial if cool
        end
    end

    % === Heater hysteresis (all Kelvin) ===
    % ON/OFF thresholds converted from °C:
    % env: 5/7°C -> 278.15/280.15 K
    % bat: 15/17°C -> 288.15/290.15 K
    env_on_K = (273.15 + 5);
    env_off_K = (273.15 + 5);
    bat_on_K = (273.15 + 15);
    bat_off_K = (273.15 + 5);
    pump_on  = 0; % rpm threshold (unchanged)

    if cmdHeater == 0
        % Turn ON only if all “on” conditions satisfied
        if (envT_K < env_on_K) && (clntBatTin_K < bat_on_K) && (pumpN > pump_on)
            cmdHeater = 1;
        end
    else
        % Stay ON unless any “off” condition is met
        if (envT_K > env_on_K) || (clntBatTin_K > bat_off_K) || (pumpN <= pump_on)
            cmdHeater = 0;
        end
    end
    if thermalMode ~= prevMode
        fprintf('[supervisor] Mode changed: %d → %d at envT=%.1fK compN=%.0f\n', ...
                prevMode, thermalMode, envT_K, compN);
    end

    % === Plant-side valve commands (sent to the Simscape plant) ===
    % These reproduce the rule-based Simulink blocks in Controls/Valves Control and
    % Controls/Coolant Loop Control/Parallel-Serial Mode one to one. The cmd* fields
    % above are left unchanged because the NMPC prediction model uses them as
    % 0/1 switches.
    clntInvTout_C = clntInvTout_K - 273.15;
    clntBatTin_C  = clntBatTin_K  - 273.15;

    % Refrigerant and Air 3-Way Valve: off -> hold, cooling -> 0, heat pump -> 1
    if thermalMode == 1
        plantRefrigBypass = 0;
    elseif thermalMode == 2
        plantRefrigBypass = 1;
    else
        plantRefrigBypass = plantRefrigBypassHold;
    end
    plantRefrigBypassHold = plantRefrigBypass;

    % Radiator 3-Way Valve: relay on T_coolant_inverter_out (on >= 50 °C -> 0,
    % off <= 25 °C -> 1, starts off = bypassed); off -> hold, cooling -> 0, heat pump -> relay
    if clntInvTout_C >= 50
        radRelayOn = true;
    elseif clntInvTout_C <= 25
        radRelayOn = false;
    end
    if thermalMode == 1
        plantRadBypass = 0;
    elseif thermalMode == 2
        plantRadBypass = double(~radRelayOn);
    else
        plantRadBypass = plantRadBypassHold;
    end
    plantRadBypassHold = plantRadBypass;

    % Chiller Expansion Valve: relay on T_coolant_battery_in (on >= 35 °C -> 0.024,
    % off <= 25 °C -> 0) through 1/(30s+1) (exact discretisation, 1 s step); cooling only
    if clntBatTin_C >= 35
        chillerRelayOn = true;
    elseif clntBatTin_C <= 25
        chillerRelayOn = false;
    end
    chillerTfState = chillerTfState + (1 - exp(-1/30)) * (0.024*chillerRelayOn - chillerTfState);
    plantExvChiller = chillerTfState * (thermalMode == 1);

    % Evaporator Expansion Valve: cooling -> lookup on T_env ([20 40] °C -> [0.006 0.009], clipped)
    if thermalMode == 1
        plantExvEvap = interp1([20 40], [0.006 0.009], min(max(envT_C, 20), 40));
    else
        plantExvEvap = 0;
    end

    % Heat Pump Expansion Valve: off -> 0, cooling -> 1,
    % heat pump -> lookup on T_env ([-10 0] °C -> [0.007 0.006], clipped)
    if thermalMode == 2
        plantExvHeatpump = interp1([-10 0], [0.007 0.006], min(max(envT_C, -10), 0));
    elseif thermalMode == 1
        plantExvHeatpump = 1;
    else
        plantExvHeatpump = 0;
    end

    % Parallel-Serial Mode: heat pump -> serial (1); otherwise relay on
    % max(T_env, T_coolant_inverter_out) (on >= 35 °C -> parallel 0, off <= 30 °C -> serial 1)
    if max(envT_C, clntInvTout_C) >= 35
        psRelayOn = true;
    elseif max(envT_C, clntInvTout_C) <= 30
        psRelayOn = false;
    end
    if thermalMode > 1
        plantParallelSerial = 1;
    else
        plantParallelSerial = double(~psRelayOn);
    end
    % === Populate Output Structure ===
    controlOut.setpoint            = setpoint;
    controlOut.thermalMode         = thermalMode;
    controlOut.cmdRefrigBypass     = cmdRefrigBypass;
    controlOut.cmdBlendAir         = cmdBlendAir;
    controlOut.cmdRadBypass        = cmdRadBypass;
    controlOut.cmdExvChiller       = cmdExvChiller;
    controlOut.cmdExvEvap          = cmdExvEvap;
    controlOut.cmdParallelSerial   = cmdParallelSerial;
    controlOut.clntBatTinFiltered  = clntBatTinFiltered; % K
    controlOut.cmdExvHeatpump      = cmdExvHeatpump;
    controlOut.cmdHeater           = cmdHeater;
    controlOut.prev_ctrlmode           = thermalMode; % Keep in sync with main loop
    controlOut.plantRefrigBypass   = plantRefrigBypass;
    controlOut.plantBlendAir       = plantRefrigBypass;  % same valve block drives both
    controlOut.plantRadBypass      = plantRadBypass;
    controlOut.plantExvChiller     = plantExvChiller;
    controlOut.plantExvEvap        = plantExvEvap;
    controlOut.plantExvHeatpump    = plantExvHeatpump;
    controlOut.plantParallelSerial = plantParallelSerial;

    % === Estimate Physical Parameters for the Current State ===
    estimatedParams = estimateParameters(state, Input, heatMeasured);
end
