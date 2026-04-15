function [controlOut, estimatedParams] = thermalLogicAndEstimation(state, Input, heatMeasured)

    % === Input Extraction (Kelvin) ===
    envT_K        = Input.envT;         % K
    clntInvTout_K = Input.clntInvTout_K;  % K
    clntBatTin_K  = Input.clntBatTin_K;   % K
    compN         = Input.compN;
    pumpN         = Input.pumpN;
    prevMode      = Input.prevMode;    % Received from the main loop

    % === Persistent Variables for State-Holding Logic ===
    persistent clntBatTinFiltered cmdExvChillerSet initialized cmdHeater
    if isempty(initialized)
        clntBatTinFiltered = clntBatTin_K;  % K
        cmdExvChillerSet   = 0;
        cmdHeater          = 0;
        initialized        = true;
    end

    % Optional: simple low-pass on clntBatTin (comment out if not desired)
    alphaFilt = 0.2; % 0..1
    clntBatTinFiltered = (1-alphaFilt)*clntBatTinFiltered + alphaFilt*clntBatTin_K;


    if envT_K >= (273.15 + 21)
        setpoint = 1;  % Cooling Mode
    elseif envT_K < (273.15 + 19)
        setpoint = 2;  % Heat Pump Mode
    else
        setpoint = 0;  % Off
    end

    % === Determine Actual Thermal Mode (with startup/shutdown logic) ===
    if prevMode ~= 0 && setpoint == 0
        thermalMode = 0;  % Turn off if running and setpoint is off
    elseif prevMode == 0 && setpoint ~= 0 && compN >= 10
        thermalMode = setpoint;  % Start new mode if off and compressor is running
    else
        thermalMode = prevMode;  % Hold current mode
    end

    % === Generate Actuator Commands based on Thermal Mode ===

    cmdRefrigBypass = (thermalMode == 2);
    cmdBlendAir     = (thermalMode == 2);

    if thermalMode == 2
        cmdRadBypass = (clntInvTout_K <= (273.15 + 50));
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

    estimatedParams = estimateParameters(state, Input, heatMeasured);
end
