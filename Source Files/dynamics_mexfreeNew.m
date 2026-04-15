function [xdot,mdlOut_sym,J] = dynamics_mexfreeNew(x, u, p)

    [mdlInput, estimatedParams, controlOut, P] = build_runtime_context(x, u, p);

    % --- 2) Component map / accumulator ---
    cmp = struct();

    % ---- Flow: coolant/refrigerant path + auxiliaries ----
    cmp = pumpMdl(mdlInput, estimatedParams, controlOut, cmp);
    cmp = compressorMdl(mdlInput, estimatedParams, controlOut, cmp);
    cmp = innerCondenserMdl(mdlInput, estimatedParams, controlOut, cmp);
    cmp = hpExvMdl(mdlInput, estimatedParams, controlOut, cmp);
    cmp = condEvapMdl(mdlInput, estimatedParams, controlOut, cmp);

    % Loop solver (sets mdlInput.u.htxClntTinK)
    [T0, ~, ~] = solve_loop_htx_in_explicit(cmp, mdlInput, estimatedParams, controlOut);
    mdlInput.u.htxClntTinK = T0;

    cmp = htxMdl(mdlInput, estimatedParams, controlOut, cmp, P);
    cmp = evapExv(mdlInput, estimatedParams, controlOut, cmp);
    cmp = evaporatorMdl(mdlInput, estimatedParams, controlOut, cmp);
    cmp = chillerMdl(mdlInput, estimatedParams, controlOut, cmp);
    cmp = heaterMdl(mdlInput, estimatedParams, controlOut, cmp);

    % ---- Electric powertrain & cabin ----
    cmp = batMdl(mdlInput, estimatedParams, controlOut, cmp, P);
    cmp = dcdcMdl(mdlInput, estimatedParams, controlOut, cmp, P);
    cmp = motMdl(mdlInput, estimatedParams, controlOut, cmp, P);
    cmp = invMdl(mdlInput, estimatedParams, controlOut, cmp, P);
    cmp = cabMdl(mdlInput, estimatedParams, controlOut, cmp, P);

    % ---- Pressure state dynamics ----
    cmp = pressureStateCalcMdl(mdlInput, estimatedParams, controlOut, cmp, P);
    cmp = pwrEstimation(mdlInput, estimatedParams, controlOut, cmp);

    % ---- 3) Assemble state derivatives (9x1) ----
    xdot = [
        cmp.mot.output.motT_dt;        % 1 motor T dot
        cmp.inv.output.invT_dt;        % 2 inverter T dot
        cmp.dcdc.output.dcdcT_dt;      % 3 dcdc T dot
        cmp.bat.output.batSoc_dt;      % 4 SOC dot
        cmp.bat.output.batT_dt;        % 5 battery T dot
        cmp.pressure.output.compPin_dt;% 6 compressor inlet pressure dot
        cmp.pressure.output.compPout_dt;%7 compressor outlet pressure dot
        cmp.cab.output.cabIntT_dt;     % 8 cabin interior mass T dot
        cmp.cab.output.cabAirT_dt      % 9 cabin air T dot
    ];
     mdlOut_sym = [
            % ---- Coolant temperatures (K) ----
            cmp.inv.output.clntToutK;            %  1 inverter coolant outlet
            cmp.mot.output.clntToutK;            %  2 motor coolant outlet
            cmp.bat.input.clntTinK;              %  3 battery coolant inlet (from heater)
            cmp.bat.output.clntToutK;            %  4 battery coolant outlet
            cmp.dcdc.output.clntToutK;           %  5 DCDC coolant outlet
            cmp.htx.output.clntToutK;            %  6 HTX coolant outlet
            cmp.heater.output.clntToutK;         %  7 heater coolant outlet
            cmp.chiller.output.clntToutK;        %  8 chiller coolant outlet
        
            % ---- Refrigerant temperatures (K) ----
            cmp.compressor.output.compToutK;     %  9 compressor outlet temperature
            cmp.innerCond.output.refToutK;       % 10 inner condenser ref outlet
            cmp.condEvap.output.refToutK;        % 11 condenser/evap ref outlet
            cmp.evap.output.refToutK;            % 12 evaporator ref outlet
            cmp.chiller.output.refToutK;         % 13 chiller ref outlet
            cmp.htx.output.refToutK;             % 14 HTX ref outlet
        
            % ---- Air-side temperatures (K) ----
            cmp.innerCond.output.cabAirToutK;    % 15 air after inner condenser (vent air)
            cmp.condEvap.output.airToutK;        % 16 air after front heat exchanger
            cmp.evap.output.airToutK; 
            % 17 air after evaporator
        
            % ---- Flows (kg/s) ----
            cmp.motPump.output.clntMf;           % 18 motor loop coolant mf
            cmp.batPump.output.clntMf;           % 19 battery loop coolant mf
            cmp.compressor.output.refMf;         % 20 refrigerant mass flow
            mdlInput.u.cabAirMfIn;               % 21 cabin air mass flow (blower)
        
            % ---- Powers (W) ----
            cmp.pwrEst.out.pwrCompElec;          % 22 compressor electrical power
            cmp.pwrEst.out.pwrBlowElec;          % 23 blower electrical power
            cmp.pwrEst.out.pwrPumpMotElec;       % 24 motor-pump electrical power
            cmp.pwrEst.out.pwrPumpBatElec;       % 25 battery-pump electrical power
            cmp.pwrEst.out.pwrTem;               % 26 total TEM power (same as J)
            cmp.innerCond.input.airTinK;         % 27

            cmp.mot.output.clntToutK; %28
            cmp.mot.input.clntTinK;%29

            cmp.heater.input.clntTinK;%30
            cmp.hpExv.output.refToutK;%31
            ];


     J = cmp.pwrEst.out.pwrTem;

end

% ========================= runtime context builder =========================
function [mdlInput, estimatedParams, controlOut, tunePara] = build_runtime_context(x, u, p)
    % States -> mdlInput.state
    mdlInput.state.motT       = x(1);
    mdlInput.state.invT       = x(2);
    mdlInput.state.dcdcT      = x(3);
    mdlInput.state.batSoc     = x(4);
    mdlInput.state.batT       = x(5);
    mdlInput.state.compPinPa  = x(6);
    mdlInput.state.compPoutPa = x(7);
    mdlInput.state.cabIntT    = x(8);
    mdlInput.state.cabAirT    = x(9);

    mdlInput.u.compNrpm    = u.compNrpm;
    mdlInput.u.cabAirMfIn  = u.cabAirMfIn + 1e-3;
    mdlInput.u.motPumpNrpm = u.motPumpNrpm;
    mdlInput.u.batPumpNrpm = u.batPumpNrpm;
    mdlInput.u.heaterPwr   = u.heaterPwr;
    mdlInput.u.radFanrpm   = u.radFanrpm;

    estimatedParams = p.est;
    controlOut      = p.ctrl;
    tunePara        = p.tuneParam;
    mdlInput.u.batImeas = estimatedParams.batImeas + 1e-3;
end



% ======================================================================
% Thermal components
% ======================================================================
function [T0, alpha, beta] = solve_loop_htx_in_explicit(cmp, mdlInput, estimatedParams, controlOut)
    % Flows & cp
    clntCp = estimatedParams.clntCp;
    m_mot  = cmp.motPump.output.clntMf;
    m_tot_bat = cmp.batPump.output.clntMf;
    split  = 0.11;
    m_dcdc = split * m_tot_bat;
    m_bat  = m_tot_bat - m_dcdc;
    m_mix  = m_bat + m_dcdc;                                

    [a_htx, b_htx] = local_htx_affine(cmp, estimatedParams, m_mot);
    [a_heat, b_heat] = local_heater_affine(estimatedParams, controlOut, m_tot_bat, clntCp, mdlInput);

    a_T2_T0 = a_heat * a_htx;
    b_T2    = a_heat * b_htx + b_heat;

    [a_bat,  b_bat]  = local_branch_affine_bat(cmp, estimatedParams, m_bat,  clntCp, mdlInput.state.batT);
    [a_dcdc, b_dcdc] = local_branch_affine_dcdc(cmp, estimatedParams, m_dcdc, clntCp, mdlInput.state.dcdcT);

    w_bat  = m_bat /(m_bat + m_dcdc + 1e-12);
    w_dcdc = 1 - w_bat;
    a_mix  = w_bat*a_bat + w_dcdc*a_dcdc;
    b_mix  = w_bat*b_bat + w_dcdc*b_dcdc;

    cmdPS = controlOut.cmdParallelSerial;
    a_Tmotin_T0 = cmdPS * (a_mix * a_T2_T0) + (1 - cmdPS) * a_htx;
    b_Tmotin    = cmdPS * (a_mix * b_T2 + b_mix) + (1 - cmdPS) * b_htx;

    [a_mot, b_mot] = local_component_affine_mot(cmp, estimatedParams, m_mot, clntCp, mdlInput.state.motT);
    [a_inv, b_inv] = local_component_affine_inv(cmp, estimatedParams, m_mot, clntCp, mdlInput.state.invT);

    a_chain = a_mot * a_Tmotin_T0;
    b_chain = a_mot * b_Tmotin + b_mot;

    alpha = a_inv * a_chain;
    beta  = a_inv * b_chain + b_inv;

    T0 = beta / (1 - alpha + 1e-6);
end

% ===================== Local affine builders =====================


function [a_htx, b_htx] = local_htx_affine(cmp, estimatedParams, m_mot)
    refArea  = 7.854e-05; clntArea = 0.00028353;
    totalHxArea = 64*pi*0.0035*0.1;
    ntuCorrectionFactor = 0.9;
    NTU_MAX = 10;

    refCp  = estimatedParams.condEvapRefCp;  refRho = estimatedParams.condEvapRefRho;
    refK   = estimatedParams.condEvapRefK;   refMu  = estimatedParams.condEvapRefMu;
    clntCp = estimatedParams.clntCp;         clntRho = estimatedParams.clntRho;
    clntK  = estimatedParams.clntK;          clntMu  = estimatedParams.clntMu;

    refMf  = cmp.compressor.output.refMf;
    clntMf = m_mot;
    refTinK = cmp.condEvap.output.refToutK;

    refD  = sqrt((4*refArea)/pi);  clntD = sqrt((4*clntArea)/pi);
    refV  = refMf /(refRho*refArea  + 1e-6);
    clntV = clntMf/(clntRho*clntArea + 1e-6);
    refRe  = (refRho*refV *refD )/(refMu  + 1e-6);
    clntRe = (clntRho*clntV*clntD)/(clntMu + 1e-6);
    refPr  = (refCp *refMu )/(refK  + 1e-6);
    clntPr = (clntCp*clntMu)/(clntK + 1e-6);
    refNu  = 0.023*(refRe^0.8 )*(refPr^0.33);
    clntNu = 0.023*(clntRe^0.8)*(clntPr^0.33);
    h_ref  = (refNu *refK )/(refD  + 1e-6);
    h_clnt = (clntNu*clntK)/(clntD + 1e-6);

    thermalResistance = (1/(h_ref*totalHxArea + 1e-6)) + (1/(h_clnt*totalHxArea + 1e-6));
    UA = 1/(thermalResistance + 1e-6);

    c_clnt = clntMf*clntCp;

    NTU_raw = (ntuCorrectionFactor*UA) / (c_clnt + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    A = 1 - exp(-NTU);

    a_htx = 1 - A;
    b_htx = A * refTinK;
end


function [a_heat, b_heat] = local_heater_affine(estimatedParams, controlOut, m_tot_bat, clntCp, mdlInput)
    heaterEfficiency = 0.85;
    heatTransferCorrection = 1.5;

    clntMf = m_tot_bat;
    heaterActualPowerW = heaterEfficiency * mdlInput.u.heaterPwr;
    denominator = heatTransferCorrection*clntMf*clntCp;
    dT = heaterActualPowerW / (denominator + 1e-6);

    a_heat = 1;
    b_heat = dT;
end

function [a_bat, b_bat] = local_branch_affine_bat(cmp, estimatedParams, m_bat, clntCp, T_batK)
    clntChannelD = 0.0092; hxAreaFactor = 0.736;
    convectionTuningFactor = 200; conductionTuningFactor = 0.7;
    NTU_MAX = 10;

    clntMf = m_bat;
    h_conv = 3.66*estimatedParams.clntK/clntChannelD;
    UA = h_conv*hxAreaFactor*convectionTuningFactor;
    c_clnt = clntMf*clntCp;

    % NTU cap
    NTU_raw = (0.1*UA) / (c_clnt + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    k_cond = conductionTuningFactor*hxAreaFactor/clntChannelD;
    k_cond_raw = k_cond / (c_clnt + 1e-6);
    k_cond_eff = NTU_MAX * tanh(k_cond_raw / NTU_MAX);

    S_bat = effectiveness + k_cond_eff;

    a_bat = 1 - S_bat;
    b_bat = S_bat * T_batK;
end

function [a_dcdc, b_dcdc] = local_branch_affine_dcdc(cmp, estimatedParams, m_dcdc, clntCp, T_dcdcK)
    clntChannelD = 0.0092; hxAreaFactor = 0.736;
    convectionTuningFactor = 1; conductionTuningFactor = 0.245;
    NTU_MAX = 10;

    clntMf = m_dcdc;
    h_conv = 3.66*0.254/clntChannelD;
    UA = h_conv*hxAreaFactor*convectionTuningFactor;
    c_clnt = clntMf*clntCp;

    NTU_raw = UA / (c_clnt + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);


    k_cond = conductionTuningFactor*hxAreaFactor/clntChannelD;
    k_cond_raw = k_cond / (c_clnt + 1e-6);
    k_cond_eff = NTU_MAX * tanh(k_cond_raw / NTU_MAX);

    S_dcdc = effectiveness + k_cond_eff;

    a_dcdc = 1 - S_dcdc;
    b_dcdc = S_dcdc * T_dcdcK;
end

function [a_mot, b_mot] = local_component_affine_mot(cmp, estimatedParams, m_mot, clntCp, T_motK)
    clntChannelD = 0.0092; hxAreaFactor = 0.736;
    convectionTuningFactor = 6.0; conductionTuningFactor = 0.254;
    NTU_MAX = 10;

    clntMf = m_mot;
    h_conv = 3.66*estimatedParams.clntK/clntChannelD;
    UA = h_conv*hxAreaFactor*convectionTuningFactor;
    c_clnt = clntMf*clntCp;


    NTU_raw = UA / (c_clnt + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    k_cond = conductionTuningFactor*hxAreaFactor/clntChannelD;
    k_cond_raw = k_cond / (c_clnt + 1e-6);
    k_cond_eff = NTU_MAX * tanh(k_cond_raw / NTU_MAX);

    S = effectiveness + k_cond_eff;

    a_mot = 1 - S;
    b_mot = S * T_motK;
end

function [a_inv, b_inv] = local_component_affine_inv(cmp, estimatedParams, m_mot, clntCp, T_invK)
    clntChannelD = 0.0092; hxAreaFactor = 0.736;
    convectionTuningFactor = 6.0; conductionTuningFactor = 0.254;
    NTU_MAX = 10;

    clntMf = m_mot;
    h_conv = 3.66*estimatedParams.clntK/clntChannelD;
    UA = h_conv*hxAreaFactor*convectionTuningFactor;
    c_clnt = clntMf*clntCp;

    NTU_raw = (0.1*UA) / (c_clnt + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    k_cond = conductionTuningFactor*hxAreaFactor/clntChannelD;
    k_cond_raw = k_cond / (c_clnt + 1e-6);
    k_cond_eff = NTU_MAX * tanh(k_cond_raw / NTU_MAX);

    S = effectiveness + k_cond_eff;

    a_inv = 1 - S;
    b_inv = S * T_invK;
end

% ======================================================================

function cmp = htxMdl(mdlInput, estimatedParams, controlOut, cmp, P)
    refArea  = 7.854e-05;
    clntArea = 0.00028353;
    totalHxArea = 64*pi*0.0035*0.1;
    ntuCorrectionFactor = 0.9;
    NTU_MAX = 10;

    clntMf = cmp.motPump.output.clntMf;
    refMf  = cmp.compressor.output.refMf;
    refTinK  = cmp.condEvap.output.refToutK;
    clntTinK = mdlInput.u.htxClntTinK;

    refCp  = estimatedParams.condEvapRefCp;
    refRho = estimatedParams.condEvapRefRho;
    refK   = estimatedParams.condEvapRefK;
    refMu  = estimatedParams.condEvapRefMu;
    clntCp  = estimatedParams.clntCp;
    clntRho = estimatedParams.clntRho;
    clntK   = estimatedParams.clntK;
    clntMu  = estimatedParams.clntMu;

    refD  = sqrt((4*refArea)/pi);
    clntD = sqrt((4*clntArea)/pi);
    refV  = refMf /(refRho *refArea  + 1e-6);
    clntV = clntMf/(clntRho*clntArea + 1e-6);
    refRe  = (refRho *refV *refD )/(refMu  + 1e-6);
    clntRe = (clntRho*clntV*clntD)/(clntMu + 1e-6);
    refPr  = (refCp *refMu )/(refK  + 1e-6);
    clntPr = (clntCp*clntMu)/(clntK + 1e-6);
    refNu  = 0.023*(refRe^0.8 )*(refPr^0.33);
    clntNu = 0.023*(clntRe^0.8)*(clntPr^0.33);
    h_ref  = (refNu *refK )/(refD  + 1e-6);
    h_clnt = (clntNu*clntK)/(clntD + 1e-6);

    thermalResistance = (1/(h_ref*totalHxArea + 1e-6)) + (1/(h_clnt*totalHxArea + 1e-6));
    UA = 1/(thermalResistance + 1e-6);

    c_clnt = clntMf*clntCp;
    c_min  = clntMf*clntCp;

    NTU_raw = (ntuCorrectionFactor*UA) / (c_min + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    qMax    = c_min*(clntTinK - refTinK);
    qActual = effectiveness*qMax;

    delta_T_clnt = qActual / (c_clnt + 1e-6);
    clntToutK    = clntTinK - delta_T_clnt;

    refToutK = refTinK;

    cmp.htx.output.clntToutK = clntToutK;
    cmp.htx.output.refToutK  = refToutK;
    cmp.htx.output.qActualW  = qActual;
end

function cmp = cabMdl(mdlInput, estimatedParams, controlOut, cmp, P)
    glassArea_m2 = 3.5;  glassU_W_m2K = 40;  glassThick_m = 0.002; glassK_W_mK = 0.96;
    glassMass_kg = 36.4; glassCp_J_kgK = 840;
    doorsArea_m2 = 4.0; doorsU_W_m2K = 40; doorsThick_m = 0.03; doorsK_W_mK = 0.08;
    doorsMass_kg = 75.0; doorsCp_J_kgK = 500;
    roofArea_m2 = 2.0; roofU_W_m2K = 40; roofThick_m = 0.02; roofK_W_mK = 0.08;
    roofMass_kg = 30.0; roofCp_J_kgK = 500;

    cabVolume_m3 = 3.0; airDensity_kg_m3 = 1.225; qHuman_W = 70.0;
    airCp_J_kgK = 1006;

    alpha_R_glass = 0.01217; alpha_R_doors = 0.15076; alpha_R_roof = 0.63668;
    alpha_dt_int_num = 42237.0; alpha_dt_int_den = 2287.85;
    alpha_dt_air_num = 0.01611; alpha_R_air2int = 0.02475;
    alpha_Q_in2out = 1.0; alpha_Q_int2air = 1.0;

    cabAirT_K = mdlInput.state.cabAirT;
    cabIntT_K = mdlInput.state.cabIntT;
    airMf_In     = mdlInput.u.cabAirMfIn;
    airTempVent_K = cmp.innerCond.output.cabAirToutK;
    envT_K = estimatedParams.envT;

    R_glass = alpha_R_glass*(1/(glassU_W_m2K*glassArea_m2) + glassThick_m/(glassK_W_mK*glassArea_m2));
    R_doors = alpha_R_doors*(1/(doorsU_W_m2K*doorsArea_m2) + doorsThick_m/(doorsK_W_mK*doorsArea_m2));
    R_roof  = alpha_R_roof *(1/(roofU_W_m2K *roofArea_m2 ) + roofThick_m /(roofK_W_mK *roofArea_m2));
    R_total_env2int = R_glass + R_doors + R_roof;

    totalMass_int = glassMass_kg + doorsMass_kg + roofMass_kg;
    avgCp_int = (glassMass_kg*glassCp_J_kgK + doorsMass_kg*doorsCp_J_kgK + roofMass_kg*roofCp_J_kgK) / totalMass_int;
    thermalMass_int = totalMass_int*avgCp_int;

    q_env2int = (envT_K - cabIntT_K) / (R_total_env2int + 1e-6);
    q_air2int = alpha_Q_in2out*(cabAirT_K - cabIntT_K) / (R_total_env2int + 1e-6);
    qNet_int  = q_env2int + q_air2int;

    cabIntT_dt = (P(1)*alpha_dt_int_num*qNet_int) / (alpha_dt_int_den*thermalMass_int + 1e-6);

    thermalMass_air = cabVolume_m3*airDensity_kg_m3*airCp_J_kgK;
    qHvac = airMf_In*airCp_J_kgK*(airTempVent_K - cabAirT_K);
    R_int2air = alpha_R_air2int*R_total_env2int;
    q_int2air = alpha_Q_int2air*(cabIntT_K - cabAirT_K) / (R_int2air + 1e-6);

    cabAirT_dt = (P(2)*alpha_dt_air_num/(thermalMass_air + 1e-6)) * (qHvac + qHuman_W + P(8)*q_int2air);

    cmp.cab.output.cabIntT_dt = cabIntT_dt;
    cmp.cab.output.cabAirT_dt = cabAirT_dt;
end

function cmp = pumpMdl(mdlInput, estimatedParams, controlOut, cmp)
    pumpVolEff        = 0.92;
    pumpDispLperRev   = 0.02;
    densityCorrection = 1.0660;

    batPumpNrpm = mdlInput.u.batPumpNrpm;
    motPumpNrpm = mdlInput.u.motPumpNrpm;
    clntRho = 1100;

    pumpDispM3perRev = pumpDispLperRev*1e-3;

    batPumpNrevPerS      = batPumpNrpm/60;
    batPumpVolFlowM3perS = pumpDispM3perRev*batPumpNrevPerS;
    batClntMf = clntRho * densityCorrection * batPumpVolFlowM3perS * pumpVolEff + 1e-4;

    motPumpNrevPerS      = motPumpNrpm/60;
    motPumpVolFlowM3perS = pumpDispM3perRev*motPumpNrevPerS;
    motClntMf = clntRho * densityCorrection * motPumpVolFlowM3perS * pumpVolEff + 1e-4;

    cmp.motPump.output.clntMf = motClntMf;
    cmp.batPump.output.clntMf = batClntMf;
end

function cmp = compressorMdl(mdlInput, estimatedParams, controlOut, cmp)
    compDispM3perRev = 80e-6;
    isenEff = 0.65;
    mechEff = 0.9;
    massFlowCorrection = 0.12;

    compPinPa  = mdlInput.state.compPinPa;
    compPoutPa = mdlInput.state.compPoutPa;
    compNrpm   = mdlInput.u.compNrpm + 1e-5;

    h_in   = estimatedParams.compHin;
    h_outS = estimatedParams.compHoutS;
    vNominalIn = estimatedParams.compVnominalIn;

   
    p_in_floor  = 0.3e5;  
    eps_pFloor  = 1e6;    
    compPinSafe = (compPinPa + p_in_floor + sqrt((compPinPa - p_in_floor)^2 + eps_pFloor)) / 2;

    pressureRatio = compPoutPa / compPinSafe;
    volEff_calc = -0.0286*pressureRatio + 1.0286;
    k_smooth = 50;
    volEff = 0.01 + (1/k_smooth)*log(1 + exp(k_smooth*(volEff_calc - 0.01)));

    angularVel = compNrpm*(2*pi/60);
    refMf = volEff * angularVel * compDispM3perRev * massFlowCorrection / vNominalIn + 1e-4;

    deltaH_isentropic = h_outS - h_in;
    h_out = h_in + deltaH_isentropic/isenEff;

    refTsatHighPK = estimatedParams.refTsatHighPK;
    refCpVapHighP = estimatedParams.refCpVapHighP;

    temp_rise = 2*((h_outS - h_in)/(refCpVapHighP + 1e-6));
    compToutK = refTsatHighPK + temp_rise;

    powerW = refMf*(h_out - h_in)/mechEff;

    cmp.compressor.output.refMf     = refMf;
    cmp.compressor.output.compToutK = compToutK;
    cmp.compressor.output.h_out     = h_out;
    cmp.compressor.output.h_in      = h_in;
    cmp.compressor.output.powerW    = powerW;
end

function cmp = innerCondenserMdl(mdlInput, estimatedParams, controlOut, cmp)
    refArea = 7.854e-05;  airArea = 0.04;
    totalHxArea = 1.1088 + 0.9952;
    wallThermalResistance = 4.1868e-07;
    ntuAreaFactor = 18*40;
    NTU_MAX = 10;

    refMf      = cmp.compressor.output.refMf;
    cabAirMfIn = mdlInput.u.cabAirMfIn;
    refTinK    = cmp.compressor.output.compToutK;
    cabAirTinK = mdlInput.state.cabAirT;

    cabAirCp  = estimatedParams.cabAirCp;
    cabAirK   = estimatedParams.cabAirK;
    cabAirMu  = estimatedParams.cabAirMu;
    cabAirRho = estimatedParams.cabAirRho;
    refCp  = estimatedParams.innerCondRefCp;
    refRho = estimatedParams.innerCondRefRho;
    refK   = estimatedParams.innerCondRefK;
    refMu  = estimatedParams.innerCondRefMu;

    cmdBlendAir = controlOut.cmdBlendAir;

    refD = 0.0100; airD = 0.0190;

    refV = refMf/(refRho*refArea + 1e-6);
    airV = (cmdBlendAir*cabAirMfIn/(cabAirRho*airArea + 1e-6)) + 1e-6;

    refRe = (refRho*refV*refD)/(refMu + 1e-6);
    airRe = (cabAirRho*airV*airD)/(cabAirMu + 1e-6);
    refPr = (refCp*refMu)/(refK + 1e-6);
    airPr = (cabAirCp*cabAirMu)/(cabAirK + 1e-6);
    refNu = 0.023*(refRe^0.8)*(refPr^0.33);
    airNu = 0.023*(airRe^0.8)*(airPr^0.33);
    refU = (refNu*refK)/(refD + 1e-6);
    airU = (airNu*cabAirK)/(airD + 1e-6);

    thermalResistance = (1/(refU*refArea + 1e-6)) + (1/(airU*airArea + 1e-6)) + wallThermalResistance;

    c_air = cabAirMfIn*cabAirCp;
    c_min = c_air;

    c_clnt_air = cabAirMfIn;
    c0_air = 1e-4; alpha_air = 5e4;
    z_air = alpha_air*(c_clnt_air - c0_air);
    sigmoid_gate_c_air = 0.5*(1 + tanh(z_air));

    NTU_raw = (ntuAreaFactor*totalHxArea) / (thermalResistance*c_min + 1e-6);
    NTU_raw = NTU_raw * sigmoid_gate_c_air;
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    airTinK = cabAirTinK;
    qMax    = c_min*(refTinK - airTinK);
    qActual = effectiveness*qMax;

    delta_T_air = (qActual/(c_air + 1e-6)) * sigmoid_gate_c_air;
    cabAirToutK = cabAirTinK + delta_T_air;

    c_ref = refMf*refCp;
    c_clnt_ref = refMf;
    c0_ref = 1e-4; alpha_ref = 5e4;
    z_ref = alpha_ref*(c_clnt_ref - c0_ref);
    sigmoid_gate_c_ref = 0.5*(1 + tanh(z_ref));

    delta_T_ref = (0.35*qActual/(c_ref + 1e-6)) * sigmoid_gate_c_ref;
    refToutK = refTinK - delta_T_ref;

    h_subcooled = estimatedParams.refHsatLiqHighP + refCp*(refToutK - estimatedParams.refTsatLiqHighPK);

    cmp.innerCond.output.cabAirToutK = cabAirToutK;
    cmp.innerCond.output.refToutK    = refToutK;
    cmp.innerCond.output.qActualW    = qActual;
    cmp.innerCond.output.h_out       = h_subcooled;
    cmp.innerCond.input.airTinK      = airTinK;
end

function cmp = hpExvMdl(mdlInput, estimatedParams, controlOut, cmp)
    refToutK_cond    = cmp.innerCond.output.refToutK;
    refTsatLiqHighPK = estimatedParams.refTsatLiqHighPK;
    refHsatLiqHighP  = estimatedParams.refHsatLiqHighP;
    refCpLiqHighP    = estimatedParams.refCpLiqHighP;
    h_comp_out       = cmp.compressor.output.h_out;
    refTsatLiqLowPK  = estimatedParams.refTsatLiqLowPK;
    cmdExvHeatpump   = controlOut.cmdExvHeatpump;

    h_subcooled_liquid = refHsatLiqHighP + refCpLiqHighP*(refToutK_cond - refTsatLiqHighPK);
    refToutK_expanded  = refTsatLiqLowPK;

    h_out        = (h_subcooled_liquid*cmdExvHeatpump) + (h_comp_out*(1 - cmdExvHeatpump));
    refToutK_out = (refToutK_expanded *cmdExvHeatpump) + (refToutK_cond*(1 - cmdExvHeatpump));

    cmp.hpExv.output.refToutK = refToutK_out;
    cmp.hpExv.output.h_out    = h_out;
end

function cmp = condEvapMdl(mdlInput, estimatedParams, controlOut, cmp)
    refArea = 7.854e-05;  airArea = 0.04;
    totalHxArea = 1.1088 + 0.9952;
    NTU_MAX = 10;

    refMf   = cmp.compressor.output.refMf;
    refTinK = cmp.hpExv.output.refToutK;

    tau_fan   = 5.0;
    dt_plant  = 1;
    alpha_fan = exp(-dt_plant / tau_fan);
    fanRpm_cmd    = mdlInput.u.radFanrpm;
    fanRpm_approx = alpha_fan * estimatedParams.fanRpm_prev + (1 - alpha_fan) * fanRpm_cmd;

    airMf_raw = 0.000258 * estimatedParams.speed / 3.6 + 0.000012 * fanRpm_cmd/60;
    airMf_min = 1e-8;
    eps_airMf = 1e-12;

    airMf = (airMf_raw + airMf_min + sqrt((airMf_raw - airMf_min)^2 + eps_airMf)) / 2;
    airMf = 0;
    airTinK   = estimatedParams.envT;

    refCp  = estimatedParams.condEvapRefCp;
    refRho = estimatedParams.condEvapRefRho;
    refK   = estimatedParams.condEvapRefK; 
    refMu  = estimatedParams.condEvapRefMu;
    airCp  = estimatedParams.condEvapAirCp;
    airRho = estimatedParams.condEvapAirRho;
    airK   = estimatedParams.condEvapAirK;
    airMu  = estimatedParams.condEvapAirMu;

    thermalMode = controlOut.thermalMode;

    refD = sqrt((4*refArea)/pi);
    airD = sqrt((4*airArea)/pi);
    refV = refMf/(refRho*refArea + 1e-6);
    airV = airMf/(airRho*airArea + 1e-6);

    refRe = (refRho*refV*refD)/(refMu + 1e-6);
    airRe = (airRho*airV*airD)/(airMu + 1e-6);
    refPr = (refCp*refMu)/(refK + 1e-6);
    airPr = (airCp*airMu)/(airK + 1e-6);

    eps_Re = 1e-4;
    refNu = 0.023*((refRe + eps_Re)^0.8)*(refPr^0.33);
    airNu = 0.023*((airRe + eps_Re)^0.8)*(airPr^0.33);

    h_ref = (refNu*refK)/(refD + 1e-6);
    h_air = (airNu*airK)/(airD + 1e-6);

    thermalResistance = (1/(h_ref*totalHxArea + 1e-6)) + (1/(h_air*totalHxArea + 1e-6));
    UA = 1/(thermalResistance + 1e-6);

    c_ref = refMf*refCp;
    c_air = airMf*airCp;

    isHeatPumpMode = 1/(1 + exp(-10*(thermalMode - 1)));

    c_min_evap = c_air; qMax_evap = c_min_evap*(airTinK - refTinK);
    c_min_cond = c_air; qMax_cond = c_min_cond*(refTinK - airTinK);

    % c_min = (c_min_evap*isHeatPumpMode) + (c_min_cond*(1 - isHeatPumpMode));
    qMax  = (qMax_evap *isHeatPumpMode) + (qMax_cond *(1 - isHeatPumpMode));

    % NTU cap
    NTU_raw = UA / (c_air + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    qActual = effectiveness*qMax;

    delta_T_ref = qActual / (c_ref + 1e-6);
    refTempChangeDirection = (1*isHeatPumpMode) + (-1*(1 - isHeatPumpMode));
    refToutK = refTinK + refTempChangeDirection*delta_T_ref;

    delta_T_air = qActual / (c_air + 1e-6);
    airTempChangeDirection = (-1*isHeatPumpMode) + (1*(1 - isHeatPumpMode));
    airToutK = airTinK + airTempChangeDirection*delta_T_air;

    cmp.condEvap.output.refToutK = refToutK;
    cmp.condEvap.output.airToutK = airToutK;
    cmp.condEvap.output.qActualW = qActual;
end

function cmp = evapExv(mdlInput, estimatedParams, controlOut, cmp)%% not used in cold climate 
    cmp.evapExv.output.refToutK = 0; %% not used in cold climate 
    cmp.evapExv.output.h_out    = 0;
end

function cmp = evaporatorMdl(mdlInput, estimatedParams, controlOut, cmp)%% not used in cold climate 
    refArea = 7.854e-05;  airArea = 0.00028353; 
    totalHxArea = 64*pi*0.0035*0.1;
    ntuCorrectionFactor = 1.3;
    NTU_MAX = 10;

    refMf  = cmp.compressor.output.refMf;
    airMf  = mdlInput.u.cabAirMfIn;
    refTinK = cmp.htx.output.refToutK;

    airCp  = estimatedParams.cabAirCp;
    airRho = estimatedParams.cabAirRho;
    airK   = estimatedParams.cabAirK;
    airMu  = estimatedParams.cabAirMu;
    refCp  = estimatedParams.evapRefCp;
    refRho = estimatedParams.evapRefRho;
    refK   = estimatedParams.evapRefK;
    refMu  = estimatedParams.evapRefMu;

    envT_K    = estimatedParams.envT;
    cabAirT_K = mdlInput.state.cabAirT;
    airTinK   = 0.1*envT_K + 0.9*cabAirT_K;
    cmdExvEvap = controlOut.cmdExvEvap;

    refD = sqrt((4*refArea)/pi);
    airD = sqrt((4*airArea)/pi);
    refV = refMf/(refRho*refArea  + 1e-6);
    airV = airMf/(airRho*airArea  + 1e-6);

    refRe = (refRho*refV*refD)/(refMu + 1e-6);
    airRe = (airRho*airV*airD)/(airMu + 1e-6);
    refPr = (refCp*refMu)/(refK + 1e-6);
    airPr = (airCp*airMu)/(airK + 1e-6);
    refNu = 0.023*(refRe^0.8)*(refPr^0.33);
    airNu = 0.023*(airRe^0.8)*(airPr^0.33);
    h_ref = (refNu*refK)/(refD + 1e-6);
    h_air = (airNu*airK)/(airD + 1e-6);

    thermalResistance = (1/(h_ref*totalHxArea + 1e-6)) + (1/(h_air*totalHxArea + 1e-6));
    UA = 1/(thermalResistance + 1e-6);

    c_air = airMf*airCp;
    c_min = c_air;

    % NTU cap
    NTU_raw = (ntuCorrectionFactor*UA) / (c_min + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    qMax         = c_min*(airTinK - refTinK);
    qTheoretical = effectiveness*qMax;
    qActual      = qTheoretical*cmdExvEvap;

    delta_T_air = qActual / (c_air + 1e-6);
    airToutK    = airTinK - delta_T_air;
    refToutK    = refTinK;

    cmp.evap.output.airToutK = airToutK;
    cmp.evap.output.refToutK = refToutK;
    cmp.evap.output.qActualW = qActual;
end

function cmp = chillerMdl(mdlInput, estimatedParams, controlOut, cmp)%% not used in cold climate 
    refArea = 7.854e-05;  clntArea = 0.00028353;
    totalHxArea = 64*pi*0.0035*0.1;
    ntuCorrectionFactor = 1.3;
    NTU_MAX = 10;

    clntMf = cmp.batPump.output.clntMf;
    refMf  = cmp.compressor.output.refMf;

    refTinK  = cmp.evapExv.output.refToutK * controlOut.cmdExvChiller;
    h_in     = cmp.evapExv.output.h_out    * controlOut.cmdExvChiller;
    clntTinK = cmp.htx.output.clntToutK;

    refCp  = estimatedParams.chilRefCp;
    refRho = estimatedParams.chilRefRho;
    refK   = estimatedParams.chilRefK;
    refMu  = estimatedParams.chilRefMu;
    clntCp  = estimatedParams.clntCp;
    clntRho = estimatedParams.clntRho;
    clntK   = estimatedParams.clntK;
    clntMu  = estimatedParams.clntMu;
    cmdExvChiller = controlOut.cmdExvChiller;

    refD  = sqrt((4*refArea )/pi);
    clntD = sqrt((4*clntArea)/pi);
    refV  = refMf /(refRho *refArea  + 1e-6);
    clntV = clntMf/(clntRho*clntArea + 1e-6);
    refRe  = (refRho *refV *refD )/(refMu  + 1e-6);
    clntRe = (clntRho*clntV*clntD)/(clntMu + 1e-6);
    refPr  = (refCp *refMu )/(refK  + 1e-6);
    clntPr = (clntCp*clntMu)/(clntK + 1e-6);
    refNu  = 0.023*(refRe^0.8 )*(refPr^0.33);
    clntNu = 0.023*(clntRe^0.8)*(clntPr^0.33);
    h_ref  = (refNu *refK )/(refD  + 1e-6);
    h_clnt = (clntNu*clntK)/(clntD + 1e-6);

    thermalResistance = (1/(h_ref*totalHxArea + 1e-6)) + (1/(h_clnt*totalHxArea + 1e-6));
    UA = 1/(thermalResistance + 1e-6);

    c_clnt = clntMf*clntCp;
    c_min  = c_clnt;

    % NTU cap
    NTU_raw = (ntuCorrectionFactor*UA) / (c_min + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    qMax         = c_min*(clntTinK - refTinK);
    qTheoretical = effectiveness*qMax;
    qActual      = qTheoretical*cmdExvChiller;

    delta_T_clnt = qActual / (c_clnt + 1e-6);
    clntToutK    = clntTinK - delta_T_clnt;

    refToutK = refTinK;
    h_out    = h_in + (qActual/(refMf + 1e-6));

    cmp.chiller.output.clntToutK = clntTinK;
    cmp.chiller.output.refToutK  = refToutK;
    cmp.chiller.output.h_out     = h_out;
    cmp.chiller.output.qActualW  = qActual;
end

function cmp = heaterMdl(mdlInput, estimatedParams, controlOut, cmp)
    heaterEfficiency = 0.85;
    heatTransferCorrection = 1.5;

    clntTinK = cmp.chiller.output.clntToutK;
    clntMf   = cmp.batPump.output.clntMf;
    clntCp   = estimatedParams.clntCp;

    heaterActualPowerW = heaterEfficiency*mdlInput.u.heaterPwr;

    denominator = heatTransferCorrection*clntMf*clntCp;
    delta_T = heaterActualPowerW / (denominator + 1e-6);

    clntToutK = clntTinK + delta_T;

    cmp.heater.output.clntToutK = clntToutK;
    cmp.heater.input.clntTinK   = clntTinK;
end

function cmp = batMdl(mdlInput, estimatedParams, controlOut, cmp, P)
    clntChannelD = 0.0092;
    hxAreaFactor = 0.736;
    NTU_MAX = 10;

    cellMassKg = 2.5; cellsInSeries = 20; packsInParallel = 4;
    batMassKg = cellMassKg*cellsInSeries*packsInParallel;
    batCp = 795;
    convectionTuningFactor = 200;
    conductionTuningFactor = 0.7;
    jouleHeatingTuningFactor = 70;
    socCapacityFactor = 44*20;
    thermalMassTuningFactor = 0.75;
    dcdcFlowSplitRatio = 0.11;

    batT_K = mdlInput.state.batT;
    batSoc = mdlInput.state.batSoc;
    batI   = mdlInput.u.batImeas;
    clntTinK = cmp.heater.output.clntToutK;
    clntCp   = estimatedParams.clntCp;

    clntTotalMf = cmp.batPump.output.clntMf;
    clntMf_dcdc = dcdcFlowSplitRatio*clntTotalMf;
    clntMf_bat  = clntTotalMf - clntMf_dcdc;

    h_conv = 3.66*estimatedParams.clntK/clntChannelD;
    UA = h_conv*hxAreaFactor*convectionTuningFactor;
    c_clnt = clntMf_bat*clntCp;

    % NTU cap
    NTU_raw = (P(7)*UA) / (c_clnt + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    qMax_conv = c_clnt*(batT_K - clntTinK);
    qConv = effectiveness*qMax_conv;

    qCond  = (conductionTuningFactor*hxAreaFactor/clntChannelD)*(batT_K - clntTinK);
    qTotal = qConv + qCond;

    p1 = 0.0075; p2 = -4.5401; p3 = 962.67;
    capacityAh = p1*batT_K^2 + p2*batT_K + p3;

    x = batT_K; y = batSoc;
    p00=0.5345; p10=-0.00348; p01=-0.02749; p20=5.748e-06; p11=7.589e-05; p02=0.02773; ...
    p21=9.909e-08; p12=-0.0001321; p03=0.008301;
    resistanceOhm_ploy = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + ...
                    p21*x^2*y + p12*x*y^2 + p03*y^3;
    r_floor   = 1e-8;
    eps_res   = 1e-18;
    resistanceOhm = (resistanceOhm_ploy + r_floor + ...
                     sqrt((resistanceOhm_ploy - r_floor)^2 + eps_res)) / 2;

    denominator_soc = capacityAh*4*socCapacityFactor;
    batSoc_dt = -batI/(denominator_soc + 1e-6);

    qJoule = jouleHeatingTuningFactor*(batI^2)*resistanceOhm;
    thermalMass = thermalMassTuningFactor*batMassKg*batCp;
    batT_dt = P(3)*(0.98*qJoule - qTotal)/(thermalMass + 1e-6);

    clntToutK = clntTinK + qTotal/(c_clnt + 1e-6);

    cmp.bat.output.batT_dt   = batT_dt;
    cmp.bat.output.batSoc_dt = batSoc_dt;
    cmp.bat.output.clntToutK = clntToutK;
    cmp.bat.output.clntMf    = clntMf_bat;
    cmp.bat.input.clntTinK   = clntTinK;
end

function cmp = dcdcMdl(mdlInput, estimatedParams, controlOut, cmp, P)
    clntChannelD = 0.0092;
    hxAreaFactor = 0.736;
    NTU_MAX = 10;

    dcdcMassKg = 1.0; dcdcCp = 447;
    convectionTuningFactor = 1;
    conductionTuningFactor = 0.245;
    thermalMassTuningFactor = 0.1;
    flowSplitRatio = 0.11;

    dcdcT_K     = mdlInput.state.dcdcT;
    clntTinK    = cmp.heater.output.clntToutK;
    clntCp      = estimatedParams.clntCp;
    clntTotalMf = cmp.batPump.output.clntMf;
    qGenerated  = estimatedParams.dcdcQmeas;
    clntMf      = flowSplitRatio*clntTotalMf;

    h_conv = 3.66*0.254/clntChannelD;
    UA = h_conv*hxAreaFactor*convectionTuningFactor;

    c_clnt = clntMf*clntCp;
    C0     = (1e-4)*clntCp;
    k_gate = 500;
    gate   = 1./(1 + exp(-k_gate*(c_clnt - C0)));

    % NTU cap
    NTU_raw = UA / (c_clnt + 1e-6) * gate;
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    qMax_conv = c_clnt*(dcdcT_K - clntTinK);
    qConv = effectiveness*qMax_conv;
    qCond = (conductionTuningFactor*hxAreaFactor/clntChannelD)*(dcdcT_K - clntTinK);
    qRemoved = qConv + qCond;

    thermalMass = thermalMassTuningFactor*dcdcMassKg*dcdcCp;
    dcdcT_dt = 0.05*(qGenerated - qRemoved)/(thermalMass + 1e-6);

    clntToutK = clntTinK + (qRemoved/(c_clnt + 1e-6))*gate;

    cmp.dcdc.output.dcdcT_dt  = dcdcT_dt;
    cmp.dcdc.output.clntToutK = clntToutK;
    cmp.dcdc.output.clntMf    = clntMf;
end

function cmp = invMdl(mdlInput, estimatedParams, controlOut, cmp, P)
    clntChannelD = 0.0092;
    hxAreaFactor = 0.736;
    NTU_MAX = 10;

    invMassKg = 1.0; invCp = 447;
    convectionTuningFactor = 6.0;
    conductionTuningFactor = 0.254;

    invT_K     = mdlInput.state.invT;
    clntCp     = estimatedParams.clntCp;
    clntMf     = cmp.motPump.output.clntMf;
    qGenerated = estimatedParams.invQmeas;
    clntTinK   = cmp.mot.output.clntToutK;

    h_conv = 3.66*estimatedParams.clntK/clntChannelD;
    UA = h_conv*hxAreaFactor*convectionTuningFactor;
    c_clnt = clntMf*clntCp;

    NTU_raw = (0.1*UA) / (c_clnt + 1e-6);
    NTU = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    qMax_conv = c_clnt*(invT_K - clntTinK);
    qConv = effectiveness*qMax_conv;
    qCond = (conductionTuningFactor*hxAreaFactor/clntChannelD)*(invT_K - clntTinK);
    qRemoved = qConv + qCond;

    thermalMass = invMassKg*invCp;
    invT_dt = P(4)*(qGenerated - qRemoved)/(thermalMass + 1e-6);

    clntToutK = clntTinK + qRemoved/(c_clnt + 1e-6);

    cmp.inv.output.invT_dt   = invT_dt;
    cmp.inv.output.clntToutK = clntToutK;
end

function cmp = pressureStateCalcMdl(mdlInput, estimatedParams, controlOut, cmp, P)
    q_evap = cmp.htx.output.qActualW + cmp.condEvap.output.qActualW;
    q_cond = cmp.innerCond.output.qActualW;

    refMf = cmp.compressor.output.refMf;

    h1 = cmp.compressor.output.h_in;  h2 = cmp.compressor.output.h_out;
    h3 = cmp.innerCond.output.h_out;  h4 = cmp.innerCond.output.h_out; % placeholder 

    ye = estimatedParams.evapVoidFrac;  yc = estimatedParams.condVoidFrac;

    Ve = estimatedParams.evapVol_m3;  Vc = estimatedParams.condVol_m3;

    Mwe_ce = estimatedParams.evapWallThermalMass_J_K;  Mwc_cc = estimatedParams.condWallThermalMass_J_K;

    dTesat_dp1  = estimatedParams.dTesat_dp1;   d_rhoh_l_dp1 = estimatedParams.d_rhoh_l_dp1;  d_rhoh_g_dp1 = estimatedParams.d_rhoh_g_dp1;
    dTcsat_dp2  = estimatedParams.dTcsat_dp2;   d_rhoh_l_dp2 = estimatedParams.d_rhoh_l_dp2;  d_rhoh_g_dp2 = estimatedParams.d_rhoh_g_dp2;

    f1 = (P(9)*q_evap + refMf*(h4 - h1));

    term1_D11 = (1 - ye)*d_rhoh_l_dp1;  term2_D11 = ye*d_rhoh_g_dp1;  term4_D11 = -1;  term5_D11 = (Mwe_ce/Ve)*dTesat_dp1;
    D11 = 1*Ve*(term1_D11 + term2_D11 + term4_D11 + term5_D11);

    f2 = -q_cond + P(6)*refMf*(h2 - h3);
    term1_D22 = (1 - yc)*d_rhoh_l_dp2;  term2_D22 = yc*d_rhoh_g_dp2;  term4_D22 = -1;  term5_D22 = (Mwc_cc/Vc)*dTcsat_dp2;
    D22 = 1*Vc*(term1_D22 + term2_D22 + term4_D22 + term5_D22);

    k_gate = 100;  sigmoid_gate_D11 = 1/(1 + exp(-k_gate*(abs(D11) - 1e-4)));  sigmoid_gate_D22 = 1/(1 + exp(-k_gate*(abs(D22) - 1e-4)));

    compPin_dt  = (P(11)*f1 /(500*D11 + 1e-6)) * sigmoid_gate_D11; % 500:
    compPout_dt = (P(10)*f2 /(1*D22  + 1e-6))  * sigmoid_gate_D22;

    cmp.pressure.output.compPin_dt  = compPin_dt;
    cmp.pressure.output.compPout_dt = compPout_dt;
end

function cmp = motMdl(mdlInput, estimatedParams, controlOut, cmp, P)
    clntChannelD = 0.0092; hxAreaFactor = 0.736;
    motMassKg = 46.0; motCp = 447;
    convectionTuningFactor = 6.0; conductionTuningFactor = 0.254;
    NTU_MAX = 10;

    motT_K     = mdlInput.state.motT;
    clntCp     = estimatedParams.clntCp;
    clntMf     = cmp.motPump.output.clntMf;
    qGenerated = estimatedParams.motQmeas;

    batClntToutK  = cmp.bat.output.clntToutK;  batClntMf  = cmp.bat.output.clntMf;
    dcdcClntToutK = cmp.dcdc.output.clntToutK; dcdcClntMf = cmp.dcdc.output.clntMf;

    eps_flow = 1e-12;
    totalBatLoopMf = batClntMf + dcdcClntMf;
    mixedBatLoopToutK = ((batClntToutK*batClntMf) + (dcdcClntToutK*dcdcClntMf)) ...
                        / (totalBatLoopMf + eps_flow);

    htxClntToutK      = cmp.htx.output.clntToutK;
    cmdParallelSerial = controlOut.cmdParallelSerial;
    clntTinK = (cmdParallelSerial*mixedBatLoopToutK) + ((1 - cmdParallelSerial)*htxClntToutK);

    h_conv = 3.66*estimatedParams.clntK/clntChannelD;
    UA = h_conv*hxAreaFactor*convectionTuningFactor;
    c_clnt = clntMf*clntCp;

    flow_threshold = 1e-3;
    k_smooth = 1000;
    gate = 0.5*(1 + tanh(k_smooth*(clntMf - flow_threshold)));

    NTU_raw = (UA/(c_clnt + 1e-6)) * gate;
    NTU     = NTU_MAX * tanh(NTU_raw / NTU_MAX);
    effectiveness = 1 - exp(-NTU);

    qMax_conv = c_clnt*(motT_K - clntTinK);
    qConv     = effectiveness*qMax_conv;
    qCond     = (conductionTuningFactor*hxAreaFactor/clntChannelD)*(motT_K - clntTinK);
    qRemoved  = qConv + qCond;

    thermalMass = motMassKg*motCp;
    motT_dt   = P(5)*(qGenerated - qRemoved)/(thermalMass + 1e-6);
    clntToutK = clntTinK + (qRemoved/(c_clnt + 1e-6))*gate;

    cmp.mot.output.motT_dt          = motT_dt;
    cmp.mot.output.clntToutK        = clntToutK;
    cmp.mot.input.clntTinK          = clntTinK;
    cmp.mot.input.mixedBatLoopToutK = mixedBatLoopToutK;
end

function cmp = pwrEstimation(mdlInput, estimatedParams, controlOut, cmp)
    h2 = cmp.compressor.output.h_out;
    h1 = cmp.compressor.output.h_in;
    mechanical_effi = 0.9;
    electric_effi   = 0.9;

    mf_compRef  = cmp.compressor.output.refMf;
    mf_blower   = mdlInput.u.cabAirMfIn;
    density_glycol = 1100;
    mf_bat      = cmp.batPump.output.clntMf;
    mf_clntMot  = cmp.motPump.output.clntMf;

    kinematic_visco = 5e-6;
    coolant_dia     = 0.0092;
    aggregate_equi  = 0.736;
    pipe_len        = 0.4;
    surf_rough      = 1.5e-5;
    friction_coeff  = (kinematic_visco*64/(2*coolant_dia^2*surf_rough))*((pipe_len+aggregate_equi)/2);

    p_com           = mf_compRef*(h2 - h1)/mechanical_effi;
    p_comElecDemand = p_com./electric_effi;

    C = 11.6064; blowerEff = 0.8;
    blowerPwr = C*mf_blower/blowerEff;

    pump_effi       = 0.88;
    delta_P_batpump = 2*friction_coeff*mf_clntMot^2 + 2*friction_coeff*mf_clntMot^2 + 2*friction_coeff*mf_clntMot^2;
    CoolantPump_HydraulicPower = (mf_clntMot*delta_P_batpump)/density_glycol;
    batpump_pwrEst  = CoolantPump_HydraulicPower/pump_effi;

    htNomiPresDrop = 2e-4*1e6;
    htNomiMf       = 0.15;
    k_heater       = htNomiPresDrop/htNomiMf^2;
    p_DropHeater   = k_heater*mf_bat^2;

    btNomiPresDrop = 1e-5*1e6;
    btNomiMf       = 0.02;
    k_bat          = btNomiPresDrop/btNomiMf^2;
    p_DropBat      = k_bat*mf_bat^2;

    p_DropAtMotPump          = p_DropHeater + p_DropBat;
    MotorPump_HydraulicPower = (mf_clntMot*p_DropAtMotPump)/density_glycol;
    motPump_pwrEst           = MotorPump_HydraulicPower/pump_effi;

    P_fan_mech_nom = 1701;
    eta_fan_motor  = 0.85;
    eta_fan_drive  = 0.95;
    eta_fan_total  = eta_fan_motor*eta_fan_drive;
    N_fan_ref      = 3000;
    radFanNrpm     = mdlInput.u.radFanrpm;
    N_ratio        = radFanNrpm/N_fan_ref;
    P_fan_mech     = P_fan_mech_nom*N_ratio^3;
    radFan_pwrEst  = P_fan_mech/eta_fan_total;

    cmp.pwrEst.out.pwrCompElec    = p_comElecDemand;
    cmp.pwrEst.out.pwrBlowElec    = blowerPwr;
    cmp.pwrEst.out.pwrPumpMotElec = motPump_pwrEst;
    cmp.pwrEst.out.pwrPumpBatElec = batpump_pwrEst;
    cmp.pwrEst.out.pwrTem         = p_comElecDemand + blowerPwr ...
                                  + motPump_pwrEst + batpump_pwrEst ...
                                  + mdlInput.u.heaterPwr + radFan_pwrEst;
end
