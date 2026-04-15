function [estimatedParams] = estimateParameters(state, input, heatMeasured)


    % === Extract Inputs and Define Constants (with standardized names) ===
    compPinPa  = state(6);
    compPoutPa = state(7);
    cabAirTinC = state(9);
    
 
    
    fluidRef = 'R134a';
    fluidAir = 'Air';
    pAtmPa   = 101325; % Atmospheric pressure (Pa)

    estimatedParams.superheatK = heatMeasured.superHeat;

    nominalPinBar  = 3;      % [bar]
    nominalTinC    = 11;     % [C]
    nominalPratio  = 4.6;    % [-]
    
    nominalPinPa  = nominalPinBar * 1e5;
    nominalTinK   = nominalTinC + 273.15;
    
    rhoNominalIn = py.CoolProp.CoolProp.PropsSI('D', 'P', nominalPinPa, 'T', nominalTinK, fluidRef);
    
    estimatedParams.compVnominalIn = 1 / rhoNominalIn; % [m^3/kg]

    % === Refrigerant Properties at Compressor Inlet/Outlet ===
    estimatedParams.refTsatLowPK  = py.CoolProp.CoolProp.PropsSI('T', 'P', compPinPa, 'Q', 1, fluidRef);
    estimatedParams.compTinK      = estimatedParams.refTsatLowPK + abs(estimatedParams.superheatK);

    if abs(estimatedParams.compTinK - estimatedParams.refTsatLowPK) < 0.5
        h1 = py.CoolProp.CoolProp.PropsSI('H', 'P', compPinPa, 'Q', 1, fluidRef);
    else
        h1 = py.CoolProp.CoolProp.PropsSI('H', 'P', compPinPa, 'T', estimatedParams.compTinK, fluidRef);
    end
    s1 = py.CoolProp.CoolProp.PropsSI('S', 'P', compPinPa, 'H', h1, fluidRef);
    
    try
        h2s = py.CoolProp.CoolProp.PropsSI('H', 'P', compPoutPa, 'S', s1, fluidRef);
        estimatedParams.refTsatHighPK = py.CoolProp.CoolProp.PropsSI('T', 'P', compPoutPa, 'Q', 1, fluidRef);
        estimatedParams.refCpVapHighP = py.CoolProp.CoolProp.PropsSI('C', 'P', compPoutPa, 'Q', 1, fluidRef);
    catch
        h2s = py.CoolProp.CoolProp.PropsSI('H', 'P', compPoutPa + 100, 'S', s1, fluidRef); 
        estimatedParams.refTsatHighPK = py.CoolProp.CoolProp.PropsSI('T', 'P', compPoutPa + 100, 'Q', 1, fluidRef);
        estimatedParams.refCpVapHighP = py.CoolProp.CoolProp.PropsSI('C', 'P', compPoutPa + 100, 'Q', 1, fluidRef);
    end

    estimatedParams.compHin   = h1;
    estimatedParams.compHoutS = h2s; % Isentropic outlet enthalpy
    estimatedParams.compSin   = s1;
    
    % === Inner Condenser Properties ===
    innerCondAirTinK = cabAirTinC;
    estimatedParams.cabAirCp      = py.CoolProp.CoolProp.PropsSI('C', 'T', innerCondAirTinK, 'P', pAtmPa, fluidAir);
    estimatedParams.cabAirK       = py.CoolProp.CoolProp.PropsSI('L', 'T', innerCondAirTinK, 'P', pAtmPa, fluidAir);
    estimatedParams.cabAirMu      = py.CoolProp.CoolProp.PropsSI('V', 'T', innerCondAirTinK, 'P', pAtmPa, fluidAir);
    estimatedParams.cabAirRho     = py.CoolProp.CoolProp.PropsSI('D', 'T', innerCondAirTinK, 'P', pAtmPa, fluidAir);
    
    estimatedParams.innerCondRefRho = py.CoolProp.CoolProp.PropsSI('D', 'P', compPoutPa, 'Q', 1, fluidRef);
    estimatedParams.innerCondRefK   = py.CoolProp.CoolProp.PropsSI('L', 'P', compPoutPa, 'Q', 1, fluidRef);
    estimatedParams.innerCondRefMu  = py.CoolProp.CoolProp.PropsSI('V', 'P', compPoutPa, 'Q', 1, fluidRef);
    estimatedParams.innerCondRefCp  = py.CoolProp.CoolProp.PropsSI('C', 'P', compPoutPa, 'Q', 1, fluidRef);
    estimatedParams.innerCondRefHliq = py.CoolProp.CoolProp.PropsSI('H', 'P', compPoutPa, 'Q', 0, fluidRef);
    estimatedParams.innerCondRefHvap = py.CoolProp.CoolProp.PropsSI('H', 'P', compPoutPa, 'Q', 1, fluidRef);


    estimatedParams.refRhoVapHighP = py.CoolProp.CoolProp.PropsSI('D', 'P', compPoutPa, 'Q', 1, fluidRef); % Vapor density
    estimatedParams.refRhoLiqHighP = py.CoolProp.CoolProp.PropsSI('D', 'P', compPoutPa, 'Q', 0, fluidRef); % Liquid density


    estParams.evapSideRefRho = py.CoolProp.CoolProp.PropsSI('D', 'P', compPinPa, 'Q', 0, fluidRef);
    estParams.evapSideRefK   = py.CoolProp.CoolProp.PropsSI('L', 'P', compPinPa, 'Q', 0, fluidRef);
    estParams.evapSideRefMu  = py.CoolProp.CoolProp.PropsSI('V', 'P', compPinPa, 'Q', 0, fluidRef);
    estParams.evapSideRefCp  = py.CoolProp.CoolProp.PropsSI('C', 'P', compPinPa, 'Q', 0, fluidRef);
    
    % Assign consolidated properties to specific components
    estimatedParams.condEvapRefRho = estParams.evapSideRefRho;
    estimatedParams.condEvapRefK   = estParams.evapSideRefK;
    estimatedParams.condEvapRefMu  = estParams.evapSideRefMu;
    estimatedParams.condEvapRefCp  = estParams.evapSideRefCp;
    
    estimatedParams.evapRefRho = estParams.evapSideRefRho;
    estimatedParams.evapRefK   = estParams.evapSideRefK;
    estimatedParams.evapRefMu  = estParams.evapSideRefMu;
    estimatedParams.evapRefCp  = estParams.evapSideRefCp;
    
    estimatedParams.chilRefRho = estParams.evapSideRefRho;
    estimatedParams.chilRefK   = estParams.evapSideRefK;
    estimatedParams.chilRefMu  = estParams.evapSideRefMu;
    estimatedParams.chilRefCp  = estParams.evapSideRefCp;

    % Ambient Air Properties (assuming fixed temperature, can be made dynamic later)
    envAirT = 263.15; % -10 C in Kelvin
    estimatedParams.condEvapAirRho = py.CoolProp.CoolProp.PropsSI('D', 'T', envAirT, 'P', pAtmPa, fluidAir);
    estimatedParams.condEvapAirK   = py.CoolProp.CoolProp.PropsSI('L', 'T', envAirT, 'P', pAtmPa, fluidAir);
    estimatedParams.condEvapAirMu  = py.CoolProp.CoolProp.PropsSI('V', 'T', envAirT, 'P', pAtmPa, fluidAir);
    estimatedParams.condEvapAirCp  = py.CoolProp.CoolProp.PropsSI('C', 'T', envAirT, 'P', pAtmPa, fluidAir);

    % === General Thermodynamic Properties ===
    estimatedParams.refTsatLiqHighPK = py.CoolProp.CoolProp.PropsSI('T', 'P', compPoutPa, 'Q', 0, fluidRef);
    estimatedParams.refHsatLiqHighP  = py.CoolProp.CoolProp.PropsSI('H', 'P', compPoutPa, 'Q', 0, fluidRef);
    estimatedParams.refCpLiqHighP    = py.CoolProp.CoolProp.PropsSI('C', 'P', compPoutPa, 'Q', 0, fluidRef);

    estimatedParams.refHsatVapLowP   = py.CoolProp.CoolProp.PropsSI('H', 'P', compPinPa, 'Q', 1, fluidRef);
    estimatedParams.refTsatLiqLowPK  = py.CoolProp.CoolProp.PropsSI('T', 'P', compPinPa, 'Q', 0, fluidRef);
    estimatedParams.refCpLiqLowP     = py.CoolProp.CoolProp.PropsSI('C', 'P', compPinPa, 'Q', 0, fluidRef);

    % === Coolant Properties (fixed values) ===
    estimatedParams.clntRho = 1113;    % kg/m^3
    estimatedParams.clntK   = 0.254;   % W/m.K
    estimatedParams.clntMu  = 16.1e-3; % Pa.s
    estimatedParams.clntCp  = 2430;    % J/kg.K
    
    % === Pass Through Inputs Needed by mdlDynamics ===
    % estimatedParams.condEvapAirMf = heatMeasured.condEvapAirMf;
    estimatedParams.speed         = heatMeasured.speed;
    estimatedParams.htxClntTinK   = input.clntInvTout_K;
    estimatedParams.envT          = input.envT;
    estimatedParams.dcdcQmeas     = heatMeasured.dcdcQ;
    estimatedParams.motQmeas      = heatMeasured.motQ;
    estimatedParams.invQmeas      = heatMeasured.invQ;



    evapVol_m3 = 0.001;  % Evaporator internal volume [m^3]
    condVol_m3 = 0.001;  % Condenser internal volume [m^3]
    evapWallThermalMass_J_K = 5000; % Mwe*ce for evaporator walls [J/K]
    condWallThermalMass_J_K = 5000; % Mwc*cc for condenser walls [J/K]
    

    estimatedParams.evapVoidFrac = 0.8; % Average vapor void fraction in evaporator
    estimatedParams.condVoidFrac = 0.5; % Average vapor void fraction in condenser


    dp = 100; % Pressure step in Pascals for finite difference calculation
    
    % --- Derivatives at Low Pressure (p1) ---
    p1_plus = compPinPa + dp;
    p1_minus = compPinPa - dp;
    
    % dT/dp at p1
    Tsat_plus = py.CoolProp.CoolProp.PropsSI('T', 'P', p1_plus, 'Q', 1, fluidRef);
    Tsat_minus = py.CoolProp.CoolProp.PropsSI('T', 'P', p1_minus, 'Q', 1, fluidRef);
    estimatedParams.dTesat_dp1 = (Tsat_plus - Tsat_minus) / (2 * dp);

    % d(rho*h)/dp for liquid at p1
    rho_l_p = py.CoolProp.CoolProp.PropsSI('D', 'P', p1_plus, 'Q', 0, fluidRef);
    h_l_p = py.CoolProp.CoolProp.PropsSI('H', 'P', p1_plus, 'Q', 0, fluidRef);
    rho_l_m = py.CoolProp.CoolProp.PropsSI('D', 'P', p1_minus, 'Q', 0, fluidRef);
    h_l_m = py.CoolProp.CoolProp.PropsSI('H', 'P', p1_minus, 'Q', 0, fluidRef);
    estimatedParams.d_rhoh_l_dp1 = (rho_l_p * h_l_p - rho_l_m * h_l_m) / (2 * dp);

    % d(rho*h)/dp for vapor at p1
    rho_g_p = py.CoolProp.CoolProp.PropsSI('D', 'P', p1_plus, 'Q', 1, fluidRef);
    h_g_p = py.CoolProp.CoolProp.PropsSI('H', 'P', p1_plus, 'Q', 1, fluidRef);
    rho_g_m = py.CoolProp.CoolProp.PropsSI('D', 'P', p1_minus, 'Q', 1, fluidRef);
    h_g_m = py.CoolProp.CoolProp.PropsSI('H', 'P', p1_minus, 'Q', 1, fluidRef);
    estimatedParams.d_rhoh_g_dp1 = (rho_g_p * h_g_p - rho_g_m * h_g_m) / (2 * dp);
    
    % --- Derivatives at High Pressure (p2) ---
    p2_plus = compPoutPa + dp;
    p2_minus = compPoutPa - dp;

    % dT/dp at p2
    Tsat_plus = py.CoolProp.CoolProp.PropsSI('T', 'P', p2_plus, 'Q', 1, fluidRef);
    Tsat_minus = py.CoolProp.CoolProp.PropsSI('T', 'P', p2_minus, 'Q', 1, fluidRef);
    estimatedParams.dTcsat_dp2 = (Tsat_plus - Tsat_minus) / (2 * dp);

    % d(rho*h)/dp for liquid at p2
    rho_l_p = py.CoolProp.CoolProp.PropsSI('D', 'P', p2_plus, 'Q', 0, fluidRef);
    h_l_p = py.CoolProp.CoolProp.PropsSI('H', 'P', p2_plus, 'Q', 0, fluidRef);
    rho_l_m = py.CoolProp.CoolProp.PropsSI('D', 'P', p2_minus, 'Q', 0, fluidRef);
    h_l_m = py.CoolProp.CoolProp.PropsSI('H', 'P', p2_minus, 'Q', 0, fluidRef);
    estimatedParams.d_rhoh_l_dp2 = (rho_l_p * h_l_p - rho_l_m * h_l_m) / (2 * dp);

    % d(rho*h)/dp for vapor at p2
    rho_g_p = py.CoolProp.CoolProp.PropsSI('D', 'P', p2_plus, 'Q', 1, fluidRef);
    h_g_p = py.CoolProp.CoolProp.PropsSI('H', 'P', p2_plus, 'Q', 1, fluidRef);
    rho_g_m = py.CoolProp.CoolProp.PropsSI('D', 'P', p2_minus, 'Q', 1, fluidRef);
    h_g_m = py.CoolProp.CoolProp.PropsSI('H', 'P', p2_minus, 'Q', 1, fluidRef);
    estimatedParams.d_rhoh_g_dp2 = (rho_g_p * h_g_p - rho_g_m * h_g_m) / (2 * dp);
    
    % Store other needed parameters
    estimatedParams.evapVol_m3 = evapVol_m3;
    estimatedParams.condVol_m3 = condVol_m3;
    estimatedParams.evapWallThermalMass_J_K = evapWallThermalMass_J_K;
    estimatedParams.condWallThermalMass_J_K = condWallThermalMass_J_K;
    estimatedParams.batImeas = heatMeasured.batImeas;

end