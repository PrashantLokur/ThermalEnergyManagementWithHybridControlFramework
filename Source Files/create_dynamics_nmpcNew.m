function [F, U_ordering, packU, packParams] = create_dynamics_nmpcNew()


% import casadi.*

% ---------- States ----------
x = casadi.SX.sym('x', 9, 1);


U_ordering.inputSignal = { ...
    'compNrpm', ...      
    'cabAirMfIn', ...    
    'motPumpNrpm', ...    
    'batPumpNrpm',...
    'heaterPwr',...
    'radFanrpm'
};

% ---------- Feed-forward controls (parameters, not decisions) ----------
U_ordering.controlOut = { ...
    'cmdHeater', ...
    'cmdParallelSerial', ...
    'cmdBlendAir', ...
    'cmdExvEvap', ...
    'cmdExvChiller', ...
    'thermalMode', ...
    'cmdExvHeatpump'
};

% ---------- Estimated parameters (parameters, not decisions) ----------
U_ordering.estimatedParams = { ...
    'envT', ...
    'clntCp','clntK','clntRho','clntMu', ...
    'cabAirCp','cabAirK','cabAirRho','cabAirMu', ...
    'innerCondRefCp','innerCondRefK','innerCondRefRho','innerCondRefMu', ...
    'condEvapRefCp','condEvapRefK','condEvapRefRho','condEvapRefMu', ...
    'condEvapAirCp','condEvapAirK','condEvapAirRho','condEvapAirMu', ...
    'evapRefCp','evapRefK','evapRefRho','evapRefMu', ...
    'chilRefCp','chilRefK','chilRefRho','chilRefMu', ...
    'speed', ...
    'compHin','compHoutS','compVnominalIn', ...
    'refTsatHighPK','refCpVapHighP', ...
    'refHsatLiqHighP','refTsatLiqHighPK','refTsatLiqLowPK', ...
    'motQmeas','invQmeas','dcdcQmeas', ...
    'evapVoidFrac','condVoidFrac', ...
    'evapVol_m3','condVol_m3', ...
    'evapWallThermalMass_J_K','condWallThermalMass_J_K', ...
    'dTesat_dp1','d_rhoh_l_dp1','d_rhoh_g_dp1', ...
    'dTcsat_dp2','d_rhoh_l_dp2','d_rhoh_g_dp2', ...
    'batImeas','refCpLiqHighP', ...
    'fanRpm_prev'
};

% ---------- Tuning vector (p1..p11) as parameters ----------
U_ordering.tunePara = {'p1','p2','p3','p4','p5','p6','p7','p8','p9','p10','p11'};

% Sizes
nu = numel(U_ordering.inputSignal);
np_est  = numel(U_ordering.estimatedParams);
np_ctrl = numel(U_ordering.controlOut);
np_tune = numel(U_ordering.tunePara);
np = np_est + np_ctrl + np_tune;

% Symbols
Udec = casadi.SX.sym('U', nu, 1);      
p    = casadi.SX.sym('P',  np, 1);     

% ---------- Unpack helpers (symbolic) ----------
[est_sym, ctrl_sym, tune_sym] = unpackPsymbolic(p, U_ordering);


u_struct = struct();
for k = 1:numel(U_ordering.inputSignal)
    nm = U_ordering.inputSignal{k};
    u_struct.(nm) = Udec(k);
end
p_struct = struct('est', est_sym, 'ctrl', ctrl_sym, 'tuneParam', tune_sym);

% ---------- Call model ----------

[xdot_sym, y_sym,pwrEst] = dynamics_mexfreeNew(x, u_struct, p_struct);

% ---------- CasADi function ----------
F = casadi.Function('F', {x, Udec, p}, {xdot_sym, y_sym,pwrEst});

% ---------- Numeric packers ----------
packU = @(inputs) local_packU(inputs, U_ordering);
packParams = @(est,ctrl,tune) local_packParams(est, ctrl, tune, U_ordering);

end

% ================== helpers ==================
function [est, ctrl, tune] = unpackPsymbolic(p, U_ordering)

np_est  = numel(U_ordering.estimatedParams);
np_ctrl = numel(U_ordering.controlOut);
% est
est = struct();
ofs = 1;
for k = 1:np_est
    est.(U_ordering.estimatedParams{k}) = p(ofs); ofs = ofs+1;
end
% ctrl
ctrl = struct();
for k = 1:np_ctrl
    ctrl.(U_ordering.controlOut{k}) = p(ofs); ofs = ofs+1;
end
% tune
tune = casadi.SX.sym('tune', numel(U_ordering.tunePara), 1);
for k = 1:numel(U_ordering.tunePara)
    tune(k) = p(ofs); ofs = ofs+1;
end
end

function Uvec = local_packU(inputs, U_ordering)
lst = cell(numel(U_ordering.inputSignal),1);
for k = 1:numel(U_ordering.inputSignal)
    lst{k} = inputs.(U_ordering.inputSignal{k});
end
Uvec = vertcat(lst{:});
end

function pvec = local_packParams(est, ctrl, tune, U_ordering)
lst = {};
for k = 1:numel(U_ordering.estimatedParams)
    lst{end+1,1} = est.(U_ordering.estimatedParams{k});
end
for k = 1:numel(U_ordering.controlOut)
    lst{end+1,1} = ctrl.(U_ordering.controlOut{k});
end
for k = 1:numel(U_ordering.tunePara)
    lst{end+1,1} = tune.(U_ordering.tunePara{k});
end
pvec = vertcat(lst{:});
end
