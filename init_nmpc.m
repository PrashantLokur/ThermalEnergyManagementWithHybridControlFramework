function init_nmpc()

repoRoot = fileparts(mfilename('fullpath'));
addpath(repoRoot);
addpath(fullfile(repoRoot, 'Data'));
addpath(fullfile(repoRoot, 'casadi-3.7.1-windows64-matlab2018b'));

if ~exist('casadi.SX', 'class')
    error('NMPC:missingDependency', ...
          ['CasADi is required but not found on the MATLAB path.\n', ...
           'Download it from: https://web.casadi.org/get/\n', ...
           'Then add it using: addpath(''<your_casadi_folder> in the init_nmpc file'')']);
end

try
    val = py.CoolProp.CoolProp.PropsSI('C', 'T', 273.15, 'P', 101325, 'Air');
    if ~isfinite(val) || val <= 0
        error('CoolProp returned an invalid value.');
    end
    fprintf('CoolProp is installed and working (cp_air = %.2f J/kg/K)\n', val);
catch ME
    error('NMPC:missingDependency', ...
          ['CoolProp Python wrapper not found or not working.\n', ...
           'Ensure CoolProp is installed in your Python environment:\n', ...
           '    pip install CoolProp\n', ...
           'Also verify MATLAB is linked to the correct Python:\n', ...
           '    pyenv\n\n', ...
           'Original error: %s'], ME.message);
end
% 2) Load tuning/resampled data used by MPC (if you still rely on it)
S = load('tuning_data_10.mat','resampled');
assignin('base','resampled', S.resampled);

Stuned = load('tunedVariables_minus5_minus10.mat');
assignin('base','Stuned', Stuned);

N  = 30;     % prediction horizon [steps]
Ts = 1;      % MPC sample time [s]
nx = 9;      % number of states
nu = 6;      % number of control inputs

MPC_INPUT_LEN = nx + 6*N + 3;

N_MAX = N;   % 
assignin('base', 'N_MAX', N_MAX);

ControlSampleTime = Ts; 


if N > N_MAX
    error('init_nmpc:HorizonTooLarge', ...
          'N=%d exceeds N_MAX=%d. Increase N_MAX in init_nmpc.m', N, N_MAX);
end


assignin('base', 'N',             N);
assignin('base', 'Ts',            Ts);
assignin('base', 'nx',            nx);
assignin('base', 'nu',            nu);
assignin('base', 'MPC_INPUT_LEN', MPC_INPUT_LEN);


fprintf('[init_nmpc] ✓ MPC parameters set: N=%d, Ts=%.1fs, nx=%d, nu=%d\n', ...
        N, Ts, nx, nu);
fprintf('[init_nmpc]   Input vector length: MPC_INPUT_LEN = %d\n', MPC_INPUT_LEN);


V = load('ElectricVehicleThermalManagementWithHeatPumpVehicleSpeed.mat');
DC = V.DriveCycle;
speed_ts = DC{1};          
t_grid = speed_ts.Time(:);
v_grid = speed_ts.Data(:);
a_grid = gradient(v_grid, t_grid);

assignin('base','t_grid', t_grid);
assignin('base','v_grid', v_grid);
assignin('base','a_grid', a_grid);

disp('[init_nmpc] Loaded CasADi + data + speed profile.');
disp('*** init_nmpc ran ***');

%% ── 5. Summary ───────────────────────────────────────────────────────────
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║           init_nmpc — Initialization Summary    ║\n');
fprintf('╠══════════════════════════════════════════════════╣\n');
fprintf('║  Horizon  N  : %d steps                          \n', N);
fprintf('║  Sample time : %.1f s                            \n', Ts);
fprintf('║  States   nx : %d                                \n', nx);
fprintf('║  Inputs   nu : %d                                \n', nu);
fprintf('║  Input vec   : %d elements                       \n', MPC_INPUT_LEN);
fprintf('╚══════════════════════════════════════════════════╝\n');

% Confirm key workspace variables are present
whos('N', 'Ts', 'nx', 'nu', 'MPC_INPUT_LEN', 'Stuned', 'resampled');
end