function [out, info] = MPCController_step_coreLQRNew( ...
    x_meas, ...
    Tamb_C, ...
    Tref_K, ...
    clntInvTout_K_meas, ...
    clntBatTin_K_meas, ...
    dcdcQ, motQ, invQ, ...
    speed, ...
    batImeas)

%% ── Persistent objects ───────────────────────────────────────────────────
persistent built ...
    solver F Uord packParams idx_cmdPS ...
    dt M h N nx nu np ...
    umin umax xmin xmax xpref_lo xpref_hi sx ...
    Sx SxInv Su SuInv ...
    w_cost w_all p_all g lbg ubg lbw ubw ...
    n_w nX nU idxSy idxSx_lo idxSx_hi idxSdu ...
    Xsym Usym Sysym Sx_losym Sx_hisym Sdu_sym ...
    prev_w_opt prev_lam_x prev_lam_g prev_Pk ...
    inputInit Stuned resampled ...
    ns_soft idx_soft_state fanRpm_actual_tracker ...
    Tref_default Gfun u_last ...
    cache_file first_step_saved infeas_count...
    Q_s q_cab w_pwr pwrEstMax R_s Rd_s ...
    w_ySlack w_du_slack w_stateSlack_mot w_stateSlack_inv ...
    w_stateSlack_dcdc w_stateSlack_bat w_stateSlack_cab ...
    du_max_s lambda_du int_offset int_e_prev ...
    K_lqr_last xT_lqr_last uT_lqr_last
%% ── Build once ───────────────────────────────────────────────────────────
if isempty(built)

    resampled = evalin('base', 'resampled');
    Stuned    = evalin('base', 'Stuned');

    [F, Uord, ~, packParams] = create_dynamics_nmpcNew();
    idx_cmdPS = get_idx_cmdPS(Uord);

    N  = double(evalin('base', 'N'));
    dt = double(evalin('base', 'Ts'));
    M  = 1;      % single RK4 step — sufficient at Ts=5s for all tau>5s
    h  = dt / M;
    nx = 9;
    nu = numel(Uord.inputSignal);

    fprintf('[coreLQR] Building NLP: N=%d, dt=%.1fs, M=%d, nx=%d, nu=%d\n', N, dt, M, nx, nu);

    %% ── Input bounds ─────────────────────────────────────────────────────
    umin = [  10;  0.00;  300;  300;    0;    0];
    umax = [6000;  0.15; 1000; 1000; 4000; 3000];

    %% ── State bounds ─────────────────────────────────────────────────────
    Tmin_phys     = -60 + 273.15;   Tmax_phys     = 140 + 273.15;
    Tbmin_phys    = -40 + 273.15;   Tbmax_phys    =  70 + 273.15;
    Tmin_cab_phys = -50 + 273.15;   Tmax_cab_phys =  60 + 273.15;
    Pmin_in_phys  =  0.5e5;         Pmax_in_phys  = 20.0e5;
    Pmin_out_phys =  0.5e5;         Pmax_out_phys = 50.0e5;

    xmin = [Tmin_phys; Tmin_phys; Tmin_phys; 0.0; Tbmin_phys; ...
            Pmin_in_phys; Pmin_out_phys; Tmin_cab_phys; Tmin_cab_phys];
    xmax = [Tmax_phys; Tmax_phys; Tmax_phys; 1.0; Tbmax_phys; ...
            Pmax_in_phys; Pmax_out_phys; Tmax_cab_phys; Tmax_cab_phys];

    %% ── Soft preferred bounds ────────────────────────────────────────────
    xpref_lo = [10 + 273.15;   % motT
                10 + 273.15;   % invT
                10 + 273.15;   % dcdcT
                0.05;          % SOC
                15 + 273.15;   % batT
                1.0e5;         % p_in  [Pa]  (not used — idx_soft_state excludes 6,7)
                8.0e5;         % p_out [Pa]  (not used)
                15 + 273.15;   % cabIntT
                15 + 273.15];  % cabAirT

    xpref_hi = [75 + 273.15;   % motT
                75 + 273.15;   % invT
                75 + 273.15;   % dcdcT
                1.0;           % SOC
                40 + 273.15;   % batT
                3.8e5;         % p_in
                30.0e5;        % p_out
                40 + 273.15;   % cabIntT
                35 + 273.15];  % cabAirT

    %% ── Scaling ──────────────────────────────────────────────────────────
    sx = max(abs(xmin), abs(xmax));
    sx(6) = Pmax_in_phys;
    sx(7) = Pmax_out_phys;

    su = [max(umax(1), max(resampled.compNrpm));
          max(umax(2), max(resampled.cabAirMfIn));
          max(umax(3), max(resampled.motPumpNrpm));
          max(umax(4), max(resampled.motPumpNrpm));
          umax(5);
          umax(6)];

    Sx    = diag(sx);
    SxInv = diag(1./sx);
    Su    = diag(su);
    SuInv = diag(1./su);

    %% ── WEIGHTS — all O(1), directly on scaled [0,1] variables ──────────
    
    q_cab = 10.0;
    Q_s   = zeros(nx);
    Q_s(9,9) = q_cab;

    pwrEstMax = 11.0e3;   % [W]

    % ── Power cost ───────────────────────────────────────────────────────
    w_pwr = 1e-4;

    % ── Input magnitude (Usym in [0,1]) ──────────────────────────────────
    % Very small — energy cost already handled by w_pwr.
    % Just prevents unbounded optimum at exactly zero error.
    R_s = diag([1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-3]);

    % ── Move penalties (dUsym in [0,1]) ──────────────────────────────────
  
    Rd_s = diag([0.090, ...   % compNrpm:    crossover ~2.5 K cabin error
                 3e-2,  ...   % cabAirMfIn:  crossover ~0.5 K
                 1e-4,  ...   % motPumpNrpm: light damping
                 1e-4,  ...   % batPumpNrpm: light damping
                 2e-4,  ...   % heaterPwr:   crossover ~1 K
                 1e-4]);      % radFanrpm:   lag filter in dynamics is dominant

    lambda_du = linspace(1.2, 1.0, N);
    K_lqr_last  = [];
    xT_lqr_last = [];
    uT_lqr_last = [];
   
    w_stateSlack_mot  = 0.1 / sx(1)^2;   % ~5.8e-7
    w_stateSlack_inv  = 0.1 / sx(2)^2;   % ~5.8e-7
    w_stateSlack_dcdc = 0.1 / sx(3)^2;   % ~5.8e-7
    w_stateSlack_bat  = 0.1 / sx(5)^2;   % ~1.1e-6
    w_stateSlack_cab  = 0.5 / sx(9)^2;   % ~4.5e-6


    w_ySlack = 1.0 / (15.0^2);   % = 0.00444

    % ── Ramp-rate slack (Sdu_sym, SCALED [0,1] units) ────────────────────
    % Full compressor step violation (scaled): (5000-2500)/su(1) ~ 0.417
    % Need: w_du_slack * 0.417^2 > cold-start tracking / 2 = 0.22
    % => w_du_slack > 1.26.  Use 5.0 for strong enforcement.
    w_du_slack = 5.0;

    % ── Rate limits ───────────────────────────────────────────────────────
    du_max_per_sec = [500;    ... % compNrpm    [rpm/s]
                      0.05;   ... % cabAirMfIn  [kg/s/s]
                      500;    ... % motPumpNrpm [rpm/s]
                      500;    ... % batPumpNrpm [rpm/s]
                      4000;   ... % heaterPwr   [W/s]
                      600];       % radFanrpm   [rpm/s]
    du_max_phys = du_max_per_sec * dt;
    du_max_s    = du_max_phys ./ su;

    %% ── CasADi decision variables ────────────────────────────────────────
    Xsym    = casadi.SX.sym('X',    nx, N+1);
    Usym    = casadi.SX.sym('U',    nu, N);
    Sysym   = casadi.SX.sym('S_y',  4,  N);
    Sdu_sym = casadi.SX.sym('S_du', nu, N);

    idx_soft_state = [1; 2; 3; 5; 8; 9];   % pressure excluded
    ns_soft        = numel(idx_soft_state);
    Sx_losym = casadi.SX.sym('Sx_lo', ns_soft, N+1);
    Sx_hisym = casadi.SX.sym('Sx_hi', ns_soft, N+1);

    %% ── Parameters ───────────────────────────────────────────────────────
    p_x0          = casadi.SX.sym('p_x0',         nx,    1);
    p_Tref        = casadi.SX.sym('p_Tref',        1,     N+1);
    p_xT          = casadi.SX.sym('p_xT',          nx,    1);
    p_uT          = casadi.SX.sym('p_uT',          nu,    1);
    p_P           = casadi.SX.sym('p_P',           nx*nx, 1);
    p_u_last_phys = casadi.SX.sym('p_u_last_phys', nu,    1);

    %% ── Dummy packParams call to get np ──────────────────────────────────
    x0_dummy = [20+273.15; 20+273.15; 20+273.15; 0.8; 20+273.15; ...
                2e5; 10e5; 20+273.15; 20+273.15];
    inputInit0.envT          = Tamb_C + 273.15;
    inputInit0.clntInvTout_K = clntInvTout_K_meas;
    inputInit0.clntBatTin_K  = clntBatTin_K_meas;
    inputInit0.compN         = 3000;
    inputInit0.pumpN         = 100;
    inputInit0.prevMode      = 0;
    inputInit0.heaterPwr     = 1000;
    heat0.dcdcQ    = 0; heat0.motQ = 0; heat0.invQ = 0;
    heat0.speed    = 0.05; heat0.batImeas = 0; heat0.superHeat = 2;
    tuneS0        = prepParaVec(x0_dummy, Tamb_C, Stuned.tunedVariables);
    [ctrl0, est0] = thermalLogicAndEstimation(x0_dummy, inputInit0, heat0);
    est0.speed = 0; est0.fanRpm_prev = 1000;
    p_sample = packParams(est0, ctrl0, tuneS0);
    np       = numel(p_sample);

    Pstage = casadi.SX.sym('Pstage', np, N);
    p_all  = vertcat(p_x0, p_Tref(:), Pstage(:), p_xT, p_uT, p_P, p_u_last_phys);

    %% ── NLP objective and constraints ────────────────────────────────────
    w_cost = casadi.SX(0);
    g      = {};
    lbg    = [];
    ubg    = [];

    % Reference for cabin tracking (scaled)
    Xref_s      = casadi.SX.zeros(nx, N+1);
    Xref_s(9,:) = p_Tref / Sx(9,9);

    %% ── HARD initial condition ───────────────────────────────────────────
    g{end+1} = Xsym(:,1) - (SxInv * p_x0);
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];

    %% ── Global slack penalties ───────────────────────────────────────────
    w_cost = w_cost + w_ySlack   * sumsqr(Sysym(:));
    w_cost = w_cost + w_du_slack * sumsqr(Sdu_sym(:));

    i_mot  = find(idx_soft_state == 1);
    i_inv  = find(idx_soft_state == 2);
    i_dcdc = find(idx_soft_state == 3);
    i_bat  = find(idx_soft_state == 5);
    i_cab  = find(idx_soft_state == 8 | idx_soft_state == 9);

    if ~isempty(i_mot)
        w_cost = w_cost + w_stateSlack_mot * ...
            (sumsqr(Sx_losym(i_mot,:)) + sumsqr(Sx_hisym(i_mot,:)));
    end
    if ~isempty(i_inv)
        w_cost = w_cost + w_stateSlack_inv * ...
            (sumsqr(Sx_losym(i_inv,:)) + sumsqr(Sx_hisym(i_inv,:)));
    end
    if ~isempty(i_dcdc)
        w_cost = w_cost + w_stateSlack_dcdc * ...
            (sumsqr(Sx_losym(i_dcdc,:)) + sumsqr(Sx_hisym(i_dcdc,:)));
    end
    if ~isempty(i_bat)
        w_cost = w_cost + w_stateSlack_bat * ...
            (sumsqr(Sx_losym(i_bat,:)) + sumsqr(Sx_hisym(i_bat,:)));
    end
    if ~isempty(i_cab)
        w_cost = w_cost + w_stateSlack_cab * ...
            (sumsqr(Sx_losym(i_cab,:)) + sumsqr(Sx_hisym(i_cab,:)));
    end
    % Initialise integrator states on first call

    if isempty(int_offset),  int_offset  = 0; end
    if isempty(int_e_prev),  int_e_prev  = 0; end

    clntLow  = -51 + 273.15;
    clntHigh = 121 + 273.15;

    %% ── Stage loop ───────────────────────────────────────────────────────
    for k = 1:N
        xk_phys = Sx * Xsym(:,k);
        pk      = Pstage(:,k);
        Uk_phys = Su * Usym(:,k);

        xtmp = xk_phys;
        for j = 1:M
            [f1,~,~]        = F(xtmp,           Uk_phys, pk);
            [f2,~,~]        = F(xtmp+0.5*h*f1,  Uk_phys, pk);
            [f3,~,~]        = F(xtmp+0.5*h*f2,  Uk_phys, pk);
            [f4,yavg_k,pwr] = F(xtmp+    h*f3,  Uk_phys, pk);
            xtmp = xtmp + (h/6)*(f1 + 2*f2 + 2*f3 + f4);
        end

        % Hard dynamics defect (scaled)
        g{end+1} = SxInv * (Sx*Xsym(:,k+1) - xtmp);
        lbg = [lbg; zeros(nx,1)];
        ubg = [ubg; zeros(nx,1)];

   
        eXk    = Xsym(:,k) - Xref_s(:,k);    % 
        w_cost = w_cost ...
                 + eXk.' * Q_s * eXk ...              % 
                 + w_pwr * (pwr / pwrEstMax)^2 ...    % 
                 + Usym(:,k).' * R_s * Usym(:,k);     %

        % Parallel/serial pump coupling
        mPS          = Pstage(idx_cmdPS, k);
        g{end+1}     = mPS * (Usym(3,k) - Usym(4,k));
        lbg(end+1,1) = 0;
        ubg(end+1,1) = 0;

        % Move cost
        if k == 1
            dUk_s = Usym(:,1) - (SuInv * p_u_last_phys);
        else
            dUk_s = Usym(:,k) - Usym(:,k-1);
        end
        w_cost = w_cost + lambda_du(k) * (dUk_s.' * Rd_s * dUk_s);  %

        % Soft ramp-rate constraints
        Sdu_k = Sdu_sym(:,k);
        g{end+1} = dUk_s + Sdu_k;
        lbg = [lbg; -du_max_s];
        ubg = [ubg;  inf(nu,1)];

        g{end+1} = dUk_s - Sdu_k;
        lbg = [lbg; -inf(nu,1)];
        ubg = [ubg;  du_max_s];

        % Algebraic outputs
        innerCondRefToutK = yavg_k(10);
        innerCondAirTinK  = yavg_k(27);
        htxClntToutK      = yavg_k(6);
        refTempCompInK    = yavg_k(14);
        invClntToutK      = yavg_k(1);
        motClntToutK      = yavg_k(2);
        dcdcClntToutK     = yavg_k(5);
        heaterClntToutK   = yavg_k(7);
        heaterClntTInK    = yavg_k(30);

        Sy1k = Sysym(1,k);
        Sy2k = Sysym(2,k);
        Sy3k = Sysym(3,k);
        Sy4k = Sysym(4,k);

        g{end+1}     = innerCondRefToutK - innerCondAirTinK + Sy1k;
        lbg(end+1,1) = 0; ubg(end+1,1) = inf;

        g{end+1}     = heaterClntToutK - heaterClntTInK - Sy2k;
        lbg(end+1,1) = -inf; ubg(end+1,1) = 18;

        g{end+1}     = (Pstage(38,k) + 2) - refTempCompInK + Sy3k;
        lbg(end+1,1) = -2; ubg(end+1,1) = 10;

        for clntVal = {htxClntToutK, heaterClntToutK, invClntToutK, motClntToutK, dcdcClntToutK}
            g{end+1}     = clntVal{1} - clntHigh - Sy4k;
            lbg(end+1,1) = -inf; ubg(end+1,1) = 0;
            g{end+1}     = clntLow - clntVal{1} - Sy4k;
            lbg(end+1,1) = -inf; ubg(end+1,1) = 0;
        end
    end

    %% ── Soft preferred state bounds ──────────────────────────────────────
    for k = 1:(N+1)
        xk_phys = Sx * Xsym(:,k);
        for j = 1:ns_soft
            ii  = idx_soft_state(j);
            slo = Sx_losym(j,k);
            shi = Sx_hisym(j,k);
            g{end+1}     = xpref_lo(ii) - xk_phys(ii) - slo;
            lbg(end+1,1) = -inf; ubg(end+1,1) = 0;
            g{end+1}     = xk_phys(ii) - xpref_hi(ii) - shi;
            lbg(end+1,1) = -inf; ubg(end+1,1) = 0;
        end
    end

    %% ── Terminal cost ────────────────────────────────────────────────────
 
    Pmat   = reshape(p_P, nx, nx);
    eXN    = Xsym(:,N+1) - (SxInv * p_xT);
    w_cost = w_cost + 1.0 * (eXN.' * Pmat * eXN);

    %% ── Decision variable vector ─────────────────────────────────────────
    w_all = [Xsym(:); Usym(:); Sysym(:); ...
             Sx_losym(:); Sx_hisym(:); Sdu_sym(:)];
    n_w   = numel(w_all);

    %% ── Variable bounds ──────────────────────────────────────────────────
    lbw = -inf(n_w, 1);
    ubw =  inf(n_w, 1);

    xmin_s = SxInv * xmin;
    xmax_s = SxInv * xmax;
    for k = 1:(N+1)
        i0 = (k-1)*nx;
        lbw(i0+(1:nx)) = xmin_s;
        ubw(i0+(1:nx)) = xmax_s;
    end

    umin_s = SuInv * umin;
    umax_s = SuInv * umax;
    offU   = numel(Xsym(:));
    for k = 1:N
        u0 = offU + (k-1)*nu;
        lbw(u0+(1:nu)) = umin_s;
        ubw(u0+(1:nu)) = umax_s;
    end

    nX   = numel(Xsym(:));
    nU   = numel(Usym(:));

    nSy      = 4*N;
    idxSy    = nX + nU + (1:nSy);

    nSx      = ns_soft*(N+1);
    idxSx_lo = nX + nU + nSy + (1:nSx);
    idxSx_hi = nX + nU + nSy + nSx + (1:nSx);

    nSdu     = nu*N;
    idxSdu   = nX + nU + nSy + 2*nSx + (1:nSdu);

    lbw(idxSx_lo) = 0;    ubw(idxSx_lo) = inf;
    lbw(idxSx_hi) = 0;    ubw(idxSx_hi) = inf;
    lbw(idxSdu)   = 0;    ubw(idxSdu)   = inf;
    lbw(idxSy)    = -1e6; ubw(idxSy)    =  1e6;

    idxSy2 = idxSy(2:4:end);
    idxSy4 = idxSy(4:4:end);
    lbw(idxSy2) = 0;
    lbw(idxSy4) = 0;

    Gfun = casadi.Function('Gfun', {w_all, p_all}, {vertcat(g{:})});
    prob = struct('f', w_cost, 'x', w_all, 'g', vertcat(g{:}), 'p', p_all);

    %% ── IPOPT options ────────────────────────────────────────────────────
    opts                    = struct;
    opts.error_on_fail      = false;
    opts.print_time         = false;
    opts.calc_lam_p         = false;

    DEBUG_IPOPT = false;
    if DEBUG_IPOPT
        opts.ipopt.output_file = 'ipopt_debug.txt';
        opts.ipopt.print_level = 5;
    else
        opts.ipopt.print_level = 0;
    end

    opts.ipopt.max_iter        = 2000;
    opts.ipopt.tol             = 1e-4;   
    opts.ipopt.constr_viol_tol = 1e-4;
    opts.ipopt.dual_inf_tol    = 1e-2;
    opts.ipopt.compl_inf_tol   = 1e-3;

    opts.ipopt.acceptable_tol             = 1e-3;
    opts.ipopt.acceptable_iter            = 3;
    opts.ipopt.acceptable_constr_viol_tol = 1e-3;
    opts.ipopt.acceptable_dual_inf_tol    = 5e-1;
    opts.ipopt.acceptable_compl_inf_tol   = 1e-2;
    opts.ipopt.acceptable_obj_change_tol  = 1e-4;

    opts.ipopt.hessian_approximation = 'exact';
    opts.ipopt.linear_solver         = 'ma97';

    opts.ipopt.mu_strategy = 'adaptive';
    opts.ipopt.mu_oracle   = 'quality-function';
    opts.ipopt.alpha_for_y = 'min-dual-infeas';
    opts.ipopt.recalc_y    = 'yes';


    opts.ipopt.bound_relax_factor              = 1e-8;
    opts.ipopt.honor_original_bounds           = 'yes';
    opts.ipopt.bound_push                      = 1e-2;
    opts.ipopt.bound_frac                      = 1e-2;
    opts.ipopt.warm_start_bound_push           = 1e-8;
    opts.ipopt.warm_start_mult_bound_push      = 1e-8;
    opts.ipopt.nlp_scaling_method              = 'gradient-based';
    opts.ipopt.nlp_scaling_max_gradient        = 10;   % reduced from 100
    opts.ipopt.nlp_scaling_obj_target_gradient = 1;
    opts.ipopt.derivative_test                 = 'none';

    solver = casadi.nlpsol('solver', 'ipopt', prob, opts);

    prev_w_opt = [];
    prev_lam_x = [];
    prev_lam_g = [];

    u_last = [3000; 0.08; 800; 800; 4000; 1000];

    inputInit.envT          = Tamb_C + 273.15;
    inputInit.clntInvTout_K = Tamb_C + 273.15;
    inputInit.clntBatTin_K  = Tamb_C + 273.15;
    inputInit.compN         = u_last(1);
    inputInit.pumpN         = u_last(3);
    inputInit.prevMode      = 0;
    inputInit.heaterPwr     = u_last(5);

    prev_Pk               = repmat(p_sample, 1, N);
    fanRpm_actual_tracker = 1000;
    Tref_default          = 294.15;
    infeas_count          = 0;

    cache_file       = fullfile(pwd, 'nmpc_warm_cache_v4.mat');
    first_step_saved = false;
    n_g_built        = numel(lbg);

    if isfile(cache_file)
        try
            c = load(cache_file);
            dims_ok = isfield(c,'n_w_cache')   && (c.n_w_cache == n_w)      && ...
                      isfield(c,'np_cache')    && (c.np_cache  == np)        && ...
                      isfield(c,'N_cache')     && (c.N_cache   == N)         && ...
                      isfield(c,'nu_cache')    && (c.nu_cache  == nu)        && ...
                      isfield(c,'nx_cache')    && (c.nx_cache  == nx)        && ...
                      isfield(c,'n_g_cache')   && (c.n_g_cache == n_g_built) && ...
                      isfield(c,'w_opt_cache') && all(isfinite(c.w_opt_cache));
            if dims_ok
                prev_w_opt = c.w_opt_cache;
                prev_lam_x = c.lam_x_cache;
                prev_lam_g = c.lam_g_cache;
                fprintf('[coreLQR] Loaded warm-start cache: %s\n', cache_file);
            else
                fprintf('[coreLQR] Cache dimension mismatch — ignoring.\n');
            end
        catch ME
            fprintf('[coreLQR] Cache load failed: %s\n', ME.message);
        end
    end

    built = true;
    fprintf('[coreLQR] NLP built: n_w=%d, n_g=%d, np=%d\n', n_w, n_g_built, np);
end

%% ── Per-step ─────────────────────────────────────────────────────────────

x_meas(6) = max(min(x_meas(6), xmax(6)), xmin(6));
x_meas(7) = max(min(x_meas(7), xmax(7)), xmin(7));

min_p_ratio = 1.5;
if x_meas(7) < min_p_ratio * x_meas(6)
    x_meas_pk    = x_meas;
    x_meas_pk(6) = max(x_meas(6), 2.0e5);
    x_meas_pk(7) = max(x_meas(7), max(x_meas_pk(6)*min_p_ratio, 6.0e5));
    fprintf('[coreLQR] Cold-start guard: p_in=%.2f bar  p_out=%.2f bar\n', ...
            x_meas_pk(6)/1e5, x_meas_pk(7)/1e5);
else
    x_meas_pk = x_meas;
end

if ~isfinite(Tref_K) || isempty(Tref_K)
    Tref_K = Tref_default;
elseif Tref_K < 200
    Tref_K = Tref_K + 273.15;
end
Tref_K = max(15+273.15, min(40+273.15, Tref_K));

inputInit.clntInvTout_K = clntInvTout_K_meas;
inputInit.clntBatTin_K  = clntBatTin_K_meas;
inputInit.envT          = Tamb_C + 273.15;

if isscalar(dcdcQ)
    heatk.dcdcQ    = repmat(dcdcQ,              N, 1);
    heatk.motQ     = repmat(motQ,               N, 1);
    heatk.invQ     = repmat(invQ,               N, 1);
    heatk.speed    = repmat(speed(1),           N, 1);
    heatk.batImeas = repmat(max(batImeas,1e-3), N, 1);
else
    heatk.dcdcQ    = dcdcQ(:);
    heatk.motQ     = motQ(:);
    heatk.invQ     = invQ(:);
    heatk.speed    = speed(:);
    heatk.batImeas = max(batImeas(:), 1e-3);
end
heatk.superHeat = 2;

tau_fan = 5.0;
alpha   = exp(-dt / tau_fan);

tuneS       = prepParaVec(x_meas_pk, Tamb_C, Stuned.tunedVariables);
[ctrl, est] = thermalLogicAndEstimation(x_meas_pk, inputInit, heatk);
est.speed   = heatk.speed;

Pk = zeros(np, N);
fanRpm_pred = fanRpm_actual_tracker;
for i = 1:N
    est_i            = est;
    est_i.speed      = heatk.speed(i);
    est_i.batImeas   = heatk.batImeas(i);
    est_i.motQmeas   = heatk.motQ(i);
    est_i.invQmeas   = heatk.invQ(i);
    est_i.dcdcQmeas  = heatk.dcdcQ(i);
    est_i.superheatK = heatk.superHeat;
    fanRpm_pred      = alpha * fanRpm_pred + (1 - alpha) * u_last(6);
    est_i.fanRpm_prev = fanRpm_pred;
    pk_col = packParams(est_i, ctrl, tuneS);
    if ~all(isfinite(pk_col))
        warning('coreLQR:nanParams','NaN in pk_col at stage %d — fallback.', i);
        pk_col = prev_Pk(:,i);
    end
    Pk(:,i) = pk_col;
end
prev_Pk = Pk;

%% ── Terminal data ────────────────────────────────────────────────────────
Ki                = 0.02;          
windup_threshold  = 5.0;            
int_offset_max    = 8.0;            

e_k = x_meas(9) - Tref_K;        

if abs(e_k) < windup_threshold
    int_offset = int_offset + Ki * e_k;
end

% Hard anti-windup clamp
int_offset = max(-int_offset_max, min(int_offset_max, int_offset));


Tref_shifted = Tref_K - int_offset;
Tref_shifted = max(15+273.15, min(40+273.15, Tref_shifted));  % safety clamp

% Use Tref_shifted throughout the horizon
Tref_path = Tref_shifted * ones(N+1, 1);

% Store for diagnostics
int_e_prev = e_k;

fprintf('[integral] e=%.2fK  int_offset=%.2fK  Tref_shifted=%.2fC\n', ...
        e_k, int_offset, Tref_shifted-273.15);

% Tref_path = Tref_K * ones(N+1, 1);
w_term    = struct('cab', 2e3, 'energy', 1e-3);

[xT, uT, Pterm, Kterm] = compute_terminal_data( ...
    F, nx, nu, h, M, Pk(:,N), xmin, xmax, umin, umax, Tref_path(end), w_term);

if ~isempty(Kterm) && all(isfinite(Kterm(:))) && ...
   ~isempty(xT)    && all(isfinite(xT(:)))    && ...
   ~isempty(uT)    && all(isfinite(uT(:)))
    K_lqr_last  = Kterm;
    xT_lqr_last = xT;
    uT_lqr_last = uT;
end

fprintf('[terminal] xT(9) = %.2f C  (target = %.2f C)\n', xT(9)-273.15, Tref_path(end)-273.15);

% Cap pressure entries to prevent DARE instability corrupting P
Pterm(6,:) = 0; Pterm(:,6) = 0; Pterm(6,6) = 1e-4;
Pterm(7,:) = 0; Pterm(:,7) = 0; Pterm(7,7) = 1e-4;


P_scaled_raw = Sx.' * Pterm * Sx;
P_scale_9    = P_scaled_raw(9,9);
if P_scale_9 > 0
    P_norm_factor = min(1.0, (10 * Q_s(9,9)) / P_scale_9);
else
    P_norm_factor = 1.0;
end
P_scaled = P_norm_factor * P_scaled_raw;

Pterm_diag    = diag(Pterm);
P_scaled_diag = diag(P_scaled);
fprintf('[terminal] P(phys):   p_in=%.2e  p_out=%.2e  cabAirT=%.2e\n', ...
        Pterm_diag(6), Pterm_diag(7), Pterm_diag(9));
fprintf('[terminal] P(scaled): p_in=%.2e  p_out=%.2e  cabAirT=%.2e  (norm=%.3e)\n', ...
        P_scaled_diag(6), P_scaled_diag(7), P_scaled_diag(9), P_norm_factor);

pvec = [x_meas; Tref_path; Pk(:); xT; uT; P_scaled(:); u_last];
assert(numel(pvec) == numel(p_all), ...
    '[coreLQR] pvec size %d != p_all size %d', numel(pvec), numel(p_all));

%% ── Warm start ───────────────────────────────────────────────────────────
w0 = zeros(n_w, 1);

Xs0 = repmat(SxInv*x_meas, 1, N+1);
w0(1:nX) = Xs0(:);

Us0 = repmat(SuInv*u_last, 1, N);
w0(nX+(1:nU)) = Us0(:);

w0(idxSy)  = 0;
w0(idxSdu) = 0;

Xphys0 = Sx * reshape(w0(1:nX), nx, N+1);
Sx_lo0 = zeros(ns_soft, N+1);
Sx_hi0 = zeros(ns_soft, N+1);
for kk = 1:(N+1)
    for j = 1:ns_soft
        ii = idx_soft_state(j);
        Sx_lo0(j,kk) = max(0, xpref_lo(ii) - Xphys0(ii,kk));
        Sx_hi0(j,kk) = max(0, Xphys0(ii,kk) - xpref_hi(ii));
    end
end
w0(idxSx_lo) = Sx_lo0(:);
w0(idxSx_hi) = Sx_hi0(:);

if ~isempty(prev_w_opt)
    w0 = shift_warm_start_full(prev_w_opt, w0, nx, nu, N, nX, nU, ...
                               idxSy, idxSx_lo, idxSx_hi, idxSdu, ...
                               idx_soft_state, ns_soft, xpref_lo, xpref_hi, Sx);
end

%% ── Solve ────────────────────────────────────────────────────────────────
n_g  = numel(lbg);
args = {'x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg, 'p', pvec};
if ~isempty(prev_lam_x) && numel(prev_lam_x) == n_w
    args(end+1:end+2) = {'lam_x0', prev_lam_x};
end
if ~isempty(prev_lam_g) && numel(prev_lam_g) == n_g
    args(end+1:end+2) = {'lam_g0', prev_lam_g};
end

t_solver    = tic;
sol         = solver(args{:});
t_solver_ms = toc(t_solver) * 1000;

st    = solver.stats();
w_opt = full(sol.x);

U_opt_s     = reshape(w_opt(nX+(1:nU)), nu, N);
u_cmd       = Su * U_opt_s(:,1);
X_opt_s     = reshape(w_opt(1:nX), nx, N+1);
x_pred_next = Sx * X_opt_s(:,2);

%% ── Diagnostics ──────────────────────────────────────────────────────────
g_val  = full(Gfun(w_opt, pvec));
g_viol = max([lbg - g_val, g_val - ubg, zeros(size(g_val))], [], 2);
fprintf('\n[diag] Constraints: max_abs=%.2e  max_viol=%.2e  n_viol=%d/%d\n', ...
        max(abs(g_val)), max(g_viol), nnz(g_viol>1e-6), numel(g_viol));

state_names_diag = {'motT','invT','dcdcT','SOC','batT','p_in','p_out','cabIntT','cabAirT'};

% Raw dynamics defect
raw_defect = zeros(nx, N);
for k = 1:N
    xkp = Sx * X_opt_s(:,k);
    ukp = Su * U_opt_s(:,k);
    xt  = xkp;
    for j = 1:M
        [f1,~,~] = F(xt,          ukp, Pk(:,k));
        [f2,~,~] = F(xt+0.5*h*f1, ukp, Pk(:,k));
        [f3,~,~] = F(xt+0.5*h*f2, ukp, Pk(:,k));
        [f4,~,~] = F(xt+    h*f3,  ukp, Pk(:,k));
        xt = xt + (h/6)*(f1+2*f2+2*f3+f4);
    end
    raw_defect(:,k) = full(SxInv*(Sx*X_opt_s(:,k+1) - xt));
end
fprintf('[diag] Dyn defect: ');
for i = [6,7,9]
    fprintf('%s=%.1e  ', state_names_diag{i}, max(abs(raw_defect(i,:))));
end
fprintf('\n');

% Predicted trajectory
X_opt_phys = Sx * X_opt_s;
U_opt_phys = Su * U_opt_s;

fprintf('\n[pred] Trajectory:\n');
fprintf('  %-4s %-7s %-10s %-10s %-8s\n','k','cabAirT','p_in[bar]','p_out[bar]','compN');
for ki = [1,3,5,10,15,20,25,30,31]
    if ki > N+1, continue; end
    xp = X_opt_phys(:,ki);
    up = U_opt_phys(:,min(ki,N));
    fprintf('  %-4d %-7.2f %-10.3f %-10.3f %-8.0f\n', ...
            ki-1, xp(9)-273.15, xp(6)/1e5, xp(7)/1e5, up(1));
end
fprintf('  [ref] %.2f C\n', Tref_K-273.15);

fprintf('\n[pred] Inputs k=1..5:\n');
fprintf('  %-4s %-7s %-7s %-8s %-6s\n','k','compN','blower','heater','radFan');
for ki = 1:min(5,N)
    up = U_opt_phys(:,ki);
    fprintf('  %-4d %-7.0f %-7.4f %-8.1f %-6.0f\n',ki,up(1),up(2),up(5),up(6));
end

% Cost breakdown (for tuning diagnostics)
track_cost = sum(Q_s(9,9) * ((X_opt_phys(9,1:N) - Tref_K) / sx(9)).^2);
fprintf('\n[cost] Total=%.4f  Tracking=%.4f  (ratio=%.1f%%)\n', ...
        full(sol.f), track_cost, 100*track_cost/max(full(sol.f),1e-10));

fprintf('║  Solver: %.1f ms  iters: %d  status: %s\n', ...
        t_solver_ms, double(st.iter_count), char(st.return_status));

%% ── Infeasibility recovery ───────────────────────────────────────────────
%% ── Infeasibility recovery ───────────────────────────────────────────────
if ~st.success
    infeas_count = infeas_count + 1;

    truly_infeasible = strcmp(char(st.return_status), 'Infeasible_Problem_Detected') || ...
                       strcmp(char(st.return_status), 'Restoration_Failed');

    if truly_infeasible
        if ~isempty(K_lqr_last) && ~isempty(xT_lqr_last) && ~isempty(uT_lqr_last) && ...
                all(isfinite(K_lqr_last(:))) && all(isfinite(xT_lqr_last)) && all(isfinite(uT_lqr_last))

            dx = x_meas - xT_lqr_last;
            u_lqr = uT_lqr_last - K_lqr_last * dx;

            % clamp to actuator bounds
            u_cmd = min(umax, max(umin, u_lqr));

            % optional safeguard: keep blower above minimum practical value
            if nu >= 2
                u_cmd(2) = max(u_cmd(2), 0.05);
            end

            fprintf('[coreLQR] NMPC failed -> using LQR fallback (consec=%d, status=%s)\n', ...
                    infeas_count, char(st.return_status));

            fprintf('[coreLQR] LQR fallback: comp=%.1f rpm, blower=%.4f, motPump=%.1f, batPump=%.1f, heater=%.1f W, fan=%.1f rpm\n', ...
                    u_cmd(1), u_cmd(2), u_cmd(3), u_cmd(4), u_cmd(5), u_cmd(6));

        else
            % if LQR data unavailable, fall back to last input
            u_cmd = min(umax, max(umin, u_last));
            warning('coreLQR:noLQRFallback', ...
                    '[coreLQR] NMPC failed and no valid LQR fallback available. Using u_last.');
        end
    else
        % for acceptable but not fully converged cases, keep best iterate
        u_cmd = min(umax, max(umin, u_cmd));
        fprintf('[coreLQR] Using best iterate (status=%s, iter=%d)\n', ...
                char(st.return_status), double(st.iter_count));
    end

    warning('coreLQR:infeasible', ...
            '[coreLQR] Not converged (consecutive: %d) — status: %s', ...
            infeas_count, char(st.return_status));
else
    infeas_count = 0;
end
fanRpm_actual_tracker = alpha * fanRpm_actual_tracker + (1-alpha)*u_cmd(6);
prev_w_opt = w_opt;
if isfield(sol,'lam_x'), prev_lam_x = full(sol.lam_x); else, prev_lam_x = []; end
if isfield(sol,'lam_g'), prev_lam_g = full(sol.lam_g); else, prev_lam_g = []; end

if ~first_step_saved && st.success
    solved_ok  = strcmp(char(st.return_status),'Solve_Succeeded') || ...
                 strcmp(char(st.return_status),'Solved_To_Acceptable_Level');
    iter_count = double(st.iter_count);
    if solved_ok && iter_count < 500 && isfinite(full(sol.f))
        try
            w_opt_cache=w_opt; lam_x_cache=prev_lam_x; lam_g_cache=prev_lam_g;
            n_w_cache=n_w; np_cache=np; N_cache=N; nu_cache=nu;
            nx_cache=nx; n_g_cache=n_g;
            save(cache_file,'w_opt_cache','lam_x_cache','lam_g_cache', ...
                 'n_w_cache','np_cache','N_cache','nu_cache','nx_cache','n_g_cache');
            first_step_saved = true;
            fprintf('[coreLQR] Cache saved (iter=%d): %s\n', iter_count, cache_file);
        catch ME
            warning('coreLQR:cacheSaveFailed','Cache save failed: %s', ME.message);
        end
    end
end

inputInit.compN     = u_cmd(1);
inputInit.pumpN     = u_cmd(3);
inputInit.heaterPwr = u_cmd(5);
if isfield(ctrl,'thermalMode'), inputInit.prevMode = ctrl.thermalMode; end
u_last = u_cmd;

ctrl_vec = packControlOut(ctrl);
iter_out = NaN;
if isfield(st,'iter_count'), iter_out = double(st.iter_count); end

out = [u_cmd; x_pred_next; double(st.success); iter_out; full(sol.f); ctrl_vec];
info.success     = logical(st.success);
info.iter        = iter_out;
info.obj         = full(sol.f);
info.t_solver_ms = t_solver_ms;
if isfield(st,'return_status')
    info.status = string(st.return_status);
else
    info.status = "";
end

end % ── end main function ───────────────────────────────────────────────────


%% ═══════════════════════════════════════════════════════════════════════════
%% Helpers
%% ═══════════════════════════════════════════════════════════════════════════

function w0_next = shift_warm_start_full(prev_w_opt, w0, nx, nu, N, nX, nU, ...
                                          idxSy, idxSx_lo, idxSx_hi, idxSdu, ...
                                          idx_soft_state, ns_soft, xpref_lo, xpref_hi, Sx)
    w0_next = w0;
    Xs = reshape(prev_w_opt(1:nX), nx, N+1);
    Xs_shift = [Xs(:,2:end), Xs(:,end)];
    w0_next(1:nX) = Xs_shift(:);
    Us = reshape(prev_w_opt(nX+(1:nU)), nu, N);
    Us_shift = [Us(:,2:end), Us(:,end)];
    w0_next(nX+(1:nU)) = Us_shift(:);
    Xphys_shift = Sx * Xs_shift;
    Sx_lo = zeros(ns_soft, N+1);
    Sx_hi = zeros(ns_soft, N+1);
    for kk = 1:(N+1)
        for j = 1:ns_soft
            ii = idx_soft_state(j);
            Sx_lo(j,kk) = max(0, xpref_lo(ii) - Xphys_shift(ii,kk));
            Sx_hi(j,kk) = max(0, Xphys_shift(ii,kk) - xpref_hi(ii));
        end
    end
    w0_next(idxSx_lo) = Sx_lo(:);
    w0_next(idxSx_hi) = Sx_hi(:);
    if ~isempty(idxSy)
        try w0_next(idxSy) = prev_w_opt(idxSy); catch, end
    end
    nnu = numel(idxSdu) / N;
    Sdu = reshape(prev_w_opt(idxSdu), nnu, N);
    Sdu_shift = [Sdu(:,2:end), Sdu(:,end)];
    w0_next(idxSdu) = Sdu_shift(:);
end

function idx = get_idx_cmdPS(Uord)
    names = [Uord.estimatedParams(:); Uord.controlOut(:); Uord.tunePara(:)];
    idx   = find(strcmp(names,'cmdParallelSerial'), 1);
    if isempty(idx)
        error('coreLQR:missingParam','cmdParallelSerial not found.');
    end
end

function ctrl_vec = packControlOut(controlOut)
    fields = {'setpoint','thermalMode','cmdRefrigBypass','cmdBlendAir', ...
              'cmdRadBypass','cmdExvChiller','cmdExvEvap','cmdParallelSerial', ...
              'clntBatTinFiltered','cmdExvHeatpump','cmdHeater','prev_ctrlmode'};
    ctrl_vec = zeros(numel(fields),1);
    for k = 1:numel(fields)
        ctrl_vec(k) = getStructFieldNumeric(controlOut, fields{k}, 0);
    end
end

function val = getStructFieldNumeric(S, name, defaultVal)
    if isfield(S,name) && ~isempty(S.(name)) && isscalar(S.(name)) && ...
            (isnumeric(S.(name)) || islogical(S.(name))) && isfinite(double(S.(name)))
        val = double(S.(name));
    else
        val = defaultVal;
    end
end


%% ═══════════════════════════════════════════════════════════════════════════
%% compute_terminal_data
%% ═══════════════════════════════════════════════════════════════════════════

function [xT, uT, P, Kt] = compute_terminal_data(F, nx, nu, h, M, pkN, ...
                                                 xmin, xmax, umin, umax, TrefN, w)

    persistent Phi_fun_p PhiJ_fun_p solverT lbwT ubwT lbgT ubgT ...
               nparam_p nx_p nu_p M_p h_p built_ok tref_mode_p ...
               xT_prev uT_prev xT_cached uT_cached P_cached Kt_cached ...
               pkN_cached TrefN_cached step_counter

    Kt = [];

    tref_mode = 'cost';
    %% ── Defaults ─────────────────────────────────────────────────────────
    if nargin < 13 || isempty(tref_mode)
        tref_mode = 'constraint';
    end
    if ~ismember(tref_mode, {'cost','constraint'})
        error('compute_terminal_data:badMode', ...
              'tref_mode must be ''cost'' or ''constraint''.');
    end

    K_recompute     = 50;
    dT_threshold    = 1.5;    % K  — retrigger if ambient shifts by this much
    dTref_threshold = 0.5;    % K  — retrigger if reference shifts by this much

    if isempty(step_counter), step_counter = 0; end
    step_counter = step_counter + 1;

    %% ── Check if rebuild needed ──────────────────────────────────────────
    need_build = isempty(Phi_fun_p) || isempty(PhiJ_fun_p) || isempty(solverT);
    if ~need_build
        need_build = (nx_p  ~= nx)            || ...
                     (nu_p  ~= nu)            || ...
                     (nparam_p ~= numel(pkN)) || ...
                     (M_p   ~= M)             || ...
                     abs(h_p - h) > 1e-12     || ...
                     isempty(built_ok)         || ...
                     ~strcmp(tref_mode, tref_mode_p);   %
    end

    %% ── Build CasADi functions and terminal NLP solver ───────────────────
    if need_build
        fprintf('[terminal] Building terminal NLP (tref_mode = ''%s'')...\n', tref_mode);

        nparam_p    = numel(pkN);
        nx_p        = nx;
        nu_p        = nu;
        M_p         = M;
        h_p         = h;
        tref_mode_p = tref_mode;

        %% RK4 map: Phi(x,u,p) = x after M steps of size h
        x = casadi.SX.sym('x', nx, 1);
        u = casadi.SX.sym('u', nu, 1);
        p = casadi.SX.sym('p', nparam_p, 1);

        xt = x;
        for j = 1:M
            [k1,~,~] = F(xt,           u, p);
            [k2,~,~] = F(xt+0.5*h*k1,  u, p);
            [k3,~,~] = F(xt+0.5*h*k2,  u, p);
            [k4,~,~] = F(xt+    h*k3,  u, p);
            xt = xt + h/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        Phi_sx = xt;

        Phi_fun_p  = casadi.Function('Phi_fun_p',  {x,u,p}, {Phi_sx});
        PhiJ_fun_p = casadi.Function('PhiJ_fun_p', {x,u,p}, ...
                         {Phi_sx, jacobian(Phi_sx,x), jacobian(Phi_sx,u)});

        %% Terminal NLP decision variables and parameters
        xT_sym   = casadi.SX.sym('xT',   nx, 1);
        uT_sym   = casadi.SX.sym('uT',   nu, 1);
        s_sym    = casadi.SX.sym('s',    nx, 1);   % fixed-point residual slack
        p_sym    = casadi.SX.sym('pp',   nparam_p, 1);
        Tref_sym = casadi.SX.sym('tref', 1, 1);

        Phi_once = casadi.Function('Phi_once', {x,u,p}, {Phi_sx});
        r = xT_sym - Phi_once(xT_sym, uT_sym, p_sym);   % fixed-point residual

        %% Smooth L1 + L2 penalty on slack
        eps1 = 1e-8;
        L1s  = sum(sqrt(s_sym.^2 + eps1));
        L2s  = sumsqr(s_sym);

        %% Input regularisati
        u_reg = 1e-4 * sumsqr(uT_sym);

        %% Cabin tracking weight in terminal NLP cost
        w_cabT = max(1e3, 1e3 * w.cab);

        switch tref_mode
            case 'cost'
      
                ell = 1e4*L1s + 1e-2*L2s + u_reg ...
                      + w_cabT * (xT_sym(9) - Tref_sym)^2;
                g_T   = r - s_sym;              % nx x 1
                lbgT  = zeros(nx, 1);
                ubgT  = zeros(nx, 1);

            case 'constraint'
           
                ell = 1e4*L1s + 1e-2*L2s + u_reg;
                g_T   = [r - s_sym;
                         xT_sym(9) - Tref_sym]; % (nx+1) x 1
                lbgT  = zeros(nx+1, 1);
                ubgT  = zeros(nx+1, 1);
        end

        probT = struct('f', ell, ...
                       'x', vertcat(xT_sym, uT_sym, s_sym), ...
                       'g', g_T, ...
                       'p', vertcat(p_sym, Tref_sym));

        optsT                      = struct;
        optsT.print_time           = false;
        optsT.error_on_fail        = false;
        optsT.ipopt.print_level    = 0;
        optsT.ipopt.linear_solver  = 'MA97';
        optsT.ipopt.max_iter       = 150;
        optsT.ipopt.tol            = 1e-6;
        optsT.ipopt.acceptable_tol = 1e-2;
        optsT.ipopt.acceptable_iter = 3;

        solverT = casadi.nlpsol('solverT', 'ipopt', probT, optsT);

        lbwT = [xmin; umin; -inf(nx,1)];
        ubwT = [xmax; umax;  inf(nx,1)];

        %% Reset cache
        built_ok     = true;
        xT_cached    = [];
        uT_cached    = [];
        P_cached     = [];
        pkN_cached   = [];
        TrefN_cached = [];
        step_counter = 1;

        fprintf('[terminal] Build complete. Constraints: %d  tref_mode: %s\n', ...
                numel(lbgT), tref_mode);
    end

    %% ── Cache check ──────────────────────────────────────────────────────
    have_cache = ~isempty(xT_cached) && ~isempty(P_cached) && ~isempty(pkN_cached);

    if have_cache
        p_rel = norm((pkN - pkN_cached) ./ max(abs(pkN_cached), 1), inf);
        do_recompute = (mod(step_counter, K_recompute) == 1)          || ...
                       abs(pkN(1) - pkN_cached(1)) > dT_threshold     || ...
                       abs(TrefN  - TrefN_cached)  > dTref_threshold  || ...
                       p_rel > 0.05;
    else
        do_recompute = true;
    end

    if ~do_recompute
        xT = xT_cached;
        uT = uT_cached;
        P  = P_cached;
        Kt = Kt_cached;
        return
    end

    %% ── Solve terminal fixed-point NLP ───────────────────────────────────
    u_guess = [4200; 0.11; 750; 750; 4000; 1000];
    x_guess = [18.8+273.15; 18.8+273.15; 18.8+273.15; 0.94; 18.8+273.15; ...
               2.0e5; 10.0e5; 21.3+273.15; 21.3+273.15];

    if ~isempty(xT_prev) && all(isfinite(xT_prev))
        x_guess = xT_prev;
        u_guess = uT_prev;
    end

    solT  = solverT('x0',  [x_guess; u_guess; zeros(nx,1)], ...
                    'lbx', lbwT, 'ubx', ubwT, ...
                    'lbg', lbgT, 'ubg', ubgT, ...
                    'p',   [pkN; TrefN]);
    wstar = full(solT.x);

    if isempty(wstar) || any(~isfinite(wstar))
        warning('compute_terminal_data:terminalSolveFailed', ...
                'Terminal NLP failed — using previous or default guess.');
        xT = x_guess;
        uT = u_guess;
    else
        xT = wstar(1:nx);
        uT = wstar(nx+(1:nu));
    end

    %% Diagnostics
    fp_res = norm(xT - full(Phi_fun_p(xT, uT, pkN)));
    fprintf('[terminal] xT(9)=%.2f C  FP_res=%.2e  tref_mode=%s\n', ...
            xT(9)-273.15, fp_res, tref_mode);

    %% Clamp to physical bounds
    clamp = @(v,lo,hi) min(max(v,lo),hi);
    xT    = clamp(xT, xmin, xmax);
    uT    = clamp(uT, umin, umax);
    if nu >= 2, uT(2) = max(uT(2), 0.05); end   % blower minimum

    xT_prev = xT;
    uT_prev = uT;

    %% ── Linearise at terminal point ──────────────────────────────────────
    [~, A, B] = PhiJ_fun_p(xT, uT, pkN);
    A = full(A);
    B = full(B);

    %% Print time constants
    tau_diag    = -(M*h) ./ log(max(abs(diag(A)), 1e-10));
    state_names = {'motT','invT','dcdcT','SOC','batT','p_in','p_out','cabIntT','cabAirT'};
    fprintf('[terminal] State time constants:\n');
    for i = 1:numel(state_names)
        if A(i,i) > 1.0
            lbl = '<- UNSTABLE';
        elseif tau_diag(i) > 3000
            lbl = '(integrator)';
        else
            lbl = '';
        end
        fprintf('  %-8s  tau=%8.1f s  A(%d,%d)=%+.4f  %s\n', ...
                state_names{i}, tau_diag(i), i, i, A(i,i), lbl);
    end

    %% ── Schur stabilisation for DARE ─────────────────────────────────────
    rho_A = max(abs(eig(A)));
    if rho_A >= 1
        [U_s, T_s] = schur(A, 'complex');
        d          = diag(T_s);
        mask       = abs(d) >= 1.0;
        if any(mask)
            d(mask)         = 0.99 * d(mask) ./ abs(d(mask));
            T_s(1:nx+1:end) = d;
            A_stab          = real(U_s * T_s * U_s');
            fprintf('[terminal] Stabilised %d eigenvalue(s) for DARE.\n', sum(mask));
        else
            A_stab = A;
        end
    else
        A_stab = A;
    end

    %% ── DARE cost matrices ───────────────────────────────────────────────
    Q        = zeros(nx);
    Q(9,9)   = max(1e3, 5e2 * w.cab);
    R        = max(1e-2, w.energy) * eye(nu);
    Q        = Q + 1e-6 * eye(nx);   % regularisation
    R        = R + 1e-4 * eye(nu);

    if any(~isfinite(A(:))) || any(~isfinite(B(:)))
        warning('compute_terminal_data:invalidAB', ...
                'A or B contains non-finite values — using scaled identity P.');
        P = (trace(Q)/nx + 1e-2) * eye(nx);
        P(6,6) = min(P(6,6), 1e-2);
        P(7,7) = min(P(7,7), 1e-2);
        xT_cached=xT; uT_cached=uT; P_cached=P; pkN_cached=pkN; TrefN_cached=TrefN;
        return
    end

    %% ── DARE fallback cascade ────────────────────────────────────────────
    P = [];

    % Attempt 1 — nominal DARE on Schur-stabilised A
    if isempty(P)
        try
            [Pt,Kt_try,~] = dare(A_stab, B, Q, R);
            if all(isfinite(Pt(:))) && all(eig((Pt+Pt')/2) > 0)
                P  = Pt;
                Kt = Kt_try;
                fprintf('[terminal] DARE: nominal succeeded.\n');
            end
        catch
        end
    end

    % Attempt 2 — discounted DARE (handles near-unstable modes)
    if isempty(P)
        try
            rho_s = max(abs(eig(A_stab)));
            if rho_s >= 1
                g = (0.95 / rho_s)^2;
            else
                g = 0.98;
            end
            [Pt,Kt_try,~] = dare(sqrt(g)*A_stab, sqrt(g)*B, Q, R);
            if all(isfinite(Pt(:))) && all(eig((Pt+Pt')/2) > 0)
                P  = Pt;
                Kt = Kt_try;
                fprintf('[terminal] DARE: discounted (gamma=%.4f) succeeded.\n', g);
            end
        catch
        end
    end

    % Attempt 3 — finite-horizon Riccati iteration (80 steps)
    if isempty(P)
        try
            PkR = Q;
            for k = 1:80
                S   = R + B'*PkR*B + 1e-8*eye(nu);
                PkR = Q + A'*PkR*A - A'*PkR*B*(S\(B'*PkR*A));
                PkR = 0.5*(PkR + PkR');
            end
           if all(isfinite(PkR(:))) && all(eig((PkR+PkR')/2) > 0)
                S = R + B' * PkR * B + 1e-8*eye(nu);
                Kt = S \ (B' * PkR * A_stab);
                P  = PkR;
                fprintf('[terminal] DARE: finite-horizon Riccati succeeded.\n');
           end
        catch
        end
    end

    % Attempt 4 — Lyapunov equation on scaled A
    if isempty(P)
        try
            rho  = max(abs(eig(A)));
            Abar = A;
            if rho >= 1, Abar = (0.98/rho) * A; end
            Pt = dlyap(Abar', Q);
            if all(isfinite(Pt(:)))
                P = Pt;
                fprintf('[terminal] DARE: Lyapunov fallback succeeded.\n');
            end
        catch
        end
    end

    % Attempt 5 — scaled identity (last resort)
    if isempty(P)
        P = (trace(Q)/nx + 1e-2) * eye(nx);
        warning('compute_terminal_data:identityFallback', ...
                'All DARE attempts failed — using scaled identity P.');
    end

    %% ── Cap pressure entries (prevents DARE instability corrupting P) ────
    P(6,6) = min(P(6,6), 1e-2);
    P(7,7) = min(P(7,7), 1e-2);
    P(6,:) = 0;  P(:,6) = 0;  P(6,6) = min(P(6,6), 1e-4);
    P(7,:) = 0;  P(:,7) = 0;  P(7,7) = min(P(7,7), 1e-4);

    fprintf('[terminal] P(9,9)=%.2e  P(6,6)=%.2e  P(7,7)=%.2e\n', ...
            P(9,9), P(6,6), P(7,7));

    %% ── Update cache ─────────────────────────────────────────────────────
    xT_cached    = xT;
    uT_cached    = uT;
    P_cached     = P;
    Kt_cached    = Kt;
    pkN_cached   = pkN;
    TrefN_cached = TrefN;
end