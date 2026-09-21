function res = e5_subgoal (varargin)
% E5: sub-goal horizon ablation.
%
% Remark rem:design-window claims the safety--convergence window
%     w2max*Gr/(w1min*R_target)  <=  k_a/k_r  <=  w2_*Gr_safe/(w1^*L)
% is EMPTY at global scale (L = D_max) and becomes NON-EMPTY once the active
% target is a sub-goal at horizon L << D_max.  This experiment fixes a gain
% ratio inside the L = 3 m window and sweeps L, so the predicted transition
% (unsafe / non-convergent at large L, safe and convergent at small L) is
% either reproduced or refuted.
%
% It simultaneously tests Theorem thm:gnron-hybrid:
%   (i)  N_sw <= ceil(D_max / Delta)
%   (ii) d_obs >= r_safety across switching instants
%   (iii) convergence to the residual ball after the final switch
%
% Options: 'ratio', 'Ls', 'seeds', 'nstart', 'nobs', 'Delta', 'dt', 'T_max'

  opt = struct ('ratio', 5.5, 'Ls', [], 'seeds', 1:6, 'nstart', 8, ...
                'nobs', 45, 'Delta', 0.5, 'dt', 2e-3, 'T_max', 60);
  for i = 1:2:numel (varargin), opt.(varargin{i}) = varargin{i+1}; end

  Pbase = fapf_params ();
  if (isempty (opt.Ls)), opt.Ls = [Pbase.D_max, 8, 5, 3, 2]; end

  k_r = Pbase.k_r;
  k_a = opt.ratio * k_r;
  P   = fapf_params ('k_a', k_a, 'k_r', k_r, 'dt', opt.dt, 'T_max', opt.T_max);

  Gr_safe = fapf_gradUr_mag (P.r_safety, P);
  [w1bar, w2und] = near_field_weights (P);
  R_target = 4.0;
  lo = P.w2_max * P.G_r / (P.w1_min * R_target);

  printf ('\n=== E5  Sub-goal horizon ablation (GNRON supervisory layer) ===\n');
  printf ('k_a = %.2f, k_r = %.2f  ->  k_a/k_r = %.2f\n', P.k_a, P.k_r, opt.ratio);
  printf ('residual radius R = %.2f m   (R_target = %.1f m needs ratio >= %.2f)\n', ...
          P.R, R_target, lo);
  printf ('Delta = %.2f m  ->  switch bound ceil(D_max/Delta) = %d\n\n', ...
          opt.Delta, ceil (P.D_max / opt.Delta));

  printf ('%-7s %-10s %-8s %-9s %-9s %-9s %-9s %-8s\n', ...
          'L [m]', 'safe_bnd', 'window', 'succ[%]', 'minclr', 'sw_mean', 'sw_max', 'sw_ok');
  rows = [];

  for L = opt.Ls
    hi   = w2und * Gr_safe / (w1bar * L);
    inwin = (lo <= hi) && (opt.ratio >= lo) && (opt.ratio <= hi);

    % dwell time of Assumption ass:dwell
    lam_m = P.w1_min * P.k_a;
    tauD  = log (L / P.rho) / lam_m;

    nrun = 0; nsucc = 0; clr = inf; sw = []; nviol = 0;
    for s = opt.seeds
      env = fapf_env (opt.nobs, s, P, 'random');
      for a = linspace (0, 2*pi, opt.nstart+1)(1:opt.nstart)
        q0 = env.goal + 0.9 * P.D_max * [cos(a), sin(a)];
        if (fapf_dobs (q0, env.obs) < P.r_safety), continue; end
        o = run_supervised (q0, env, P, L, opt.Delta, tauD);
        nrun++;
        nsucc += o.success;
        clr = min (clr, o.dmin);
        nviol += (o.dmin < P.r_safety - 1e-9);
        sw(end+1) = o.nsw;
      end
    end

    bound = ceil (P.D_max / opt.Delta);
    sw_ok = isempty (sw) || (max (sw) <= bound);
    m_sw = NaN; if ~isempty (sw), m_sw = mean (sw); end
    printf ('%-7.1f %-10.3f %-8s %-9.1f %-9.4f %-9.2f %-9d %-8s\n', ...
            L, hi, ternary (inwin, 'IN', 'OUT'), 100*nsucc/max(nrun,1), ...
            clr, m_sw, max ([sw, 0]), ternary (sw_ok, 'YES', 'NO'));
    rows(end+1,:) = [L, hi, inwin, 100*nsucc/max(nrun,1), clr, ...
                     m_sw, max([sw,0]), nviol];
  end

  printf ('\ncolumns: safe_bnd = safety upper bound on k_a/k_r at horizon L;\n');
  printf ('         window   = whether the implemented ratio lies in the design window;\n');
  printf ('         minclr   = min clearance over all runs (r_safety = %.2f m).\n', P.r_safety);

  res.P = P; res.rows = rows; res.Ls = opt.Ls;
  res.switch_bound = ceil (P.D_max / opt.Delta);
end

% =====================================================================

function o = run_supervised (q0, env, P, L, Delta, tauD)
% Holonomic closed loop with the GNRON supervisor of Section
% subsec:gnron-enhancement.  Stall is detected from the convergence metric
% Eq. (convergence-metric-section4) and its windowed rate
% Eq. (convergence-rate-section4).

  dt = P.dt;
  n  = round (P.T_max / dt);
  q  = q0(:).';

  gamma_d  = 1.0;
  T_window = 0.5;                       % Eq. (convergence-rate-section4)
  Nw       = max (2, round (T_window / dt));
  eps_conv = 5e-3;                      % epsilon_convergence
  T_thresh = max (1.0, tauD);           % Assumption ass:dwell
  rho_wp   = 0.30;                      % waypoint capture radius

  Vbuf = nan (1, Nw);  bi = 0;
  t_stall = 0; t_last = -inf;
  target = env.goal; mode = 0; nsw = 0;
  dmin = inf; cause = 'timeout'; k = 0;

  for k = 1:n
    t = (k-1) * dt;
    [F, I] = fapf_force (q, target, env.obs, P);
    dmin = min (dmin, I.d);
    e_goal = norm (q - env.goal);

    if (e_goal <= P.rho), cause = 'goal'; break; end

    % --- supervisor -----------------------------------------------------
    V = e_goal^2 + gamma_d * max (0, P.d0 - I.d);
    bi = mod (bi, Nw) + 1;
    Vold = Vbuf(bi);
    Vbuf(bi) = V;
    if (!isnan (Vold))
      Vdot = (V - Vold) / T_window;
      if (abs (Vdot) < eps_conv), t_stall += dt; else, t_stall = 0; end
    end

    if (mode == 1 && norm (q - target) <= rho_wp)
      target = env.goal; mode = 0; t_stall = 0; Vbuf(:) = nan;
    end

    if (t_stall > T_thresh && (t - t_last) >= tauD)
      [wp, ok] = fapf_subgoal (q, env.goal, env.obs, P, L, Delta);
      if (ok)
        target = wp; mode = 1; nsw++; t_last = t;
        t_stall = 0; Vbuf(:) = nan;
      else
        cause = 'no_waypoint'; break;      % supervisor reports failure
      end
    end

    q = q + dt * F;
    if (norm (q - env.goal) > 3 * P.D_max), cause = 'diverged'; break; end
  end

  o.q_end   = q;
  o.e_end   = norm (q - env.goal);
  o.cause   = cause;
  o.success = strcmp (cause, 'goal');
  o.dmin    = dmin;
  o.nsw     = nsw;
  o.t_end   = k * dt;
end

function [w1bar, w2und] = near_field_weights (P)
  dd = linspace (0, P.r_safety, 501);
  a = zeros (size (dd)); b = zeros (size (dd));
  for i = 1:numel (dd)
    [a(i), b(i)] = fapf_weights (dd(i), P);
  end
  w1bar = max (a);
  w2und = min (b);
end

function s = ternary (c, a, b)
  if (c), s = a; else, s = b; end
end
