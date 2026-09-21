function res = e4_timestep (varargin)
% E4: sampled-data penetration study for Theorem thm:collision-safety(b),
% which asserts  h(t) >= -v_max * dt:  any excursion below the safe set is
% bounded by one sample of travel and therefore vanishes linearly in dt.
%
% The worst case of the proof is reproduced as closely as a moving trajectory
% allows: the robot starts at the maximal goal distance D_max with a single
% obstacle directly between it and the goal, so it enters the influence region
% travelling radially inward at the largest speed the attractive term can
% command.
%
% Four settings are reported:
%   (A) implemented gains -- the continuous-time margin at h = 0 is large, so
%       the robot is turned away well before the boundary and no penetration
%       occurs at any step size; the sampled-data term is never exercised;
%   (B) NEAR-CRITICAL gains -- k_a at a fraction 'margin_frac' of the value
%       at which Eq. (safety-gain-condition) holds with equality;
%   (C) SUPERCRITICAL gains -- k_a a factor 'over_frac' above that value, so
%       Eq. (safety-gain-condition) FAILS.  Part (a) then gives no guarantee
%       and penetration is genuinely produced, which is the only regime in
%       which the one-sample bound of part (b) is a binding constraint and
%       the predicted slope of 1 can be measured at all;
%   (D) random fields at the implemented gains, to confirm (A) generically.
%
% Options: 'dts', 'seeds', 'nobs', 'T_max', 'margin_frac', 'over_frac'

  opt = struct ('dts', [1e-4 2e-4 5e-4 1e-3 2e-3 5e-3 1e-2], ...
                'seeds', 1:4, 'nobs', 40, 'T_max', 30, ...
                'margin_frac', 0.98, 'over_frac', 3.0);
  for i = 1:2:numel (varargin), opt.(varargin{i}) = varargin{i+1}; end

  P0 = fapf_params ();
  Gr_safe = fapf_gradUr_mag (P0.r_safety, P0);
  [w1bar, w2und] = near_field_weights (P0);
  k_a_crit = w2und * P0.k_r * Gr_safe / (w1bar * P0.D_max);

  printf ('\n=== E4  Sampled-data penetration vs integration step ===\n');
  printf ('Theory: pen := max(0, r_safety - min_t d_obs) <= v_max * dt.\n');
  printf ('near-field weights  w1^ = %.4f,  w2_ = %.4f,  Gr_safe = %.4f\n', ...
          w1bar, w2und, Gr_safe);
  printf ('safety bound on k_a/k_r = %.4f  ->  critical k_a = %.4f ', ...
          w2und * Gr_safe / (w1bar * P0.D_max), k_a_crit);
  printf ('(implemented %.2f)\n\n', P0.k_a);

  % ---------------- (A), (B), (C): single-obstacle radial approach -------
  labels = {'A  implemented gains', 'B  near-critical gains', ...
            'C  supercritical gains (safety condition FAILS)'};
  kas    = [P0.k_a, opt.margin_frac * k_a_crit, opt.over_frac * k_a_crit];
  out_ab = cell (1, numel (kas));
  slopes = nan (1, numel (kas));

  for c = 1:numel (kas)
    printf ('--- (%s):  k_a = %.4f, k_a/k_r = %.4f ---\n', ...
            labels{c}, kas(c), kas(c)/P0.k_r);
    printf ('%-10s %-13s %-13s %-11s %-8s\n', ...
            'dt [s]', 'pen [m]', 'v_max*dt', 'Fmax', 'bound');
    tbl = zeros (numel (opt.dts), 4);
    for i = 1:numel (opt.dts)
      dt  = opt.dts(i);
      P   = fapf_params ('k_a', kas(c), 'dt', dt, 'T_max', opt.T_max);
      [q0, env] = worst_case_env (P);
      o   = fapf_sim_holo (q0, env, P);
      Fmax = max (o.log(:,3));
      pen  = max (0, P.r_safety - o.dmin);
      if (pen < 1e-12), pen = 0; end          % suppress round-off at h(0) = 0
      printf ('%-10.0e %-13.4e %-13.4e %-11.4f %-8s\n', ...
              dt, pen, Fmax*dt, Fmax, ternary (pen <= Fmax*dt + 1e-12, 'OK', 'BROKEN'));
      tbl(i,:) = [dt, pen, Fmax*dt, Fmax];
    end
    slopes(c) = loglog_slope (tbl(:,1), tbl(:,2));
    if (isnan (slopes(c)))
      printf ('  no penetration at any step size -- the continuous-time\n');
      printf ('  margin of thm:collision-safety(a) is never exhausted.\n\n');
    else
      printf ('  fitted slope d log(pen) / d log(dt) = %.3f  (theory: 1)\n\n', slopes(c));
    end
    out_ab{c} = tbl;
  end

  % ---------------- (D) random fields ------------------------------------
  printf ('--- (D) random fields, implemented gains, %d seeds x 8 starts ---\n', ...
          numel (opt.seeds));
  printf ('%-10s %-13s %-13s %-11s %-8s\n', ...
          'dt [s]', 'pen [m]', 'v_max*dt', 'Fmax', 'bound');
  tc = zeros (numel (opt.dts), 4);
  for i = 1:numel (opt.dts)
    dt = opt.dts(i);
    P  = fapf_params ('dt', dt, 'T_max', opt.T_max);
    pen = 0; Fmax = 0;
    for s = opt.seeds
      env = fapf_env (opt.nobs, s, P, 'random');
      for a = linspace (0, 2*pi, 9)(1:8)
        q0 = env.goal + 0.9 * P.D_max * [cos(a), sin(a)];
        if (fapf_dobs (q0, env.obs) < P.r_safety), continue; end
        o = fapf_sim_holo (q0, env, P);
        pen  = max (pen, max (0, P.r_safety - o.dmin));
        Fmax = max (Fmax, max (o.log(:,3)));
      end
    end
    if (pen < 1e-12), pen = 0; end
    printf ('%-10.0e %-13.4e %-13.4e %-11.4f %-8s\n', ...
            dt, pen, Fmax*dt, Fmax, ternary (pen <= Fmax*dt + 1e-12, 'OK', 'BROKEN'));
    tc(i,:) = [dt, pen, Fmax*dt, Fmax];
  end
  sc = loglog_slope (tc(:,1), tc(:,2));
  if (isnan (sc))
    printf ('  no penetration at any step size.\n\n');
  else
    printf ('  fitted slope d log(pen) / d log(dt) = %.3f  (theory: 1)\n\n', sc);
  end

  res.implemented   = out_ab{1};
  res.near_critical = out_ab{2};
  res.supercritical = out_ab{3};
  res.random        = tc;
  res.k_a_crit      = k_a_crit;
  res.slope_implemented   = slopes(1);
  res.slope_near_critical = slopes(2);
  res.slope_supercritical = slopes(3);
  res.slope_random        = sc;
  res.bound_respected = all (out_ab{1}(:,2) <= out_ab{1}(:,3) + 1e-12) && ...
                        all (out_ab{2}(:,2) <= out_ab{2}(:,3) + 1e-12) && ...
                        all (out_ab{3}(:,2) <= out_ab{3}(:,3) + 1e-12) && ...
                        all (tc(:,2) <= tc(:,3) + 1e-12);
  % the scaling law is only measurable where penetration is actually non-zero
  res.slope_testable = !isnan (slopes(3));
  res.slope_ok = !res.slope_testable || abs (slopes(3) - 1) < 0.35;

  printf ('One-sample bound  pen <= v_max*dt  respected everywhere: %s\n', ...
          ternary (res.bound_respected, 'YES', 'NO'));
  printf ('First-order scaling measurable (supercritical case): %s', ...
          ternary (res.slope_testable, 'YES', 'NO'));
  if (res.slope_testable)
    printf ('  (slope %.3f)', slopes(3));
  end
  printf ('\n');
end

% =====================================================================

function [q0, env] = worst_case_env (P)
% Single obstacle on the robot-goal line, with the robot starting at the
% maximal goal distance D_max and OUTSIDE the influence region, so it enters
% that region travelling radially inward: the approach in which the bound of
% thm:collision-safety(a) is tightest.
  r_obs = 0.30;
  env.goal = [0, 0];
  q0 = [-P.D_max, 0];
  u  = (env.goal - q0) / norm (env.goal - q0);
  env.obs = [q0 + (P.d0 + r_obs + 1.0) * u, r_obs];
end

function [w1bar, w2und] = near_field_weights (P)
% Eq. (near-field-weights): extremes of w1, w2 over d in [0, r_safety].
  dd = linspace (0, P.r_safety, 501);
  a = zeros (size (dd)); b = zeros (size (dd));
  for i = 1:numel (dd)
    [a(i), b(i)] = fapf_weights (dd(i), P);
  end
  w1bar = max (a);
  w2und = min (b);
end

function s = loglog_slope (x, y)
  m = (y > 0);
  if (sum (m) < 2), s = NaN; return; end
  c = polyfit (log (x(m)), log (y(m)), 1);
  s = c(1);
end

function s = ternary (c, a, b)
  if (c), s = a; else, s = b; end
end
