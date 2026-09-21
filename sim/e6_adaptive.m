function res = e6_adaptive (varargin)
% E6: adaptive membership tuning -- Theorem thm:adaptive-membership-stability-section4.
%
%   (ii)  Adaptation preserves the convergence RATE.  Remark
%         rem:adapt-rate-correction explicitly retracts the earlier "<5% rate
%         loss" claim and asserts no rate loss at all, because the Lyapunov
%         function V = 0.5||qtil||^2 is weight-independent.  Part 1 below
%         measures the decay rate with adaptation OFF and ON.
%
%   (iii) Adaptation strengthens exactly one condition, the steering-gain
%         condition, through
%             D_adapt <= D + nu (k_a D_max + k_r G_r) / F_min,   Eq. (eq:D-adapt)
%         with nu = L_w (vmax_c + vmax_sigma).  Part 2 estimates L_w
%         numerically, measures D and D_adapt from the trajectories, and
%         checks the inequality.  Part 3 sweeps k_omega across the threshold
%         k_omega = D_adapt / sin(beta*) of Eq. (komega-condition).
%
% Options: 'seeds', 'nstart', 'nobs', 'dt', 'T_max', 'komegas'

  opt = struct ('seeds', 1:6, 'nstart', 6, 'nobs', 40, ...
                'dt', 2e-3, 'T_max', 60, 'komegas', []);
  for i = 1:2:numel (varargin), opt.(varargin{i}) = varargin{i+1}; end

  P = fapf_params ('dt', opt.dt, 'T_max', opt.T_max);
  beta_star = atan (P.w1_min / P.w1_max);      % lambda_-/lambda_+, k_a cancels

  printf ('\n=== E6  Adaptive membership tuning ===\n');
  printf ('beta* = atan(lambda_-/lambda_+) = %.4f rad,  sin(beta*) = %.4f\n\n', ...
          beta_star, sin (beta_star));

  % ---------- part 1: convergence rate, adaptation OFF vs ON -------------
  printf ('--- (1) convergence rate (Theorem ...(ii): no rate loss) ---\n');
  printf ('%-10s %-10s %-10s %-10s %-10s\n', ...
          'adapt', 'succ[%]', 'rate[1/s]', 't_goal[s]', 'minclr[m]');

  stats = struct ();
  for ad = [false, true]
    acc = collect (P, opt, ad);
    printf ('%-10s %-10.1f %-10.4f %-10.3f %-10.4f\n', ...
            ternary (ad, 'ON', 'OFF'), acc.succ, acc.rate, acc.t_mean, acc.clr);
    if (ad), stats.on = acc; else, stats.off = acc; end
  end
  dr = 100 * (stats.off.rate - stats.on.rate) / max (stats.off.rate, eps);
  printf ('rate change with adaptation = %+.2f %%  (theory predicts ~0)\n', -dr);
  printf ('lambda_- = w1_min*k_a = %.4f 1/s is the holonomic reference rate\n\n', ...
          P.w1_min * P.k_a);

  % ---------- part 2: the D_adapt bound ----------------------------------
  L_w = lipschitz_weights (P);
  nu  = L_w * (P.vmax_c + P.vmax_s);
  F_min = min (stats.off.Fmin, stats.on.Fmin);
  extra = nu * (P.k_a * P.D_max + P.k_r * P.G_r) / max (F_min, eps);
  D_meas  = stats.off.D;
  Da_meas = stats.on.D;
  Da_bnd  = D_meas + extra;

  printf ('--- (2) steering-gain condition under adaptation (Eq. eq:D-adapt) ---\n');
  printf ('  L_w (numeric Lipschitz const of (c,sigma) -> (w1,w2)) = %.4f\n', L_w);
  printf ('  nu = L_w (vmax_c + vmax_sigma)                        = %.4f 1/s\n', nu);
  printf ('  F_min observed on the operating region                = %.4f\n', F_min);
  printf ('  D      measured, adaptation OFF                       = %.4f 1/s\n', D_meas);
  printf ('  D_adapt measured, adaptation ON                       = %.4f 1/s\n', Da_meas);
  printf ('  D_adapt bound  D + nu(k_a D_max + k_r G_r)/F_min      = %.4f 1/s\n', Da_bnd);
  printf ('  bound respected: %s\n', ternary (Da_meas <= Da_bnd, 'YES', 'NO'));
  printf ('  required k_omega > D/sin(beta*)       = %.2f\n', D_meas / sin (beta_star));
  printf ('  required k_omega > D_adapt/sin(beta*) = %.2f   (implemented %.2f)\n\n', ...
          Da_meas / sin (beta_star), P.k_omega);

  % ---------- part 3: steering-gain sweep --------------------------------
  if (isempty (opt.komegas))
    kreq = max (1e-3, Da_meas / sin (beta_star));
    opt.komegas = kreq * [0.25, 0.5, 1.0, 2.0, 4.0];
  end
  printf ('--- (3) steering-gain sweep (Eq. komega-condition) ---\n');
  printf ('%-10s %-10s %-10s %-12s %-10s\n', ...
          'k_omega', 'cond', 'succ[%]', 'max|beta|', 'minclr[m]');
  sweep = [];
  for kw = opt.komegas
    Pk  = fapf_params ('dt', opt.dt, 'T_max', opt.T_max, 'k_omega', kw);
    acc = collect (Pk, opt, true);
    ok  = kw > Da_meas / sin (beta_star);
    printf ('%-10.2f %-10s %-10.1f %-12.4f %-10.4f\n', ...
            kw, ternary (ok, 'MET', 'unmet'), acc.succ, acc.beta_max, acc.clr);
    sweep(end+1,:) = [kw, ok, acc.succ, acc.beta_max, acc.clr];
  end

  res.P = P; res.beta_star = beta_star;
  res.off = stats.off; res.on = stats.on;
  res.L_w = L_w; res.nu = nu; res.F_min = F_min;
  res.D = D_meas; res.D_adapt = Da_meas; res.D_adapt_bound = Da_bnd;
  res.sweep = sweep;
  res.pass_rate_invariance = (abs (dr) < 5);
  res.pass_D_adapt_bound   = (Da_meas <= Da_bnd);
end

% =====================================================================

function acc = collect (P, opt, adapt)
% Run the unicycle model over the benchmark set and aggregate the quantities
% needed by parts 1-3.

  nrun = 0; nsucc = 0; clr = inf; ts = [];
  rates = []; Dmax_seen = 0; Fmin = inf; beta_max = 0;

  for s = opt.seeds
    env = fapf_env (opt.nobs, s, P, 'random');
    for a = linspace (0, 2*pi, opt.nstart+1)(1:opt.nstart)
      q0 = env.goal + 0.9 * P.D_max * [cos(a), sin(a)];
      if (fapf_dobs (q0, env.obs) < P.r_safety), continue; end
      th0 = atan2 (env.goal(2)-q0(2), env.goal(1)-q0(1));
      out = fapf_sim_nonholo (q0, th0, env, P, struct ('adapt', adapt));

      nrun++;
      nsucc += out.success;
      clr = min (clr, out.dmin);
      if (out.success), ts(end+1) = out.t_end; end

      lg = out.log;
      t = lg(:,1); e = lg(:,2); nF = lg(:,3); thd = lg(:,4);
      beta_max = max (beta_max, max (abs (lg(:,8))));

      % Assumption ass:theta-d-rate is asserted on the region where
      % ||F|| >= F_min > 0, i.e. outside the residual ball.
      m = (e > 1.2 * P.R) | (nF > 0.2);
      if (sum (m) > 10)
        Fmin = min (Fmin, min (nF(m)));
        dth = wrap (diff (thd(m)));
        dtv = diff (t(m));
        ok  = dtv > 0;
        if (any (ok))
          Dmax_seen = max (Dmax_seen, quantile_ (abs (dth(ok) ./ dtv(ok)), 0.99));
        end
      end

      r = decay_rate (t, e, P);
      if (!isnan (r)), rates(end+1) = r; end
    end
  end

  acc.succ     = 100 * nsucc / max (nrun, 1);
  acc.rate     = mean_ (rates);
  acc.t_mean   = mean_ (ts);
  acc.clr      = clr;
  acc.D        = Dmax_seen;
  acc.Fmin     = Fmin;
  acc.beta_max = beta_max;
  acc.nrun     = nrun;
end

function r = decay_rate (t, e, P)
% Least-squares slope of log||qtil|| over the exponential phase, i.e. while
% the error is still well outside the residual ball of radius R.
  m = (e > max (2 * P.rho, 0.3 * e(1))) & (e > 0);
  if (sum (m) < 20), r = NaN; return; end
  c = polyfit (t(m), log (e(m)), 1);
  r = -c(1);
end

function L = lipschitz_weights (P)
% Numeric Lipschitz constant of (c_1..c_N, sigma) -> (w1, w2), via the
% largest singular value of the Jacobian over a distance grid.
  dd = linspace (0, P.d_meas, 121);
  hstep = 1e-5;
  L = 0;
  for i = 1:numel (dd)
    J = zeros (2, P.N + 1);
    for j = 1:P.N
      cp = P.centers; cp(j) += hstep;
      cm = P.centers; cm(j) -= hstep;
      [a1, b1] = fapf_weights (dd(i), P, cp, P.sigma);
      [a2, b2] = fapf_weights (dd(i), P, cm, P.sigma);
      J(:,j) = [a1-a2; b1-b2] / (2*hstep);
    end
    [a1, b1] = fapf_weights (dd(i), P, P.centers, P.sigma + hstep);
    [a2, b2] = fapf_weights (dd(i), P, P.centers, P.sigma - hstep);
    J(:,end) = [a1-a2; b1-b2] / (2*hstep);
    L = max (L, max (svd (J)));
  end
end

function a = wrap (x)
  a = atan2 (sin (x), cos (x));
end

function m = mean_ (x)
  if (isempty (x)), m = NaN; else, m = mean (x); end
end

function v = quantile_ (x, p)
% Plain empirical quantile; avoids depending on the statistics package.
  x = sort (x(:));
  if (isempty (x)), v = 0; return; end
  v = x(max (1, min (numel (x), round (p * numel (x)))));
end

function s = ternary (c, a, b)
  if (c), s = a; else, s = b; end
end
