function res = e3_barrier (varargin)
% E3: closed-loop verification of collision safety and the barrier certificate.
%
%   Theorem thm:collision-safety(a)  -- forward invariance of  S = {d_obs >= r_safety}
%   Proposition prop:cbf-validity    -- min_t [ hdot + alpha0 h ] >= 0
%
% The barrier margin is evaluated for BOTH candidate class-K gains:
%   alpha0_formula  = Eq. (alpha0-choice), certified only at the two endpoints
%   alpha0_required = the true supremum over [r_safety, d0]
% so that the endpoint-vs-interior gap found in E1 is exposed on trajectories
% rather than only on the scalar function phi.
%
% Options: 'seeds', 'nobs', 'nstart', 'dt', 'T_max', 'model' ('holo'|'nonholo'|'both')

  opt = struct ('seeds', 1:5, 'nobs', 40, 'nstart', 8, ...
                'dt', 2e-3, 'T_max', 40, 'model', 'both');
  for i = 1:2:numel (varargin), opt.(varargin{i}) = varargin{i+1}; end

  P = fapf_params ('dt', opt.dt, 'T_max', opt.T_max);
  alphas = [P.alpha0_formula, P.alpha0_required];

  printf ('\n=== E3  Safety and barrier verification along trajectories ===\n');
  printf ('r_safety = %.2f m,  d0 = %.2f m,  k_a/k_r = %.2f,  dt = %.0e s\n', ...
          P.r_safety, P.d0, P.k_a/P.k_r, opt.dt);

  % --- does the design satisfy the hypothesis of thm:collision-safety(a)? ---
  Gr_safe = fapf_gradUr_mag (P.r_safety, P);
  [w1b, w2u] = near_field_weights (P);
  lhs = w2u * P.k_r * Gr_safe;
  rhs = w1b * P.k_a * P.D_max;
  printf ('Safety gain condition  w2_ * k_r * Gr_safe >= w1^ * k_a * D_max :\n');
  printf ('  %.3f >= %.3f  -> %s   (bound on k_a/k_r = %.3f)\n\n', ...
          lhs, rhs, ternary (lhs >= rhs, 'HOLDS', 'FAILS'), ...
          w2u * Gr_safe / (w1b * P.D_max));

  models = {};
  switch (opt.model)
    case 'holo',    models = {'holo'};
    case 'nonholo', models = {'nonholo'};
    otherwise,      models = {'holo', 'nonholo'};
  end

  res.P = P;
  for im = 1:numel (models)
    md = models{im};
    nrun = 0; nsucc = 0; clear_min = inf; nviol = 0;
    marg = inf (1, numel (alphas));
    e_end_max = 0; causes = struct ('goal',0,'stall',0,'timeout',0,'diverged',0);

    for s = opt.seeds
      env = fapf_env (opt.nobs, s, P, 'random');
      for a = linspace (0, 2*pi, opt.nstart+1)(1:opt.nstart)
        q0 = env.goal + 0.9 * P.D_max * [cos(a), sin(a)];
        if (fapf_dobs (q0, env.obs) < P.r_safety), continue; end   % h(q0) >= 0

        if (strcmp (md, 'holo'))
          out = fapf_sim_holo (q0, env, P);
        else
          th0 = atan2 (env.goal(2)-q0(2), env.goal(1)-q0(1));
          out = fapf_sim_nonholo (q0, th0, env, P);
        end

        nrun++;
        nsucc += out.success;
        causes.(out.cause)++;
        clear_min = min (clear_min, out.dmin);
        nviol += (out.dmin < P.r_safety - 1e-9);
        e_end_max = max (e_end_max, out.e_end);

        h = out.log(:,6); hdot = out.log(:,7);
        for ia = 1:numel (alphas)
          marg(ia) = min (marg(ia), min (hdot + alphas(ia) * h));
        end
      end
    end

    printf ('--- model: %s ---\n', md);
    printf ('  runs = %d,  reached terminal ball = %d (%.0f%%)\n', ...
            nrun, nsucc, 100*nsucc/max(nrun,1));
    printf ('  causes: goal %d | stall %d | timeout %d | diverged %d\n', ...
            causes.goal, causes.stall, causes.timeout, causes.diverged);
    printf ('  min clearance over all runs = %.4f m  (r_safety = %.2f)  -> %s\n', ...
            clear_min, P.r_safety, ternary (nviol == 0, 'SAFE', 'VIOLATED'));
    printf ('  runs breaching r_safety     = %d\n', nviol);
    printf ('  barrier margin, alpha0 = %6.2f (Eq. alpha0-choice)   = %+8.4f  -> %s\n', ...
            alphas(1), marg(1), ternary (marg(1) >= -1e-9, 'PASS', 'FAIL'));
    printf ('  barrier margin, alpha0 = %6.2f (interior supremum)   = %+8.4f  -> %s\n\n', ...
            alphas(2), marg(2), ternary (marg(2) >= -1e-9, 'PASS', 'FAIL'));

    r.nrun = nrun; r.nsucc = nsucc; r.clear_min = clear_min; r.nviol = nviol;
    r.margin = marg; r.causes = causes; r.e_end_max = e_end_max;
    res.(md) = r;
  end

  res.alphas = alphas;
  res.safety_condition_holds = (lhs >= rhs);
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

function s = ternary (c, a, b)
  if (c), s = a; else, s = b; end
end
