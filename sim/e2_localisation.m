function res = e2_localisation ()
% E2: test Theorem thm:no-local-minima-fuzzy.
%   (a) locate equilibria of F on a fine grid and check they lie in the
%       annulus  eps <= ||q - goal|| <= R   (Eq. annulus);
%   (b) sweep k_a/k_r and compare the measured maximum equilibrium radius
%       against the predicted residual radius R (Corollary cor:design-rule).

  printf ('\n=== E2  Equilibrium localisation and gain sweep ===\n');

  ratios = [0.6, 1.36, 2.5, 4.8, 10.0];
  k_r    = 2.0;
  seeds  = 1:8;
  nobs   = 60;

  printf ('Equilibria are sought on the SAFE SET d_obs >= r_safety, where the\n');
  printf ('gradient bound G_r of property (P3) is valid.\n\n');
  printf ('%-8s %-8s %-10s %-12s %-12s %-8s %-8s\n', ...
          'k_a/k_r', 'k_a', 'R_pred[m]', 'max|q*|[m]', '#eq(safe)', 'inside', '#eq(uns)');
  rows = [];

  for ir = 1:numel (ratios)
    k_a = ratios(ir) * k_r;
    P   = fapf_params ('k_a', k_a, 'k_r', k_r);

    maxr = 0; total = 0; allin = true; n_unsafe = 0;
    for s = seeds
      env = fapf_env (nobs, s, P, 'random');
      [r, ~, runsafe] = find_equilibria (env, P);
      total += numel (r);
      n_unsafe += numel (runsafe);
      if (~isempty (r))
        maxr = max (maxr, max (r));
        if (max (r) > P.R + 1e-6), allin = false; end
      end
    end

    printf ('%-8.2f %-8.2f %-10.2f %-12.3f %-12d %-8s %-8d\n', ...
            ratios(ir), k_a, P.R, maxr, total, ternary (allin, 'YES', 'NO'), n_unsafe);
    rows(end+1,:) = [ratios(ir), k_a, P.R, maxr, total, allin, n_unsafe];
  end

  res.rows = rows;
  res.header = {'ratio','k_a','R_pred','max_eq_radius','n_eq','all_inside'};
  res.pass = all (rows(:,6) == 1);
  printf ('\nLocalisation bound respected in every case: %s\n', ...
          ternary (res.pass, 'YES', 'NO'));

  % how informative is the bound?
  printf ('Bound is non-vacuous (R < workspace radius 9 m) for ratio >= %.2f\n', ...
          min (rows(rows(:,3) < 9, 1)));
end

% -------------------------------------------------------------------------
function [radii, pts, radii_unsafe] = find_equilibria (env, P)
% Grid search for zeros of F.  Coarse scan for local minima of ||F||,
% then refinement by local descent.  Equilibria are classified by whether
% they lie in the safe set d_obs >= r_safety, since the bound G_r of (P3)
% -- and hence the residual radius R -- is only valid there.

  g  = linspace (-P.D_max, P.D_max, 221);
  [X, Y] = meshgrid (g, g);
  NF = inf (size (X));

  for i = 1:numel (X)
    q = [X(i), Y(i)];
    if (norm (q - env.goal) > P.D_max), continue; end
    [d, ~] = fapf_dobs (q, env.obs);
    if (d <= 0), continue; end               % inside an obstacle
    F = fapf_force (q, env.goal, env.obs, P);
    NF(i) = norm (F);
  end

  radii = []; pts = []; radii_unsafe = [];
  [nr, nc] = size (NF);
  for a = 2:nr-1
    for b = 2:nc-1
      v = NF(a,b);
      if (~isfinite (v) || v > 0.5), continue; end
      w = NF(a-1:a+1, b-1:b+1);
      if (v <= min (w(:)) + 1e-12)
        q = refine ([X(a,b), Y(a,b)], env, P);
        if (isempty (q)), continue; end
        F = fapf_force (q, env.goal, env.obs, P);
        r = norm (q - env.goal);
        [dq, ~] = fapf_dobs (q, env.obs);
        if (norm (F) < 1e-3 && r > P.rho)
          if (dq >= P.r_safety)
            radii(end+1) = r; pts(end+1,:) = q;
          else
            radii_unsafe(end+1) = r;
          end
        end
      end
    end
  end
end

function q = refine (q0, env, P)
% Coordinate descent on ||F|| with shrinking step.
  q = q0; step = 0.05;
  fv = norm (fapf_force (q, env.goal, env.obs, P));
  for it = 1:400
    improved = false;
    for dvec = [1 0; -1 0; 0 1; 0 -1]'
      qn = q + step * dvec.';
      [d, ~] = fapf_dobs (qn, env.obs);
      if (d <= 0 || norm (qn - env.goal) > P.D_max), continue; end
      fn = norm (fapf_force (qn, env.goal, env.obs, P));
      if (fn < fv), q = qn; fv = fn; improved = true; end
    end
    if (~improved)
      step = step / 2;
      if (step < 1e-6), break; end
    end
  end
end

function s = ternary (c, a, b)
  if (c), s = a; else, s = b; end
end
