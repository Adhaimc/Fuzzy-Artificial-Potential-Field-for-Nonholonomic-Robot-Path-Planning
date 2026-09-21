function res = e1_constants ()
% E1: verify the closed-form constants asserted in the revised theory.
%   - G_r from Eq. (Gr-def) and Table tab:repulsive-models
%   - residual radius R, Eq. (residual-radius)
%   - design window, Eq. (design-window), at global and sub-goal scale
%   - scalar barrier condition phi(d) >= 0, Eq. (cbf-scalar-condition)

  P = fapf_params ();
  printf ('\n=== E1  Theory constants ===\n');

  Gr_cf = (P.d0 - P.r_safety) / (P.r_safety^2 + P.eps_reg);
  printf ('G_r  numeric = %.4f   closed form = %.4f   (diff %.2e)\n', ...
          P.G_r, Gr_cf, abs (P.G_r - Gr_cf));

  printf ('R    = w2max*k_r*G_r/(w1min*k_a) = %.2f m   (workspace radius %.1f m)\n', ...
          P.R, P.D_max);
  if (P.R > P.D_max)
    printf ('       -> VACUOUS at these gains, as stated in rem:R-numerical\n');
  end

  % ---- design window ----------------------------------------------------
  bar_w1    = P.w1_min;      % w1 is increasing in d, so near-field max is w1_min
  underw2   = P.w2_max;      % w2 is decreasing in d, so near-field min is w2_max
  R_target  = 4.0;
  lo = P.w2_max * P.G_r / (P.w1_min * R_target);
  hi_glob = underw2 * P.G_r / (bar_w1 * P.D_max);
  L = 3.0;
  hi_sub  = underw2 * P.G_r / (bar_w1 * L);

  printf ('\nDesign window on k_a/k_r  (R_target = %.1f m):\n', R_target);
  printf ('  convergence lower bound      = %.3f\n', lo);
  printf ('  safety upper bound, D_max=%.1f = %.3f   -> window %s\n', ...
          P.D_max, hi_glob, ternary (lo <= hi_glob, 'NON-EMPTY', 'EMPTY'));
  printf ('  safety upper bound, L    =%.1f = %.3f   -> window %s\n', ...
          L, hi_sub, ternary (lo <= hi_sub, 'NON-EMPTY', 'EMPTY'));
  printf ('  implemented ratio k_a/k_r    = %.3f  (satisfies safety bound: %d)\n', ...
          P.k_a/P.k_r, (P.k_a/P.k_r) <= hi_glob);

  % ---- scalar barrier condition ----------------------------------------
  dd  = linspace (P.r_safety, P.d0, 20001);
  phi = zeros (size (dd));
  for i = 1:numel (dd)
    [w1, w2] = fapf_weights (dd(i), P);
    phi(i) = w2 * P.k_r * fapf_gradUr_mag (dd(i), P) ...
           + P.alpha0 * (dd(i) - P.r_safety) ...
           - w1 * P.k_a * P.D_max;
  end
  [phi_min, im] = min (phi);
  printf ('\nBarrier condition (alpha0 = %.2f 1/s from Eq. alpha0-choice):\n', P.alpha0);
  printf ('  min phi(d) = %.4f at d = %.3f m  -> %s\n', ...
          phi_min, dd(im), ternary (phi_min >= 0, 'SATISFIED', 'VIOLATED'));

  % endpoint values only -- what prop:cbf-validity actually checks
  printf ('  phi(r_safety) = %.4f ,  phi(d0) = %.4f  (endpoints OK: %d)\n', ...
          phi(1), phi(end), (phi(1) >= 0) && (phi(end) >= -1e-9));

  % smallest alpha0 for which phi >= 0 on the whole interval
  num = zeros (size (dd)); 
  for i = 1:numel (dd)
    [w1, w2] = fapf_weights (dd(i), P);
    num(i) = w1 * P.k_a * P.D_max - w2 * P.k_r * fapf_gradUr_mag (dd(i), P);
  end
  msk = dd > P.r_safety + 1e-9;
  a_req = max (num(msk) ./ (dd(msk) - P.r_safety));
  printf ('  alpha0 required for interior validity = %.2f 1/s  (factor %.2f larger)\n', ...
          a_req, a_req / P.alpha0);

  res.P = P; res.phi = phi; res.dd = dd;
  res.phi_min = phi_min; res.d_argmin = dd(im);
  res.alpha0_required = a_req;
  res.lo = lo; res.hi_glob = hi_glob; res.hi_sub = hi_sub;
  res.pass_phi = (phi_min >= 0);
  res.window_empty_global = (lo > hi_glob);
  res.window_open_subgoal = (lo <= hi_sub);
end

function s = ternary (c, a, b)
  if (c), s = a; else, s = b; end
end
