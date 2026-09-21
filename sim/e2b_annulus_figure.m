function e2b_annulus_figure ()
% Standalone figure: equilibrium localisation annulus (Theorem
% thm:no-local-minima-fuzzy / Corollary cor:design-rule), comparing the
% implemented gain ratio k_a/k_r = 0.6 (vacuous R) against the tuned ratio
% k_a/k_r = 4.8 (R_target = 4 m). Does not touch anything under
% sim/figs/E*.png or the Experiments/matlab pipeline.

  printf ('\n=== E2b  Annulus figure at implemented vs tuned gain ratio ===\n');

  seed = 3; nobs = 60; k_r = 2.0;
  ratios = [0.6, 4.8];
  names  = {'Implemented (k_a/k_r = 0.6)', 'Tuned (k_a/k_r = 4.8)'};

  fig = figure ('visible', 'off', 'position', [0 0 1200 620]);

  for j = 1:2
    k_a = ratios(j) * k_r;
    P   = fapf_params ('k_a', k_a, 'k_r', k_r);
    env = fapf_env (nobs, seed, P, 'random');
    [radii, pts] = find_equilibria (env, P);

    subplot ('position', [0.06 + (j-1)*0.50, 0.10, 0.40, 0.68]);
    hold on;
    th = linspace (0, 2*pi, 40);
    for i = 1:size (env.obs, 1)
      fill (env.obs(i,1) + env.obs(i,3)*cos(th), ...
            env.obs(i,2) + env.obs(i,3)*sin(th), ...
            [0.75 0.75 0.8], 'EdgeColor', 'none');
    end

    tt = linspace (0, 2*pi, 200);
    R_draw = min (P.R, 1.15 * P.D_max);
    plot (env.goal(1) + R_draw*cos(tt), env.goal(2) + R_draw*sin(tt), ...
          'r--', 'linewidth', 2);
    plot (env.goal(1) + env.eps_clear*cos(tt), env.goal(2) + env.eps_clear*sin(tt), ...
          'r:', 'linewidth', 1.5);

    if (~isempty (pts))
      plot (pts(:,1), pts(:,2), 'k.', 'markersize', 14);
    end
    plot (env.goal(1), env.goal(2), 'p', 'markersize', 16, ...
          'markerfacecolor', [1 0.6 0], 'markeredgecolor', 'k');

    axis equal; axis (1.05*P.D_max*[-1 1 -1 1]); grid on; box on;
    vacuous = P.R > P.D_max;
    ttl = sprintf ('%s,  R = %.1f m%s', names{j}, P.R, ...
                   ternary (vacuous, ' (vacuous)', ''));
    title (ttl, 'interpreter', 'none', 'fontsize', 11);
    xlabel ('x [m]'); ylabel ('y [m]');
  end

  ax = axes ('position', [0 0.90 1 0.09], 'visible', 'off');
  xlim (ax, [0 1]); ylim (ax, [0 1]);
  text (ax, 0.5, 0.78, ...
        'Equilibrium localisation annulus: implemented vs. tuned gain ratio', ...
        'horizontalalignment', 'center', 'fontsize', 13, 'fontweight', 'bold');
  text (ax, 0.5, 0.22, ...
        'grey = obstacles;  orange = goal;  black = equilibria;  dashed/dotted red = annulus [\epsilon, R]', ...
        'horizontalalignment', 'center', 'fontsize', 9);

  outdir = 'figs';
  if (~exist (outdir, 'dir')), mkdir (outdir); end
  print (fullfile (outdir, 'E2b_annulus_comparison.png'), '-dpng', '-r150');
  close (fig);
  printf ('Saved figs/E2b_annulus_comparison.png\n');
end

% -------------------------------------------------------------------------
function [radii, pts] = find_equilibria (env, P)
% Grid search for zeros of F on the safe set (same method as e2_localisation.m).
  g  = linspace (-P.D_max, P.D_max, 221);
  [X, Y] = meshgrid (g, g);
  NF = inf (size (X));

  for i = 1:numel (X)
    q = [X(i), Y(i)];
    if (norm (q - env.goal) > P.D_max), continue; end
    [d, ~] = fapf_dobs (q, env.obs);
    if (d <= 0), continue; end
    F = fapf_force (q, env.goal, env.obs, P);
    NF(i) = norm (F);
  end

  radii = []; pts = [];
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
        if (norm (F) < 1e-3 && r > P.rho && dq >= P.r_safety)
          radii(end+1) = r; pts(end+1,:) = q;
        end
      end
    end
  end
end

function q = refine (q0, env, P)
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
