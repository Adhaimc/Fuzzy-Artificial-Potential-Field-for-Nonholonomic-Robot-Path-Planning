function res = run_all (varargin)
% Runs E1-E6 and prints a consolidated PASS/FAIL table against the claims of
% the revised theory.  Pass 'quick', true for a reduced-cost sanity run.

  opt = struct ('quick', false, 'only', []);
  for i = 1:2:numel (varargin), opt.(varargin{i}) = varargin{i+1}; end

  if (opt.quick)
    a3 = {'seeds', 1:2, 'nstart', 4, 'nobs', 25, 'dt', 5e-3, 'T_max', 25};
    a4 = {'dts', [5e-4 1e-3 5e-3 1e-2], 'seeds', 1:2, 'nobs', 25, 'T_max', 20};
    a5 = {'seeds', 1:2, 'nstart', 4, 'nobs', 25, 'dt', 5e-3, 'T_max', 30, ...
          'Ls', [9, 5, 3]};
    a6 = {'seeds', 1:2, 'nstart', 3, 'nobs', 25, 'dt', 5e-3, 'T_max', 30};
  else
    a3 = {}; a4 = {}; a5 = {}; a6 = {};
  end

  want = @(n) isempty (opt.only) || any (opt.only == n);
  res = struct ();

  if (want (1)), res.e1 = e1_constants (); end
  if (want (2)), res.e2 = e2_localisation (); end
  if (want (3)), res.e3 = e3_barrier (a3{:}); end
  if (want (4)), res.e4 = e4_timestep (a4{:}); end
  if (want (5)), res.e5 = e5_subgoal (a5{:}); end
  if (want (6)), res.e6 = e6_adaptive (a6{:}); end

  printf ('\n\n================ CONSOLIDATED SUMMARY ================\n');
  printf ('%-46s %-8s\n', 'claim', 'verdict');
  printf ('%s\n', repmat ('-', 1, 56));

  if (isfield (res, 'e1'))
    row ('E1  G_r matches closed form (Table tab:repulsive-models)', ...
         abs (res.e1.P.G_r - (res.e1.P.d0 - res.e1.P.r_safety) / ...
              (res.e1.P.r_safety^2 + res.e1.P.eps_reg)) < 1e-9);
    row ('E1  design window empty at global scale (rem:design-window)', ...
         res.e1.window_empty_global);
    row ('E1  design window opens at L = 3 m (rem:design-window)', ...
         res.e1.window_open_subgoal);
    row ('E1  phi(d) >= 0 on all of [r_safety,d0] (prop:cbf-validity)', ...
         res.e1.pass_phi);
  end
  if (isfield (res, 'e2'))
    row ('E2  equilibria confined to ||qtil|| <= R (thm:no-local-minima)', ...
         res.e2.pass);
  end
  if (isfield (res, 'e3'))
    row ('E3  d_obs >= r_safety on all trajectories (thm:collision-safety a)', ...
         res.e3.holo.nviol == 0);
    row ('E3  barrier margin >= 0 with Eq. (alpha0-choice)', ...
         res.e3.holo.margin(1) >= -1e-9);
    row ('E3  barrier margin >= 0 with interior-valid alpha0', ...
         res.e3.holo.margin(2) >= -1e-9);
  end
  if (isfield (res, 'e4'))
    row ('E4  penetration <= v_max*dt (thm:collision-safety b)', ...
         res.e4.bound_respected);
    row ('E4  penetration first order in dt (supercritical case)', ...
         res.e4.slope_ok);
  end
  if (isfield (res, 'e5'))
    row ('E5  switch count <= ceil(D_max/Delta) (thm:gnron-hybrid i)', ...
         all (res.e5.rows(:,7) <= res.e5.switch_bound));
    row ('E5  safety preserved across switches (thm:gnron-hybrid ii)', ...
         all (res.e5.rows(:,8) == 0));
    row ('E5  success improves as horizon L decreases', ...
         res.e5.rows(end,4) >= res.e5.rows(1,4));
  end
  if (isfield (res, 'e6'))
    row ('E6  no convergence-rate loss under adaptation (thm ii)', ...
         res.e6.pass_rate_invariance);
    row ('E6  D_adapt within Eq. (eq:D-adapt) bound (thm iii)', ...
         res.e6.pass_D_adapt_bound);
  end
  printf ('%s\n', repmat ('=', 1, 56));
end

function row (txt, ok)
  printf ('%-46s %-8s\n', txt, ternary (ok, 'PASS', 'FAIL'));
end

function s = ternary (c, a, b)
  if (c), s = a; else, s = b; end
end
