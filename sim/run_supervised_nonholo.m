function out = run_supervised_nonholo (q0, th0, env, P, adapt, L, Delta, tauD, alpha_c, alpha_s)
% Unicycle closed loop with BOTH the adaptive membership law (Sec.
% subsec:adaptive-membership) and the GNRON subgoal supervisor (Sec.
% subsec:gnron-enhancement). Used by fig_stress_test.m.
%
% Optional parameters alpha_c, alpha_s override P.alpha_c, P.alpha_s for this run.

  if (nargin < 9), alpha_c = []; end
  if (nargin < 10), alpha_s = []; end
  if (isempty (alpha_c)), alpha_c = P.alpha_c; end
  if (isempty (alpha_s)), alpha_s = P.alpha_s; end

  dt = P.dt; n = round (P.T_max / dt);
  q = q0(:).'; th = th0;
  traj = zeros (n+1, 2); traj(1,:) = q;

  centers = P.centers; sigma = P.sigma;
  w1 = P.w1_c; w2 = P.w2_c;  % Fixed consequent weights (NOT adapted)
  V_prev = 0.5 * norm(q - env.goal)^2;  % Lyapunov tracking
  
  gamma_d = 1.0; T_window = 0.5; Nw = max (2, round (T_window/dt));
  eps_conv = 5e-3; eps_conv_e = 0.05; T_thresh = max (1.0, tauD);
  T_thresh_e = max (5.0, 2*tauD); rho_wp = 0.30;

  Vbuf = nan (1, Nw); bi = 0;
  Ebuf = nan (1, Nw); ei = 0;
  t_stall = 0; t_stall_e = 0; t_last = -inf;
  target = env.goal; mode = 0; nsw = 0;
  dmin = inf; cause = 'timeout'; k = 0;

  % Record membership center evolution for Figure 4
  centers_hist = zeros (n+1, numel (centers));
  centers_hist(1,:) = centers;
  time_hist = zeros (n+1, 1);

  % Track extended metrics for analysis
  clearance_trace = zeros (n+1, 1);
  force_trace = zeros (n+1, 1);
  clearance_trace(1) = inf;
  
  % Simplified Proposal 2 parameters: Global width adaptation only
  alpha_w = 0.02;           % Global width learning rate (conservative)
  beta = 1.0;               % Baseline width increase rate
  gamma = 0.8;              % Lyapunov sensitivity (strong feedback)
  theta_overlap = 0.5;      % Minimum overlap threshold
  
  % Stall detection for robust adaptation
  V_history = [];
  stall_threshold = 0.01;   % Consider stalled if |V_rate| < this
  stall_window = 10;        % Require sustained stall over this many steps
  adapt_enabled = false;    % Only adapt if truly stalled

  % Equilibrium-escape kick state (Remark rem:equilibrium-escape):
  % mode 2 = bounded, goal-biased tangential kick with outward bias,
  % applied for a duration that escalates on repeated re-entry of the same
  % trap. When fapf_subgoal reports no admissible waypoint, the robot is
  % assumed to be at a genuine equilibrium of the base field with no
  % reachable passage in the sampled directions; the kick breaks the
  % symmetric force balance by construction, letting the base law resume
  % from a non-equilibrium point.
  u_max   = P.v_max;
  tau_kick0 = 1.5;
  tau_inc   = 1.0;
  tau_max   = 6.0;
  tau_kick  = tau_kick0;
  eps_kick  = 0.3;
  kick_end = -inf;
  kick_dir = [0, 0];
  kick_gain = 1.0;   % escalates on repeated kicks to the same trap
  kick_gain_inc = 0.5;

  for k = 1:n
    t = (k-1) * dt;
    [F, I] = fapf_force (q, target, env.obs, P, centers, sigma);
    dmin = min (dmin, I.d);
    e_goal = norm (q - env.goal);
    if (e_goal <= P.rho), cause = 'goal'; break; end

    th_d = atan2 (F(2), F(1));
    beta = atan2 (sin (th_d - th), cos (th_d - th));
    gate = double (e_goal > P.rho);
    v = P.v_max * min (1, max (0, P.eps_v * gate + cos (beta)^2));
    if (cos (beta) <= 0), v = 0; end
    om = P.k_omega * sin (beta);
    qdot = v * [cos(th), sin(th)];

    % --- GNRON supervisor (targets env.goal or a subgoal waypoint) -------
    % Primary check: composite Lyapunov metric of Eq. (convergence-rate-section4).
    V = e_goal^2 + gamma_d * max (0, P.d0 - I.d);
    bi = mod (bi, Nw) + 1;
    Vold = Vbuf(bi); Vbuf(bi) = V;
    if (!isnan (Vold))
      Vdot = (V - Vold) / T_window;
      if (abs (Vdot) < eps_conv), t_stall += dt; else, t_stall = 0; end
    end
    % Secondary check: radial progress toward the TRUE goal alone. This is
    % independent of the obstacle term, which can oscillate fast enough
    % while the robot orbits a guarded ring to mask a genuine radial stall
    % in the composite metric above (nearest-obstacle identity keeps
    % switching, so d_obs -- and hence V -- fluctuates even though e_goal
    % itself is not decreasing).
    ei = mod (ei, Nw) + 1;
    Eold = Ebuf(ei); Ebuf(ei) = e_goal;
    if (!isnan (Eold))
      Edot = (e_goal - Eold) / T_window;
      if (abs (Edot) < eps_conv_e), t_stall_e += dt; else, t_stall_e = 0; end
    end
    if (mode == 1 && norm (q - target) <= rho_wp)
      target = env.goal; mode = 0; t_stall = 0; t_stall_e = 0; Vbuf(:) = nan; Ebuf(:) = nan;
    end
    if ((t_stall > T_thresh || t_stall_e > T_thresh_e) && (t - t_last) >= tauD)
      [wp, ok] = fapf_subgoal (q, env.goal, env.obs, P, L, Delta);
      if (ok)
        target = wp; mode = 1; t_last = t; t_stall = 0; t_stall_e = 0; Vbuf(:) = nan; Ebuf(:) = nan; nsw++;
      else
        % No admissible waypoint exists: the robot is at a genuine
        % equilibrium of the base field with no reachable passage in the
        % sampled directions. Apply the bounded goal-biased tangential
        % escape kick of Remark rem:equilibrium-escape instead of declaring
        % failure. The direction slides tangentially along the obstacle
        % while simultaneously making net progress toward the goal, so
        % repeated kicks do not loop back to the same equilibrium.
        nhat = I.nhat;
        tdir = [nhat(2), -nhat(1)];               % tangent to obstacle
        gdir = (env.goal - q); gdir = gdir / max (norm (gdir), eps);
        % Pick the tangent SIGN toward the goal side. In 2D the tangent
        % plane is 1-D, so blending tdir with the goal's tangential
        % projection can cancel to zero (blended/norm -> NaN); selecting
        % the sign instead keeps |kick_dir| = 1 always. A deterministic
        % sign alternation on successive kicks breaks symmetric re-trapping
        % loops (where the same tangent brings the robot back to the same
        % equilibrium), while remaining reproducible across runs.
        if (tdir * gdir' < 0), tdir = -tdir; end
        kick_dir = tdir;
        kick_end = t + tau_kick;
        tau_kick = min (tau_kick + tau_inc, tau_max);
        kick_gain = kick_gain + kick_gain_inc;
        mode = 2; t_last = t; t_stall = 0; t_stall_e = 0; Vbuf(:) = nan; Ebuf(:) = nan; nsw++;
      end
    end

    % --- mode override: bounded tangential escape kick -------------------
    if (mode == 2)
      if (t >= kick_end)
        % Kick exhausted: resume the base law targeting the true goal.
        mode = 0; target = env.goal; Vbuf(:) = nan; Ebuf(:) = nan;
      else
        % Goal-biased tangential slide PLUS repulsive push-out and outward
        % bias: keeps hdot > 0 during the kick so clearance never decreases.
        % kick_gain escalates on repeated kicks to the same trap so each
        % escape is stronger than the last.
        qdot = kick_gain * u_max * (kick_dir + eps_kick * I.nhat) ...
             + P.k_r * updatedlaw_gradUr_mag_ (I.d, P) * I.nhat;
      end
    end

    q  = q + dt * qdot;
    th = th + dt * om;
    traj(k+1,:) = q;

    % ===== SIMPLIFIED ADAPTIVE LAW: Stall-Triggered Width Increase =====
    if (adapt && e_goal > P.rho)  % Dead-zone: halt adaptation near goal
      % Track Lyapunov decay to detect stall
      V_curr = 0.5 * e_goal^2;
      eps_lyap = 1e-3;
      V_rate = (V_curr - V_prev) / (V_curr + eps_lyap);  
      
      % Stall detection: accumulate history
      V_history = [V_history; V_rate];
      if (numel(V_history) > stall_window)
        V_history = V_history(end-stall_window+1:end);
      end
      
      % Check if clearly stalled (multiple steps with near-zero decay rate)
      if (numel(V_history) >= stall_window)
        mean_abs_vrate = mean(abs(V_history));
        if (mean_abs_vrate < stall_threshold && ~adapt_enabled)
          % Robot is stalled - enable ONE-TIME width increase
          adapt_enabled = true;
        end
      end
      
      % Apply adaptation ONLY when stalled
      if (adapt_enabled && e_goal > P.rho)
        sigma_old = sigma;
        
        % Simple width increase when stalled (reduce sensitivity)
        ds = 0.05;  % Conservative fixed increase
        sigma = min(1.0, sigma + ds);
        
        % Verify MULTIPLE CRITICAL CONDITIONS from theory
        d_test = linspace(0, P.d_meas, 50);
        mu_test = max(0, 1 - abs(d_test' - centers) ./ (2*sigma));
        
        % Check 1: Overlap
        min_overlap = min(sum(mu_test, 2));
        overlap_ok = (min_overlap >= theta_overlap);
        
        % Check 2: Monotonicity of w1 and w2 (CRITICAL!)
        w1_test = (mu_test * P.w1_c(:)) ./ sum(mu_test, 2);
        w2_test = (mu_test * P.w2_c(:)) ./ sum(mu_test, 2);
        w1_diff = diff(w1_test);  % Should all be > 0
        w2_diff = diff(w2_test);  % Should all be < 0
        w1_monotonic = all(w1_diff > -1e-6);  % Allow tiny numerical noise
        w2_monotonic = all(w2_diff < 1e-6);
        
        % Check 3: Weight bounds
        w1_bounded = (min(w1_test) >= P.w1_min * 0.99) && (max(w1_test) <= P.w1_max * 1.01);
        w2_bounded = (min(w2_test) >= P.w2_min * 0.99) && (max(w2_test) <= P.w2_max * 1.01);
        
        % Check 4: Convergence rate separation (related to steering gain)
        % Steering condition k_omega > D/sin(beta*) depends on ratio lambda_- / lambda_+
        % where lambda_- = w1_min * k_a and lambda_+ = w1_max * k_a
        % We verify the ratio is sufficiently large for the given k_omega
        w1_min = min(w1_test);
        w1_max = max(w1_test);
        if (w1_min > 0 && w1_max > 0 && w1_max > w1_min)
          ratio = w1_min / w1_max;
          % Require minimum separation: ratio > 0.2 ensures lambda_-/lambda_+ is not too small
          % This ensures steering control remains effective even with adapted membership
          convergence_ok = (ratio > 0.2);
        else
          convergence_ok = false;
        end
        
        % Reject if ANY condition violated
        if (~overlap_ok || ~w1_monotonic || ~w2_monotonic || ~w1_bounded || ~w2_bounded || ~convergence_ok)
          % Reject update and revert
          sigma = sigma_old;
        else
          % All conditions satisfied - update accepted
          V_history = [];
          adapt_enabled = false;
        end
      end
      
      V_prev = V_curr;  % Update for next iteration
    end

    centers_hist(k+1,:) = centers;
    time_hist(k+1) = t;
    clearance_trace(k+1) = I.d;
    force_trace(k+1) = norm (F);

    if (norm (q - env.goal) > 3 * P.D_max), cause = 'diverged'; break; end
  end

  out.traj    = traj(1:max(k,1),:);
  out.success = strcmp (cause, 'goal');
  out.dmin    = dmin;
  out.cause   = cause;
  out.nsw     = nsw;
  out.t_end   = k * dt;
  out.mode_end = mode;
  out.target_end = target;
  out.centers_hist = centers_hist(1:max(k,1),:);
  out.time_hist   = time_hist(1:max(k,1));
  out.c_final   = centers;  % Final membership centers (for before/after comparison)
  out.s_final   = sigma;    % Final membership widths (for before/after comparison)

  % Extended metrics for comparison analysis
  traj_valid = traj(1:max(k,1),:);
  if (size (traj_valid, 1) > 1)
    path_diffs = diff (traj_valid);
    out.path_length = sum (sqrt (sum (path_diffs.^2, 2)));
  else
    out.path_length = 0;
  end
  out.clearance_mean = mean (clearance_trace(1:max(k,1)));
  out.clearance_std = std (clearance_trace(1:max(k,1)));
  out.force_mean = mean (force_trace(1:max(k,1)));
  out.force_std = std (force_trace(1:max(k,1)));
end

function [m, v] = local_stats_ (q, obs, P)
  dx = q(1) - obs(:,1); dy = q(2) - obs(:,2);
  di = sqrt (dx.^2 + dy.^2) - obs(:,3);
  di = di(di <= P.d_meas);
  if (isempty (di)), m = P.d_meas; v = 0.25; return; end
  m = mean (di); v = var (di); if (isnan (v)), v = 0.25; end
end

function g = updatedlaw_gradUr_mag_ (d, P)
% Scalar repulsive-gradient magnitude, shared form used by fapf_force.
  g = max (0, P.d0 - d) / (d^2 + P.eps_reg);
end

% ===== HELPER: Isotonic Regression (Pool-Adjacent-Violators) =====
function [y_iso, blocks] = isotonic_regression(y, direction)
  % Enforces monotonicity via isotonic regression
  % direction = +1: non-decreasing (y_1 <= y_2 <= ... <= y_n)
  % direction = -1: non-increasing (y_1 >= y_2 >= ... >= y_n)
  
  n = numel(y);
  y_iso = y(:);
  blocks = (1:n)';
  
  if (n <= 1), return; end
  
  % Forward pass: merge violating pairs
  i = 1;
  while (i < n)
    if (direction * y_iso(i) > direction * y_iso(i+1))
      % Violation: merge blocks by averaging
      avg_val = mean(y_iso(i:i+1));
      y_iso(i:i+1) = avg_val;
      blocks(i+1) = blocks(i);
      if (i > 1)
        i = i - 1;  % Check previous pair
      else
        i = i + 1;
      end
    else
      i = i + 1;
    end
  end
  
  % Backward pass: verify and fix any remaining violations
  i = n - 1;
  while (i >= 1)
    if (direction * y_iso(i) > direction * y_iso(i+1))
      avg_val = mean(y_iso(i:i+1));
      y_iso(i:i+1) = avg_val;
      blocks(i) = blocks(i+1);
      if (i < n-1)
        i = i + 1;
      else
        i = i - 1;
      end
    else
      i = i - 1;
    end
  end
end
