function out = run_adaptive_only_nonholo (q0, th0, env, P, alpha_c, alpha_s)
% Unicycle closed loop with adaptive membership law only (NO supervisor).
% Used by fig_stress_test.m to diagnose whether adaptation alone helps.
%
% This is like fapf_sim_nonholo but with the adaptive law applied to centers and sigma.

  if (nargin < 5), alpha_c = []; end
  if (nargin < 6), alpha_s = []; end
  if (isempty (alpha_c)), alpha_c = P.alpha_c; end
  if (isempty (alpha_s)), alpha_s = P.alpha_s; end

  dt = P.dt; n = round (P.T_max / dt);
  q = q0(:).'; th = th0;
  traj = zeros (n+1, 2); traj(1,:) = q;

  centers = P.centers; sigma = P.sigma;
  w1 = P.w1_c; w2 = P.w2_c;  % Fixed consequent weights (NOT adapted)
  V_prev = 0.5 * norm(q - env.goal)^2;  % Lyapunov tracking
  
  eps_conv = 5e-3; T_thresh = 1.0;
  dmin = inf; cause = 'timeout'; k = 0;

  % Track extended metrics for analysis
  clearance_trace = zeros (n+1, 1);
  force_trace = zeros (n+1, 1);
  clearance_trace(1) = inf;

  t_stall = 0; t_last = -inf;
  v_buf = [];
  
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

  for k = 1:n
    t = (k-1) * dt;
    [F, I] = fapf_force (q, env.goal, env.obs, P, centers, sigma);
    dmin = min (dmin, I.d);
    d_obs = I.d;  % Current measured obstacle distance

    % ===== SIMPLIFIED ADAPTIVE LAW: Stall-Triggered Width Increase =====
    e_goal = norm (q - env.goal);
    if (e_goal > P.rho)  % Dead-zone: halt adaptation near goal
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
        if (mod(k, 200) == 0)  % Print every 200 steps
          fprintf('[RATE] t=%.2f e=%.4f mean|V_rate|=%.6f\n', t, e_goal, mean_abs_vrate);
        end
        if (mean_abs_vrate < stall_threshold && ~adapt_enabled)
          % Robot is stalled - enable ONE-TIME width increase
          adapt_enabled = true;
          fprintf('[ADAPT-STALL] Stall detected at t=%.2f, e_goal=%.4f, mean|V_rate|=%.6f, sigma=%.4f\n', t, e_goal, mean_abs_vrate, sigma);
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
        w1_min = min(w1_test);
        w1_max = max(w1_test);
        if (w1_min > 0 && w1_max > 0 && w1_max > w1_min)
          ratio = w1_min / w1_max;
          convergence_ok = (ratio > 0.2);
        else
          convergence_ok = false;
        end
        
        % Reject if ANY condition violated
        violations = [~overlap_ok, ~w1_monotonic, ~w2_monotonic, ~w1_bounded, ~w2_bounded, ~convergence_ok];
        if (any(violations))
          % Reject update and revert
          sigma = sigma_old;
          fprintf('[ADAPT-REJECT] overlap=%d mono_w1=%d mono_w2=%d bound_w1=%d bound_w2=%d conv=%d sigma_old=%.4f\n', violations, sigma_old);
        else
          % All conditions satisfied - update accepted
          V_history = [];
          adapt_enabled = false;
          fprintf('[ADAPT-ACCEPT] sigma %.4f -> %.4f\n', sigma_old, sigma);
        end
      end
      
      V_prev = V_curr;  % Update for next iteration
    end

    % ===== KINEMATICS (unicycle model) =====
    v_cmd_max = norm (F);
    th_d = atan2 (F(2), F(1));
    beta = atan2 (sin (th_d - th), cos (th_d - th));
    
    % Modulate velocity with heading alignment (like base case)
    gate = double (e_goal > P.rho);
    v_cmd = P.v_max * min (1, max (0, P.eps_v * gate + cos (beta)^2));
    if (cos (beta) <= 0), v_cmd = 0; end  % Never drive backwards
    
    % Steering gain from theory (Theorem thm:nonholonomic-asymptotic)
    k_steering = P.k_omega;
    om = k_steering * sin (beta);

    qdot = v_cmd * [cos(th); sin(th)];
    q  = q + dt * qdot(:).';
    th = th + dt * om;
    traj(k+1,:) = q;

    % Check convergence
    if (e_goal < P.rho)
      cause = 'goal'; break;
    end

    % Check divergence
    dist_to_goal = norm (q - env.goal);
    if (dist_to_goal > 3 * P.D_max)
      cause = 'diverged'; break;
    end

    % Track clearance for metrics
    clearance_trace(k+1) = I.d;
    force_trace(k+1) = norm (F);
  end

  out.traj    = traj(1:max(k,1),:);
  out.success = strcmp (cause, 'goal');
  out.dmin    = dmin;
  out.cause   = cause;
  out.nsw     = 0;  % No supervisor, so no switches
  out.t_end   = k * dt;

  % Extended metrics for comparison analysis
  traj_valid = traj(1:max(k,1),:);
  if (size (traj_valid, 1) > 1)
    path_diffs = diff (traj_valid);
    out.path_length = sum (sqrt (sum (path_diffs.^2, 2)));
  else
    out.path_length = 0;
  end

  % Clearance statistics
  clr_valid = clearance_trace(1:max(k,1));
  clr_valid = clr_valid(clr_valid < inf);
  if (~isempty(clr_valid))
    out.clearance_mean = mean(clr_valid);
    out.clearance_std = std(clr_valid);
  else
    out.clearance_mean = inf;
    out.clearance_std = 0;
  end
end

% ===== HELPER: Compute global obstacle statistics =====
function [d_mean, d_var] = local_stats_ (q, obs, P)
  % Returns [d_mean, d_var] = global mean and variance of obstacle distances
  % within P.d_meas range from robot position q
  dx = q(1) - obs(:,1); dy = q(2) - obs(:,2);
  di = sqrt(dx.^2 + dy.^2) - obs(:,3);
  di = di(di <= P.d_meas);
  
  if (~isempty(di))
    d_mean = mean(di);
    d_var = var(di);
  else
    d_mean = P.d_meas;  % Default to max measurement range if no obstacles
    d_var = 0.25;       % Default variance
  end
end

