function out = fapf_sim_nonholo (q0, th0, env, P, opt)
% Unicycle closed loop with the GATED velocity law of Eq. (velocity-gated):
%     v = v_max [ eps_v * 1(||qtil|| > rho) + cos^2(beta) ]_sat
% The gate is what makes convergence to the terminal ball consistent with a
% non-zero speed floor (Remark rem:epsilon-v).

  if (nargin < 5), opt = struct (); end
  if (~isfield (opt, 'dt')),    opt.dt    = P.dt;    end
  if (~isfield (opt, 'T_max')), opt.T_max = P.T_max; end
  if (~isfield (opt, 'adapt')), opt.adapt = false;   end

  n = round (opt.T_max / opt.dt);
  q = q0(:).'; th = th0;
  traj = zeros (n+1, 2); traj(1,:) = q;
  % log columns: t, e, ||F||, theta_d, d_obs, h, hdot, beta, v
  logd = zeros (n+1, 9);

  centers = P.centers; sigma = P.sigma;
  dmin = inf; cbf_min = inf;
  k = 0; cause = 'timeout';

  for k = 1:n
    [F, I] = fapf_force (q, env.goal, env.obs, P, centers, sigma);
    dmin = min (dmin, I.d);

    h    = I.d - P.r_safety;
    e    = norm (q - env.goal);

    th_d = atan2 (F(2), F(1));
    beta = atan2 (sin (th_d - th), cos (th_d - th));

    gate = double (e > P.rho);
    v = P.v_max * min (1, max (0, P.eps_v * gate + cos (beta)^2));
    if (cos (beta) <= 0), v = 0; end          % never drive backwards
    om = P.k_omega * sin (beta);

    qdot = v * [cos(th), sin(th)];
    logd(k,:) = [(k-1)*opt.dt, e, norm(F), th_d, I.d, h, I.nhat*qdot(:), beta, v];

    if (e <= P.rho), cause = 'goal'; break; end

    cbf_min = min (cbf_min, I.nhat * qdot(:) + P.alpha0 * h);

    q  = q + opt.dt * qdot;
    th = th + opt.dt * om;
    traj(k+1,:) = q;

    % adaptive membership tuning with dead zone (Eq. adapt-deadzone)
    if (opt.adapt && e > P.rho)
      [dm, dv] = local_stats (q, env.obs, P);
      dc = P.alpha_c * (dm - centers);
      ds = P.alpha_s * (sqrt (dv) - sigma);
      dc = max (-P.vmax_c, min (P.vmax_c, dc));   % rate limits (constraint 3)
      ds = max (-P.vmax_s, min (P.vmax_s, ds));
      centers = centers + opt.dt * dc;
      sigma   = max (0.15, min (1.0, sigma + opt.dt * ds));
      centers = sort (centers);                   % preserve zone ordering
    end

    if (norm (q - env.goal) > 3 * P.D_max), cause = 'diverged'; break; end
  end

  out.log     = logd(1:max(k,1),:);
  out.traj    = traj(1:max(k,1),:);
  out.q_end   = q;
  out.e_end   = norm (q - env.goal);
  out.cause   = cause;
  out.success = strcmp (cause, 'goal');
  out.t_end   = k * opt.dt;
  out.dmin    = dmin;
  out.cbf_min = cbf_min;
  out.centers = centers;
  out.sigma   = sigma;
end

function [m, v] = local_stats (q, obs, P)
  dx = q(1) - obs(:,1); dy = q(2) - obs(:,2);
  di = sqrt (dx.^2 + dy.^2) - obs(:,3);
  di = di(di <= P.d_meas);
  if (isempty (di)), m = P.d_meas; v = 0.25; return; end
  m = mean (di);
  v = max (0.02, var (di));
end
