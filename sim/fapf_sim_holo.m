function out = fapf_sim_holo (q0, env, P, opt)
% First-order holonomic closed loop  qdot = F(q)   (Eq. holonomic-dynamics).
% This is the model the theory treats as primary and the one integrated here.
%
% Returns trajectory, termination cause, and the safety/barrier monitors
% needed to test Proposition prop:cbf-validity numerically.

  if (nargin < 4), opt = struct (); end
  if (~isfield (opt, 'dt')),    opt.dt    = P.dt;    end
  if (~isfield (opt, 'T_max')), opt.T_max = P.T_max; end
  if (~isfield (opt, 'Fstall')),opt.Fstall= 1e-3;    end

  n  = round (opt.T_max / opt.dt);
  q  = q0(:).';
  traj = zeros (n+1, 2); traj(1,:) = q;
  % log columns: t, e, ||F||, theta_d, d_obs, h, hdot
  logd = zeros (n+1, 7);

  dmin      = inf;      % closest approach to any obstacle
  cbf_min   = inf;      % min of  hdot + alpha0*h
  Fmin_seen = inf;
  k = 0; cause = 'timeout';

  for k = 1:n
    [F, I] = fapf_force (q, env.goal, env.obs, P);
    nF = norm (F);
    dmin = min (dmin, I.d);

    % barrier monitor:  h = d_obs - r_safety,  hdot = nhat' * qdot,  qdot = F
    h    = I.d - P.r_safety;
    hdot = I.nhat * F(:);
    cbf_min = min (cbf_min, hdot + P.alpha0 * h);

    e = norm (q - env.goal);
    logd(k,:) = [(k-1)*opt.dt, e, nF, atan2(F(2), F(1)), I.d, h, hdot];

    if (e <= P.rho), cause = 'goal'; break; end

    % a genuine equilibrium of the field, away from the goal
    if (nF < opt.Fstall), cause = 'stall'; break; end
    Fmin_seen = min (Fmin_seen, nF);

    q = q + opt.dt * F;
    traj(k+1,:) = q;

    if (norm (q - env.goal) > 3 * P.D_max), cause = 'diverged'; break; end
  end

  out.log       = logd(1:max(k,1),:);
  out.traj      = traj(1:max(k,1),:);
  out.q_end     = q;
  out.e_end     = norm (q - env.goal);
  out.cause     = cause;
  out.success   = strcmp (cause, 'goal');
  out.t_end     = k * opt.dt;
  out.dmin      = dmin;
  out.cbf_min   = cbf_min;
  out.Fmin_seen = Fmin_seen;
end
