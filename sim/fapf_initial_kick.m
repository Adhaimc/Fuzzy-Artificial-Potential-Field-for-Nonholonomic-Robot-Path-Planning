function [pos, kick_traj] = fapf_initial_kick (pos0, obs, P, dt)
% Pre-control clearance recovery: the kinematic law integrates position
% directly (no velocity state), so a start point placed too close to (or
% inside) an obstacle has no "current speed" to carry it clear before the
% attractive term pulls it back in. Give it an explicit initial speed
% opposite the obstacle (outward normal, at P.v_max) until the nominal
% passage margin r_robot+r_safety is met, then hand off to the force law.

  margin = P.r_robot + P.r_safety;
  pos = pos0(:).';
  kick_traj = pos;
  max_kick_steps = 500;
  for i = 1:max_kick_steps
    [d, nhat] = fapf_dobs (pos, obs);
    if (d >= margin), break; end
    pos = pos + dt * P.v_max * nhat;
    kick_traj(end+1, :) = pos;
  end
end
