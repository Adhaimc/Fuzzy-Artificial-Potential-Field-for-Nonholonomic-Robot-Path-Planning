function [d, nhat, d0_nearest] = fapf_dobs_pointmass_varying(q, obs_struct)
% Distance and direction to nearest obstacle with VARYING d0 (influence radius).
%
% Input:
%   q: robot position [x, y]
%   obs_struct: struct array with fields:
%      .center: [x, y] obstacle center
%      .d0:     influence radius for this obstacle
%
% Output:
%   d: distance to nearest obstacle center
%   nhat: unit normal pointing away from nearest obstacle
%   d0_nearest: the d0 value of the nearest obstacle
%
% For distance metric, we use the center-to-center distance (point-mass model).
% The per-obstacle d0 is returned so force law and clearance checks can use it.

  q = q(:).';
  
  if (isempty(obs_struct))
    d = inf; nhat = [1, 0]; d0_nearest = inf; return;
  end
  
  nobs = numel(obs_struct);
  dists = zeros(nobs, 1);
  for i = 1:nobs
    dists(i) = norm(q - obs_struct(i).center);
  end
  
  [d, idx] = min(dists);
  d0_nearest = obs_struct(idx).d0;
  
  if (d < 1e-10)
    nhat = [1, 0];
  else
    nhat = (q - obs_struct(idx).center) / d;
  end
end
