function [d, nhat] = fapf_dobs_pointmass(q, obs_centers)
% Distance and direction to nearest obstacle (POINT-MASS MODEL).
%
% Input:
%   q: robot position [x, y]
%   obs_centers: Nx2 array of obstacle centers (no radii; point masses)
%
% Output:
%   d: distance to nearest obstacle center
%   nhat: unit normal pointing away from nearest obstacle
%
% This is the theoretical obstacle model: point masses with d0-based influence.
% For convergence analysis, obstacles have no physical extent, only influence regions.

  q = q(:).';
  obs_centers = obs_centers(:,1:2);  % Extract only center coords, ignore any radius column
  
  dists = sqrt(sum((obs_centers - q).^2, 2));
  [d, idx] = min(dists);
  
  if (d < 1e-10)
    nhat = [1, 0];  % arbitrary direction if exactly at obstacle
  else
    nhat = (q - obs_centers(idx,:)) / d;
  end
end
