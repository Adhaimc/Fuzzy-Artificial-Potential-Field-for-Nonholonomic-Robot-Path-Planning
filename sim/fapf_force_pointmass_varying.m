function [F, info] = fapf_force_pointmass_varying(q, goal, obs_struct, P)
% Control force using POINT-MASS obstacles with VARYING d0 per obstacle.
%
% Computes attractive force + aggregate repulsion from all obstacles.
% Each obstacle has its own d0_i influence radius.
%
% Force model:
%   F = -w1(d_nearest, d0_nearest) * k_a * (q - goal)
%       + sum_i [ w2_i(d_i, d0_i) * k_r * grad_U_r(d_i, d0_i) * nhat_i ]
%
% where weights and gradients are computed per-obstacle.

  q = q(:).';
  qt = q - goal(:).';
  
  F_att = zeros(1, 2);
  F_rep_total = zeros(1, 2);
  
  % Attractive force (based on nearest obstacle's influence)
  [d_nearest, nhat_nearest, d0_nearest] = fapf_dobs_pointmass_varying(q, obs_struct);
  w1 = fapf_weights_with_d0(d_nearest, P, d0_nearest);
  F_att = -w1 * P.k_a * qt;
  
  % Aggregate repulsion from all obstacles (each with its own d0_i)
  if (~isempty(obs_struct))
    nobs = numel(obs_struct);
    for i = 1:nobs
      obs_center = obs_struct(i).center(:).';
      d0_i = obs_struct(i).d0;
      
      % Distance and direction to this obstacle
      d_i = norm(q - obs_center);
      if (d_i < 1e-10)
        nhat_i = [1, 0];
      else
        nhat_i = (q - obs_center) / d_i;
      end
      
      % Weight and gradient for this obstacle's influence region
      w2_i = fapf_weights_with_d0(d_i, P, d0_i, 'w2_only');
      grad_i = fapf_gradUr_mag_with_d0(d_i, P, d0_i);
      
      F_rep_total = F_rep_total + w2_i * P.k_r * grad_i * nhat_i;
    end
  end
  
  F = F_att + F_rep_total;
  
  info.d_nearest = d_nearest;
  info.d0_nearest = d0_nearest;
  info.w1 = w1;
  info.F_att = F_att;
  info.F_rep_total = F_rep_total;
end

function w = fapf_weights_with_d0(d, P, d0, mode)
% Compute weights for a given distance d and influence radius d0.
% mode: 'full' (default) = compute w1 and w2, or 'w2_only'
  
  if (nargin < 4), mode = 'full'; end
  
  if (strcmp(mode, 'w2_only'))
    % Just w2 for repulsion
    if (d >= d0)
      w = P.w2_min;
    else
      % Linear interpolation: w2_max at d=0, w2_min at d=d0
      w = P.w2_max - (P.w2_max - P.w2_min) * d / d0;
    end
  else
    % Full: return w1 and w2 (unused here, kept for compatibility)
    w = 0;  % placeholder
  end
end

function grad = fapf_gradUr_mag_with_d0(d, P, d0)
% Magnitude of gradient of U_r (repulsive potential) for this d0.
% Standard formula: grad U_r = k / (d - d0)^2 for d < d0, 0 for d >= d0
  
  if (d >= d0)
    grad = 0;
  else
    grad = 1.0 / max((d - d0)^2, P.eps_reg);
  end
end
