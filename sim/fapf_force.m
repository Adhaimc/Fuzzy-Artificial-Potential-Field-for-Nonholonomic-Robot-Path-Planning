function [F, info] = fapf_force (q, goal, obs, P, centers, sigma)
% Control force of Eq. (control-force):
%     F = -w1(d_obs) k_a grad U_a  -  w2(d_obs) k_r grad U_r
% with grad U_a = q - goal.  This is a designed NON-gradient field: the
% chain-rule terms of Eq. (chain-rule-terms) are deliberately absent.

  if (nargin < 5), centers = []; end
  if (nargin < 6), sigma   = []; end

  qt = q(:).' - goal(:).';
  [d, nhat] = fapf_dobs (q, obs);
  [w1, w2]  = fapf_weights (d, P, centers, sigma);

  F_att = -w1 * P.k_a * qt;
  F_rep =  w2 * P.k_r * fapf_gradUr_mag (d, P) * nhat;   % pushes outward

  F = F_att + F_rep;

  info.d    = d;
  info.nhat = nhat;
  info.w1   = w1;
  info.w2   = w2;
  info.Fatt = F_att;
  info.Frep = F_rep;
end
