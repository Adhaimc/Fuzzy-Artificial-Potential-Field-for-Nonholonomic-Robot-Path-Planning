function [F, info] = fapf_force_pointmass(q, goal, obs_centers, P)
% Control force using POINT-MASS obstacle model.
% Same as fapf_force but uses fapf_dobs_pointmass instead of fapf_dobs.

  qt = q(:).' - goal(:).';
  [d, nhat] = fapf_dobs_pointmass(q, obs_centers);
  [w1, w2]  = fapf_weights(d, P);

  F_att = -w1 * P.k_a * qt;
  F_rep =  w2 * P.k_r * fapf_gradUr_mag(d, P) * nhat;

  F = F_att + F_rep;

  info.d    = d;
  info.nhat = nhat;
  info.w1   = w1;
  info.w2   = w2;
  info.Fatt = F_att;
  info.Frep = F_rep;
end
