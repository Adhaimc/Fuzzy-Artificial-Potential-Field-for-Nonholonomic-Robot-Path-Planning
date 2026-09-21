function g = fapf_gradUr_mag (d, P)
% |grad U_r|(d) for the implemented repulsive model (Table tab:repulsive-models):
%     grad U_r = -(d0 - d)_+ / (d^2 + eps_reg) * grad d_obs
% Finite support (P2) is enforced by the positive part.

  g = max (0, P.d0 - d) ./ (d.^2 + P.eps_reg);
end
