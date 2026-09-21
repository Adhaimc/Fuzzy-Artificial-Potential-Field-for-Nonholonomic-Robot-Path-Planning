function [d, nhat, nactive] = fapf_dobs (q, obs, tol)
% Nearest-obstacle distance d_obs(q) = min_i d_i(q) and its gradient.
% obs is an M-by-3 array of [x, y, radius].
% nactive counts obstacles within tol of the minimum; nactive > 1 flags a
% Voronoi boundary where grad d_obs is undefined (Remark rem:filippov).

  if (nargin < 3), tol = 1e-6; end

  dx = q(1) - obs(:,1);
  dy = q(2) - obs(:,2);
  rr = sqrt (dx.^2 + dy.^2);
  di = rr - obs(:,3);

  [d, k] = min (di);
  nactive = sum (di <= d + tol);

  if (rr(k) < 1e-12)
    nhat = [1, 0];
  else
    nhat = [dx(k), dy(k)] / rr(k);
  end
end
