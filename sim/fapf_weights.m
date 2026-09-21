function [w1, w2, mu] = fapf_weights (d, P, centers, sigma)
% Normalised fuzzy weights (Eqs. fuzzy-weight-attractive/repulsive) from a
% triangular partition.  Only continuity and monotonicity are used by the
% theory, so no derivative of mu is ever required.

  if (nargin < 3 || isempty (centers)), centers = P.centers; end
  if (nargin < 4 || isempty (sigma)),   sigma   = P.sigma;   end

  mu = max (0, 1 - abs (d - centers) ./ (2 * sigma));

  s = sum (mu);
  if (s < 1e-12)
    % outside the partition support: clamp to the nearest zone
    if (d <= centers(1))
      mu = zeros (size (centers)); mu(1) = 1; s = 1;
    else
      mu = zeros (size (centers)); mu(end) = 1; s = 1;
    end
  end

  w1 = sum (mu .* P.w1c) / s;
  w2 = sum (mu .* P.w2c) / s;
end
