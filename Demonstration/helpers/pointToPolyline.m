function d = pointToPolyline(p, P)
%POINTTOPOLYLINE  Shortest distance [m] from point p to polyline P - the
%   cross-track error.
A = P(1:end-1, :);  B = P(2:end, :);
AB = B - A;  AP = p - A;
t  = min(1, max(0, sum(AP .* AB, 2) ./ max(sum(AB.^2, 2), eps)));
d  = min(vecnorm(AP - t .* AB, 2, 2));
end
