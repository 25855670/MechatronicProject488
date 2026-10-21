function rem = pathRemaining(p, P)
%PATHREMAINING  Length of path P [m] still ahead of a vehicle at p,
%   measured from its projection onto the nearest segment.
A = P(1:end-1, :);  B = P(2:end, :);
AB = B - A;  AP = p - A;
L2 = sum(AB.^2, 2);
t  = min(1, max(0, sum(AP .* AB, 2) ./ max(L2, eps)));
[~, k] = min(vecnorm(AP - t .* AB, 2, 2));
segLen = sqrt(L2);
rem = (1 - t(k)) * segLen(k) + sum(segLen(k+1:end));
end
