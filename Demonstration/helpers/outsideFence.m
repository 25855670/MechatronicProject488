function out = outsideFence(xy, P, areaM, margin, grace)
%OUTSIDEFENCE  True if xy [m] is more than margin outside the arena
%   [0 0]-areaM. Points within grace of the start or end of the vehicle's
%   own path P are never outside, since those sit on the arena edge.
out = any(xy < -margin) || any(xy > areaM + margin);
if out && grace > 0
    out = norm(xy - P(1, :)) > grace && norm(xy - P(end, :)) > grace;
end
end
