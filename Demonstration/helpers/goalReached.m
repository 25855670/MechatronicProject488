function [done, why] = goalReached(xy, P, goalTol, capture, onFinal)
%GOALREACHED  Has a vehicle at xy arrived at the end of path P?
%
%   'goal'    within goalTol of the last waypoint.
%   'passed'  on the final approach (onFinal), within capture of the goal
%             and past the line through the goal perpendicular to the last
%             segment. The vehicle cannot reverse, so drawing level with
%             the goal counts as arriving rather than circling back.
g = P(end, :);
d = norm(xy - g);
if d < goalTol
    done = true; why = 'goal'; return;
end
u = g - P(end-1, :);
n = norm(u);
if n > eps && dot(xy - g, u/n) >= 0 && d < capture && onFinal
    done = true; why = 'passed'; return;
end
done = false; why = '';
end
