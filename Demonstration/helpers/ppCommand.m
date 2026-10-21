function [v, w, kappa, alpha, lookPt] = ppCommand(pp, pose)
%PPCOMMAND  Pure pursuit command for one pose [x y theta].
%
%   Returns forward speed v [m/s], turn rate w [rad/s], curvature kappa
%   [1/m], heading error to the lookahead point alpha [rad] and the
%   lookahead point itself [m].
%
%   controllerPurePursuit returns a curvature, so w = v*kappa. The toolbox
%   divides by the configured lookahead distance even when the point it
%   found along the path is closer (at corners), so the curvature is
%   recomputed from the straight-line distance d to that point and clipped
%   to MaxCurvature again.
[v, kappa, lookPt] = pp(pose);

d     = norm(lookPt - pose(1:2));
alpha = ArucoTracker.wrapPi(atan2(lookPt(2) - pose(2), lookPt(1) - pose(1)) - pose(3));
if d > 1e-6
    kappa = 2*sin(alpha)/d;
    kappa = max(-pp.MaxCurvature, min(pp.MaxCurvature, kappa));
end

w = v*kappa;
end
