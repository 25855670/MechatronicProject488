function [P, margin] = mapPlanPath(csvFile, veh, planArea, areaM)
%MAPPLANPATH  One vehicle's waypoints from the planner CSV, in arena metres.
%
%   planArea [xmin ymin xmax ymax] is the planner's area and areaM [W H] the
%   arena. x and y are scaled by the same factor and the plan is centred, so
%   every vehicle gets the same mapping and the planned separations hold.
%   margin [dx dy] is the offset that centring added [m].
T  = readtable(csvFile, 'VariableNamingRule', 'preserve');
cx = sprintf('v%d_x', veh);  cy = sprintf('v%d_y', veh);
if ~all(ismember({cx, cy}, T.Properties.VariableNames))
    error('mapPlanPath:noColumns', '%s has no %s/%s columns.', csvFile, cx, cy);
end

% Trailing NaNs are the end of a shorter route; repeated points are dropped
xy = [T.(cx) T.(cy)];
xy = xy(all(~isnan(xy), 2), :);
xy = xy([true; any(abs(diff(xy)) > 1e-9, 2)], :);
if size(xy, 1) < 2
    error('mapPlanPath:shortPath', 'Vehicle %d has fewer than 2 waypoints.', veh);
end

planSz = planArea(3:4) - planArea(1:2);
s      = min(areaM ./ planSz);
margin = (areaM - s*planSz) / 2;
P      = margin + (xy - planArea(1:2))*s;
end
