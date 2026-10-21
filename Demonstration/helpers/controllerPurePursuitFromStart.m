classdef controllerPurePursuitFromStart < controllerPurePursuit
%CONTROLLERPUREPURSUITFROMSTART  controllerPurePursuit that starts on segment 1.
%
%   The stock controller searches the whole path for the closest segment on
%   its first call. A planner wait-loop returns a vehicle to its own start,
%   so several segments are 0 mm away and the controller can lock onto the
%   one after the loop and skip it. This anchors the first search to
%   waypoint 1 on segment 1; everything else is the stock controller.
%   Same fix as PurePursuitFromStart in the simulation.

    methods
        function obj = controllerPurePursuitFromStart(varargin)
            obj@controllerPurePursuit(varargin{:});
        end
    end

    methods (Access = protected)
        function computeProjectionPoint(obj, currentPose, waypoints)
            if obj.ProjectionLineIndex == 0
                obj.ProjectionPoint     = waypoints(1,1:2);
                obj.ProjectionLineIndex = cast(1, like=currentPose);
            end
            computeProjectionPoint@controllerPurePursuit(obj, currentPose, waypoints);
        end
    end
end
