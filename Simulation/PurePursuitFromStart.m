classdef PurePursuitFromStart < nav.slalgs.internal.PurePursuit
%PUREPURSUITFROMSTART  The Pure Pursuit block, made to start on segment 1.
%
%   The three pure pursuit blocks in PurePursuitControlledRobot.slx run
%   this System object instead of the stock nav.slalgs.internal.PurePursuit.
%   It is the stock block in every respect - same dialog, same ports, same
%   control law - except for how it picks the path segment on its FIRST
%   step.
%
% WHY THE STOCK BLOCK SKIPS THE PLANNER'S WAIT-LOOPS
%
%   The joint planner has no wait action, so it delays a vehicle by sending
%   it round a cell and back to its own start node:
%
%       (0.4,1.6) (0.4,1.2) (0,1.2) (0,1.6) (0.4,1.6) (0.8,1.6) ...
%        ^ wp1                               ^ wp5 == wp1
%
%   PurePursuitBase.computeProjectionPoint searches only a lookahead-long
%   window ahead of the last projection - EXCEPT on the first call, when
%   the projection is uninitialised and it searches the WHOLE path for the
%   closest segment. On a route like this, segment 1 (leaving the start),
%   segment 4 (arriving back at it) and segment 5 (leaving it again) are
%   all exactly 0 mm from the start node, and the vehicle's pose decides
%   between them by a fraction of a millimetre. In Robot.slx the ArUco
%   noise is added AFTER the transport delay, so even the t = 0 pose
%   carries sig_xy of noise. Segment 1 wins only when the noise lands in
%   one quadrant: replayed over 2000 noisy starts on trials 510 and 516 of
%   grid04_wc08, the stock block skipped the loop 75% of the time. Once it
%   has locked onto segment 4 or 5 the forward-only search can never go
%   back, so the whole lap is silently dropped.
%
%   Cross-track error cannot see this, because the lap returns to the same
%   line: the skipped route scores as near-perfect tracking.
%
% THE FIX
%
%   Every route starts at its first waypoint, so on the first step the
%   projection is anchored to waypoint 1 on segment 1. The stock forward,
%   lookahead-windowed search then runs from there exactly as it would on
%   any later step. On a route that does not come back to its start the
%   whole-path search would have picked segment 1 anyway, so those routes
%   are unaffected.
%
%   The anchor is re-applied whenever the projection is reset, which the
%   stock block already does when the waypoints input changes.
%
%   Cost of the change: a vehicle that genuinely starts part-way along its
%   route is pulled back to segment 1 rather than joining where it is. No
%   tool in this project starts a vehicle anywhere but the first waypoint.
%
%   See also nav.slalgs.internal.PurePursuit, controllerPurePursuitFromStart.

%#codegen

    methods
        function obj = PurePursuitFromStart(varargin)
            obj@nav.slalgs.internal.PurePursuit(varargin{:});
        end
    end

    methods (Access = protected)
        function computeProjectionPoint(obj, currentPose, waypoints)
            if obj.ProjectionLineIndex == 0
                obj.ProjectionPoint     = waypoints(1,1:2);
                obj.ProjectionLineIndex = cast(1, like=currentPose);
            end
            computeProjectionPoint@nav.slalgs.internal.PurePursuit(obj, currentPose, waypoints);
        end
    end
end
