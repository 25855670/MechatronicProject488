function wTurn = maxTurnRate(v, b, r, wMin, wMax)
%MAXTURNRATE  Largest turn rate [rad/s] at forward speed v [m/s] that keeps
%   the inner wheel at or above wMin and the outer wheel at or below wMax
%   [rad/s]. b is the track width and r the wheel radius [m]. The speed is
%   held; only the difference between the wheels changes.
inner = v - wMin*r;          % headroom before the inner wheel drops below wMin
outer = wMax*r - v;          % headroom before the outer wheel exceeds wMax
wTurn = 2*max(0, min(inner, outer))/b;
end
