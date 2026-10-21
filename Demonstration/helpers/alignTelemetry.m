function w = alignTelemetry(tel)
%ALIGNTELEMETRY  One vehicle's wheel telemetry on the host clock.
%
%   tel rows are [id tMs spL wL uL spR wR uR flags tHost]. tMs is the
%   vehicle's own clock and tHost the (late) time the row was read, so
%   tHost - tMs over-estimates the clock offset by the read delay. The 5th
%   percentile is taken as the offset, i.e. the least-delayed rows.
w = struct('t', [], 'tMs', [], 'spL', [], 'wL', [], 'uL', [], ...
           'spR', [], 'wR', [], 'uR', [], 'flags', []);
if isempty(tel), return; end

tMs = tel(:, 2) / 1000;
w.t     = tMs + prctile(tel(:, 10) - tMs, 5);
w.tMs   = tel(:, 2);
w.spL   = tel(:, 3);   w.wL = tel(:, 4);   w.uL = tel(:, 5);
w.spR   = tel(:, 6);   w.wR = tel(:, 7);   w.uR = tel(:, 8);
w.flags = tel(:, 9);
end
