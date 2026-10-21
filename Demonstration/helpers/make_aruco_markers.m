%% ArUco markers - writes the printable markers, one per page at exact size, to
%% aruco_markers.pdf in the Demonstration folder. Print at 100% and cut on the grey line.

%% ================= MARKER SETTINGS =================
% ArUco dictionary
FAMILY = "DICT_4X4_50";
% Arena corner marker ids and black-square side length [mm]
CORNER_IDS = [1 2 3 4];
CORNER_MM  = 160;
% Vehicle marker ids and black-square side length [mm]
VEHICLE_IDS = [5 6 7];
VEHICLE_MM  = 100;

%% ================= PAGE SETTINGS =================
% White margin around each marker [bit cells]
QUIET_CELLS = 1.0;
% Extra margin outside the white border, so a slightly inside cut is still fine [mm]
CUT_TOL_MM = 3;
% Space around the cut square for the labels and corner ticks, per side [mm]
CLEAR_MM = 12;
% Candidate paper sizes A4, A3, A2 portrait [mm] - the smallest that fits is used
PAPERS = [210 297; 297 420; 420 594];
% Output file, in the Demonstration folder
OUT = fullfile(fileparts(mfilename('fullpath')), '..', 'aruco_markers.pdf');

%% ================= GEOMETRY =================
ids = [CORNER_IDS, VEHICLE_IDS];
mm  = [repmat(CORNER_MM, 1, numel(CORNER_IDS)), repmat(VEHICLE_MM, 1, numel(VEHICLE_IDS))];

% Cells across a marker: data bits plus one border cell each side
nCells = str2double(regexp(FAMILY, 'DICT_(\d)X', 'tokens', 'once')) + 2;
% Cut-square side: marker + white margin + cutting tolerance [mm]
cut = mm + 2*(QUIET_CELLS*mm/nCells + CUT_TOL_MM);

page = PAPERS(find(max(cut) + 2*CLEAR_MM <= min(PAPERS, [], 2), 1), :);
assert(~isempty(page), 'The largest marker does not fit on A2 paper.');

%% ================= DRAW =================
% Pages are appended, so start from an empty file
if isfile(OUT), delete(OUT); end

for k = 1:numel(ids)
    % Data units are millimetres on a page of the same size, so it prints at true scale
    fig = figure('Visible', 'off', 'Units', 'centimeters', ...
                 'Position', [1 1 page/10], 'Color', 'w', ...
                 'PaperUnits', 'centimeters', 'PaperSize', page/10, ...
                 'PaperPositionMode', 'manual', 'PaperPosition', [0 0 page/10]);
    ax = axes(fig, 'Units', 'normalized', 'Position', [0 0 1 1]);
    hold(ax, 'on');
    ax.XLim = [0 page(1)];  ax.YLim = [0 page(2)];
    axis(ax, 'off');

    cx = page(1)/2;  cy = page(2)/2;
    M  = mm(k);
    s  = M / nCells;

    % Marker, drawn as one filled square per black cell
    bits = markerCells(FAMILY, ids(k), nCells);
    for r = 1:nCells
        for c = 1:nCells
            if ~bits(r, c)
                rectangle(ax, 'Position', [cx - M/2 + (c-1)*s, cy - M/2 + (nCells-r)*s, s, s], ...
                          'FaceColor', 'k', 'EdgeColor', 'none');
            end
        end
    end

    % Cut line
    C = cut(k);
    rectangle(ax, 'Position', [cx - C/2, cy - C/2, C, C], ...
              'EdgeColor', [0.62 0.62 0.62], 'LineWidth', 0.5);

    % Labels, outside the cut line
    text(ax, cx, cy + C/2 + 14, sprintf('ArUco id %d   %s   %.0f mm', ids(k), FAMILY, M), ...
         'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', ...
         'Interpreter', 'none');
    text(ax, cx, cy + C/2 + 7, ...
         sprintf('print at 100%%  -  the black square must measure %.1f mm  -  cut on the grey line', M), ...
         'HorizontalAlignment', 'center', 'FontSize', 9);

    % Corner ticks keep the exported page at full paper size
    L = 8;
    for ux = [0 page(1)]
        for uy = [0 page(2)]
            sx = 1 - 2*(ux > 0);  sy = 1 - 2*(uy > 0);
            plot(ax, [ux ux+sx*L], [uy uy], 'k-', 'LineWidth', 0.5);
            plot(ax, [ux ux], [uy uy+sy*L], 'k-', 'LineWidth', 0.5);
        end
    end

    exportgraphics(fig, OUT, 'ContentType', 'vector', ...
                   'Append', k > 1, 'BackgroundColor', 'white');
    close(fig);
end
fprintf('wrote %s  -  %d pages of %g x %g mm\n', OUT, numel(ids), page(1), page(2));

%% ================= LOCAL FUNCTIONS =================
function bits = markerCells(fam, id, nCells)
% Marker as an nCells x nCells logical grid (true = white), sampled at each cell centre.
    K    = 20;
    img  = generateArucoMarker(char(fam), id, nCells*K);
    c    = round(K/2) + (0:nCells-1)*K;
    bits = img(c, c) > 127;
end
