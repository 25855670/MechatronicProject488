%% Calibration checkerboard - writes a printable checkerboard of known square size to
%% the Demonstration folder for camera_calibrate.m. Print at 100% and check the 100 mm bar.

%% ================= BOARD SETTINGS =================
% Number of squares [across down] - one even, one odd so the orientation is unambiguous
SQUARES = [12 9];
% Square size [mm]
SQUARE_MM = 60;
% White border around the board [mm]
MARGIN_MM = 12;

%% ================= PAGE SETTINGS =================
% Paper name and size [width height] [mm]
PAPER   = "A1";
PAGE_MM = [594 841];
% Output file, named after the board, in the Demonstration folder
OUT = fullfile(fileparts(mfilename('fullpath')), '..', ...
               sprintf('checkerboard_%s_%dx%d_%gmm.pdf', PAPER, SQUARES(1), SQUARES(2), SQUARE_MM));

%% ================= LAYOUT =================
% Board size [across down] [mm]
board = SQUARES * SQUARE_MM;
% Turn the page to landscape if the board only fits that way
page = PAGE_MM;
if any(board + 2*MARGIN_MM > page), page = fliplr(page); end
assert(all(board + 2*MARGIN_MM <= page), 'The board does not fit on %s paper.', PAPER);

%% ================= DRAW =================
% Data units are millimetres on a page of the same size, so it prints at true scale
fig = figure('Visible', 'off', 'Units', 'centimeters', ...
             'Position', [1 1 page/10], 'Color', 'w', ...
             'PaperUnits', 'centimeters', 'PaperSize', page/10, ...
             'PaperPositionMode', 'manual', 'PaperPosition', [0 0 page/10]);
ax = axes(fig, 'Units', 'normalized', 'Position', [0 0 1 1]);
hold(ax, 'on');
ax.XLim = [0 page(1)];  ax.YLim = [0 page(2)];
axis(ax, 'off');

x0 = (page(1) - board(1)) / 2;
y0 = (page(2) - board(2)) / 2;
for c = 1:SQUARES(1)
    for r = 1:SQUARES(2)
        if mod(c + r, 2) == 0
            rectangle(ax, 'Position', [x0 + (c-1)*SQUARE_MM, y0 + (r-1)*SQUARE_MM, SQUARE_MM, SQUARE_MM], ...
                      'FaceColor', 'k', 'EdgeColor', 'none');
        end
    end
end

% 100 mm measuring bar, to confirm the print was not scaled
barY = y0 - 7;
plot(ax, [x0 x0+100], [barY barY], 'k-', 'LineWidth', 2);
plot(ax, [x0 x0], barY + [-2 2], 'k-', 'LineWidth', 2);
plot(ax, [x0+100 x0+100], barY + [-2 2], 'k-', 'LineWidth', 2);
text(ax, x0 + 104, barY, 'this bar must measure 100.0 mm', ...
     'FontSize', 8, 'VerticalAlignment', 'middle');

text(ax, x0, y0 + board(2) + 6, ...
     sprintf('%d x %d squares of %g mm  (%d x %d inner corners)  -  print at 100%%, DO NOT fit to page', ...
             SQUARES(1), SQUARES(2), SQUARE_MM, SQUARES(1)-1, SQUARES(2)-1), 'FontSize', 8);

print(fig, '-dpdf', '-vector', OUT);
close(fig);
fprintf('wrote %s\n', OUT);
