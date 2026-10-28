% =========================================================
% MULTI-PARAMETER COMPARISON VISUALIZATION TOOL
% =========================================================
% (Assumes 'training_curves' and the 3 parameter sets are in your workspace)

% Define your 3 parameter sets [kx, ky, kth, Kp, Ki]
candidate_params = [
    6.8272    9.4402    0.5000   29.2086   93.5769;
    6.8872 9.7055 0.5001 22.3752 94.3481
    ];

num_params = size(candidate_params, 1);
num_tracks = length(training_curves); % Uses your existing curves pool

figure('Name', 'Controller Parameter Comparison', 'Position', [50, 50, 1400, 900]);

cols = 5;
rows = ceil(num_tracks / cols);
t = tiledlayout(rows, cols, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, 'Comparative Tracking Performance: Run 1 (Blue), Run 2 (Green), Run 3 (Yellow)');

% Colors for the 3 candidate sets
line_colors = {'b-', 'g-', 'y-'};
lineWidths  = [1.5, 1.2, 1.0];

for i = 1:num_tracks
    ax = nexttile(t);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    title(ax, sprintf('Track %d', i), 'FontSize', 9);

    % Plot target reference path once per tile (Red Dashed)
    % We quickly generate reference points from the curve object for plotting
    simEngine_ref = RobotSimulationEngine(training_curves{i}, 'Ts', 0.2, ...
        'KanayamaGains', [1,1,1], 'Kp', 1, 'Ki', 1, 'EnableLogging', true);
    [~, ~, ref_logs] = simEngine_ref.run();
    plot(ax, ref_logs.LUT.pos(:,1), ref_logs.LUT.pos(:,2), 'r--', 'LineWidth', 1.2);

    % Simulate and plot each of the 3 candidate parameter sets
    for p = 1:num_params
        simEngine = RobotSimulationEngine(training_curves{i}, ...
            'Ts', 0.2, ...
            'KanayamaGains', candidate_params(p, 1:3), ...
            'Kp', candidate_params(p, 4), ...
            'Ki', candidate_params(p, 5), ...
            'EnableLogging', true);

        [~, ~, logs] = simEngine.run();

        plot(ax, logs.pos(:,1), logs.pos(:,2), line_colors{p}, 'LineWidth', lineWidths(p));
    end
end

legend(ax, {'Reference', 'Set 1 (Score 14.22)', 'Set 2', 'Set 3'}, 'Location', 'best', 'FontSize', 7);