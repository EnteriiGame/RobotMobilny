% ==========================================
% 1. SETUP CURVE & ENGINE WITH STARTING PARAMS
% ==========================================
curve = CubicBCurve();
curve.ControlPoints = [0, 10, 20, 30; 0, 10, 0, 10];
curve.SegmentTimes  = [87.6358, 82.6367, 65.2548].*3;
curve.initializeCurve();

% Initialize engine with starting parameters
simEngine = RobotSimulationEngine(curve, ...
    'Ts', 0.2, ...
    'KanayamaGains', [2.0, 1.5, 1.0], ... % [kx, ky, kth]
    'Kp', 4.0, ...                       % Wheel PI P gain
    'Ki', 21.0, ...                      % Wheel PI I gain
    'EnableLogging', true);

% ==========================================
% 2. BASELINE RUN (STARTING PARAMETERS)
% ==========================================
fprintf('========================================\n');
fprintf('  BASELINE RUN (STARTING PARAMETERS)\n');
fprintf('========================================\n');

[startScore, startBreakdown, startLogs] = simEngine.run();

fprintf('Starting Score:        %.4f\n', startScore);
fprintf('  - Distance Cost:     %.4f\n', startBreakdown.distanceCost);
fprintf('  - Angle Alignment:   %.4f\n', startBreakdown.angleCost);
fprintf('  - Acceleration Cost: %.4f\n', startBreakdown.accelCost);
fprintf('----------------------------------------\n\n');

% ==========================================
% 3. OPTIMIZATION SETUP (Optimization Toolbox)
% ==========================================
% Parameter vector: x = [kx, ky, kth, Kp, Ki]
x0 = [simEngine.KanayamaGains, simEngine.Kp, simEngine.Ki];

% Objective function handle (disables logging during optimization for speed)
objectiveFunc = @(x) evaluateObjective(simEngine, x);

% Configure options: max 320 function evaluations/runs, display iteration text
options = optimset('MaxFunEvals', 320, 'Display', 'iter');

fprintf('========================================\n');
fprintf('  STARTING OPTIMIZATION (Max 20 Runs)\n');
fprintf('========================================\n');

[xBest, bestScore] = fminsearch(objectiveFunc, x0, options);

fprintf('\n========================================\n');
fprintf('  OPTIMIZATION COMPLETE\n');
fprintf('========================================\n');
fprintf('Best Score Found: %.4f\n', bestScore);
fprintf('Best Parameters [kx, ky, kth, Kp, Ki]:\n');
disp(xBest);

% ==========================================
% 4. FINAL RUN & LOGGING WITH BEST PARAMETERS
% ==========================================
xBest=[    5.1854  200.8639    8.5952    1.4912    3.7182]
simEngine.KanayamaGains = xBest(1:3);
simEngine.Kp = xBest(4);
simEngine.Ki = xBest(5);
simEngine.EnableLogging = true; % Re-enable logging for final visualization

[finalScore, finalBreakdown, bestLogs] = simEngine.run();

fprintf('Final Run Score:       %.4f\n', finalScore);
fprintf('  - Distance Cost:     %.4f\n', finalBreakdown.distanceCost);
fprintf('  - Angle Alignment:   %.4f\n', finalBreakdown.angleCost);
fprintf('  - Acceleration Cost: %.4f\n', finalBreakdown.accelCost);
fprintf('========================================\n\n');

% ==========================================
% 5. VISUALIZE BEST RUN IN TIMELINE INSPECTOR
% ==========================================
visualize_simulation(bestLogs.LUT, startLogs, bestLogs);

% ==========================================
% HELPER FUNCTION FOR OBJECTIVE EVALUATION
% ==========================================
function score = evaluateObjective(engine, x)
if any(x <= 0)
    score = 1e6; 
    return;
end

engine.KanayamaGains = x(1:3);
engine.Kp = x(4);
engine.Ki = x(5);
engine.EnableLogging = true; % Keep logging enabled for accurate scoring

% Run simulation; memory buffers are cleared right after scoring finishes
[score, ~, ~] = engine.run(); 
end

function visualize_simulation(LUT, logs_start, logs_opt)
num_steps = length(LUT.time);

% Create interactive comparison window
fig = uifigure('Name', 'Robot Simulation Comparison Inspector', 'Position', [100, 100, 900, 700]);
ax = uiaxes(fig, 'Position', [50, 100, 800, 560]);
title(ax, 'Baseline vs. Optimized Robot Trajectory Comparison');
xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
grid(ax, 'on'); hold(ax, 'on');
axis(ax, 'equal');

% 1. Plot static paths
plot(ax, LUT.pos(:,1), LUT.pos(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference Trajectory');
plot(ax, logs_start.pos(:,1), logs_start.pos(:,2), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'Baseline Path');
plot(ax, logs_opt.pos(:,1), logs_opt.pos(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Optimized Path');

% 2. Initialize dynamic markers for step 1
h_ref_pt      = plot(ax, LUT.pos(1,1), LUT.pos(1,2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', 'Ref Point');
h_robot_start = plot(ax, logs_start.pos(1,1), logs_start.pos(1,2), 'o', 'Color', [0.4 0.4 0.4], 'MarkerSize', 8, 'MarkerFaceColor', [0.7 0.7 0.7], 'DisplayName', 'Baseline Robot');
h_robot_opt   = plot(ax, logs_opt.pos(1,1), logs_opt.pos(1,2), 'ko', 'MarkerSize', 9, 'MarkerFaceColor', 'b', 'DisplayName', 'Optimized Robot');

legend(ax, 'Location', 'best');

isPlaying = false;

% 3. UI Timeline Controls
sld = uislider(fig, 'Position', [180, 40, 400, 3], ...
    'Limits', [1, num_steps], ...
    'Value', 1, ...
    'MajorTicks', round(linspace(1, num_steps, min(num_steps, 6))), ...
    'ValueChangingFcn', @(~, event) updatePlot(round(event.Value)), ...
    'ValueChangedFcn', @(~, event) updatePlot(round(sld.Value)));

lbl = uilabel(fig, 'Position', [50, 30, 120, 22], ...
    'Text', sprintf('Step: 1 / %d', num_steps), ...
    'FontSize', 12, 'FontWeight', 'bold');

btnPlay = uibutton(fig, 'Position', [610, 25, 80, 30], ...
    'Text', 'Play', ...
    'ButtonPushedFcn', @(~, ~) togglePlay());

    function updatePlot(k)
        k = max(1, min(k, num_steps));
        sld.Value = k;

        % Update reference point
        h_ref_pt.XData = LUT.pos(k, 1);
        h_ref_pt.YData = LUT.pos(k, 2);

        % Update baseline robot marker
        if k <= size(logs_start.pos, 1)
            h_robot_start.XData = logs_start.pos(k, 1);
            h_robot_start.YData = logs_start.pos(k, 2);
        end

        % Update optimized robot marker
        if k <= size(logs_opt.pos, 1)
            h_robot_opt.XData = logs_opt.pos(k, 1);
            h_robot_opt.YData = logs_opt.pos(k, 2);
        end

        lbl.Text = sprintf('Step: %d / %d', k, num_steps);
    end

    function togglePlay()
        isPlaying = ~isPlaying;
        if isPlaying
            btnPlay.Text = 'Pause';
            startPlayback();
        else
            btnPlay.Text = 'Play';
        end
    end

    function startPlayback()
        current_k = round(sld.Value);
        if current_k >= num_steps, current_k = 1; end

        dt = 0.2; 
        if num_steps > 1, dt = LUT.time(2) - LUT.time(1); end

        while isPlaying && isvalid(fig)
            current_k = current_k + 1;
            if current_k > num_steps, current_k = 1; end
            updatePlot(current_k);
            drawnow limitrate;
            pause(max(dt, 0.01)); 
        end
    end
end