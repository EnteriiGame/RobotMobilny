clear; clc; close all;
prev_champion = [    6.8272    9.4402    0.5000   29.2086   93.5769];
initial_swarm = [
    prev_champion;
    prev_champion .* [1.05, 1.0, 1.0, 0.95, 1.0]; 
    prev_champion .* [0.95, 1.0, 1.0, 1.05, 1.0];
    ];

num_batch_runs   = 40;       
swarmSize        = 30;       
maxIterations    = 80;       
topSpeed         = 2.0;      
num_swarms       = 1;        

lb = [0.5, 0.5, 0.5, 0.1, 0.1];
ub = [20.0, 250.0, 20.0, 50.0, 100.0];

fprintf('Generating %d diverse training curves for multi-swarm race...\n', num_batch_runs);

training_curves = cell(num_batch_runs, 1);
opt = CubicCurveOptimizer(); 
opt.MaxVelocity = topSpeed; 

for i = 1:num_batch_runs
    pts = zeros(2, 4);
    pts(:, 1) = [0; 0];
    current_pt = pts(:, 1);
    
    for p = 2:4
        dist = 10 + (15 - 10) * rand(); 
        angle = (rand() - 0.5) * pi/1.5; 
        if p > 2
            prev_angle = atan2(current_pt(2) - pts(2, p-2), current_pt(1) - pts(1, p-2));
            angle = prev_angle + angle;
        end
        current_pt = current_pt + [dist * cos(angle); dist * sin(angle)];
        pts(:, p) = current_pt;
    end
    
    opt.ControlPoints = pts;
    opt_times = opt.optimize()*3; 
    curve = CubicBCurve();
    curve.ControlPoints = pts;
    curve.SegmentTimes  = opt_times;
    curve.initializeCurve();
    training_curves{i} = curve;
end

objectiveFunc = @(x) staticBatchObjective(x, training_curves);

best_overall_score = Inf;
best_overall_params = [];
swarm_results = struct();

figure('Name', 'Swarm Convergence Race', 'Position', [100, 100, 800, 500]);
hold on; grid on;
colors = lines(num_swarms);

for s = 1:num_swarms
    fprintf('\n========================================\n');
    fprintf('  LAUNCHING SWARM %d / %d (Randomly Seeded)\n', s, num_swarms);
    fprintf('========================================\n');
    
    options = optimoptions('particleswarm', ...
        'SwarmSize', swarmSize, ...
        'MaxIterations', maxIterations, ...
        'Display', 'iter', ...
        'FunctionTolerance', 1e-5,...
        'InitialSwarmMatrix', initial_swarm);
    
    [xBest_s, bestScore_s, exitflag, output] = particleswarm(objectiveFunc, 5, lb, ub, options);
    
    swarm_results(s).params = xBest_s;
    swarm_results(s).score = bestScore_s;
    
    fprintf('Swarm %d Finished! Best Score: %.4f\n', s, bestScore_s);
    disp('Parameters:'); disp(xBest_s);
    
    if bestScore_s < best_overall_score
        best_overall_score = bestScore_s;
        best_overall_params = xBest_s;
    end
end

fprintf('\n========================================\n');
fprintf('  RACE COMPLETE! Overall Best Score: %.4f\n', best_overall_score);
fprintf('  Elite Parameters Found:\n');
disp(best_overall_params);

fprintf('Running validation visualization across all training tracks with champion parameters...\n');
figure('Name', 'Champion Robust Controller Results', 'Position', [50, 50, 1400, 900]);

cols = 5;
rows = ceil(num_batch_runs / cols);
t = tiledlayout(rows, cols, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, sprintf('Champion Controller Performance (Score: %.2f)', best_overall_score));

for i = 1:num_batch_runs
    simEngine = RobotSimulationEngine(training_curves{i}, ...
        'Ts', 0.2, ...
        'KanayamaGains', best_overall_params(1:3), ...
        'Kp', best_overall_params(4), ...
        'Ki', best_overall_params(5), ...
        'EnableLogging', true);
        
    [~, ~, logs] = simEngine.run();
    
    ax = nexttile(t);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    title(ax, sprintf('Track %d', i), 'FontSize', 9);
    
    plot(ax, logs.LUT.pos(:,1), logs.LUT.pos(:,2), 'r--', 'LineWidth', 1);
    plot(ax, logs.pos(:,1), logs.pos(:,2), 'b-', 'LineWidth', 1.2);
end


function finalCost = staticBatchObjective(x, curves)
if any(x <= 0)
    finalCost = 1e6;
    return;
end

num_runs = length(curves);
run_scores = zeros(num_runs, 1);
Ts = 0.2;

for i = 1:num_runs
    simEngine = RobotSimulationEngine(curves{i}, ...
        'Ts', Ts, ...
        'KanayamaGains', x(1:3), ...
        'Kp', x(4), ...
        'Ki', x(5), ...
        'EnableLogging', false); 

    [rawScore, ~, ~] = simEngine.run();

    if isnan(rawScore) || isinf(rawScore)
        run_scores(i) = 1e5; 
        continue;
    end

    total_time = sum(curves{i}.SegmentTimes);
    num_steps = round(total_time / Ts);
    if num_steps <= 0, num_steps = 1; end

    run_scores(i) = rawScore / num_steps;
end

med_score = median(run_scores);
worst_score = max(run_scores);
finalCost = med_score + 0.2 * worst_score;
end
