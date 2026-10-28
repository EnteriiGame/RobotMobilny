% =========================================================
% BATCH TESTING SCRIPT: 15 Random B-Spline Curves
% =========================================================
num_runs = 15;

% TODO: Initialize your custom optimizer object and configure properties
opt = CubicCurveOptimizer(); 
opt.MaxVelocity = 2; 

% Colormap setup for red-to-green error mapping
cmap = turbo(256); 

for i = 1:num_runs
    % -----------------------------------------------------
    % 1. GENERATE 4 REASONABLE CONTROL POINTS (10-15 units apart)
    % -----------------------------------------------------
    pts = zeros(2, 4);
    pts(:, 1) = [0; 0]; % Start at origin
    
    current_pt = pts(:, 1);
    for p = 2:4
        dist = 10 + (15 - 10) * rand(); % Random distance between 10 and 15
        angle = (rand() - 0.5) * pi/1.5; % Random steering angle spread
        
        if p > 2
            prev_angle = atan2(current_pt(2) - pts(2, p-2), current_pt(1) - pts(1, p-2));
            angle = prev_angle + angle;
        end
        
        current_pt = current_pt + [dist * cos(angle); dist * sin(angle)];
        pts(:, p) = current_pt;
    end
    
    % -----------------------------------------------------
    % 2. OPTIMIZE SEGMENT TIMES USING CubicCurveOptimizer
    % -----------------------------------------------------
    % TODO: Invoke optimizer method to return the time vector T
    opt.ControlPoints = pts;
    opt_times = opt.optimize(); 
    
    % Initialize curve per path
    curve = CubicBCurve();
    curve.ControlPoints = pts;
    curve.SegmentTimes  = opt_times;
    curve.initializeCurve();
    
    % -----------------------------------------------------
    % 3. RUN SIMULATION ENGINE WITH TUNED GAINS
    % -----------------------------------------------------
    simEngine = RobotSimulationEngine(curve, ...
        'Ts', 0.2, ...
        'KanayamaGains', [5.1854, 200.8639, 8.5952], ...
        'Kp', 1.4912, ...
        'Ki', 3.7182, ...
        'EnableLogging', true);
        
    [~, ~, logs] = simEngine.run();
    
    % -----------------------------------------------------
    % 4. CALCULATE DISTANCE ERROR & NORMALIZE TIME
    % -----------------------------------------------------
    num_steps = length(logs.LUT.time);
    dist_errors = zeros(num_steps, 1);
    
    for k = 1:num_steps
        ref_pos = logs.LUT.pos(k, :);
        rob_pos = logs.pos(k, :);
        dist_errors(k) = norm(ref_pos - rob_pos);
    end
    
    % Create normalized time vector from 0 to 1
    t_normalized = linspace(0, 1, num_steps);
    
    % -----------------------------------------------------
    % 5. CREATE NEW FIGURE PER RUN (XY + Normalized Error)
    % -----------------------------------------------------
    figure('Name', sprintf('Run %d Analysis', i), 'Position', [100 + mod(i*25, 300), 100 + mod(i*20, 200), 900, 450]);
    
    % Subplot 1: XY Trajectory
    ax_xy = subplot(1, 2, 1);
    title(ax_xy, sprintf('Run %d: XY Trajectory', i));
    xlabel(ax_xy, 'X [m]'); ylabel(ax_xy, 'Y [m]');
    grid(ax_xy, 'on'); hold(ax_xy, 'on'); axis(ax_xy, 'equal');
    
    plot(ax_xy, logs.LUT.pos(:,1), logs.LUT.pos(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    plot(ax_xy, logs.pos(:,1), logs.pos(:,2), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Robot Path');
    legend(ax_xy, 'Location', 'best');
    
    % Subplot 2: Normalized Time Error Graph (Color-coded)
    ax_err = subplot(1, 2, 2);
    title(ax_err, sprintf('Run %d: Normalized Time Error', i));
    xlabel(ax_err, 'Normalized Time [0 to 1]'); ylabel(ax_err, 'Distance Error [m]');
    grid(ax_err, 'on'); hold(ax_err, 'on');
    
    max_err_limit = 2.0; 
    for k = 1:(num_steps-1)
        err_val = min(dist_errors(k), max_err_limit);
        color_idx = max(1, min(256, round((err_val / max_err_limit) * 255) + 1));
        c_rgb = cmap(color_idx, :);
        
        plot(ax_err, [t_normalized(k), t_normalized(k+1)], [dist_errors(k), dist_errors(k+1)], ...
             'Color', c_rgb, 'LineWidth', 1.5);
    end
    
    colormap(ax_err, cmap);
    cb = colorbar(ax_err);
    cb.Label.String = 'Error [m] (Green=Good, Red=High)';
    caxis(ax_err, [0, max_err_limit]);
end
