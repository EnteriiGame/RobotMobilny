% ==========================================
% 1. INITIALIZE CURVE & PRECOMPUTE LUT
% ==========================================
curve = CubicBCurve();
curve.ControlPoints = [0, 10, 20, 30; 0, 10, 0, 10];
curve.SegmentTimes  = [87.6358, 82.6367, 65.2548];
curve.initializeCurve();

Ts = 0.2;
total_len = curve.getTotalLength();
time_vec = 0 : Ts : total_len;
num_steps = length(time_vec);

% Preallocate L ook-Up Table (LUT)
LUT.time         = time_vec;
LUT.pos          = zeros(num_steps, 2);
LUT.vel          = zeros(num_steps, 2);
LUT.accel        = zeros(num_steps, 2);
LUT.heading      = zeros(num_steps, 2);
LUT.speed        = zeros(num_steps, 1);
LUT.frenet_frame = zeros(2, 2, num_steps); 

for k = 1:num_steps
    t = time_vec(k);
    [pos, vel, accel, heading, speed, frenet, ~, ~] = curve.evaluate(t);
    LUT.pos(k, :)             = pos(:)';
    LUT.vel(k, :)             = vel(:)';
    LUT.accel(k, :)           = accel(:)';
    LUT.heading(k, :)         = heading(:)';
    LUT.speed(k)              = speed;
    LUT.frenet_frame(:, :, k) = frenet;
end

% ==========================================
% 2. INITIALIZE ROBOT & CONTROLLER GAINS
% ==========================================
robot = RobotMobilny();
robot.initTelemetry(num_steps); % Initializes logging buffers automatically

% Kanayama gains [kx, ky, kth]
%K_kanayama = [2.0, 1.5, 1.0]; 
K_kanayama = [3.0, 6.0 , 5.5]; 

% Wheel PI Controller Gains
Kp = 4.0;
Ki = 21.0;
int_error = [0; 0]; % Integrator state for [Left; Right] wheels

% Initialize state variables for step 1
pose_global = [0, 0];
heading_vec = [1, 0];
frenet_inv  = [1, 0; 0, 1];
w_wheels    = [0, 0];

% ==========================================
% 3. SIMULATION LOOP (FULL STEP)
% ==========================================
for k = 1:num_steps
    % A. Evaluate 1 point along the curve for the current step
    pos_ref    = LUT.pos(k, :)';
    vel_ref_v  = LUT.vel(k, :)';
    accel_ref  = LUT.accel(k, :)';
    speed_ref  = LUT.speed(k);
    frenet_ref = LUT.frenet_frame(:, :, k);
    
    v_ref_struct = [0; speed_ref];       % Matches v_ref(2) in kanayama
    a_ref_struct = [norm(accel_ref); 0]; % Curvature/accel metric approximation

    % B. Compute Position Errors in Car Frame
    pos_diff_global = pos_ref - pose_global(:);
    e_xy = frenet_inv * pos_diff_global; % 2x1 [e_x; e_y] in car frame

    % C. Compute Heading Errors (sin(dphi), cos(dphi))
    e_sPhi = frenet_ref * heading_vec(:); % 2x1 orientation delta

    % D. Kanayama Controller -> Desired Body Velocities
    [v_d, w_d] = kanayama(e_xy, e_sPhi, v_ref_struct, a_ref_struct, K_kanayama);

    % E. Wheel Speed Generator (Pred_LP) -> Desired Wheel Velocities
    v_w_des = Pred_LP(v_d, w_d); % [v_wL; v_wR]

    % F. PI Controller around Wheel Speeds -> Control Inputs q = [q_L, q_R]
    error_w = v_w_des - w_wheels(:);
    int_error = int_error + error_w * Ts;
    
    % Compute normalized control signals (clamped between -1 and 1)
    q = Kp * error_w + Ki * int_error;
    q = max(min(q, 1.0), -1.0); 

    % G. Step the Robot Physics Engine
    [~, ~, ~, w_wheels, ~, ~, pose_global, heading_vec, frenet_inv] = robot.step(q, [0, 0], 0);
end

% ==========================================
% 4. EXTRACT & PLOT TELEMETRY
% ==========================================
logs = robot.getTelemetry();
visualize_simulation(LUT, logs);

function v_w = Pred_LP(v_d, w_d)
L = 0.2;
v_w = [v_d - w_d * L/2;
    v_d + w_d * L/2];
end

function [v_d, w_d] = kanayama(e_xy, e_sPhi, v_ref, a_ref, K)
kx = K(1); ky = K(2); kth = K(3);
eps_v = 1e-2;
v_r = v_ref(2);
w_r = a_ref(1) * v_r / (v_r^2 + eps_v^2);

v_d = max(v_r * e_sPhi(2) + kx * e_xy(1), 0);
w_d = w_r + v_r * (ky * e_xy(2) + kth * e_sPhi(1));
end

function visualize_simulation(LUT, logs)
    num_steps = length(LUT.time);
    
    % Create interactive figure window
    fig = uifigure('Name', 'Robot Simulation Timeline Inspector', 'Position', [100, 100, 850, 700]);
    ax = uiaxes(fig, 'Position', [50, 100, 750, 560]);
    title(ax, 'Robot Trajectory Timeline Inspector');
    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
    grid(ax, 'on'); hold(ax, 'on');
    axis(ax, 'equal');
    
    % 1. Plot static elements (Reference & Full Path)
    plot(ax, LUT.pos(:,1), LUT.pos(:,2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference Trajectory');
    plot(ax, logs.pos(:,1), logs.pos(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Robot Path');
    
    % 2. Initialize dynamic handles for current step (k = 1)
    h_ref_pt  = plot(ax, LUT.pos(1,1), LUT.pos(1,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Target Ref Point');
    h_robot   = plot(ax, logs.pos(1,1), logs.pos(1,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Robot Position');
    h_heading = quiver(ax, logs.pos(1,1), logs.pos(1,2), cos(logs.phi(1)), sin(logs.phi(1)), 0.6, 'g', 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Robot Heading');
    
    legend(ax, 'Location', 'best');
    
    % Playback state variable
    isPlaying = false;
    
    % 3. UI Slider with continuous live tracking (ValueChangingFcn)
    sld = uislider(fig, 'Position', [180, 40, 380, 3], ...
        'Limits', [1, num_steps], ...
        'Value', 1, ...
        'MajorTicks', round(linspace(1, num_steps, min(num_steps, 6))), ...
        'ValueChangingFcn', @(~, event) updatePlot(round(event.Value)), ...
        'ValueChangedFcn', @(~, event) updatePlot(round(sld.Value)));
        
    % Time/Step Indicator Label
    lbl = uilabel(fig, 'Position', [50, 30, 110, 22], ...
        'Text', sprintf('Step: 1 / %d', num_steps), ...
        'FontSize', 12, 'FontWeight', 'bold');

    % Play / Pause Button
    btnPlay = uibutton(fig, 'Position', [590, 25, 80, 30], ...
        'Text', 'Play', ...
        'ButtonPushedFcn', @(~, ~) togglePlay());

    % Nested update callback function
    function updatePlot(k)
        k = max(1, min(k, num_steps));
        sld.Value = k; % Keep slider synchronized during playback
        
        % Update reference target point
        h_ref_pt.XData = LUT.pos(k, 1);
        h_ref_pt.YData = LUT.pos(k, 2);
        
        % Update robot position and heading vector
        if k <= size(logs.pos, 1)
            rx = logs.pos(k, 1);
            ry = logs.pos(k, 2);
            rphi = logs.phi(k);
            
            h_robot.XData = rx;
            h_robot.YData = ry;
            
            h_heading.XData = rx;
            h_heading.YData = ry;
            h_heading.UData = 0.6 * cos(rphi);
            h_heading.VData = 0.6 * sin(rphi);
            
            lbl.Text = sprintf('Step: %d / %d', k, num_steps);
        end
    end

    % Toggle Play/Pause state
    function togglePlay()
        isPlaying = ~isPlaying;
        if isPlaying
            btnPlay.Text = 'Pause';
            startPlayback();
        else
            btnPlay.Text = 'Play';
        end
    end

    % Automatic looping playback loop
    function startPlayback()
        current_k = round(sld.Value);
        if current_k >= num_steps
            current_k = 1;
        end
        
        % Determine pause delta based on configured Ts (with fallback)
        dt = 0.2; 
        if num_steps > 1
            dt = LUT.time(2) - LUT.time(1);
        end
        
        while isPlaying && isvalid(fig)
            current_k = current_k + 1;
            if current_k > num_steps
                current_k = 1; % Loop continuously back to start
            end
            
            updatePlot(current_k);
            drawnow limitrate;
            
            % Match time scaling (capped for responsive UI)
            pause(max(dt, 0.01)); 
        end
    end
end