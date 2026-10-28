classdef RobotSimulationEngine < handle
    properties
        Curve           % Instance of CubicBCurve
        Robot           % Instance of RobotMobilny
        KanayamaGains = [2.0, 1.5, 1.0] % [kx, ky, kth]
        Kp = 4.0        % Wheel PI Proportional gain
        Ki = 21.0       % Wheel PI Integral gain
        Ts = 0.2        % Time step
        EnableLogging = true % Toggle telemetry logging on/off
        
        % Scoring weights for optimization
        Weights = struct('dist', 20.0, 'angle', 5.0, 'cross', 2.0, 'steer', 10.0, 'accel', 5.0);
    end
    
    properties (Access = private)
        LUT             % Precomputed Look-Up Table
        num_steps       = 0
        logs            = struct()
        scoreData       = struct()
    end
    
    methods
        function obj = RobotSimulationEngine(curveObj, varargin)
            % Constructor: Accepts a curve object and optional name-value pairs
            if nargin > 0 && ~isempty(curveObj)
                obj.Curve = curveObj;
            else
                obj.Curve = CubicBCurve(); % Default fallback
            end
            
            % Parse optional name-value arguments
            for i = 1:2:length(varargin)
                obj.(varargin{i}) = varargin{i+1};
            end
            
            obj.Robot = RobotMobilny();
            obj.Robot.Ts = obj.Ts;
        end
        
        function precomputeLUT(obj)
            total_len = obj.Curve.getTotalLength();
            time_vec = 0 : obj.Ts : total_len;
            obj.num_steps = length(time_vec);
            
            obj.LUT.time         = time_vec;
            obj.LUT.pos          = zeros(obj.num_steps, 2);
            obj.LUT.vel          = zeros(obj.num_steps, 2);
            obj.LUT.accel        = zeros(obj.num_steps, 2);
            obj.LUT.heading      = zeros(obj.num_steps, 2);
            obj.LUT.speed        = zeros(obj.num_steps, 1);
            obj.LUT.frenet_frame = zeros(2, 2, obj.num_steps); 
            
            for k = 1:obj.num_steps
                t = time_vec(k);
                [pos, vel, accel, heading, speed, frenet, ~, ~] = obj.Curve.evaluate(t);
                obj.LUT.pos(k, :)             = pos(:)';
                obj.LUT.vel(k, :)             = vel(:)';
                obj.LUT.accel(k, :)           = accel(:)';
                obj.LUT.heading(k, :)         = heading(:)';
                obj.LUT.speed(k)              = speed;
                obj.LUT.frenet_frame(:, :, k) = frenet;
            end
        end

        function [totalScore, scoreBreakdown, logs] = run(obj)
            if isempty(obj.LUT) || ~isfield(obj.LUT, 'pos')
                obj.precomputeLUT();
            end

            obj.Robot.reset();
            if obj.EnableLogging
                obj.Robot.initTelemetry(obj.num_steps);
            else
                obj.Robot.initTelemetry(0); 
            end

            int_error = [0; 0];
            pose_global = [0, 0];
            heading_vec = [1, 0];
            frenet_inv  = [1, 0; 0, 1];
            w_wheels    = [0, 0];

            % Raw history buffers for scoring (fixed, never changes)
            history.e_xy   = zeros(obj.num_steps, 2);
            history.e_sPhi = zeros(obj.num_steps, 2);
            history.v_d    = zeros(obj.num_steps, 1);
            history.w_d    = zeros(obj.num_steps, 1);

            for k = 1:obj.num_steps
                pos_ref    = obj.LUT.pos(k, :)';
                vel_ref_v  = obj.LUT.vel(k, :)';
                accel_ref  = obj.LUT.accel(k, :)';
                speed_ref  = obj.LUT.speed(k);
                frenet_ref = obj.LUT.frenet_frame(:, :, k);

                v_ref_struct = [0; speed_ref];       
                a_ref_struct = [norm(accel_ref); 0]; 

                pos_diff_global = pos_ref - pose_global(:);
                e_xy = frenet_inv * pos_diff_global; 
                e_sPhi = frenet_ref * heading_vec(:); 

                [v_d, w_d] = obj.kanayama(e_xy, e_sPhi, v_ref_struct, a_ref_struct);

                v_w_des = obj.Pred_LP(v_d, w_d);
                error_w = v_w_des - w_wheels(:);
                int_error = int_error + error_w * obj.Ts;

                q = obj.Kp * error_w + obj.Ki * int_error;
                q = max(min(q, 1.0), -1.0); 

                [~, ~, ~, w_wheels, ~, ~, pose_global, heading_vec, frenet_inv] = obj.Robot.step(q, [0, 0], 0);

                % Save raw data to history
                history.e_xy(k, :)   = e_xy(:)';
                history.e_sPhi(k, :) = e_sPhi(:)';
                history.v_d(k)       = v_d;
                history.w_d(k)       = w_d;
            end

            % Delegate grading entirely to evaluateScore using vector math
            [totalScore, scoreBreakdown] = obj.evaluateScore(history);

            if obj.EnableLogging
                logs = obj.Robot.getTelemetry();
                logs.LUT = obj.LUT;
            else
                logs = struct();
            end
            obj.logs = logs;
            obj.Robot.initTelemetry(0); 
        end

        function LUT = getLUT(obj)
            if isempty(obj.LUT.pos)
                obj.precomputeLUT();
            end
            LUT = obj.LUT;
        end

        function [totalScore, scoreBreakdown] = evaluateScore(obj, history)
            N = obj.num_steps;
            w = obj.Weights; 

            % Extract raw arrays from history
            dist_err_sq = sum(history.e_xy.^2, 2); % [e_x^2 + e_y^2] per step
            sin_dphi    = history.e_sPhi(:, 1);
            cos_dphi    = history.e_sPhi(:, 2);
            v_d         = history.v_d;
            w_d         = history.w_d;

            % --- VECTORIZED BASE PENALTIES ---
            distanceCost = w.dist * (sum(dist_err_sq) / N);

            % True squared angle penalty without atan2
            angle_pen    = 2 * (1 - cos_dphi);
            angleCost    = w.angle * (sum(angle_pen) / N);

            % Cross-penalty: distance error * |sin(dphi)|
            dist_mag     = sqrt(dist_err_sq);
            cross_pen    = dist_mag .* abs(sin_dphi);
            crossCost    = w.cross * (sum(cross_pen) / N);

            % Steering effort penalty (w_d^2)
            steerCost    = w.steer * (sum(w_d.^2) / N);

            % Tangent acceleration penalty (dv_d^2)
            dv_d         = diff([v_d(1); v_d]) / obj.Ts;
            accelCost    = w.accel * (sum(dv_d.^2) / N);

            % --- ANTI-OSCILLATION PENALTY (Command Jitter) ---
            % Penalizes rapid sign-flips or high-frequency jerks in angular velocity command w_d
            w_d_changes  = diff([w_d(1); w_d]);
            oscillationCost = 5.0 * (sum(abs(w_d_changes)) / N);

            scoreBreakdown = struct(...
                'distanceCost', distanceCost, ...
                'angleCost', angleCost, ...
                'crossCost', crossCost, ...
                'steerCost', steerCost, ...
                'accelCost', accelCost, ...
                'oscillationCost', oscillationCost ...
                );

            totalScore = distanceCost + angleCost + crossCost + steerCost + accelCost + oscillationCost;
        end

    end
    
    methods (Access = private)
        function v_w = Pred_LP(~, v_d, w_d)
            L = 0.2;
            v_w = [v_d - w_d * L/2;
                   v_d + w_d * L/2];
        end

        function [v_d, w_d] = kanayama(obj, e_xy, e_sPhi, v_ref, a_ref)
            kx = obj.KanayamaGains(1); 
            ky = obj.KanayamaGains(2); 
            kth = obj.KanayamaGains(3);
            
            eps_v = 1e-2;
            v_r = v_ref(2);
            w_r = a_ref(1) * v_r / (v_r^2 + eps_v^2);
            
            v_d = max(v_r * e_sPhi(2) + kx * e_xy(1), 0);
            w_d = w_r + v_r * (ky * e_xy(2) + kth * e_sPhi(1));
        end
    end
end