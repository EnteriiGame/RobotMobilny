classdef CubicCurveOptimizer < handle
    properties
        ControlPoints = [0, 10, 20, 30; 0, 10, 0, 10]; 
        MaxVelocity = 5.0;        % Robot max velocity limit
        TargetVelocityRatio = 0.6;% Target peak velocity (e.g., 60% of max)
        Alpha = 0.5;
        Beta = 0.4;
        
        % Spring-Mass Optimization Parameters
        MaxIterations = 50;
        SpringConstant = 0.3;
        Damping = 0.4;
    end

    methods
        function obj = CubicCurveOptimizer(varargin)
            if nargin > 0
                for i = 1:2:length(varargin)
                obj.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        function SegmentTimes = optimize(obj)
            N = size(obj.ControlPoints, 2);
            numSegments = N - 1;

            % 1. Initialization: Equal time per segment
            T = ones(1, numSegments);
            velocity_history = zeros(1, numSegments);

            for iter = 1:obj.MaxIterations
                % Build temporary curve geometry using current T
                [P, ~] = obj.computeControlPolygon(T);

                % Sample curve and estimate local velocities / curvatures
                [~, segLengths, segCurvatures] = obj.evaluateSegments(P, T);

                % 2. Spring-Mass Target Time Calculation
                targetPeakVel = obj.MaxVelocity * obj.TargetVelocityRatio;

                desiredTimes = zeros(1, numSegments);
                for i = 1:numSegments
                    localMaxVel = targetPeakVel / (1 + 2.0 * segCurvatures(i));
                    desiredTimes(i) = segLengths(i) / max(localMaxVel, 0.1);
                end

                % 3. Spring Relaxation Step with Damping
                force = obj.SpringConstant * (desiredTimes - T);
                velocity_history = (1 - obj.Damping) * velocity_history + force;
                T = T + velocity_history;

                % Hard constraints: prevent negative or zero time chunks
                T = max(0.1, T);
            end

            SegmentTimes = T;
        end
    end
    
    methods (Access = private)
        function [P, totalLength] = computeControlPolygon(obj, T)
            N = size(obj.ControlPoints, 2);
            totalLength = sum(T);
            P = zeros(2, N * 3 - 2);
            
            for i = 1:N 
                P(:, 3 * i - 2) = obj.ControlPoints(:, i); 
            end
            
            startDir = obj.ControlPoints(:, 2) - obj.ControlPoints(:, 1); 
            startDir = startDir / (norm(startDir) + eps);
            endDir = obj.ControlPoints(:, N) - obj.ControlPoints(:, N - 1); 
            endDir = endDir / (norm(endDir) + eps);
            
            P(:, 2) = P(:, 1) + (0 * T(1) / 3) * startDir; % assuming startSpeed = 0 for optimizer
            
            for i = 2:(N - 1)
                dt_prev = T(i-1);
                dt_next = T(i);
                
                incoming_chord = P(:, 3*i - 2) - P(:, 3*i - 5);
                outgoing_chord = P(:, 3*i + 1) - P(:, 3*i - 2);
                
                P(:, 3*i - 1) = P(:, 3*i - 2) + ...
                    (obj.Alpha * (outgoing_chord / dt_next) + (1 - obj.Alpha) * (incoming_chord / dt_prev)) * (obj.Beta * dt_next);
                
                ratio = dt_prev / dt_next;
                P(:, 3*i - 3) = P(:, 3*i - 2) - ratio * (P(:, 3*i - 1) - P(:, 3*i - 2));
            end 
            
            P(:, N*3 - 3) = P(:, N*3 - 2) - (0 * T(end) / 3) * endDir;
        end
        
        function [maxSegVel, segLengths, segCurvatures] = evaluateSegments(obj, P, T)
            numSegments = length(T);
            segLengths = zeros(1, numSegments);
            segCurvatures = zeros(1, numSegments);
            maxSegVel = zeros(1, numSegments);
            
            samplesPerSeg = 10;
            for k = 1:numSegments
                idx = 3 * (k - 1) + 1;
                P0 = P(:, idx); P1 = P(:, idx+1); P2 = P(:, idx+2); P3 = P(:, idx+3);
                
                segLen = 0;
                maxCurv = 0;
                prevPos = P0;
                
                for s = 1:samplesPerSeg
                    tau = s / samplesPerSeg;
                    % Sample position
                    pos = (1 - tau)^3 * P0 + 3 * (1 - tau)^2 * tau * P1 + ...
                          3 * (1 - tau) * tau^2 * P2 + tau^3 * P3;
                    segLen = segLen + norm(pos - prevPos);
                    prevPos = pos;
                    
                    % Sample velocity & acceleration derivatives to find curvature kappa
                    vel_local = 3 * (1 - tau)^2 * (P1 - P0) + ...
                                6 * (1 - tau) * tau * (P2 - P1) + ...
                                3 * tau^2 * (P3 - P2);
                    accel_local = -6 * (1 - tau) * (P1 - P0) + ...
                                  6 * (1 - 2 * tau) * (P2 - P1) + ...
                                  6 * tau * (P3 - P2);
                    
                    v_mag = norm(vel_local / T(k));
                    if v_mag > 1e-5
                        % Curvature formula: kappa = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
                        num_curv = abs(vel_local(1)*accel_local(2) - vel_local(2)*accel_local(1));
                        den_curv = (vel_local(1)^2 + vel_local(2)^2)^(3/2);
                        kappa = num_curv / (den_curv + eps);
                        maxCurv = max(maxCurv, kappa);
                    end
                end
                
                segLengths(k) = segLen;
                segCurvatures(k) = maxCurv;
                maxSegVel(k) = segLengths(k) / T(k);
            end
        end
    end
end