classdef CubicBCurve < handle
    properties 
        ControlPoints = [0, 10, 20, 30; 0, 10, 0, 10]; 
        alpha = 0.5;
        beta = 0.4;
        startSpeed = 0;
        endSpeed = 0;
        SegmentTimes = [87.6358, 82.6367, 65.2548]; 
        P = [];
        N = 0;
        T = [];
        totalLength = 0;
    end
    
    methods
        function obj = CubicBCurve(varargin)
            if nargin > 0
                for i = 1:2:nargin
                    obj.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        function obj = initializeCurve(obj)
            obj.N = size(obj.ControlPoints, 2);
            
            if isempty(obj.SegmentTimes)
                obj.T = ones(1, obj.N - 1);
            else
                obj.T = obj.SegmentTimes(:)';
            end
            
            obj.totalLength = sum(obj.T);
            obj.P = zeros(2, obj.N * 3 - 2);
            
            for i = 1:obj.N 
                obj.P(:, 3 * i - 2) = obj.ControlPoints(:, i); 
            end
            
            startDir = obj.ControlPoints(:, 2) - obj.ControlPoints(:, 1); 
            startDir = startDir / (norm(startDir) + eps);
            endDir = obj.ControlPoints(:, obj.N) - obj.ControlPoints(:, obj.N - 1); 
            endDir = endDir / (norm(endDir) + eps);
            
            obj.P(:, 2) = obj.P(:, 1) + (obj.startSpeed * obj.T(1) / 3) * startDir;
            
            for i = 2:(obj.N - 1)
                dt_prev = obj.T(i-1);
                dt_next = obj.T(i);
                
                incoming_chord = obj.P(:, 3*i - 2) - obj.P(:, 3*i - 5);
                outgoing_chord = obj.P(:, 3*i + 1) - obj.P(:, 3*i - 2);
                
                obj.P(:, 3*i - 1) = obj.P(:, 3*i - 2) + ...
                    (obj.alpha * (outgoing_chord / dt_next) + (1 - obj.alpha) * (incoming_chord / dt_prev)) * (obj.beta * dt_next);
                
                ratio = dt_prev / dt_next;
                obj.P(:, 3*i - 3) = obj.P(:, 3*i - 2) - ratio * (obj.P(:, 3*i - 1) - obj.P(:, 3*i - 2));
            end 
            
            obj.P(:, obj.N*3 - 3) = obj.P(:, obj.N*3 - 2) - (obj.endSpeed * obj.T(end) / 3) * endDir;
        end
        
        function [pos, vel, accel, heading_vec, tangent_speed, frenet_frame, accel_tan, accel_norm] = evaluate(obj, t)
            if isempty(obj.P) || size(obj.P, 2) ~= (size(obj.ControlPoints, 2) * 3 - 2)
                obj.initializeCurve();
            end
            [k, tau] = obj.getSegmentAndTau(t);
            
            numSegments = length(obj.T);
            k = max(1, min(round(double(k)), numSegments));
            
            idx = 3 * (k - 1) + 1;
            max_idx = size(obj.P, 2) - 3;
            idx = max(1, min(round(double(idx)), max_idx));
            
            P0 = obj.P(:, idx); 
            P1 = obj.P(:, idx+1); 
            P2 = obj.P(:, idx+2); 
            P3 = obj.P(:, idx+3);
            
            pos = (1 - tau)^3 * P0 + 3 * (1 - tau)^2 * tau * P1 + ...
                  3 * (1 - tau) * tau^2 * P2 + tau^3 * P3;
              
            vel_local = 3 * (1 - tau)^2 * (P1 - P0) + ...
                        6 * (1 - tau) * tau * (P2 - P1) + ...
                        3 * tau^2 * (P3 - P2);
            vel = vel_local / obj.T(k);
            tangent_speed = norm(vel);
            
            heading_vec = vel / (tangent_speed + eps);
            normal_vec = [-heading_vec(2); heading_vec(1)];
            frenet_frame = [heading_vec(:)'; normal_vec(:)'];
            
            accel_local = -6 * (1 - tau) * (P1 - P0) + ...
                          6 * (1 - 2 * tau) * (P2 - P1) + ...
                          6 * tau * (P3 - P2);
            accel = accel_local / (obj.T(k)^2);
            
            accel_tan  = dot(accel, heading_vec);
            accel_norm = dot(accel, normal_vec);
        end
        
        function [k, tau] = getSegmentAndTau(obj, t)
            t_clamped = max(0, min(t, obj.totalLength));
            numSegments = length(obj.T);
            
            cumT = [0, cumsum(obj.T)];
            
            if t_clamped >= obj.totalLength - 1e-9
                k = numSegments;
                tau = 1.0;
                return;
            end
            
            k = find(t_clamped >= cumT(1:end-1), 1, 'last');
            
            if isempty(k)
                k = 1;
            end
            
            k = max(1, min(k, numSegments));
            tau = (t_clamped - cumT(k)) / obj.T(k);
            tau = max(0.0, min(1.0, tau));
        end
        
        function len = getTotalLength(obj)
            if obj.totalLength == 0
                obj.initializeCurve();
            end
            len = obj.totalLength;
        end
    end
end