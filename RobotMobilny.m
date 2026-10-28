classdef RobotMobilny < handle
    properties
        Ts     = 0.2          
        mc     = 5.0           
        mw     = 2.0           
        Iz     = 1.00          
        Iw     = 0.2          
        R      = 0.05          
        L      = 0.20          
        A      = 0.02          
        B      = 0.15          
        g      = 10            
        mu     = 1.9           
        kx     = 20            
        ky     = 10            
        tauMax = 1.0       
        wMax   = 20        
        tauF   = 0.05      
    end
    
    properties (Access = private)
        pos   = [0, 0]        
        v     = [0, 0]        
        phi   = 0            
        phid  = 0            
        w_L   = 0            
        w_R   = 0            
        tau_L = 0          
        tau_R = 0          
        acc   = [0, 0]      
        
        % Telemetry internal buffers
        log_steps = 0
        log_idx   = 1
        log_pos   
        log_v     
        log_phi   
        log_phid  
        log_wheels
        log_tau   
    end
    
    methods
        function obj = RobotMobilny(varargin)
            obj.reset();
        end
        
        function reset(obj)
            obj.pos   = [0, 0];        
            obj.v     = [0, 0];        
            obj.phi   = 0;            
            obj.phid  = 0;            
            obj.w_L   = 0;            
            obj.w_R   = 0;            
            obj.tau_L = 0;          
            obj.tau_R = 0;          
            obj.acc   = [0, 0];      
            obj.log_idx = 1;
        end
        
        function initTelemetry(obj, max_steps)
            obj.log_steps = max_steps;
            obj.log_idx   = 1;

            % Explicitly force 2 columns for X and Y positions
            obj.log_pos    = zeros(max_steps, 2);
            obj.log_v      = zeros(max_steps, 2);
            obj.log_phi    = zeros(max_steps, 1);
            obj.log_phid   = zeros(max_steps, 1);
            obj.log_wheels = zeros(max_steps, 2);
            obj.log_tau    = zeros(max_steps, 2);
        end

        function logs = getTelemetry(obj)
            valid = 1:(obj.log_idx - 1);
            logs = struct();

            % Ensure pos is safely sliced as an N x 2 matrix
            if ~isempty(obj.log_pos)
                logs.pos = reshape(obj.log_pos(valid, :), [], 2);
            else
                logs.pos = zeros(0, 2);
            end

            logs.v      = obj.log_v(valid, :);
            logs.phi    = obj.log_phi(valid);
            logs.phid   = obj.log_phid(valid);
            logs.wheels = obj.log_wheels(valid, :);
            logs.tau    = obj.log_tau(valid, :);
        end
        function [acc_body, phid_out, phi, w_wheels, tau_wheels, v_body, pose_global, heading_vec, frenet_inv] = step(obj, q, F_ext, Mz_ext)
            Ts = obj.Ts;
            M  = obj.mc + 2*obj.mw;
            
            q_L = q(1);
            q_R = q(2);
            Fx_ext = F_ext(1);
            Fy_ext = F_ext(2);
            
            obj.tau_L = q_L*obj.tauMax - obj.tauMax/obj.wMax*obj.w_L;
            obj.tau_R = q_R*obj.tauMax - obj.tauMax/obj.wMax*obj.w_R;
            
            vx_L = obj.v(1) - obj.L/2*obj.phid;
            vx_R = obj.v(1) + obj.L/2*obj.phid;
            vy_w = obj.v(2) - obj.A*obj.phid;
            
            slip_L = obj.R*obj.w_L - vx_L;
            slip_R = obj.R*obj.w_R - vx_R;
            
            FxReq_L = obj.kx*slip_L;
            FxReq_R = obj.kx*slip_R;
            FyReq   = -obj.ky*vy_w;
            
            Fmax = obj.mu*M*obj.g*obj.B/(obj.A + obj.B)/2;
            [Fx_L, Fy_L] = obj.limitForce(FxReq_L, FyReq, Fmax);
            [Fx_R, Fy_R] = obj.limitForce(FxReq_R, FyReq, Fmax);
            
            Fx = Fx_L + Fx_R + Fx_ext;
            Fy = Fy_L + Fy_R + Fy_ext;
            
            du  = Fx/M + obj.phid*obj.v(2);
            dvy = Fy/M - obj.phid*obj.v(1);
            
            Mz    = obj.L/2*(Fx_R - Fx_L) - obj.A*(Fy_L + Fy_R) + Mz_ext;
            dphid = Mz/obj.Iz;
            
            dw_L = (obj.tau_L - obj.R*Fx_L)/obj.Iw;
            dw_R = (obj.tau_R - obj.R*Fx_R)/obj.Iw;
            obj.w_L    = obj.axleFriction(obj.w_L + dw_L*Ts);
            obj.w_R    = obj.axleFriction(obj.w_R + dw_R*Ts);
            obj.v      = obj.v + [du, dvy]*Ts;
            obj.phid   = obj.phid + dphid*Ts;
            obj.phi    = obj.phi + obj.phid*Ts;
            
            c = cos(obj.phi);
            s = sin(obj.phi);
            Vw = [c*obj.v(1) - s*obj.v(2),  s*obj.v(1) + c*obj.v(2)];
            obj.pos = obj.pos + Vw*Ts;
            obj.acc = [Fx/M, Fy/M];
            
            % =========================================================
            % RECORDING: Comment out matching lines if you skipped them above
            % =========================================================
            if obj.log_steps > 0 && obj.log_idx <= obj.log_steps
                i = obj.log_idx;
                obj.log_pos(i, :)   = obj.pos;
                obj.log_v(i, :)     = obj.v;
                obj.log_phi(i)      = obj.phi;
                obj.log_phid(i)     = obj.phid;
                obj.log_wheels(i,:) = [obj.w_L, obj.w_R];
                obj.log_tau(i, :)   = [obj.tau_L, obj.tau_R];
                
                obj.log_idx = i + 1;
            end
            
            % Outputs
            v_body        = obj.v;
            acc_body      = obj.acc;
            phid_out      = obj.phid;
            w_wheels      = [obj.w_L, obj.w_R];
            phi           = obj.phi; 
            tau_wheels    = [obj.tau_L, obj.tau_R];
            pose_global   = [obj.pos(1), obj.pos(2)]; 
            heading_vec   = [c, s];
            frenet_inv    = [c, s; -s, c];
        end
        
       
    
        function w = axleFriction(obj, w_in)
            dwF = obj.tauF*obj.Ts/obj.Iw;
            if abs(w_in) <= dwF
                w = 0;
            else
                w = w_in - dwF*sign(w_in);
            end
        end
        
        function [Fx, Fy] = limitForce(~, FxReq, FyReq, Fmax)
            Q = hypot(FxReq, FyReq);
            if Q > Fmax
                s = Fmax/Q;
            else
                s = 1;
            end
            Fx = s*FxReq;
            Fy = s*FyReq;
        end
    end
end