classdef vehPlant < handle
    %vehPlant Vehicle plant model
    %   This class stores the state of a vehicle, and acts as
    %   the plant model for the controller. This replaces the ground truth. 

    properties  % vehicle states
        vx = 0
        vy = 0
        px = 0
        py = 0
        r = 0
        psi = 0
        omega_f = 0
        omega_r = 0
        delta = 0
    end

    properties % vehicle states history
        vx_ = []
        vy_ = []
        px_ = []
        py_ = []
        r_ = []
        psi_ = []
        omega_f_ = []
        omega_r_ = []
        delta_ = []
    end

    properties % vehicle parameters
        m = 1700;   % [kg] vehicle weight
        I_z = 2385; % [kg*m^2] yaw moment of inertia
        I_omega = 3; % [kg*m^2] wheel rotating moment of inertia
        mu = 0.6;   % [] tire-road friction coefficient
        C_kappa = 2e3;  % longitudinal tire stiffness (linear tire model)
        C_alpha = 1e4;  % steering stiffness (linear tire model)
        a = 1.392;  % [m] distance from CG to front axle
        b = 1.008;  % [m] distance from CG to rear axle
        R = 0.3;    % [m] wheel radius
        g = 9.81;   % [kg*m/s^2] gravity constant
    end

    methods
        function obj = vehPlant(varargin)
            %vehPlant Construct 
            %   obj = vehPlant(m,I_z,I_omega,mu,C_kappa,C_alpha,a,b,R,g)
            %   constructs an instance of vehicle plant model
            Defaults = {1700,2385,3,0.6,2e3,1e4,1.392,1.008,0.3,9.81};
            Defaults(1:nargin) = varargin;
            obj.m = Defaults{1};
            obj.I_z = Defaults{2};
            obj.I_omega = Defaults{3};
            obj.mu = Defaults{4};
            obj.C_kappa = Defaults{5};
            obj.C_alpha = Defaults{6};
            obj.a = Defaults{7};
            obj.b = Defaults{8};
            obj.R = Defaults{9};
            obj.g = Defaults{10};
        end

        function step(obj,u,dt)
            %step Update vehicle state by a step
            %   step(obj,u,dt) updates the vehicle states using the control
            %   input u, and the control time interval dt. 

            dx = obj.vehdyn(u);
            obj.vx = obj.vx + dx(1)*dt;
            obj.vy = obj.vy + dx(2)*dt;
            obj.px = obj.px + dx(3)*dt;
            obj.py = obj.py + dx(4)*dt;
            obj.r = obj.r + dx(5)*dt;
            obj.psi = obj.psi + dx(6)*dt;
            obj.omega_f = obj.omega_f + dx(7)*dt;
            obj.omega_r = obj.omega_r + dx(8)*dt;
            obj.delta = obj.delta + dx(9)*dt;
            
            % add to states history
            obj.add_to_state_history();
        end

        function add_to_state_history(obj)
            %add_to_state_history Add current state to state history
            obj.vx_(end+1) = obj.vx;
            obj.vy_(end+1) = obj.vy;
            obj.px_(end+1) = obj.px;
            obj.py_(end+1) = obj.py;
            obj.r_(end+1) = obj.r;
            obj.psi_(end+1) = obj.psi;
            obj.omega_f_(end+1) = obj.omega_f;
            obj.omega_r_(end+1) = obj.omega_r;
            obj.delta_(end+1) = obj.delta;
        end

        function dx = vehdyn(obj, u)
            %f State-space function of the ego vehicle dynamics
            %   dx = vehdyn(obj, u) Produces the 9 state
            %   derivatives of the ego vehicle system:
            %   [vx,vy,X_c,Y_c,r,psi,omega_f,omega_r,delta]
            %   (Note: X_c,Y_c and px,py are used interchangably)

            % rename states and controls
            nu = u(1);      % [rad/s] steering rate
            tau = u(2);     % [Nm] torque command

            % intermediate variables based on states
            F_zf = obj.b/(obj.a+obj.b)*obj.m*obj.g;
            F_zr = obj.a/(obj.a+obj.b)*obj.m*obj.g;
            v_xb = cos(obj.psi)*obj.vx + sin(obj.psi)*obj.vy;
            v_yb = -sin(obj.psi)*obj.vx + cos(obj.psi)*obj.vy;
            V_xft = cos(obj.delta)*v_xb + sin(obj.delta)*(v_yb+obj.a*obj.r);
            V_xrt = v_xb;

            kappa_f = (obj.R*obj.omega_f-V_xft) / (V_xft + eps);    % eps to prevent
            kappa_r = (obj.R*obj.omega_r-V_xrt)/ (V_xrt + eps);     % divide-by-zero
            alpha_f = atan2(v_yb+obj.r*obj.a,v_xb)-obj.delta;
            alpha_r = atan2(v_yb-obj.r*obj.b,v_xb);

            % for prototype, only use the stiffness and max to model tire force
            F_xft = max(-obj.mu*F_zf, min(obj.mu*F_zf, obj.C_kappa * kappa_f));
            F_xrt = max(-obj.mu*F_zr, min(obj.mu*F_zr, obj.C_kappa * kappa_r));
            F_yft = max(-obj.mu*F_zf, min(obj.mu*F_zf, -obj.C_alpha * alpha_f));
            F_yrt = max(-obj.mu*F_zr, min(obj.mu*F_zr, -obj.C_alpha * alpha_r));

            F_xfb = cos(obj.delta)*F_xft - sin(obj.delta)*F_yft;
            F_yfb = sin(obj.delta)*F_xft + cos(obj.delta)*F_yft;
            F_xrb = F_xrt;
            F_yrb = F_yrt;

            F_xf = cos(obj.psi)*F_xfb - sin(obj.psi)*F_yfb;
            F_yf = sin(obj.psi)*F_xfb + cos(obj.psi)*F_yfb;
            F_xr = cos(obj.psi)*F_xrb - sin(obj.psi)*F_yrb;
            F_yr = sin(obj.psi)*F_xrb + cos(obj.psi)*F_yrb;

            % 1 \dot{v_x}
            dx1 = 1/obj.m * (F_xf + F_xr);
            % 2 \dot{v_y}
            dx2 = 1/obj.m * (F_yf + F_yr);
            % 3 \dot{X_c}
            dx3 = obj.vx;
            % 4 \dot{Y_c}
            dx4 = obj.vy;
            % 5 \dot{r}
            dx5 = 1/obj.I_z * (obj.a*F_yft*cos(obj.delta)+obj.a*F_xft*sin(obj.delta)-obj.b*F_yrt);
            % 6 \dot{psi}
            dx6 = obj.r;
            % 7 \dot{omega_f}
            dx7 = -1/obj.I_omega * F_xft * obj.R;
            % 8 \dot{omega_r}
            dx8 = 1/obj.I_omega * (tau - F_xrt*obj.R);
            % 9 \dot{delta}
            dx9 = nu;

            % finally, turn dx into a column vector
            dx = vertcat(dx1,dx2,dx3,dx4,dx5,dx6,dx7,dx8,dx9);
        end
    end
end