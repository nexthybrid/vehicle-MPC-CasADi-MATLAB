classdef vehPlant_m < handle
    %vehPlant_m Vehicle plant model
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
        function obj = vehPlant_m(varargin)
            %vehPlant Construct 
            %   obj = vehPlant_m(m,I_z,I_omega,mu,C_kappa,C_alpha,a,b,R,g)
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

        function set_states(obj, states)
            %set_states Set vehicle states
            %   set_states(obj, states) sets the vehicle state in the order
            %   of [vx,vy,px,py,r,psi,omega_f,omega_r,delta]
            obj.vx = states(1);
            obj.vy = states(2);
            obj.px = states(3);
            obj.py = states(4);
            obj.r = states(5);
            obj.psi = states(6);
            obj.omega_f = states(7);
            obj.omega_r = states(8);
            obj.delta = states(9);
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

        function stepRK4(obj,U,dt)
            %stepRK4 Update vehicle state by an RK4 step
            X = [obj.vx, obj.vy, obj.px, obj.py, obj.r, obj.psi,...
                obj.omega_f, obj.omega_r, obj.delta]';
            f = @obj.vehdynStatic;
            
            % Runge-Kutta 4 integration
            k1 = f(X,         U);
            k2 = f(X+dt/2*k1, U);
            k3 = f(X+dt/2*k2, U);
            k4 = f(X+dt*k3,   U);
            X_next = X + dt/6*(k1+2*k2+2*k3+k4); 

            obj.vx = X_next(1);
            obj.vy = X_next(2);
            obj.px = X_next(3);
            obj.py = X_next(4);
            obj.r = X_next(5);
            obj.psi = X_next(6);
            obj.omega_f = X_next(7);
            obj.omega_r = X_next(8);
            obj.delta = X_next(9);

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

        function sim(obj, uArray, dt)
            %sim Simulate the vehicle model multiple steps
            %   sim(obj, uArray, dt) simulates the model multiple steps by
            %   recursively calling step(). The input uArray is a sequence
            %   of all inputs in 2xN, dt is time step.
            obj.add_to_state_history(); % add the initial state to state history
            N = size(uArray,2);
            for i = 1:N
                obj.stepRK4(uArray(:,i), dt);
            end

        end

        function stateTraj = get_trajectory(obj)
            %get_trajectory Get the stored state trajectory

            stateTraj = [obj.vx_; obj.vy_; obj.px_; obj.py_; obj.r_; obj.psi_;
                                obj.omega_f_; obj.omega_r_; obj.delta_ ];
            
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
            
            states = [obj.vx, obj.vy, obj.psi, obj.r, obj.omega_f, obj.omega_r,...
                obj.px, obj.py, obj.delta]';
            inputs = [nu, 0, tau]';
            dStates = VehicleModelDifferentiable(states, inputs);
            % remap state order
            %dStates = [dvx; dvy; omega; dOmega; dwWheel; dX; dY; dDeltaF];
            dx1 = dStates(1);
            dx2 = dStates(2);
            dx3 = dStates(7);
            dx4 = dStates(8);
            dx5 = dStates(4);
            dx6 = dStates(3);
            dx7 = dStates(5);
            dx8 = dStates(6);
            dx9 = dStates(9);

            % finally, turn dx into a column vector
            dx = vertcat(dx1,dx2,dx3,dx4,dx5,dx6,dx7,dx8,dx9);
        end
    end

    methods (Static)

        function dx = vehdynStatic(X, U)
            %f State-space function of the ego vehicle dynamics
            %   dx = vehdynStatic(X, U) Produces the 9 state
            %   derivatives of the ego vehicle system:
            %   [vx,vy,X_c,Y_c,r,psi,omega_f,omega_r,delta]
            %   (Note: X_c,Y_c and px,py are used interchangably)

            % rename states and controls
            nu = U(1);      % [rad/s] steering rate
            tau = U(2);     % [Nm] torque command
            
            states = [X(1),X(2),X(6),X(5),X(7),X(8),X(3),X(4),X(9)]';
            inputs = [nu, 0, tau]';
            dStates = VehicleModelDifferentiable(states, inputs);
            % remap state order
            %dStates = [dvx; dvy; omega; dOmega; dwWheel; dX; dY; dDeltaF];
            dx1 = dStates(1);
            dx2 = dStates(2);
            dx3 = dStates(7);
            dx4 = dStates(8);
            dx5 = dStates(4);
            dx6 = dStates(3);
            dx7 = dStates(5);
            dx8 = dStates(6);
            dx9 = dStates(9);

            % finally, turn dx into a column vector
            dx = vertcat(dx1,dx2,dx3,dx4,dx5,dx6,dx7,dx8,dx9);
        end
    end
end