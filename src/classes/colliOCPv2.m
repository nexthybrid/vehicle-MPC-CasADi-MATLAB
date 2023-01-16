classdef colliOCPv2 < handle
    %colliOCPv2 A class for storing the casadi.Opti() object for solving the
    %collision avoidance OCP.
    %   The v2 stands for version 2. In version 2, the vehicle model is
    %   changed from the pushbox model to the single track bicycle body and
    %   smoothed Dugoff tire model.
    %   Prior to instantiate an object of this class, the casadi package
    %   should be imported by calling:
    %   addpath('C:\Program Files\casadi-windows-matlabR2016a-v3.5.5');
    %   import casadi.*

    properties % casadi class object
        opti
    end

    properties % decision variables
        X
        vx
        vy
        px
        py
        r
        psi
        omega_f
        omega_r
        delta

        U
        f_thr
        f_lat
        u_beta
        nu
        tau
    end

    properties % debug variables
       F_zf_dbg
       F_zr_dbg
       v_xb_dbg
       v_yb_dbg
       V_xft_dbg
       V_xrt_dbg
       kappa_f_dbg
       kappa_r_dbg
       alpha_f_dbg
       alpha_r_dbg
       flambda_f_dbg
       flambda_r_dbg
       F_xft_dbg
       F_xrt_dbg
       F_yft_dbg
       F_yrt_dbg
       F_xfb_dbg
       F_yfb_dbg
       F_xrb_dbg
       F_yrb_dbg
       F_xf_dbg
       F_yf_dbg
       F_xr_dbg
       F_yr_dbg
   end

    properties % more intermediate symbolic variables
        x_1
        x_2
        x_3
        y_1
        y_2
        y_3

        x_a
        x_b
        x_c
        y_a
        y_b
        y_c

        Xobs
        px_obs
        py_obs
        psi_obs
    end

    properties % parameters with numerical values
        T % final time [s]
        N % number of control intervals (initialized in the constructor.)
        m % vehicle mass [kg]
        I_z % vehicle yaw axis rotational inertia [kgm^2]
        I_omega % wheel rotational inertia [kgm^2]
        mu % friction coefficient
        C_kappa % longitudinal stiffness
        C_alpha % (lateral) steering stiffness
        a % ego vehicle front axles to CG distance [m]
        b % ego vehicle rear axles to CG distance [m]
        R % effective tire radius [m]
        a_z % ego vehicle front Ziegler circle to CG distance [m]
        b_z % ego vehicle rear Ziegler circle to CG distance [m]
        R_col % collision radius [m]
        aBmpr % ego vehicle CG to front bumper distance
        bBmpr % ego vehicle CG to rear bumper distance
        bdyWdth % ego vehicle body width
        g % gravity [kgm/s^2]
    end

    methods
        function obj = colliOCPv2(varargin)
            %colliOCPv2 Construct an instance of this class
            %   colliOCPv2(N, T) constructs an instance of colliOCPv2 with N
            %   control intervals across time length T(seconds).
            %   The initial time is t = 0, the final time is t = T, the
            %   time period is discretized into (N+1) steps, with step 0
            %   being t = 0, and step (N+1) being t = T.
            %   The casadi.Opti() object is self-created in the constructor
            % Inputs:
            % N: double, the number of control intervals
            Defaults = {20, 5};
            Defaults(1:nargin) = varargin;
            obj.N = Defaults{1};
            obj.T = Defaults{2};
            
            obj.opti = casadi.Opti();
        end

        function build_basics(obj)
            %build_basics Build the basic setup of the OCP
            %   build_basics(obj) builds the basic setup of the OCP
            %   including the decision variables, the valued parameters,
            %   the objective function, and the (ego) dynamics constraints.
            obj.build_decision_var();
            obj.add_parameters();
            obj.add_objective();
            obj.add_dynamic_constraints();
        end

        function build_decision_var(obj,varargin)
            %build_decision_var Builds decision variables
            %   build_decision_var(obj) builds the symbolic decision
            %   variables and store them as class properties
            
            obj.X = obj.opti.variable(9,obj.N+1); % state trajectory
            obj.vx = obj.X(1,:);
            obj.vy = obj.X(2,:);
            obj.px = obj.X(3,:);
            obj.py = obj.X(4,:);
            obj.r = obj.X(5,:);
            obj.psi = obj.X(6,:);
            obj.omega_f = obj.X(7,:);
            obj.omega_r = obj.X(8,:);
            obj.delta = obj.X(9,:);
            
            obj.U = obj.opti.variable(2,obj.N);
            obj.nu = obj.U(1,:); % [rad/s]
            obj.tau = obj.U(2,:); % [Nm]

        end

        function build_debug_var(obj)
            %build_debug_var Build debug variables
            %   build_debug_var(obj) turns some of the intermediate
            %   variables in the system equations into states or decision
            %   variables, so that they can be monitored for debug purpose.
            %
            %   To differentiate these variables with the actual variables,
            %   we use suffix of _dbg for these variable names.

            obj.F_zf_dbg = obj.b/(obj.a+obj.b)*obj.m*obj.g;
            obj.F_zr_dbg = obj.a/(obj.a+obj.b)*obj.m*obj.g;
            obj.v_xb_dbg = cos(obj.psi).*obj.vx + sin(obj.psi).*obj.vy;
            obj.v_yb_dbg = -sin(obj.psi).*obj.vx + cos(obj.psi).*obj.vy;
            obj.V_xft_dbg = cos(obj.delta).*obj.v_xb_dbg ...
                + sin(obj.delta).*(obj.v_yb_dbg+obj.a.*obj.r);
            obj.V_xrt_dbg = obj.v_xb_dbg;

            obj.kappa_f_dbg = (obj.R.*obj.omega_f-obj.V_xft_dbg)./obj.V_xft_dbg;
            obj.kappa_r_dbg = (obj.R.*obj.omega_r-obj.V_xrt_dbg)./obj.V_xrt_dbg;
            obj.alpha_f_dbg = atan2(obj.v_yb_dbg+obj.r*obj.a,obj.v_xb_dbg)-obj.delta;
            obj.alpha_r_dbg = atan2(obj.v_yb_dbg-obj.r*obj.b,obj.v_xb_dbg);

            obj.flambda_f_dbg = 0.5.*obj.mu.*obj.F_zf_dbg.*(1+obj.kappa_f_dbg)./...
                sqrt( (obj.C_kappa.*obj.kappa_f_dbg).^2 + (obj.C_alpha.*tan(obj.alpha_f_dbg)).^2 );
            obj.flambda_r_dbg = 0.5*obj.mu.*obj.F_zr_dbg.*(1+obj.kappa_r_dbg)./...
                sqrt( (obj.C_kappa.*obj.kappa_r_dbg).^2 + (obj.C_alpha.*tan(obj.alpha_r_dbg)).^2 );

%             obj.F_xft_dbg = obj.C_kappa .* obj.kappa_f_dbg ./ (1+obj.kappa_f_dbg) .* obj.flambda_f_dbg;
%             obj.F_xrt_dbg = obj.C_kappa .* obj.kappa_r_dbg ./ (1+obj.kappa_r_dbg) .* obj.flambda_r_dbg;
%             obj.F_yft_dbg = obj.C_alpha .* tan(obj.alpha_f_dbg) ./ (1+obj.kappa_f_dbg) .* obj.flambda_f_dbg;
%             obj.F_yrt_dbg = obj.C_alpha .* tan(obj.alpha_f_dbg) ./ (1+obj.kappa_r_dbg) .* obj.flambda_r_dbg;

            obj.F_xft_dbg = obj.C_kappa .* obj.kappa_f_dbg ;
            obj.F_xrt_dbg = obj.C_kappa .* obj.kappa_r_dbg;
            obj.F_yft_dbg = obj.C_alpha .* tan(obj.alpha_f_dbg);
            obj.F_yrt_dbg = obj.C_alpha .* tan(obj.alpha_r_dbg);

            obj.F_xfb_dbg = cos(obj.delta).*obj.F_xft_dbg - sin(obj.delta).*obj.F_yft_dbg;
            obj.F_yfb_dbg = sin(obj.delta).*obj.F_xft_dbg + cos(obj.delta).*obj.F_yft_dbg;
            obj.F_xrb_dbg = obj.F_xrt_dbg;
            obj.F_yrb_dbg = obj.F_yrt_dbg;

            obj.F_xf_dbg = cos(obj.psi).*obj.F_xfb_dbg - sin(obj.psi).*obj.F_yfb_dbg;
            obj.F_yf_dbg = sin(obj.psi).*obj.F_xfb_dbg + cos(obj.psi).*obj.F_yfb_dbg;
            obj.F_xr_dbg = cos(obj.psi).*obj.F_xrb_dbg - sin(obj.psi).*obj.F_yrb_dbg;
            obj.F_yr_dbg = sin(obj.psi).*obj.F_xrb_dbg + cos(obj.psi).*obj.F_yrb_dbg;


        end

        function add_parameters(obj,varargin)
            %add_parameters Adds parameters to the OCP
            %   add_parameters(obj,varargin) adds parameters to the OCP, the
            %   current parameters include vehicle mass m and more.
            % Inputs:
            % m: double, vehicle mass [kg]
            % I_z:, double, vehicle yaw axis rotational inertia [kgm^2]
            % I_omega, double, wheel rotational inertia [kgm^2]
            % mu: friction coefficient [ ]
            % C_kappa: longitudinal stiffness [ ]
            % C_alpha: steering stiffneess [ ]
            % a: double, ego vehicle front axle to CG distance [m]
            % b: double, ego vehicle rear axle to CG distance [m]
            % R: dobule, effective tire radius [m]
            % a_z: double, ego vehicle front Ziegler circle to CG distance [m]
            % b_z: double, ego vehicle rear Ziegler circle to CG distance [m]
            % R_col: double, collision radius [m]
            % aBmpr: double, ego vehicle CG to front bumper distance [m]
            % bBmpr: double, ego vehicle CG to rear bumper distance [m]
            % bdyWdth: double, ego vehicle body width [m]
            % g: double, gravity [kgm/s^2]

            Defaults = {1700,2385,3,0.6,2e3,1e4,1.392,1.008,0.3,1.5,1.5,2,2,2,2,9.81};
            Defaults(1:nargin-1) = varargin;
            obj.m = Defaults{1};
            obj.I_z = Defaults{2};
            obj.I_omega = Defaults{3};
            obj.mu = Defaults{4};
            obj.C_kappa = Defaults{5};
            obj.C_alpha = Defaults{6};
            obj.a = Defaults{7};
            obj.b = Defaults{8};
            obj.R = Defaults{9};
            obj.a_z = Defaults{10};
            obj.b_z = Defaults{11};
            obj.R_col = Defaults{12};
            obj.aBmpr = Defaults{13};
            obj.bBmpr = Defaults{14};
            obj.bdyWdth = Defaults{15};
            obj.g = Defaults{16};
        end

        function add_objective(obj)
            %add_objective Adds the objective function to the OCP
            %   add_objective(obj) applies the pre-defined OCP objective
            %   function.
            obj.opti.minimize(sum( (obj.tau/1000).^2 + (obj.nu/5).^2) + sum((obj.vy).^2) ...
                                        + sum((obj.psi).^2)  );
        end

        function modify_objective(obj)
            %modify_objective Modify the objective function to the OCP
            %   add_objective(obj) modifies the OCP objective
            %   function.

            % maximize CG-to-CG distance to the obstacle
            obj.opti.minimize(-sum((obj.px(2:end) - obj.x_b).^2 + (obj.py(2:end) - obj.y_b).^2) );
        end

        function zero_objective(obj)
            %zero_objective Force the objective to be a constant of zero
            % zero_objective Use constant zero as the objective,
            % effectively making no cost functions. So the solver will only
            % need to deal with the feasibility problem, and somehow
            % optimize itself towards a feasible solution.

            obj.opti.minimize(0);
            disp('Zero objective function applied!');
        end

        function add_dynamic_constraints(obj)
            %add_dynamic_constraints Adds the (ego) dynamics constraints
            %   add_dynamic_constraints(obj) adds the ego vehicle dynamics
            %   as equality constraints to the OCP.
            
%             f = @(x,u) [u(1)/obj.m;
%                 u(2)/obj.m;
%                 x(1).*cos(x(5))-x(2).*sin(x(5));...
%                 x(1).*sin(x(5))+x(2).*cos(x(5));...
%                 (x(1).*u(2)-x(2).*u(1))./obj.m./(x(1).^2+x(2).^2)-u(3)]; % dx/dt = f(x,u)
            f = @obj.vehdyn;
            dt = obj.T/obj.N; % length of a control interval
            for k=1:obj.N % loop over control intervals
               % Runge-Kutta 4 integration
               k1 = f(obj.X(:,k),         obj.U(:,k));
               k2 = f(obj.X(:,k)+dt/2*k1, obj.U(:,k));
               k3 = f(obj.X(:,k)+dt/2*k2, obj.U(:,k));
               k4 = f(obj.X(:,k)+dt*k3,   obj.U(:,k));
               x_next = obj.X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
               obj.opti.subject_to(obj.X(:,k+1)==x_next); % close the gaps
            end
        end

        function add_control_bounds(obj,varargin)
            %add_control_bounds Add the control bounds to the OCP
            %   add_control_bounds(obj,B_nu, B_tau) adds the
            %   lower and upper bounds for the manipulated control inputs
            %   Inputs:
            %   B_nu: 1x2 double, lower and upper bounds for nu
            %   B_tau: 1x2 double, lower and upper bounds for tau

            Defaults = {[-5, 5], [0, 1000]}; % disabling brake for now.
            % add in a brake.
            Defaults(1:nargin-1) = varargin;
            B_nu = Defaults{1};
            B_tau = Defaults{2};

            obj.opti.subject_to(B_nu(1)<=obj.nu<=B_nu(2));
            obj.opti.subject_to(B_tau(1)<=obj.tau<=B_tau(2));
        end

        function add_initial_conditions(obj,varargin)
            %add_initial_conditions Add initial conditions
            %   add_initial_conditions(obj,v_xb0,v_yb0,px0,py0,r0,psi0,omega_f0,omega_r0,delta0)
            %   
            %   initial condition for the set of decision variables
            %   selected. In this case, the set is the ego vehicle states.
            %   Inputs:
            %   vx0: double, ego vehicle initial global X speed [m/s]
            %   vy0: double, ego vehicle initial global Y  speed [m/s]
            %   px0: double, ego vehicle initial global X coordinate [m]
            %   py0: double, ego vehicle initial global Y coordinate [m]
            %   r0: double, initial yaw rate [rad/s]
            %   psi0: double, ego vehicle initial yaw angle [rad]
            %   omega_f0: double, ego vehicle initial front wheel speed [rad/s]
            %   omega_r0: double, ego vehicle initial rear wheel speed [rad/s]
            %   delta0: double, ego vehicle initial steering angle [rad]
            
            if (nargin == 2)
                % obj, X as inputs
                vx0 = varargin{1}(1);
                vy0 = varargin{1}(2);
                px0 = varargin{1}(3);
                py0 = varargin{1}(4);
                r0 = varargin{1}(5);
                psi0 = varargin{1}(6);
                omega_f0 = varargin{1}(7);
                omega_r0 = varargin{1}(8);
                delta0 = varargin{1}(9);
            elseif (nargin == 10)
                %
                Defaults = {2, -0.1, 0.2, 5, 0.1, 0.1, 30,30,0.1};
                Defaults(1:nargin-1) = varargin;
                vx0 = Defaults{1};
                vy0 = Defaults{2};
                px0 = Defaults{3};
                py0 = Defaults{4};
                r0 = Defaults{5};
                psi0 = Defaults{6};
                omega_f0 = Defaults{7};
                omega_r0 = Defaults{8};
                delta0 = Defaults{9};
            end



            obj.opti.subject_to(obj.vx(1)==vx0);
            obj.opti.subject_to(obj.vy(1)==vy0);
            obj.opti.subject_to(obj.px(1)==px0);
            obj.opti.subject_to(obj.py(1)==py0);
            obj.opti.subject_to(obj.r(1)==r0);
            obj.opti.subject_to(obj.psi(1)==psi0);
            obj.opti.subject_to(obj.omega_f(1)==omega_f0);
            obj.opti.subject_to(obj.omega_r(1)==omega_r0);
            obj.opti.subject_to(obj.delta(1)==delta0);
        end

        function add_approx_initial_conditions(obj,varargin)
            %add_approx_initial_conditions Add approximal initial conditions
            %   add_initial_conditions(obj,v_xb0,v_yb0,px0,py0,r0,psi0,omega_f0,omega_r0,delta0)
            %   
            %   initial condition for the set of decision variables
            %   selected. In this case, the set is the ego vehicle states.
            %   Inputs:
            %   vx0: double, ego vehicle initial global X speed [m/s]
            %   vy0: double, ego vehicle initial global Y  speed [m/s]
            %   px0: double, ego vehicle initial global X coordinate [m]
            %   py0: double, ego vehicle initial global Y coordinate [m]
            %   r0: double, initial yaw rate [rad/s]
            %   psi0: double, ego vehicle initial yaw angle [rad]
            %   omega_f0: double, ego vehicle initial front wheel speed [rad/s]
            %   omega_r0: double, ego vehicle initial rear wheel speed [rad/s]
            %   delta0: double, ego vehicle initial steering angle [rad]
            
            if (nargin == 2)
                % obj, X as inputs
                vx0 = varargin{1}(1);
                vy0 = varargin{1}(2);
                px0 = varargin{1}(3);
                py0 = varargin{1}(4);
                r0 = varargin{1}(5);
                psi0 = varargin{1}(6);
                omega_f0 = varargin{1}(7);
                omega_r0 = varargin{1}(8);
                delta0 = varargin{1}(9);
            elseif (nargin == 10)
                %
                Defaults = {2, -0.1, 0.2, 5, 0.1, 0.1, 30,30,0.1};
                Defaults(1:nargin-1) = varargin;
                vx0 = Defaults{1};
                vy0 = Defaults{2};
                px0 = Defaults{3};
                py0 = Defaults{4};
                r0 = Defaults{5};
                psi0 = Defaults{6};
                omega_f0 = Defaults{7};
                omega_r0 = Defaults{8};
                delta0 = Defaults{9};
            end



            obj.opti.subject_to(0.9*vx0<=obj.vx(1)<=1.1*vx0);
            obj.opti.subject_to(0.9*vy0<=obj.vy(1)<=1.1*vy0);
            obj.opti.subject_to(0.9*px0<=obj.px(1)<=1.1*px0);
            obj.opti.subject_to(0.9*py0<=obj.py(1)<=1.1*py0);
            obj.opti.subject_to(0.9*r0<=obj.r(1)<=1.1*r0);
            obj.opti.subject_to(0.9*psi0<=obj.psi(1)<=1.1*psi0);
            obj.opti.subject_to(0.9*omega_f0<=obj.omega_f(1)<=1.1*omega_f0);
            obj.opti.subject_to(0.9*omega_r0<=obj.omega_r(1)<=1.1*omega_r0);
            obj.opti.subject_to(0.9*delta0<=obj.delta(1)<=1.1*delta0);
        end

        function add_terminal_conditions(obj,varargin)
            %add_terminal_conditions Add terminal conditions for decision
            %variables
            %   add_terminal_conditions(obj,B_py) adds the terminal
            %   conditions for the decision variables selected. In this
            %   case, the selected decision variable is simply the global y
            %   coordinate of the ego vehicle.

            obj.opti.subject_to(obj.py(obj.N+1) == 3); %  bound for py
            obj.opti.subject_to(obj.px(obj.N+1) >= 10); % lower bound for px
            obj.opti.subject_to(obj.psi(obj.N+1) == 0); %  bound for psi
        end

        function add_path_constraints(obj,varargin)
            %add_path_constraints Add the path constraints
            %   add_path_constraints(obj, X_initial, Y_initial,
            %   X_move_range, Y_move_range) implements pre-defined road
            %   geometry constraints on the vehicle state. Also additional
            %   limit is added to bound the X,Y values to within a move
            %   range of the initial values. This additional limit helps
            %   reduce infeasibility dual by bounding the problem tighter.
            Defaults = {0, 5, 50, 50};
            Defaults(1:nargin-1) = varargin;
            X_initial = Defaults{1};
            Y_initial = Defaults{2};
%             obj.opti.subject_to(X_initial - X_move_range <= obj.px <= X_initial + X_move_range);
%             obj.opti.subject_to(Y_initial - Y_move_range <= obj.py <= Y_initial + Y_move_range);
            obj.opti.subject_to(-5<=obj.px(2:end)<=20); % X limit
            obj.opti.subject_to(2<=obj.py(2:end)<=7); % Y limit
            obj.opti.subject_to(2<=obj.px(end)); % X > something
            obj.opti.subject_to(-0.8<=obj.delta(2:end)<=0.8); % steering angle limit [rad]
            %obj.opti.subject_to(cos(obj.psi(2:end)).*obj.vx(2:end) + sin(obj.psi(2:end)).*obj.vy(2:end) >= 1);  % min speed
            % for initial debug, bound the wheel speeds
            %obj.opti.subject_to(-200<=obj.omega_f<=200); %
            %obj.opti.subject_to(-200<=obj.omega_r<=200); %
        end

        function add_ego_centers(obj)
            %add_ego_centers Add the ego vehicle collision centers
            %   add_ego_centers(obj) adds the ego vehicle Ziegler collision 
            %   circle centers to the OCP

            % the ego vehicle is denoted 1,2,3 for its three circle centers

            obj.x_1 = obj.px - obj.b_z.*cos(obj.psi);
            obj.y_1 = obj.py - obj.b_z.*sin(obj.psi);
            obj.x_2 = obj.px;
            obj.y_2 = obj.py;
            obj.x_3 = obj.px + obj.a_z.*cos(obj.psi);
            obj.y_3 = obj.py + obj.a_z.*sin(obj.psi);
        end

        function add_obstacle_centers(obj, varargin)
            %add_obstacle_centers Add the obstacle collision centers
            %   add_obstacle_centers(obj, x_obs, y_obs, psi_obs, L_obs)
            %   adds the (static) obstacle Ziegler collision circle centers
            %   to the OCP
            Defaults = {6, 3, 0, 4};
            Defaults(1:nargin-1) = varargin;
            x_obs_st = Defaults{1};
            y_obs_st = Defaults{2};
            psi_obs_st = Defaults{3};
            L_obs = Defaults{4};
            % the obstacle is denoted a,b,c for its three circle centers
            obj.x_a = x_obs_st + L_obs/2*cos(psi_obs_st);
            obj.y_a = y_obs_st + L_obs/2*sin(psi_obs_st);
            obj.x_b = x_obs_st;
            obj.y_b = y_obs_st;
            obj.x_c = x_obs_st - L_obs/2*cos(psi_obs_st);
            obj.y_c = y_obs_st - L_obs/2*sin(psi_obs_st);
        end

        function add_second_obstacle_centers(obj)
            %add_second_obstacle_centers Add a second obstacle
            %   add_second_obstacle_centers(obj) adds the obstacle
            %   Ziegler collision circle centers to the OCP

            % the obstacle is denoted a,b,c for its three circle centers
            obj.x_a2 = 4; obj.y_a2 = 3;
            obj.x_b2 = 6; obj.y_b2 = 3;
            obj.x_c2 = 8; obj.y_c2 = 3;
        end

        function add_dynamic_obstacle_centers(obj,x_traj,y_traj,psi_traj)
            %add_dynamic_obstacle_centers Add a dynamic obstacle collision
            %centers
            %   add_dynamic_obstacle_centers(obj) adds the obstacle Ziegler
            %   collision circle centers to the OCP using provided obstacle
            %   CG and yaw angle trajectories.
            %   Inputs:
            %   x_traj: 1x(obj.N+1) double, global x coordinate trajectory
            %   y_traj: 1x(obj.N+1) double, global x coordinate trajectory
            %   psi_traj: 1x(obj.N+1) double, yaw angle trajectory

            % apply the pre-defined CG and yaw angle trajectories
            obj.Xobs = [x_traj; y_traj; psi_traj];
            obj.px_obs = obj.Xobs(1,:);
            obj.py_obs = obj.Xobs(2,:);
            obj.psi_obs = obj.Xobs(3,:);

            obj.x_a = obj.px_obs + obj.b_z.*cos(obj.psi_obs);
            obj.y_a = obj.py_obs + obj.b_z.*sin(obj.psi_obs);
            obj.x_b = obj.px_obs;
            obj.y_b = obj.py_obs;
            obj.x_c = obj.px_obs - obj.a_z.*cos(obj.psi_obs);
            obj.y_c = obj.py_obs - obj.a_z.*sin(obj.psi_obs);
        end

        function add_collision_constraint(obj,varargin)
            %add_collision_constraint Add the collision constraint to the
            %OCP
            %   add_collision_constraint(obj,R_collision) adds the
            %   collision constraint based on the ego and the obstacle
            %   collision centers. 
            %   Inputs:
            %   R_collision: double, collision radius between any two circle
            %   centers from different objects.

            Defaults = {obj.R_col};
            Defaults(1:nargin-1) = varargin;
            R_collision = Defaults{1};
            R_sq = R_collision^2;

            obj.opti.subject_to((obj.x_1-obj.x_a).^2+(obj.y_1-obj.y_a).^2 > R_sq); % distance 1 to a
            obj.opti.subject_to((obj.x_1-obj.x_b).^2+(obj.y_1-obj.y_b).^2 > R_sq); % distance 1 to b
            obj.opti.subject_to((obj.x_1-obj.x_c).^2+(obj.y_1-obj.y_c).^2 > R_sq); % distance 1 to c
            obj.opti.subject_to((obj.x_2-obj.x_a).^2+(obj.y_2-obj.y_a).^2 > R_sq); % distance 2 to a
            obj.opti.subject_to((obj.x_2-obj.x_b).^2+(obj.y_2-obj.y_b).^2 > R_sq); % distance 2 to b
            obj.opti.subject_to((obj.x_2-obj.x_c).^2+(obj.y_2-obj.y_c).^2 > R_sq); % distance 2 to c
            obj.opti.subject_to((obj.x_3-obj.x_a).^2+(obj.y_3-obj.y_a).^2 > R_sq); % distance 3 to a
            obj.opti.subject_to((obj.x_3-obj.x_b).^2+(obj.y_3-obj.y_b).^2 > R_sq); % distance 3 to b
            obj.opti.subject_to((obj.x_3-obj.x_c).^2+(obj.y_3-obj.y_c).^2 > R_sq); % distance 3 to c
        end

        function set_solver_init(obj)
            %set_solver_init Set solver initial values
            %   set_solver_init(obj) sets the solver initial values for the
            %   set of decision variables selected

            obj.opti.set_initial(obj.vx, 3);
            obj.opti.set_initial(obj.vy, 0);
            obj.opti.set_initial(obj.px, 0);
            obj.opti.set_initial(obj.py, 5);
            obj.opti.set_initial(obj.r, 0);
            %obj.opti.set_initial(obj.psi, 0);
            obj.opti.set_initial(obj.omega_f, 10);
            obj.opti.set_initial(obj.omega_r, 10);
            obj.opti.set_initial(obj.delta, 0);

            % set control initial values
            obj.opti.set_initial(obj.nu, 0) ;
        end

        function warm_start_init(obj,sol)
            %warm_start_init Warm-start the initial condition
            % warm_start_init(obj,sol) warm-starts the intial condition by
            % using all the optimization variables from a previous solution

            obj.opti.set_initial(sol.value_variables());
        end

        function sol = solve(obj,varargin)
            %solve Solve the OCP and produce the solution object
            % sol = solve(obj,plugin_opt,solver_opt) uses the plugin
            % options and solver options to create an IPOPT solver and
            % solves the OCP.
            Defaults = {struct([]),struct([])};
            Defaults(1:nargin-1) = varargin;
            plugin_opt = Defaults{1};
            solver_opt = Defaults{2};
            obj.opti.solver('ipopt',plugin_opt,solver_opt); % set numerical backend
            sol = obj.opti.solve();   % actual solve
        end
    end

    methods (Static) % ego vehicle dynamics
        function dx = vehdyn(x, u)
            %f State-space function of the ego vehicle dynamics
            %   dx = vehdyn(obj, u) Produces the 9 state
            %   derivatives of the ego vehicle system:
            %   [vx,vy,X_c,Y_c,r,psi,omega_f,omega_r,delta]
            %   (Note: X_c,Y_c and px,py are used interchangably)

            % rename states and controls
            nu = u(1);      % [rad/s] steering rate
            tau = u(2);     % [Nm] torque command
            
            states =x;
            inputs = [nu, 0, tau]';
            dStates = VehicleModelDifferentiable(states, inputs);
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
%         function dx = vehdyn(x,u)
%             %f State-space function of the ego vehicle dynamics
%             %   dx = vehdyn(x,u) Produces the 9 state
%             %   derivatives of the ego vehicle system:
%             %   [vx,vy,X_c,Y_c,r,psi,omega_f,omega_r,delta]
%             %   (Note: X_c,Y_c and px,py are used interchangably)
%             %   
%             %   Because the inputs are symbolic variables, the function
%             %   f is symbolic-variable friendly.
% 
%             Defaults = {1700,2385,3,0.9,1e5,5e4,1.3,1.4,0.3,1.5,1.5,2,9.81};
%             %Defaults(1:nargin-2) = varargin;
%             m = Defaults{1};
%             I_z = Defaults{2};
%             I_omega = Defaults{3};
%             mu = Defaults{4};
%             C_kappa = Defaults{5};
%             C_alpha = Defaults{6};
%             a = Defaults{7};
%             b = Defaults{8};
%             R = Defaults{9};
%             a_z = Defaults{10};
%             b_z = Defaults{11};
%             R_col = Defaults{12};
%             g = Defaults{13};
% 
%             % rename states and controls
%             vx = x(1);
%             vy = x(2);
%             X_c = x(3);
%             Y_c = x(4);
%             r = x(5);
%             psi = x(6);
%             omega_f = x(7);
%             omega_r = x(8);
%             delta = x(9);
%             nu = u(1);
%             tau = u(2);
% 
%             % intermediate variables based on states
%             F_zf = b/(a+b)*m*g;
%             F_zr = a/(a+b)*m*g;
%             v_xb = cos(psi)*vx + sin(psi)*vy;
%             v_yb = -sin(psi)*vx + cos(psi)*vy;
%             V_xft = cos(delta)*v_xb + sin(delta)*(v_yb+a*r);
%             V_xrt = v_xb;
% 
%             kappa_f = (R*omega_f-V_xft) / (V_xft + eps);    % eps prevents divide by zero
%             kappa_r = (R*omega_r-V_xrt)/ (V_xrt + eps);
%             alpha_f = atan2(v_yb+r*a,v_xb)-delta;
%             alpha_r = atan2(v_yb-r*b,v_xb);
%             %alpha_f = atan((v_yb+r*a)/v_xb)-delta;
%             %alpha_r = atan((v_yb-r*b)/v_xb);
% %             flambda_f = 0.5*mu*F_zf*(1+kappa_f)/...
% %                 sqrt( (C_kappa*kappa_f)^2 + (C_alpha*tan(alpha_f))^2 );
% %             flambda_r = 0.5*mu*F_zr*(1+kappa_r)/...
% %                 sqrt( (C_kappa*kappa_r)^2 + (C_alpha*tan(alpha_r))^2 );
% 
% %             F_xft = C_kappa * kappa_f / (1+kappa_f) * flambda_f;
% %             F_xrt = C_kappa * kappa_r / (1+kappa_r) * flambda_r;
% %             F_yft = C_alpha * tan(alpha_f) / (1+kappa_f) * flambda_f;
% %             F_yrt = C_alpha * tan(alpha_r) / (1+kappa_r) * flambda_r;
% %             % for debug, make all F_**t = 0
% %             F_xft = 0;
% %             F_xrt = 0;
% %             F_yft = 0;
% %             F_yrt = 0;
% %             % for debug, only use stiffness
%             F_xft = max(-mu*F_zf, min(mu*F_zf, C_kappa * kappa_f));
%             F_xrt = max(-mu*F_zr, min(mu*F_zr, C_kappa * kappa_r));
%             F_yft = max(-mu*F_zf, min(mu*F_zf, -C_alpha * alpha_f));
%             F_yrt = max(-mu*F_zr, min(mu*F_zr, -C_alpha * alpha_r));
% %              F_xft = C_kappa * kappa_f;
% %              F_xrt = C_kappa * kappa_r;
% %              F_yft = C_alpha * tan(alpha_f);    % maybe a negative sign in front?
% %              F_yrt = C_alpha * tan(alpha_r);
% 
% 
%             F_xfb = cos(delta)*F_xft - sin(delta)*F_yft;
%             F_yfb = sin(delta)*F_xft + cos(delta)*F_yft;
%             F_xrb = F_xrt;
%             F_yrb = F_yrt;
% 
%             F_xf = cos(psi)*F_xfb - sin(psi)*F_yfb;
%             F_yf = sin(psi)*F_xfb + cos(psi)*F_yfb;
%             F_xr = cos(psi)*F_xrb - sin(psi)*F_yrb;
%             F_yr = sin(psi)*F_xrb + cos(psi)*F_yrb;
% 
%             % 1 \dot{v_x}
%             dx1 = 1/m * (F_xf + F_xr);
%             % 2 \dot{v_y}
%             dx2 = 1/m * (F_yf + F_yr);
%             % 3 \dot{X_c}
%             dx3 = vx;
%             % 4 \dot{Y_c}
%             dx4 = vy;
%             % 5 \dot{r}
%             dx5 = 1/I_z * (a*F_yft*cos(delta)+a*F_xft*sin(delta)-b*F_yrt);
%             % 6 \dot{psi}
%             dx6 = r;
%             % 7 \dot{omega_f}
%             dx7 = -1/I_omega * F_xft * R;
%             % 8 \dot{omega_r}
%             dx8 = 1/I_omega * (tau - F_xrt*R);
%             % 9 \dot{delta}
%             dx9 = nu;
% 
% 
%             % finally, turn dx into a column vector
%             dx = vertcat(dx1,dx2,dx3,dx4,dx5,dx6,dx7,dx8,dx9);
%         end

        function X_next =  system_update(obj,X0,U0,tHorizon,nHorizon)
            %system_update Update the system one step ahead
            f = @obj.vehdyn;
            dt = tHorizon/nHorizon; % length of a control interval
            % Runge-Kutta 4 integration
            k1 = f(X0,         U0);
            k2 = f(X0+dt/2*k1, U0);
            k3 = f(X0+dt/2*k2, U0);
            k4 = f(X0+dt*k3,   U0);
            X_next = X0 + dt/6*(k1+2*k2+2*k3+k4);
        end

        function Xobs = obstacle_update(obj)
            %obstacle_update Updates the obstacle position
            Xobs = [obj.x_a; obj.x_b; obj.x_c];
        end
    end

    methods % methods related to the obstacle trajectory
        function [x_traj,y_traj,psi_traj] = make_const_v_trajectory(obj,X_init,vx,vy,r)
            %make_const_v_trajectory Make a constant velocity trajectory
            %   [x_traj,y_traj,psi_traj] =
            %   make_const_v_trajectory(Xobs_init,vx,vy,r) makes a constant
            %   velocity trajectory given the intial X_init = [x0,y0,psi0]
            %   and the specified velocities vx,vy,r.
            %   Inputs:
            %   X_init: 1x3 double, the initial [x0,y0,psi0] state of a
            %   moving obstacle
            %   vx: double, (constant) global x velocity
            %   vy: double, (constant) global y velocity
            %   r: double, (constant) yaw rate
            %   Outputs:
            %   x_traj: 1x(obj.N+1) double, global x trajectory
            %   y_traj: 1x(obj.N+1) double, global y trajectory
            %   psi_traj: 1x(obj.N+1) double, psi trajectory

            x_traj = X_init(1) + linspace(0,obj.T,obj.N+1)*vx;
            y_traj = X_init(2) + linspace(0,obj.T,obj.N+1)*vy;
            psi_traj = X_init(3) + linspace(0,obj.T,obj.N+1)*r;

        end
    end
end