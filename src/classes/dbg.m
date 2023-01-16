classdef dbg
    %DBG Debug class for finding problems in OCP formulation
    %   This class hosts a collection of functions to help debug the
    %   problems in formating the OCP. Mainly it incudes re-calculation
    %   functions of the constraint violation.


    methods (Static)
        function cv = control_bounds_violation(sol, OCP, B_nu, B_tau)
            %control_bounds_violation Computes the control bounds violation
            %   control_bounds_violation(sol, OCP, B_nu, B_tau) computes how
            %   much has the solution violated the control bounds defined
            %   in the OCP. B_nu is the bounds on steering rate, B_tau is
            %   the bounds on torque.
            nu = sol.value(OCP.nu);
            tau = sol.value(OCP.tau);
            cv_nu = abs( (nu > B_nu) .* (nu - B_nu) ) + abs( (nu < -B_nu) .* (-B_nu - nu) );
            cv_tau = abs( (tau > B_tau) .* (tau - B_tau) ) + abs( (tau < -B_tau) .* (-B_tau - tau) );
            cv = sum(cv_nu + cv_tau);
        end

        function cv = initial_constraints_violation(sol, OCP, v_xb0,v_yb0,px0,py0,r0,psi0,omega_f0,omega_r0,delta0)
            %initial_constraints_violation Computes the initial constraints
            %violation
            % cv = initial_constraints_violation(sol, OCP,
            % v_xb0,v_yb0,px0,py0,r0,psi0,omega_f0,omega_r0,delta0)
            % produced the cumulative initial constraints violation
            vx = sol.value(OCP.vx);
            vy = sol.value(OCP.vy);
            px = sol.value(OCP.px);
            py = sol.value(OCP.py);
            r = sol.value(OCP.r);
            psi = sol.value(OCP.psi);
            omega_f = sol.value(OCP.omega_f);
            omega_r = sol.value(OCP.omega_r);
            delta = sol.value(OCP.delta);

            cv_vx = abs(vx(1) - v_xb0);
            threshold_vx = abs(vx(1) - v_xb0) >= 1e-4;
            if threshold_vx > 0
                disp(['detected vx0 violation: vx0 = ', num2str(vx(1))]);
            end
            cv_vy = abs(vy(1) - v_yb0);
            threshold_vy = abs(vy(1) - v_yb0) >= 1e-4;
            if threshold_vy > 0
                disp(['detected vy0 violation: vy0 = ', num2str(vy(1))]);
            end
            cv_px = abs(px(1) - px0);
            threshold_px = abs(px(1) - px0) >= 1e-4;
            if threshold_px > 0
                disp(['detected px0 violation: px0 = ', num2str(px(1))]);
            end
            cv_py = abs(py(1) - py0);
            threshold_py = abs(py(1) - py0) >= 1e-4;
            if threshold_py > 0
                disp(['detected py0 violation: py0 = ', num2str(py(1))]);
            end
            cv_r = abs(r(1)  - r0);
            threshold_r = abs(r(1) - r0) >= 1e-4;
            if threshold_r > 0
                disp(['detected r0 violation: r0 = ', num2str(r(1))]);
            end
            cv_psi = abs(psi(1) - psi0);
            threshold_psi = abs(psi(1) - psi0) >= 1e-4;
            if threshold_psi > 0
                disp(['detected psi0 violation: psi0 = ', num2str(psi(1))]);
            end
            cv_omega_f = abs( omega_f(1) - omega_f0);
            threshold_omega_f = abs(omega_f(1) - omega_f0) >= 1e-4;
            if threshold_omega_f > 0
                disp(['detected omega_f0 violation: omega_f0 = ', num2str(omega_f(1))]);
            end
            cv_omega_r = abs( omega_r(1) - omega_r0);
            threshold_omega_r = abs(omega_r(1) - omega_r0) >= 1e-4;
            if threshold_omega_r > 0
                disp(['detected omega_r0 violation: omega_r0 = ', num2str(omega_r(1))]);
            end
            cv_delta = abs(delta(1) - delta0);
            threshold_delta = abs(delta(1) - delta0) >= 1e-4;
            if threshold_delta > 0
                disp(['detected delta violation: delta = ', num2str(delta(1))]);
            end
            cv = cv_vx + cv_vy + cv_px + cv_py + cv_r + cv_psi ...
                + cv_omega_f + cv_omega_r + cv_delta;
        end

        function cv = path_constraints_violation(sol,OCP, X_initial, Y_initial, X_move_range, Y_move_range)
            %path_constraints_violation Computes the path constraints
            %violation
            %   cv = path_constraints_violation(sol,OCP, X_initial,
            %   Y_initial, X_move_range, Y_move_range) computes how much
            %   has the solution violated the path constraints. X_initial,
            %   Y_initial are the initial X,Y positions, X_move_range and
            %   Y_move_range are the allowed range of movement in X, Y
            %   directions.

            px = sol.value(OCP.px);
            py = sol.value(OCP.py);
            cv_px = abs( (px > X_initial + X_move_range) .* (px - X_initial - X_move_range) ) ...
                + abs( (px < X_initial - X_move_range) .* (X_initial - X_move_range - px) );
            cv_py = abs( (py > Y_initial + Y_move_range) .* (py - Y_initial - Y_move_range) ) ...
                + abs( (py < Y_initial - Y_move_range) .* (Y_initial - Y_move_range - py) );
            cv = sum(cv_px + cv_py);
        end

        function cv = collision_constraints_violation(sol,OCP)
            %collision_constraints_violation Compute collision constraint
            %violation
            %   cv = collision_constraints_violation(sol,OCP) computes how
            %   much the solution has violated the collision constraints.
            R_sq = OCP.R_col^2;

            D_sq_1_a = (sol.value(OCP.x_1)-OCP.x_a).^2+(sol.value(OCP.y_1)-OCP.y_a).^2;
            cv_1_a = sum( ( D_sq_1_a < R_sq ) .* (R_sq - D_sq_1_a) );
            if cv_1_a > 0
                disp('Detected collision violation between point 1 and point a.');
            end
            D_sq_1_b = (sol.value(OCP.x_1)-OCP.x_b).^2+(sol.value(OCP.y_1)-OCP.y_b).^2;
            cv_1_b = sum( ( D_sq_1_b < R_sq ) .* (R_sq - D_sq_1_b) );
            if cv_1_b > 0
                disp('Detected collision violation between point 1 and point b.');
            end
            D_sq_1_c = (sol.value(OCP.x_1)-OCP.x_c).^2+(sol.value(OCP.y_1)-OCP.y_c).^2;
            cv_1_c = sum( ( D_sq_1_c < R_sq ) .* (R_sq - D_sq_1_c) );
            if cv_1_c > 0
                disp('Detected collision violation between point 1 and point c.');
            end
            D_sq_2_a = (sol.value(OCP.x_2)-OCP.x_a).^2+(sol.value(OCP.y_2)-OCP.y_a).^2;
            cv_2_a = sum( ( D_sq_2_a < R_sq ) .* (R_sq - D_sq_2_a) );
            if cv_2_a > 0
                disp('Detected collision violation between point 2 and point a.');
            end
            D_sq_2_b = (sol.value(OCP.x_2)-OCP.x_b).^2+(sol.value(OCP.y_2)-OCP.y_b).^2;
            cv_2_b = sum( ( D_sq_2_b < R_sq ) .* (R_sq - D_sq_2_b) );
            if cv_2_b > 0
                disp('Detected collision violation between point 2 and point b.');
            end
            D_sq_2_c = (sol.value(OCP.x_2)-OCP.x_c).^2+(sol.value(OCP.y_2)-OCP.y_c).^2;
            cv_2_c = sum( ( D_sq_2_c < R_sq ) .* (R_sq - D_sq_2_c) );
            if cv_2_c > 0
                disp('Detected collision violation between point 2 and point c.');
            end
            D_sq_3_a = (sol.value(OCP.x_1)-OCP.x_a).^2+(sol.value(OCP.y_3)-OCP.y_a).^2;
            cv_3_a = sum( ( D_sq_3_a < R_sq ) .* (R_sq - D_sq_3_a) );
            if cv_3_a > 0
                disp('Detected collision violation between point 3 and point a.');
            end
            D_sq_3_b = (sol.value(OCP.x_3)-OCP.x_b).^2+(sol.value(OCP.y_3)-OCP.y_b).^2;
            cv_3_b = sum( ( D_sq_3_b < R_sq ) .* (R_sq - D_sq_3_b) );
            if cv_3_b > 0
                disp('Detected collision violation between point 3 and point b.');
            end
            D_sq_3_c = (sol.value(OCP.x_3)-OCP.x_c).^2+(sol.value(OCP.y_3)-OCP.y_c).^2;
            cv_3_c = sum( ( D_sq_3_c < R_sq ) .* (R_sq - D_sq_3_c) );
            if cv_3_c > 0
                disp('Detected collision violation between point 3 and point c.');
            end
            cv = cv_1_a + cv_1_b + cv_1_c + cv_2_a + cv_2_b + cv_2_c ...
                + cv_3_a + cv_3_b + cv_3_c;
        end

        function cv = dynamic_constraints_violation(sol, OCP)
            %dynamic_constraints_violation Computes dynamic constraints
            %violation
            %   cv = dynamic_constraints_violation(sol, OCP) computes the
            %   sum of error for dynamic propagation for all n steps

            % The math:
            % There are in total OCP.N control steps, and OCP.N+1 result
            % steps. The task is to check whether the OCP.N dynamic
            % propagations are obeying to the vehicle dynamics modelled in
            % OCP.vehdyn. In each of the OCP.N steps, this can be checked
            % by computing the difference bewteen the (k+1)-th step system
            % state and the one-step-propagation step from k-th step.

            
            X = sol.value(OCP.X);   % Nstate x (OCP.N+1)
            U = sol.value(OCP.U);   % Ncontrol x (OCP.N+1)
            f = @OCP.vehdyn;
            dt = OCP.T/OCP.N; % length of a control interval
            cv_sum = 0;
            %sample iteration
            %k=1;    % iterator for k = 1:OCP.N
            for k = 1:OCP.N
               % Runge-Kutta 4 integration
               k1 = f(X(:,k), U(:,k));
               k2 = f(X(:,k)+dt/2*k1, U(:,k));
               k3 = f(X(:,k)+dt/2*k2, U(:,k));
               k4 = f(X(:,k)+dt*k3, U(:,k));
               x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4);    % X_rk4(:, k+1)
               cv_sum = cv_sum + sum( abs(X(:,k+1) - x_next) );
            end
            cv = cv_sum;

        end
    end

    methods (Static)    % Replay related
        function replay(sol, OCP)
            %replay Replays the simulation using current control inputs
            %   replay(sol, OCP) replays the simulation by applying the
            %   controls in sol to the same vehicle model, then plot the
            %   results. This helps indentify any discrepancy between
            %   casadi solution and simple simulation.
            N = 20;
            dt = 0.025;
            a = vehPlant_m();
            InitCond = [10,0,0,0,0,0,30,30,0];
            prep.SetInitCond(a, InitCond);
            uArray = sol.value(OCP.U);  % 2 x N
            a.sim(uArray, dt);
            % trajectory of the replay
            stateTraj_replay = a.get_trajectory();       % 9 x (N+1)
            timeArray_replay = linspace(0,N*dt, N+1);
            timeArray_sol = linspace(0,N*dt, N+1);

            pp.rectangle_overlap(OCP,sol);
            hold on;
            h_replay = plot(stateTraj_replay(3,:), stateTraj_replay(4,:),'LineWidth',2);
            legend(h_replay, "Replay");
            
            figure(); hold on;
            h_sol = plot(timeArray_sol, sol.value(OCP.vx),'LineWidth',2);
            h_replay = plot(timeArray_replay, stateTraj_replay(1,:),'LineWidth',2);
            legend([h_sol, h_replay], "Solution", "Replay");
        end

    end
end