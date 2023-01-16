classdef t_vehModelTest_SimulinkComparison_fixture < matlab.unittest.TestCase
% compare script-RK4 based simulation with Simulink-based simulation
% the purpose is to cross-check the correctness of integration methods and
% identify potential vehicle dynamics error.

    properties  % variables to be passed between Setup, Test, and TearDown
            N
            dt
            InitCond
            NuFcn
            TauFcn
            test_title
    end

    methods(TestClassSetup)
        % Shared setup for the entire test class
        function DefaultDiscretization(tc)
            %DefaultDiscretization Set default discretization for N and dt
            %   this can be overwritten in each test if desired.
            tc.N = 300;
            tc.dt = 0.01;
        end

        function DefaultInitCond(tc)
            %DefaultInitCond Set default initial condition
            %   this can be overwritten in each test if desired.
            tc.InitCond = [10,0,0,0,0,0,30,30,0];
        end
    end

    methods(TestMethodSetup)
        % Setup for each test
    end

    methods(TestMethodTeardown)
        function SimAndPlot(tc)
            %SimAndPlot run the simulations and plot figure
            %   as a wrap-up routine that is repeated in every test,
            %   run the simulations and plot the figure.
            a = vehPlant_m();
            prep.SetInitCond(a, tc.InitCond);
            uArray = prep.GenInputSeq(tc.NuFcn, tc.TauFcn, tc.N, tc.N*tc.dt);
            a.sim(uArray, tc.dt);
            % trajectory of the simulation a
            stateTraj_a = a.get_trajectory();       % 9 x (N+1)
            timeArray_a = linspace(0,tc.N*tc.dt, tc.N+1);
            clear a;

            modelFile = "tests\vehPlant_m_simulink.slx";
            prep.SetSimulinkInitCond(modelFile, tc.InitCond);
            prep.SetSimulinkControl(modelFile, tc.NuFcn, tc.TauFcn, tc.N, tc.dt);
            prep.SetSimulinkStopTime(modelFile, tc.N*tc.dt);
            simOut = sim(modelFile);
            
            % trajectory of the simulation b
            size_1 = size(simOut.states,1);
            size_3 = size(simOut.states,3);
            stateTraj_b = reshape(simOut.states, [size_1 size_3]);       % 9 x (N+1)
            timeArray_b = simOut.tout';

            figure();
            tiledlayout(2,1);
            nexttile;
            hold on;
            hCurve_a = plot(timeArray_a, stateTraj_a(1,:));
            hCurve_b = plot(timeArray_b, stateTraj_b(1,:));
            legend([hCurve_a, hCurve_b], "MATLAB", "Simulink");
            xlabel("Time (s)");
            ylabel("Vx (m/s)");
            title(tc.test_title);
            nexttile;
            hold on;
            hCurve_a = plot(stateTraj_a(3,:), stateTraj_a(4,:));
            hCurve_b = plot(stateTraj_b(3,:), stateTraj_b(4,:));
            xlabel("X (m)");
            ylabel("Y (m)");
            axis equal;
            legend([hCurve_a, hCurve_b], "MATLAB", "Simulink");
            tc.verifyTrue(true);
        end

    end

    methods(Test)
        % Test methods

        function zeroInput_comparison(tc)
            % test models with a zero input, the vehicle should keep
            % near-constant speed afterwards.

            tc.NuFcn = @(x) x*0;
            tc.TauFcn = @(x) x*0;
            tc.test_title = "Zero Input Simulation Comparison";
        end

        function acceleration_comparison(tc)
            % test models with constant positive tau input, the vehicle
            % should accelerate.
            tc.NuFcn = @(x) x*0;
            tc.TauFcn = @(x) x*0 + 500;
            tc.test_title = "Acceleration Simulation Comparison";
        end

        function accel_coast_comparison(tc)
            % test models with constant positive tau input for 2 seconds,
            % then coast for 2 seconds.
            tc.N = 400;
            tc.dt = 0.01;
            tc.NuFcn = @(x) x*0;
            tc.TauFcn = @(x) (x <= 2).* 500 + (x > 2).*0;
            tc.test_title = "Acceleration-Coast Simulation Comparison";
        end

        function coast_turn_comparison(tc)
            % test models with constant zero tau input but a constant
            % steering speed to the left (positive) for 3 seconds.
            tc.NuFcn = @(x) x*0 + 0.1;
            tc.TauFcn = @(x) x*0;
            tc.test_title = "Coast-LeftTurn Simulation Comparison";
        end

        function accel_turn_comparison(tc)
            % test models with constant positive tau input but a constant
            % steering speed to the left (positive) for 3 seconds.
            tc.NuFcn = @(x) x*0 + 0.1;
            tc.TauFcn = @(x) x*0 + 700;
            tc.test_title = "Accel-LeftTurn Simulation Comparison";
        end
    end

end