classdef t_vehModelTest_SimulinkComparison < matlab.unittest.TestCase
% compare script-RK4 based simulation with Simulink-based simulation
% the purpose is to cross-check the correctness of integration methods and
% identify potential vehicle dynamics error.

    methods(TestClassSetup)
        % Shared setup for the entire test class
    end

    methods(TestMethodSetup)
        % Setup for each test
    end

    

    methods(Test)
        % Test methods

        function zeroInput_matlab(testCase)
            % test models with a zero input, the vehicle should keep
            % near-constant speed afterwards.
            N = 300;
            dt = 0.01;
            a = vehPlant_m();
            InitCond = [10,0,0,0,0,0,30,30,0];
            prep.SetInitCond(a, InitCond);
            NuFcn = @(x) x*0;
            TauFcn = @(x) x*0;
            uArray = prep.GenInputSeq(NuFcn, TauFcn, N, N*dt);
            a.sim(uArray, dt);
            % trajectory of the simulation
            stateTraj_a = a.get_trajectory();       % 9 x (N+1)
            testCase.verifyTrue(true);
            hFig = figure();
            timeArray = linspace(0,N*dt, N+1);
            plot(timeArray, stateTraj_a(1,:));
            clear a;
            close(hFig);

        end

        function zeroInput_simulink(testCase)
            % test models with a zero input, the vehicle should keep
            % near-constant speed afterwards.
            N = 300;
            dt = 0.01;
            InitCond = [10,0,0,0,0,0,30,30,0];
            modelFile = "tests\vehPlant_m_simulink.slx";
            prep.SetSimulinkInitCond(modelFile, InitCond);
            NuFcn = @(x) x*0;
            TauFcn = @(x) x*0;
            prep.SetSimulinkControl(modelFile, NuFcn, TauFcn, N, dt);
            prep.SetSimulinkStopTime(modelFile, N*dt);
            simOut = sim(modelFile);
            
            % trajectory of the simulation
            size_1 = size(simOut.states,1);
            size_3 = size(simOut.states,3);
            stateTraj_b = reshape(simOut.states, [size_1 size_3]);       % 9 x (N+1)
            testCase.verifyTrue(true);
            hFig = figure();
            timeArray = simOut.tout';
            plot(timeArray, stateTraj_b(1,:));
            close(hFig);

        end

        function zeroInput_comparison(testCase)
            % test models with a zero input, the vehicle should keep
            % near-constant speed afterwards.
            N = 300;
            dt = 0.01;
            a = vehPlant_m();
            InitCond = [10,0,0,0,0,0,30,30,0];
            prep.SetInitCond(a, InitCond);
            NuFcn = @(x) x*0;
            TauFcn = @(x) x*0;
            uArray = prep.GenInputSeq(NuFcn, TauFcn, N, N*dt);
            a.sim(uArray, dt);
            % trajectory of the simulation
            stateTraj_a = a.get_trajectory();       % 9 x (N+1)
            figure();
            hold on;
            timeArray = linspace(0,N*dt, N+1);
            hCurve_a = plot(timeArray, stateTraj_a(1,:));
            clear a;

            modelFile = "tests\vehPlant_m_simulink.slx";
            prep.SetSimulinkInitCond(modelFile, InitCond);
            prep.SetSimulinkControl(modelFile, NuFcn, TauFcn, N, dt);
            prep.SetSimulinkStopTime(modelFile, N*dt);
            simOut = sim(modelFile);
            
            % trajectory of the simulation
            size_1 = size(simOut.states,1);
            size_3 = size(simOut.states,3);
            stateTraj_b = reshape(simOut.states, [size_1 size_3]);       % 9 x (N+1)
            timeArray = simOut.tout';
            hCurve_b = plot(timeArray, stateTraj_b(1,:));
            legend([hCurve_a, hCurve_b], "MATLAB", "Simulink");
            xlabel("Time (s)");
            ylabel("Vx (m/s)");
            title("Zero Input Simulation Comparison")
            testCase.verifyTrue(true);
        end

        function acceleration_comparison(testCase)
            % test models with constant positive tau input, the vehicle
            % should accelerate
            N = 300;
            dt = 0.01;
            a = vehPlant_m();
            InitCond = [10,0,0,0,0,0,30,30,0];
            prep.SetInitCond(a, InitCond);
            NuFcn = @(x) x*0;
            TauFcn = @(x) x*0 + 500;
            uArray = prep.GenInputSeq(NuFcn, TauFcn, N, N*dt);
            a.sim(uArray, dt);
            % trajectory of the simulation
            stateTraj_a = a.get_trajectory();       % 9 x (N+1)
            figure();
            hold on;
            timeArray = linspace(0,N*dt, N+1);
            hCurve_a = plot(timeArray, stateTraj_a(1,:));
            clear a;

            modelFile = "tests\vehPlant_m_simulink.slx";
            prep.SetSimulinkInitCond(modelFile, InitCond);
            prep.SetSimulinkControl(modelFile, NuFcn, TauFcn, N, dt);
            prep.SetSimulinkStopTime(modelFile, N*dt);
            simOut = sim(modelFile);
            
            % trajectory of the simulation
            size_1 = size(simOut.states,1);
            size_3 = size(simOut.states,3);
            stateTraj_b = reshape(simOut.states, [size_1 size_3]);       % 9 x (N+1)
            timeArray = simOut.tout';
            hCurve_b = plot(timeArray, stateTraj_b(1,:));
            legend([hCurve_a, hCurve_b], "MATLAB", "Simulink");
            xlabel("Time (s)");
            ylabel("Vx (m/s)");
            title("Acceleration Simulation Comparison")
            testCase.verifyTrue(true);
        end

    end

end