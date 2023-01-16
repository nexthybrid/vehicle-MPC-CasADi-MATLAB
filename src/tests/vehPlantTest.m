classdef vehPlantTest < matlab.unittest.TestCase

    methods(TestClassSetup)
        % Shared setup for the entire test class
    end

    methods(TestMethodSetup)
        % Setup for each test
    end

    methods(Test)
        % Test methods

        function initializeToZero(testCase)
            a = vehPlant();
            testCase.verifyEqual([a.vx,a.vy,a.px,a.py,a.r,a.psi,a.omega_f,a.omega_r,a.delta]',...
                [0,0,0,0,0,0,0,0,0]');
        end

        function zeroInputToDynamics(testCase)
            % test step() under zero input, the state should stay the same
            a = vehPlant();
            dx = a.vehdyn([0;0]);
            testCase.verifyEqual(dx,[0,0,0,0,0,0,0,0,0]');

        end

        function zeroStepInput(testCase)
            % test step() under zero input, the state should stay the same
            a = vehPlant();
            dt = 0.01; %s
            a.step([0;0],dt);
            testCase.verifyEqual([a.vx,a.vy,a.px,a.py,a.r,a.psi,a.omega_f,a.omega_r,a.delta]',...
                [0,0,0,0,0,0,0,0,0]');
        end

        function smallStepInput(testCase)
            % test step() under a small input, the vehicle should keep
            % constant speed afterwards
            a = vehPlant();
            dt = 0.01;
            a.step([0;100],dt);
            testCase.assertTrue(a.vx == 0);
            a.step([0;0],dt);
            testCase.assertTrue(a.vx > 0);
            constant_speed = a.vx;
            for i = 1:20
                a.step([0;0],dt);
                 testCase.verifyEqual(a.vx, constant_speed);
            end

            figure();
            plot(a.vx_);

        end

    end

end