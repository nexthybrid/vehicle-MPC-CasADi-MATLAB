classdef prep
    %prep Test preparation class that hosts test preparation functions


    methods (Static)

        function uArray = GenInputSeq(NuFcn, TauFcn, NCtrl, T)
            %GenInputSeq Generate input sequence from input function
            % GenInputSeq(NuFcn, TauFcn, NCtrl, T) generates discrete input 
            % sequence according to the input function InFcn. There are
            % NCtrl control intervals, (NCtrl + 1) total time points, and
            % the toal time length is T (seconds). InFcn is a function (@f)
            
            % prepare discrete time stamps at control intervals
            tArray = linspace(0, (NCtrl-1)/NCtrl*T, NCtrl);
            nuArray = NuFcn(tArray);    % 1xN
            tauArray = TauFcn(tArray);  % 1xN
            uArray = [nuArray; tauArray];   % 2xN
        end

        function LoadInitCond(vehicle, filename, index)
            %LoadInitCond Load initial condition set from file
            % LoadInitCond(vehicle, filename, index) loads the initial
            % condtion stored in filename with the index into the vehicle
            % object.
            
        end

        function SetInitCond(vehicle, InitCond)
            %SetInitCond Set the initial ocndition
            % SetInitCond(vehicle, InitCond) sets the initial condition set
            % InitCond to the vehicle object
            vehicle.set_states(InitCond);   % works for the vehPlant_m class
        end

        function SetSimulinkInitCond(mdlName, InitCond)
            %SetSimulinkInitCond Set the simulink model initial condition
            %   SetSimulinkInitCond(mdlName, InitCond) assigns the
            %   initial condition InitCond into the model workspace of the
            %   Simulink model with mdlName.
            mdl = load_system(mdlName);
            mdlWks = get_param(mdl,'ModelWorkspace');
            assignin(mdlWks,"InitCond", InitCond);
        end

        function SetSimulinkControl(mdlName, NuFcn, TauFcn, N, dt)
            %SetSimulinkControl Set the simulink model control lookup
            %   SetSimulinkInitCond(mdlName, InitCond) assigns the
            %   control sequence time-based lookup data into the
            %   Simulink model with mdlName.
            mdl = load_system(mdlName);
            mdlWks = get_param(mdl,'ModelWorkspace');
            timeBreakPoints = linspace(0,(N-1)*dt, N);
            uArray = prep.GenInputSeq(NuFcn, TauFcn, N, N*dt);

            assignin(mdlWks,"timeBreakPoints", timeBreakPoints);
            assignin(mdlWks, "nuDataPoints", uArray(1,:));
            assignin(mdlWks, "tauDataPoints", uArray(2,:));
        end

        function SetSimulinkStopTime(mdlName, stopTime)
            %SetSimulinkStopTime Set the Simulink stop time
            mdl = load_system(mdlName);
            set_param(mdl, 'StopTime', num2str(stopTime));
        end

    end
end