classdef pp
    %pp A collection of post-processing functions
    %   This class contains post-processing function relevant to the
    %   vehicle collision avoidance simulations. For example, functions to
    %   draw vehicle shapes and make videos.


    methods (Static)

        function make_video_rectangles(OCP,sol,videoName,foundSol)
            %make_video_rectangles Make a video with the name videoName
            %   make_video_rectangles(OCP,sol,videoName,foundSol)
            if ~foundSol
                sol = OCP.opti.debug;
            end
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            v = VideoWriter([videoName '.avi']);
            v.FrameRate = 10;
            open(v);
            hFigure = figure();
            hold on;
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            if isprop(OCP,'delta')
                hasWheels = true;
                delta_arr = sol.value(OCP.delta);
            end
            a = OCP.a;
            b = OCP.b;
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % dynamic obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(4,3); % obstacle color: all black

            for i = 1:length(sol.value(OCP.px))
                plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
                hold on;
                pp.drawVehicleBox(X_c_arr(i),Y_c_arr(i),psi_arr(i),OCP.a+OCP.b+OCP.R_col,OCP.R_col);
                if hasWheels
                    pp.drawWheels(X_c_arr(i),Y_c_arr(i),psi_arr(i),delta_arr(i),a,b);
                end
                if staticObs
                    pp.drawVehicleBox(Xobs_c,Yobs_c,psiobs_c,OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
                else
                    pp.drawVehicleBox(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
                end
                xlim([-5, 25]);
                ylim([-5, 15]);
                xlabel('X'); ylabel('Y')
                axis equal;
                frame = getframe(hFigure);
                writeVideo(v,frame);
                hold off;
                %ClearLinesFromAxes(); % delete all line objects
            end
            close(v);
            close(hFigure);
            clear hFigure;
            disp(['Video saved as: ', pwd , filesep, videoName ,'.avi'])
        end

        function make_video_circles(OCP,sol,videoName,foundSol)
            %make_video_circles Make a video with the name videoName
            %   make_video_circles(OCP,sol,videoName,foundSol)
            if ~foundSol
                sol = OCP.opti.debug;
            end
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end

            v = VideoWriter([videoName '.avi']);
            v.FrameRate = 10;
            open(v);
            hFigure = figure();
            hold on;
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(4,3); % obstacle color: all black
            for i = 1:length(sol.value(OCP.px))
                plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
                hold on;
                pp.drawVehicleCircles(X_c_arr(i),Y_c_arr(i),psi_arr(i),OCP.a,OCP.R_col/2);
                if staticObs
                    pp.drawVehicleCircles(Xobs_c,Yobs_c,psiobs_c,...
                        OCP.a,OCP.R_col/2,obs_colors);  
                else
                    pp.drawVehicleCircles(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),...
                        OCP.a,OCP.R_col/2,obs_colors);  
                end
                xlim([-5, 25]);
                ylim([-5, 15]);
                xlabel('X'); ylabel('Y')
                axis equal;
                frame = getframe(hFigure);
                writeVideo(v,frame);
                hold off;
                %ClearLinesFromAxes(); % delete all line objects
            end 
            close(v);
            close(hFigure);
            clear hFigure;
            disp(['Video saved as: ', pwd , filesep, videoName ,'.avi'])
        end

        function [cost, cost_vx, cost_vy, cost_py] = calculate_cost(OCP,sol,foundSol)
            %calculate_cost Calculate the cost (objective) function value
            if ~foundSol
                sol = OCP.opti.debug;
            end
            
            cost_vx = 0.1*sum((sol.value(OCP.vx) - OCP.vxRef).^2);
            cost_vy = 0.1*sum((sol.value(OCP.vy) - OCP.vyRef).^2);
            cost_py = sum((sol.value(OCP.py) - OCP.pyRef).^2);
            cost = cost_vx + cost_vy + cost_py;
        end

        function basic_plots(OCP,sol)
            %basic_plots Plot the basic system states: velocity, position, yaw
            %rate, BEV (bird eye view) trajectory
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            figure('Position',[100 100 800 400]);
            tiledlayout(2,2)
            nexttile;
            hold on
            plot(sol.value(OCP.vx),'LineWidth',2);
            plot(sol.value(OCP.vy),'LineWidth',2);
            legend('vx','vy');
            title('Ego velocity')
            nexttile;
            hold on
            plot(sol.value(OCP.px),'LineWidth',2);
            plot(sol.value(OCP.py),'LineWidth',2);
            legend('px','py')
            title('Ego vehicle CG position');
            nexttile;
            hold on
            plot(sol.value(OCP.psi),'LineWidth',2);
            legend('psi')
            title('Ego vehicle yaw rate')
            nexttile;
            plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            xlabel('X'); ylabel('Y')
            obs_colors = zeros(4,3); % obstacle color: all black
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            if staticObs
                pp.drawVehicleBox(Xobs_c,Yobs_c,psiobs_c,...
                    OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
            else
                for i = 1:length(OCP.px_obs)
                    pp.drawVehicleBox(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),...
                        OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
                end
            end
            axis equal
        end

        function basic_plots_dugoff(OCP,sol)
            %basic_plots_dugoff Plot the basic system states with the
            %dugoff tire model
        
            figure('Position',[100 100 800 600]);
            tiledlayout(3,2)
            nexttile;
            hold on
            plot(sol.value(OCP.vx),'LineWidth',2);
            plot(sol.value(OCP.vy),'LineWidth',2);
            legend('$v_{x}$','$v_{y}$','interpreter','latex');
            title('Ego velocity in Global XY Coordinates')
            nexttile;
            hold on
            plot(sol.value(OCP.px),'LineWidth',2);
            plot(sol.value(OCP.py),'LineWidth',2);
            legend('$p_x$','$p_y$','interpreter','latex')
            title('Ego vehicle CG position');
            nexttile;
            hold on
            plot(sol.value(OCP.psi),'LineWidth',2);
            legend('$\psi$','interpreter','latex')
            title('Ego vehicle yaw rate')
            nexttile;
            plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            xlabel('X'); ylabel('Y'); axis equal; grid on;
            title('Birdeye View');
            obs_colors = zeros(4,3); % obstacle color: all black
            Xobs_c_arr = sol.value(OCP.px_obs);
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            for i = 1:length(OCP.px_obs)
                pp.drawVehicleBox(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),OCP.a+OCP.b,OCP.R_col,obs_colors);
            end
            nexttile;
            plot(sol.value(OCP.omega_f),'LineWidth',2);
            hold on
            plot(sol.value(OCP.omega_r),'LineWidth',2);
            legend('$\omega_f$','$\omega_r$','interpreter','latex');
            nexttile;
            plot(sol.value(OCP.delta),'LineWidth',2);
            legend('$\delta$','interpreter','latex');
            grid on;
        end

        function controls_plot(OCP,sol)
            %controls_plot Plot the control signals
            figure('Position',[100 100 800 200])
            tiledlayout(1,2);
            nexttile
            plot(sol.value(OCP.f_thr),'LineWidth',2);
            hold on;
            plot(sol.value(OCP.f_lat),'LineWidth',2);
            title('Longitudinal and Lateral Force Input');
            legend('$f_{thr}$','$f_{lat}$','interpreter','latex')
            nexttile;
            plot(sol.value(OCP.u_beta),'LineWidth',2);
            legend('$u_{\beta}$','interpreter','latex');
            title('Sideslip Control Input');
        end

        function controls_plot_dugoff(OCP,sol)
            %controls_plot_dugoff Plot the control signals in the dugoff
            %model with bicycle body model
            figure('Position',[100 100 800 200])
            tiledlayout(1,2);
            nexttile
            plot(sol.value(OCP.nu),'LineWidth',2);
            title('Steering Input');
            legend('$\nu [rad/s]$','interpreter','latex')
            nexttile;
            plot(sol.value(OCP.tau),'LineWidth',2);
            title('Torque Input')
            legend('$\tau [Nm]$','interpreter','latex');
            title('Torque Input');
        end

        function debug_plot(OCP,sol,foundSol)
            %debug_plot Plot the debug signals
            if ~foundSol
                sol = OCP.opti.debug;
            end
            figure('Position',[100 100 800 1200]);
            tiledlayout(6,2)
            nexttile;
            hold on
            plot(sol.value(OCP.v_xb_dbg),'LineWidth',2);
            plot(sol.value(OCP.v_yb_dbg),'LineWidth',2);
            legend('$v_{xb}$','$v_{yb}$','interpreter','latex');
            title('Ego velocity in Body Frame')
            nexttile;
            hold on
            plot(sol.value(OCP.V_xft_dbg),'LineWidth',2);
            plot(sol.value(OCP.V_xrt_dbg),'LineWidth',2);
            legend('$v_{xft}$','$v_{xrt}$','interpreter','latex')
            title('Longitudinal Velocity in Tire Frame');
            nexttile;
            hold on
            plot(sol.value(OCP.kappa_f_dbg),'LineWidth',2);
            plot(sol.value(OCP.kappa_r_dbg),'LineWidth',2);
            legend('$\kappa_{f}$','$\kappa_{r}$','interpreter','latex');
            title('Longitudinal Tire Slip Ratio')
            nexttile;
            hold on
            plot(sol.value(OCP.alpha_f_dbg),'LineWidth',2);
            plot(sol.value(OCP.alpha_r_dbg),'LineWidth',2);
            legend('$\alpha_{f}$','$\alpha_{r}$','interpreter','latex');
            title('Lateral Tire Slip Angle')
            nexttile;
            hold on
            plot(sol.value(OCP.F_zf_dbg),'LineWidth',2);
            plot(sol.value(OCP.F_zr_dbg),'LineWidth',2);
            legend('$F_{zf}$','$F_{zr}$','interpreter','latex');
            title('Vertical Axle Load')
            nexttile;
            hold on
            plot(sol.value(OCP.flambda_f_dbg),'LineWidth',2);
            plot(sol.value(OCP.flambda_r_dbg),'LineWidth',2);
            legend('$f(\lambda_{f})$','$f(\lambda_{r})$','interpreter','latex');
            title('F(Lambda)');
            nexttile;
            hold on
            plot(sol.value(OCP.F_xft_dbg),'LineWidth',2);
            plot(sol.value(OCP.F_yft_dbg),'LineWidth',2);
            legend('$F_{xft}$','$F_{yft}$','interpreter','latex');
            title('Front Tire Force in Tire Frame');
            nexttile;
            hold on
            plot(sol.value(OCP.F_xrt_dbg),'LineWidth',2);
            plot(sol.value(OCP.F_yrt_dbg),'LineWidth',2);
            legend('$F_{xrt}$','$F_{yrt}$','interpreter','latex');
            title('Rear Tire Force in Tire Frame');
            nexttile;
            hold on
            plot(sol.value(OCP.F_xfb_dbg),'LineWidth',2);
            plot(sol.value(OCP.F_yfb_dbg),'LineWidth',2);
            legend('$F_{xfb}$','$F_{yfb}$','interpreter','latex');
            title('Front Axle Force in Body Frame');
            nexttile;
            hold on
            plot(sol.value(OCP.F_xrb_dbg),'LineWidth',2);
            plot(sol.value(OCP.F_yrb_dbg),'LineWidth',2);
            legend('$F_{xrb}$','$F_{yrb}$','interpreter','latex');
            title('Rear Axle Force in Body Frame');
            nexttile;
            hold on
            plot(sol.value(OCP.F_xf_dbg),'LineWidth',2);
            plot(sol.value(OCP.F_yf_dbg),'LineWidth',2);
            legend('$F_{xf}$','$F_{yf}$','interpreter','latex');
            title('Front Axle Force in Global Frame');
            nexttile;
            hold on
            plot(sol.value(OCP.F_xr_dbg),'LineWidth',2);
            plot(sol.value(OCP.F_yr_dbg),'LineWidth',2);
            legend('$F_{xr}$','$F_{yr}$','interpreter','latex');
            title('Rear Axle Force in Global Frame');
    end

        function hFig = rectangle_overlap(OCP,sol)
            %rectangle_overlap Draw the ego-obstacle vehicles rectangles in overlap
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            if isprop(OCP,'delta') % check if OCP has the property delta
                hasWheels = true;
                delta_arr = sol.value(OCP.delta);
            else
                hasWheels = false;
            end
            hFig = figure();
            hold on;
            plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(4,3); % obstacle color: all black
            pp.drawVehicleBox(X_c_arr,Y_c_arr,psi_arr,OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth);
            if staticObs
                pp.drawVehicleBox(Xobs_c,Yobs_c,psiobs_c,...
                    OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
            else
                pp.drawVehicleBox(Xobs_c_arr,Yobs_c_arr,psiobs_c_arr,...
                    OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
            end
            if hasWheels
                pp.drawWheels(X_c_arr,Y_c_arr,psi_arr,delta_arr,OCP.a,OCP.b);
            end
            
            xlabel('X'); ylabel('Y')
            axis equal
        end

        function hFig = circles_overlap(OCP,sol)
            %circles_overlap Draw the ego-obstacle vehicles Ziegler 3-circles in overlap
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            hFig = figure();
            hold on;
            plot(sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = 0;
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(3,3); % obstacle color: all black
            if staticObs
                for i = 1:length(sol.value(OCP.px))
                    pp.drawVehicleCircles(X_c_arr(i),Y_c_arr(i),psi_arr(i),OCP.a,OCP.R_col/2);
                end
                pp.drawVehicleCircles(Xobs_c,Yobs_c,psiobs_c,...
                    OCP.a,OCP.R_col/2,obs_colors);
            else
                for i = 1:length(sol.value(OCP.px))
                    pp.drawVehicleCircles(X_c_arr(i),Y_c_arr(i),psi_arr(i),OCP.a,OCP.R_col/2);
                    pp.drawVehicleCircles(Xobs_c_arr(i),Yobs_c_arr(i),psiobs_c_arr(i),OCP.a,OCP.R_col/2,obs_colors);
                end
            end
            
            xlabel('X'); ylabel('Y')
            axis equal
        end
    end

    methods (Static)    % 2D drawing functions
        function p = drawVehicleBox(X_c,Y_c,psi,L,W,varargin)
                    %drawVehicleBox Draw vehicle box shape using line command
                    % p = drawVehicleBox(X_c,Y_c,psi,L,W,colors)
                    % Drawing vehicle box shape using a combination of line
                    % commands, which doesn't require the existing figure to have
                    % hold on property.
                    % Inputs:
                    % X_c, Y_c: 1xn double , vehicle CG X,Y coordinates [m]
                    % psi: 1xn double , yaw angle [rad]
                    % L: double, vehicle length [m]
                    % W: double, vehicle width [m]
                    % colors: 4x3 double[0-1], vehicle body edge colors (f,rear,rgt,l)
                    % Output:
                    % p: 1x4 handles, the line object handles to the four lines.
                    
                    % calculating vertices of vehicle rectangular box vertices,
                    % with subscript order:
                    % 1 - Left Front, 2 - Right Front, 3 - Left Rear, 4 - Right
                    % Rear, illustrated below:
                    % 
                    % + 3 --------------------------------------- + 1
                    % |                                                     |
                    % |                          .                          |
                    % |                        CG                        |
                    % |                                                     |
                    % + 4 --------------------------------------- + 2
        
                    Defaults = {[0 0 0; 0 0 1; 0 1 0; 1 0 0]};
                    Defaults(1:nargin-5) = varargin;
                    colors = Defaults{1}; % edge colors (front, right, rear, left)
        
                    
                    xVert = [L/2    L/2     -L/2    -L/2];
                    yVert = [W/2    -W/2    W/2     -W/2];
                    
                    Verticies = zeros(length(X_c),4,2);  % initialize verticies XY coords
                    for i = 1:4
                        for j = 1:length(X_c)
                            Verticies(j,i,:) = [cos(psi(j)) -sin(psi(j)); sin(psi(j)) cos(psi(j))] * ...
                                [xVert(i); yVert(i)] + [X_c(j); Y_c(j)];
                        end
                    end
                    
                    XVert = Verticies(:,:,1);
                    YVert = Verticies(:,:,2);
                    for j = 1:length(X_c)
                        p(1) = line([XVert(j,1) XVert(j,2)],[YVert(j,1) YVert(j,2)],'Color',colors(1,:),'LineStyle','-');
                        p(2) = line([XVert(j,2) XVert(j,4)],[YVert(j,2) YVert(j,4)],'Color',colors(2,:),'LineStyle','-');
                        p(3) = line([XVert(j,4) XVert(j,3)],[YVert(j,4) YVert(j,3)],'Color',colors(3,:),'LineStyle','-');
                        p(4) = line([XVert(j,3) XVert(j,1)],[YVert(j,3) YVert(j,1)],'Color',colors(4,:),'LineStyle','-');
                    end
                    axis equal
        end

        function drawWheels(X_c,Y_c,psi,delta,a,b,varargin)
            %drawWheels Draw four wheels
            % drawWheels(X_c,Y_c,psi,delta,a,b,tw,tireWidth,tireRadius) draws four wheels
            % based on vehicle CG, yaw angle, front steering angle, front and rear axle
            % distance, track width, tire width and radius.
            % Inputs:
            % X_c, Y_c: 1xn double , vehicle CG X,Y coordinates [m]
            % psi: 1xn double , yaw angle [rad]
            % delta: 1xn double, vehicle front axle steering angle [rad]
            % a,b : double, vehicle length [m]
            % tw: double, vehicle track width between left and right wheels [m]
            % tireWidth, tireRadius: double, tire width and tire radius [m]
            % colors: 4x3 double[0-1], vehicle tire edge colors (f,rear,rgt,l)
            % Output:
            % p: 1x4 handles, the line object handles to the four lines.
        
            % calculating positions of vehicle wheel centers on the body,
            % with subscript order:
            % 1 - Left Front, 2 - Right Front, 3 - Left Rear, 4 - Right Rear,
            % illustrated below:
            % 
            % + 3 --------------------------------------- + 1
            % |                                                     |
            % |                          .                          |
            % |                        CG                        |
            % |                                                     |
            % + 4 --------------------------------------- + 2
        
            Defaults = {1.6,0.25,0.3,[0 0 0; 0 0 0; 0 0 0; 0 0 0]};
            Defaults(1:nargin-6) = varargin;
            tw = Defaults{1}; % track width [m]
            tireWidth = Defaults{2}; % tire width [m]
            tireRadius = Defaults{3}; % tire radius [m]
            colors = Defaults{4}; % edge colors (front, right, rear, left)
            
            %%% STEP 1 Calculate the wheel center locations
            xVert = [a    a     -b    -b];
            yVert = [tw/2    -tw/2    tw/2     -tw/2];
            % vertices of the wheel centers
            Verticies = zeros(length(X_c),4,2);  % initialize verticies XY coords
            for i = 1:4 % i = 1,2,3,4: four body corners
                for j = 1:length(X_c) % j = 1,...,n: number of poses
                    Verticies(j,i,:) = [cos(psi(j)) -sin(psi(j)); sin(psi(j)) cos(psi(j))] * ...
                        [xVert(i); yVert(i)] + [X_c(j); Y_c(j)];
                end
            end
            
            XVert = Verticies(:,:,1);
            YVert = Verticies(:,:,2);
            
            %%% STEP 2 Calculate the wheel rubber corner locations
            numWheels = 4;
            xVert = [tireRadius    tireRadius     -tireRadius    -tireRadius];
            yVert = [tireWidth/2    -tireWidth/2    tireWidth/2     -tireWidth/2];
            psiFront = psi+delta;       % front wheel yaw rangle w.r.t. ground, 1xn
            psiRear = psi;                  % rear wheel yaw angle w.r.t. ground, 1xn
            psiWheels = [psiFront; psiFront; psiRear; psiRear]; % 4xn
            Verticies = zeros(length(X_c),numWheels,4,2);  % initialize verticies XY coords
            for j = 1:length(X_c)  % j = 1,...,n: number of poses
                % configure each of the four wheels' four vertices
                for k = 1:4 % k = 1,2,3,4: wheel center locations
                    XWhlCntr = XVert(j,k);
                    YWhlCntr = YVert(j,k);
                    % vertices of the wheel rubber corners, in total nxnumWheelsx4
                    % corners, each with two cordinate values
                    
                    for m = 1:4 % m = 1,2,3,4: four wheel rubber corners
                        Verticies(j,m,k,:) = [cos(psiWheels(k,j)) -sin(psiWheels(k,j));...
                            sin(psiWheels(k,j)) cos(psiWheels(k,j))] * ...
                            [xVert(m); yVert(m)] + [XWhlCntr; YWhlCntr];
                    end
                end
            end
        
            XVertTr = Verticies(:,:,:,1); % n x numWheels x 4
            YVertTr = Verticies(:,:,:,2); % n x numWheels x 4
            for j = 1:length(X_c)
                for m = 1:4 % m = 1,2,3,4: four wheel rubber corners
                    line([XVertTr(j,1,m) XVertTr(j,2,m)],[YVertTr(j,1,m) YVertTr(j,2,m)],'Color',colors(1,:),'LineStyle','-');
                    line([XVertTr(j,2,m) XVertTr(j,4,m)],[YVertTr(j,2,m) YVertTr(j,4,m)],'Color',colors(2,:),'LineStyle','-');
                    line([XVertTr(j,4,m) XVertTr(j,3,m)],[YVertTr(j,4,m) YVertTr(j,3,m)],'Color',colors(3,:),'LineStyle','-');
                    line([XVertTr(j,3,m) XVertTr(j,1,m)],[YVertTr(j,3,m) YVertTr(j,1,m)],'Color',colors(4,:),'LineStyle','-');
                end
            end
        
            axis equal
        
        end

        function p = drawVehicleCircles(X_c,Y_c,psi,d,R,varargin)
            %drawVehicleCircles Draw vehicle Ziegler 3-circle shape
            %   p = drawVehicleCircles(X_c,Y_c,psi,d,R,colors)
            
            %   Inputs:
            %   X_c, Y_c: 1xn double , vehicle CG X,Y coordinates [m]
            %   psi: 1xn double , yaw angle [rad]
            %   d: double, neiguboring circle center distance [m]
            %   R: double, vehicle width [m]
            %   colors: 3x3 double[0-1], vehicle 3-circle colors (front,mid,rear)
            %   Output:
            %   p: 1x3 handles, the line object handles to the three circles.
            
            %   The math:
            %   For the three circle centers [x_f,y_f], [x_m,y_m], [x_r,y_r]
            %   x_f = X_c + d*cos(psi);
            %   y_f = Y_c + d*sin(psi);
            %   x_m = X_c;
            %   y_m = Y_c;
            %   x_r = X_c - d*cos(psi);
            %   y_r = Y_c - d*sin(psi);
            
            Defaults = {[0 0 1; 0 1 0; 1 0 0]};
            Defaults(1:nargin-5) = varargin;
            colors = Defaults{1}; % edge colors (front, mid, rear)
            
            x_f = X_c + d*cos(psi);
            y_f = Y_c + d*sin(psi);
            x_m = X_c;
            y_m = Y_c;
            x_r = X_c - d*cos(psi);
            y_r = Y_c - d*sin(psi);
            
            for j = 1:length(X_c)
                p(1) = pp.circle(x_f(j),y_f(j),R,colors(1,:));
                p(2) = pp.circle(x_m(j),y_m(j),R,colors(2,:));
                p(3) = pp.circle(x_r(j),y_r(j),R,colors(3,:));
            end
            axis equal;
        end
        
        function hCirc = circle(x,y,r,varargin)
            %circle Draw a circle
            %   hCirc = circle(x,y,r,colorr) draws a circle centered at [x,y] with a
            %   radius of r, with the color of colorr
            %   0.01 is the angle step, bigger values will draw the circle faster but
            %   you might notice imperfections (not very smooth).
                Defaults = {[0 0 0]};
                Defaults(1:nargin-3) = varargin;
                colorr = Defaults{1};
                ang=0:0.01:2*pi; 
                xp=r*cos(ang);
                yp=r*sin(ang);
                hCirc = plot(x+xp,y+yp,'Color',colorr);
        end

    end

    methods (Static) % methods specific to MATLAB app
        function app_rectangle_overlap(hAx,OCP,sol)
            %app_rectangle_overlap Draw the ego-obstacle vehicles rectangles in overlap
            % Extra Inputs (app specific)
            % hAx: handle of axes
            cla(hAx); % clear content of the axes before the new plot.
            if isempty(OCP.px_obs)
                staticObs = true;
            else
                staticObs = false;
            end
            if isprop(OCP,'delta') % check if OCP has the property delta
                hasWheels = true;
                delta_arr = sol.value(OCP.delta);
            else
                hasWheels = false;
            end
            plot(hAx,sol.value(OCP.px),sol.value(OCP.py),'LineWidth',2);
            hold(hAx,'on');
            X_c_arr = sol.value(OCP.px);
            Y_c_arr = sol.value(OCP.py);
            psi_arr = sol.value(OCP.psi);
            Xobs_c = sol.value(OCP.x_b); % static obstacle
            Yobs_c = sol.value(OCP.y_b);
            psiobs_c = atan2(OCP.y_c-OCP.y_b,OCP.x_c-OCP.x_b);
            Xobs_c_arr = sol.value(OCP.px_obs); % moving obstacle
            Yobs_c_arr = sol.value(OCP.py_obs);
            psiobs_c_arr = sol.value(OCP.psi_obs);
            
            obs_colors = zeros(4,3); % obstacle color: all black
            pp.app_drawVehicleBox(hAx,X_c_arr,Y_c_arr,psi_arr,OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth);
            if staticObs
                pp.app_drawVehicleBox(hAx,Xobs_c,Yobs_c,psiobs_c,...
                    OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
            else
                pp.app_drawVehicleBox(hAx,Xobs_c_arr,Yobs_c_arr,psiobs_c_arr,...
                    OCP.aBmpr+OCP.bBmpr,OCP.bdyWdth,obs_colors);
            end
            if hasWheels
                pp.app_drawWheels(hAx,X_c_arr,Y_c_arr,psi_arr,delta_arr,OCP.a,OCP.b);
            end
            
            xlabel(hAx,'X'); ylabel(hAx,'Y');
            axis(hAx,'equal');
            hold(hAx,'off');
        end

        function p = app_drawVehicleBox(hAx,X_c,Y_c,psi,L,W,varargin)
            %app_drawVehicleBox Draw vehicle box shape using line command
            % p = app_drawVehicleBox(hAx,X_c,Y_c,psi,L,W,colors)
            % Drawing vehicle box shape using a combination of line
            % commands, which doesn't require the existing figure to have
            % hold on property. This is a MATLAB app special version of the
            % function drawVehicleBox, with an additional first argument of
            % the axes handle.
            % Inputs:
            % X_c, Y_c: 1xn double , vehicle CG X,Y coordinates [m]
            % psi: 1xn double , yaw angle [rad]
            % L: double, vehicle length [m]
            % W: double, vehicle width [m]
            % colors: 4x3 double[0-1], vehicle body edge colors (f,rear,rgt,l)
            % Output:
            % p: 1x4 handles, the line object handles to the four lines.
            
            % calculating vertices of vehicle rectangular box vertices,
            % with subscript order:
            % 1 - Left Front, 2 - Right Front, 3 - Left Rear, 4 - Right
            % Rear, illustrated below:
            % 
            % + 3 --------------------------------------- + 1
            % |                                                     |
            % |                          .                          |
            % |                        CG                        |
            % |                                                     |
            % + 4 --------------------------------------- + 2

            Defaults = {[0 0 0; 0 0 1; 0 1 0; 1 0 0]};
            Defaults(1:nargin-6) = varargin;
            colors = Defaults{1}; % edge colors (front, right, rear, left)

            
            xVert = [L/2    L/2     -L/2    -L/2];
            yVert = [W/2    -W/2    W/2     -W/2];
            
            Verticies = zeros(length(X_c),4,2);  % initialize verticies XY coords
            for i = 1:4
                for j = 1:length(X_c)
                    Verticies(j,i,:) = [cos(psi(j)) -sin(psi(j)); sin(psi(j)) cos(psi(j))] * ...
                        [xVert(i); yVert(i)] + [X_c(j); Y_c(j)];
                end
            end
            
            XVert = Verticies(:,:,1);
            YVert = Verticies(:,:,2);
            for j = 1:length(X_c)
                p(1) = line(hAx,[XVert(j,1) XVert(j,2)],[YVert(j,1) YVert(j,2)],'Color',colors(1,:),'LineStyle','-');
                p(2) = line(hAx,[XVert(j,2) XVert(j,4)],[YVert(j,2) YVert(j,4)],'Color',colors(2,:),'LineStyle','-');
                p(3) = line(hAx,[XVert(j,4) XVert(j,3)],[YVert(j,4) YVert(j,3)],'Color',colors(3,:),'LineStyle','-');
                p(4) = line(hAx,[XVert(j,3) XVert(j,1)],[YVert(j,3) YVert(j,1)],'Color',colors(4,:),'LineStyle','-');
            end
        end

        function app_drawWheels(hAx,X_c,Y_c,psi,delta,a,b,varargin)
            %app_drawWheels Draw four wheels
            % app_drawWheels(hAx,X_c,Y_c,psi,delta,a,b,tw,tireWidth,tireRadius)
            % draws four wheels based on vehicle CG, yaw angle, front
            % steering angle, front and rear axle distance, track width,
            % tire width and radius. 
            % This is an app-specific variant of drawWheels() with an
            % additional first argument of hAX.
            % Inputs:
            % hAx: axes handle, the axes to be drawn upon
            % X_c, Y_c: 1xn double , vehicle CG X,Y coordinates [m]
            % psi: 1xn double , yaw angle [rad]
            % delta: 1xn double, vehicle front axle steering angle [rad]
            % a,b : double, vehicle length [m]
            % tw: double, vehicle track width between left and right wheels [m]
            % tireWidth, tireRadius: double, tire width and tire radius [m]
            % colors: 4x3 double[0-1], vehicle tire edge colors (f,rear,rgt,l)
            % Output:
            % p: 1x4 handles, the line object handles to the four lines.
        
            % calculating positions of vehicle wheel centers on the body,
            % with subscript order:
            % 1 - Left Front, 2 - Right Front, 3 - Left Rear, 4 - Right Rear,
            % illustrated below:
            % 
            % + 3 --------------------------------------- + 1
            % |                                                     |
            % |                          .                          |
            % |                        CG                        |
            % |                                                     |
            % + 4 --------------------------------------- + 2
        
            Defaults = {1.6,0.25,0.3,[0 0 0; 0 0 0; 0 0 0; 0 0 0]};
            Defaults(1:nargin-6) = varargin;
            tw = Defaults{1}; % track width [m]
            tireWidth = Defaults{2}; % tire width [m]
            tireRadius = Defaults{3}; % tire radius [m]
            colors = Defaults{4}; % edge colors (front, right, rear, left)
            
            %%% STEP 1 Calculate the wheel center locations
            xVert = [a    a     -b    -b];
            yVert = [tw/2    -tw/2    tw/2     -tw/2];
            % vertices of the wheel centers
            Verticies = zeros(length(X_c),4,2);  % initialize verticies XY coords
            for i = 1:4 % i = 1,2,3,4: four body corners
                for j = 1:length(X_c) % j = 1,...,n: number of poses
                    Verticies(j,i,:) = [cos(psi(j)) -sin(psi(j)); sin(psi(j)) cos(psi(j))] * ...
                        [xVert(i); yVert(i)] + [X_c(j); Y_c(j)];
                end
            end
            
            XVert = Verticies(:,:,1);
            YVert = Verticies(:,:,2);
            
            %%% STEP 2 Calculate the wheel rubber corner locations
            numWheels = 4;
            xVert = [tireRadius    tireRadius     -tireRadius    -tireRadius];
            yVert = [tireWidth/2    -tireWidth/2    tireWidth/2     -tireWidth/2];
            psiFront = psi+delta;       % front wheel yaw rangle w.r.t. ground, 1xn
            psiRear = psi;                  % rear wheel yaw angle w.r.t. ground, 1xn
            psiWheels = [psiFront; psiFront; psiRear; psiRear]; % 4xn
            Verticies = zeros(length(X_c),numWheels,4,2);  % initialize verticies XY coords
            for j = 1:length(X_c)  % j = 1,...,n: number of poses
                % configure each of the four wheels' four vertices
                for k = 1:4 % k = 1,2,3,4: wheel center locations
                    XWhlCntr = XVert(j,k);
                    YWhlCntr = YVert(j,k);
                    % vertices of the wheel rubber corners, in total nxnumWheelsx4
                    % corners, each with two cordinate values
                    
                    for m = 1:4 % m = 1,2,3,4: four wheel rubber corners
                        Verticies(j,m,k,:) = [cos(psiWheels(k,j)) -sin(psiWheels(k,j));...
                            sin(psiWheels(k,j)) cos(psiWheels(k,j))] * ...
                            [xVert(m); yVert(m)] + [XWhlCntr; YWhlCntr];
                    end
                end
            end
        
            XVertTr = Verticies(:,:,:,1); % n x numWheels x 4
            YVertTr = Verticies(:,:,:,2); % n x numWheels x 4
            for j = 1:length(X_c)
                for m = 1:4 % m = 1,2,3,4: four wheel rubber corners
                    line(hAx,[XVertTr(j,1,m) XVertTr(j,2,m)],[YVertTr(j,1,m) YVertTr(j,2,m)],'Color',colors(1,:),'LineStyle','-');
                    line(hAx,[XVertTr(j,2,m) XVertTr(j,4,m)],[YVertTr(j,2,m) YVertTr(j,4,m)],'Color',colors(2,:),'LineStyle','-');
                    line(hAx,[XVertTr(j,4,m) XVertTr(j,3,m)],[YVertTr(j,4,m) YVertTr(j,3,m)],'Color',colors(3,:),'LineStyle','-');
                    line(hAx,[XVertTr(j,3,m) XVertTr(j,1,m)],[YVertTr(j,3,m) YVertTr(j,1,m)],'Color',colors(4,:),'LineStyle','-');
                end
            end
        end

    end
        

end