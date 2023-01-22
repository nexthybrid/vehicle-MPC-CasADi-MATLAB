% Car race along a track
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.

addpath('C:\Program Files\casadi-windows-matlabR2016a-v3.5.5');
import casadi.*

N = 100; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(2,N+1); % state trajectory
pos   = X(1,:);
speed = X(2,:);
U = opti.variable(1,N);   % control trajectory (throttle)
T = opti.variable();      % final time

% ---- objective          ---------
opti.minimize(T); % race in minimal time

% ---- dynamic constraints --------
f = @(x,u) [x(2);u-x(2)]; % dx/dt = f(x,u)

dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- path constraints -----------
limit = @(pos) 1-sin(2*pi*pos)/2;
opti.subject_to(speed<=limit(pos)); % track speed limit
opti.subject_to(0<=U<=1);           % control is limited

% ---- boundary conditions --------
opti.subject_to(pos(1)==0);   % start at position 0 ...
opti.subject_to(speed(1)==0); % ... from stand-still 
opti.subject_to(pos(N+1)==1); % finish line at position 1

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
opti.set_initial(speed, 1);
opti.set_initial(T, 1);


%% Extracting NLP from Opti;

f = opti.f;
g = opti.g;
x = opti.x;
p = opti.p;
lbg = opti.lbg;
ubg = opti.ubg;

x0 = opti.debug.value(x,opti.initial);
p  = opti.debug.value(p);

%% Solve with fmincon, with CasADi gradients

% Helper function to compute objective and its gradient
f = casadi.Function('f',{x,p},{f,gradient(f,x)});

% Helper function to compute constraint vector, its Jacobian, and bounds
g = casadi.Function('g',{x,p},{g,jacobian(g,x),lbg,ubg});

options = optimoptions('fmincon',...
                       'Display','iter',...
                       'Algorithm','sqp',...
                       'SpecifyObjectiveGradient',true,...
                       'SpecifyConstraintGradient',true);

[x_opt,fval] = fmincon(@(x) obj_casadi(f,x,p),x0,[],[],[],[],[],[], @(x) nonlin_casadi(g,x,p),options);


%% Post-processing


opti.set_initial(x, x_opt);

t = linspace(0,opti.value(T,opti.initial),N+1);
figure
hold on
plot(t,opti.debug.value(speed,opti.initial));
plot(t,opti.debug.value(pos,opti.initial));
plot(t,limit(opti.debug.value(pos,opti.initial)),'r--');
stairs(t(1:end-1),opti.debug.value(U,opti.initial),'k');
xlabel('Time [s]');
legend('speed','pos','speed limit','throttle','Location','northwest')

