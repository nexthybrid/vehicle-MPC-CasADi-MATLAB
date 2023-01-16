clear; clc;

vehicle = LoadVehicleParameters();

X = 40;          % initial X position
N = 1000;       % number of control discretizations
vx = 10;        % current velocity
Ts = 0.01;  
vxRef = 20;     % target velocity
ref = DLCReference(X, vx, vxRef, Ts, N, vehicle);

linspace(X, X+vx*N*Ts, N+1);
figure;
plot(linspace(X, X+vx*N*Ts, N+1),ref(end,:));