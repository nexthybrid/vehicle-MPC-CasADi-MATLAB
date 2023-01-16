function vehicle = LoadVehicleParameters()
% LOAD VEHICLE PARAMETERS
%   The function creates and returns vehicle object based on the specified
%   parameters.

lf = 1.311; % distance between the centre of gravity and the front axle
lr = 1.311;  % distance between the centre of gravity and the rear axle
w = 0.793;  % half of the vehicles width
m = 1599.98;   % mass
Jz = 2393.665;  % moment of inertia around z axis

cw = 0.37;  % air drag coefficient
Aw = 2.156;   % projected area in a transversal view

frontAxle.Cx = 3e5; % front tire longitudinal stiffness
frontAxle.Cy = 117950; % front tire cornering stiffness

frontAxle.Rw = 0.336705; % wheel moment of inertia
frontAxle.Jw = 2.084; % wheel moment of inertia
frontAxle.rollRes = 0.008;

rearAxle.Cx = 3e5; % rear tire longitudinal stiffness
rearAxle.Cy = 143700; % rear tire cornering stiffness

rearAxle.Rw = 0.33601;  % tire radius
rearAxle.Jw = 1.985; % wheel moment of inertia
rearAxle.rollRes = 0.008;

steerRatio = 13.4684; % steering wheel ratio

Ku = m*(rearAxle.Cy*lr - frontAxle.Cy*lf)/(rearAxle.Cy*frontAxle.Cy*(lr+lf));

vehicle = Vehicle(m, Jz, lf, lr, w, cw, Aw, frontAxle, rearAxle, steerRatio, Ku); % create vehicle object
end

