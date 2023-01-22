function dStates = VehicleModelDifferentiable(states, input)
% VEHICLE MODEL 
%   The function accepts current system states and inputs, and calculates 
%   state derivatives.

dStates = zeros(9,1);   % for Simulink size determination

% Define functions
gt_diff = @(x,x0) 0.5*(tanh( 10 * (x - x0)) + 1);
satur = @(x, low, upp) gt_diff(x,low).*(1 - gt_diff(x,upp)).*(x - low) + gt_diff(x,upp)*(upp - low) + low;

% State definition
vx = states(1,:);
vy = states(2,:);
psi = states(3,:);
omega = states(4,:);
wWheel = states(5:6,:);
deltaF = states(9, :);

% Input definition
dDeltaF = input(1,:);
T = input(2:end,:);

% Vehicle parameters
vehicle = LoadVehicleParameters();
g = 9.81; % gravity acceleration
mi = 0.9;

% deltaF = deltaSW/vehicle.steerRatio;


lf = vehicle.lf;
lr = vehicle.lr;
m = vehicle.m;
Jz = vehicle.Jz;

% Wheel parameters
% Front axle
Rwf = vehicle.frontAxle.Rw;
Jwf = 2*vehicle.frontAxle.Jw;
rollResF = 2*vehicle.frontAxle.rollRes;

% Rear axle
Rwr = vehicle.rearAxle.Rw;
Jwr = 2*vehicle.rearAxle.Jw;
rollResR = 2*vehicle.rearAxle.rollRes;

% Rolling resistance
rollRes = [rollResF; rollResR];

% VEHICLE VELOCITIES

% Lateral velocities
vyf = vy + lf*omega;
vyr = vy - lr*omega;
% Longitudinal velocities
vxf = vx;
vxr = vx;

% TIRE VELOCITIES

% Cornering velocities
vcf = vyf.*cos(deltaF) - vxf.*sin(deltaF);
vcr = vyr;

% Longitudinal velocities
vlf = vyf.*sin(deltaF) + vxf.*cos(deltaF);
vlr = vxr;

% SLIP RATIOS
vl = [vlf; vlr]; % longitudinal wheel velocities
vw = [Rwf; Rwr].*wWheel;
sDrive = (vw-vl)./(vw+eps); % Driving
sBrake = (vw-vl)./(vl+eps); % Braking
relativeVel = vw - vl;
s = gt_diff(relativeVel,0).*sDrive + (1 - gt_diff(relativeVel,0)).*sBrake; % differentiable
% s =  sDrive;

% TIRE SLIP ANGLES
alpha_f = atan2(vcf,vlf);
alpha_r = atan2(vcr,vlr);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOAD TRANSFER

Fzf = g*m*lr/(2*(lf+lr));
Fzr = g*m*lf/(2*(lf+lr));
Fz = [Fzf; Fzr];

% TIRE FORCES

% Longitudinal forces
Flf = satur(vehicle.frontAxle.Cx*s(1,:), -mi*Fzf, +mi*Fzf);
% Flf = vehicle.frontAxle.Cx*s(1,:);
Flr = satur(vehicle.rearAxle.Cx*s(2,:), -mi*Fzr, +mi*Fzr);
% Flr = vehicle.rearAxle.Cx*s(2,:);
Fl = 2*[Flf; Flr];

% Conrnering forces
Fcf = satur(-vehicle.frontAxle.Cy*alpha_f, -mi*Fzf, +mi*Fzf);
% Fcf = -vehicle.frontAxle.Cy*alpha_f;
Fcr = satur(-vehicle.rearAxle.Cy*alpha_r, -mi*Fzr, +mi*Fzr);
% Fcr = -vehicle.rearAxle.Cy*alpha_r;
Fc = 2*[Fcf; Fcr];

beta = atan2(Fc, Fl);
F_rez = sqrt(Fl.^2 + Fc.^2);

Fl_comb = F_rez.*cos(beta);
Fc_comb = F_rez.*sin(beta);

Flf_comb = Fl_comb(1,:);
Flr_comb = Fl_comb(2,:);

Fcf_comb = Fc_comb(1,:);
Fcr_comb = Fc_comb(2,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AXLE FORCES

% Lateral forces
Fyf = Flf_comb.*sin(deltaF) + Fcf_comb.*cos(deltaF);
Fyr = Fcr_comb;
% Longitudinal forces
Fxf = Flf_comb.*cos(deltaF) - Fcf_comb.*sin(deltaF);
Fxr = Flr_comb;

% VEHICLE DYNAMICS MODEL

dvx = vy.*omega + 1/m *(Fxf + Fxr);
dvy = -vx.*omega + 1/m *(Fyf + Fyr);
dOmega = lf/Jz*Fyf - lr/Jz*Fyr;
dwWheel = (T - [Rwf; Rwr].*Fl_comb - rollRes.*Fz)./[Jwf; Jwr];
dX = vx.*cos(psi) - vy.*sin(psi);
dY = vx.*sin(psi) + vy.*cos(psi);

dStates = [dvx; dvy; omega; dOmega; dwWheel; dX; dY; dDeltaF];

end