clear; clc;

tanh_coeff = 5; % change to other values to see the effect
gt_diff = @(x,x0) 0.5*(tanh(tanh_coeff*(x - x0)) + 1);
satur = @(x, low, upp) gt_diff(x,low).*(1 - gt_diff(x,upp)).*(x - low) + gt_diff(x,upp)*(upp - low) + low;

low = -5;
upp = 5;
x = [-50:0.01:50];
plot(x, max(low, min(x, upp))); hold; grid;
plot(x, satur(x, low, upp));
