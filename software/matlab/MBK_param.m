% Initialize paramters for the MBK Control System in Simulink
s = tf('s');

dt = 0.01;
CF = 100;
tau = 0.05;
beta = exp(-dt/tau);

% Motor Paramters
Rw = 8.2;
Lw = 8.6e-3;
n = 324;
Kb = 0.00375;
Kt = 0.00281;
Bm = 0.00538; % Bm and Jm have been reflected to output shaft
Jm = 4.03e-5;

% Load parameters
Bl = 0;
Jl = 0;

% Calculating system models

Ye = 1/(s*Lw+Rw);
Ym = 1/(s*(Jm+Jl)+(Bm+Bl));

Gp = Ye*Ym*n*Kt/(1+n^2*Kb*Kt*Ye*Ym)*1/s;

Ga = 14; % motor driver acts as a gain based on the supply voltage

Hs = 1; % assume encoder delay is negligable

% Control Design

GHs = minreal(Ga*Gp*Hs); 
% use pzplot(GHs) to get find wd
wd = 133;
tauf = 1/(10*wd);
beta = exp(-dt/tauf); % beta for the IRR filter in feedback path
Nf = beta/(1-beta); 

N = 0.5+Nf;
Hc = (CF/Hs)/(N*s+CF); % if Hs is a tf use dcgain(Hs) instead

G = Gp*Ga;
H = Hc*Hs;