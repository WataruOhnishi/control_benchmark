%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4th order benchmark problem:
% ------------------------
% Descr.:   Benchmark problem for two MP systems and one NMP system 
% System:   4th order model of nano stage 1 
% Author:   Wataru Ohnishi, Koseki lab, the University of Tokyo, 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;
close all;

s = tf('s');
%% plant definition
Pmp1 = func_plant_stage1(0); % minimum phase system (collocated)
Pmp2 = func_plant_stage1(0.085); % minimum phase system (non-collocated) 
Pnmp = func_plant_stage1(0.30); % non-minimum phases system (non-collocated)

% 2nd order approximation
[z_Pmp1, p_Pmp1, k_Pmp1] = zpkdata(Pmp1,'v');
z_Pmp1 = sort(z_Pmp1); p_Pmp1 = sort(p_Pmp1);
P2 = zpk([],[p_Pmp1(1:2)],k_Pmp1)/dcgain(zpk(p_Pmp1(3:4),z_Pmp1,1));
figure('name','Bode plot'); bode(P2,Pmp1,Pmp2,Pnmp); legend('P2','Pmp1','Pmp2','Pnmp');

%% Pmp1
P = Pmp1;
figure('name','plant'); bode(P);

%% controller design
% please design controller
% PID controller as an example
kp = 1; ki = 0; kd = 0; tau = 0;
C = kp + ki/s + kd*s/(s+tau); 

%{
% Hints

% 1. Try and error design

% 2. PID pole placement design using 2nd order model
% Ref. G. C. Goodwin, S. F. Graebe, and M. E. Salgado, Control System Design. 2000.
% Section 7.2. Theorem 7.1 (Sylvester's theorem)

% 3. Add notch filter or phase lead-lag filter

% 4. Disturbance observer

% 5. State feedback

% etc..
%}

%% verify
% reference response
Gyr = feedback(P*C,1);
figure('name','reference response'); step(Gyr);

% disturbance response
Gyd = feedback(P,C);
figure('name','disturbance response'); step(Gyd);

% sensitivity function
S = feedback(1,P*C);
figure('name','sensitivity function'); bode(S);

% margin
try M = allmargin(P*C) % Octave compatibility
catch figure; margin(P*C); end

% nyquist diagram
figure('name','nyquist diagram'); nyquist(P*C);
xlim([-100,100]);ylim([-100,100]);

