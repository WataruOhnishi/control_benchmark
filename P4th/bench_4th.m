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
figure;  bode(P2,Pmp1,Pmp2,Pnmp);

%% Pmp1
P = Pmp1;
C = 1; % please design controller

% verify
% reference response
Gyr = feedback(P*C,1);
figure; step(Gyr);

% disturbance response
Gyd = feedback(P,C);
figure; step(Gyd);

% sensitivity function
S = feedback(1,P*C);
figure; bode(S);

% margin
try M = allmargin(P*C) % Octave compatibility
catch figur; margin(P*C); end

% nyquist diagram
figure; nyquist(P*C);

