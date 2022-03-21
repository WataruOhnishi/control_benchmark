clear; close all;

%% Plant definition
s = tf('s');
z_G = 1.0e+03 * [...
 -0.153329147774075 + 1.825673570553010i;
 -0.153329147774075 - 1.825673570553010i;
 -0.039126477290724 + 0.601973241891408i;
 -0.039126477290724 - 0.601973241891408i;
 -0.003170161097733 + 0.273922740595655i;
 -0.003170161097733 - 0.273922740595655i;
 ];

p_G = 1.0e+03 * [...
 -0.131098843525143 + 1.868124153338216i;
 -0.131098843525143 - 1.868124153338216i;
 -0.022700067819691 + 0.559791345771639i;
 -0.022700067819691 - 0.559791345771639i;
 -0.005446668039055 + 0.191363603871908i;
 -0.005446668039055 - 0.191363603871908i;
 -0.002100774089528 + 0.000000000000000i;
 ];
G_k = 0.990921091595854;

Gvel = zpk(z_G,p_G,G_k);
Gn1vel = zpk([],[p_G(end)],G_k);
Gn1vel = Gn1vel * dcgain(minreal(Gvel))/dcgain(minreal(Gn1vel));

figure;bode(Gvel,Gn1vel);

%% Controller design
% edit from here ---------------------
% vel PI
% [kvp,kvi] = designpi(Gn1vel,1*2*pi)
kvp = 3;
kvi = 20;

% pos P
kpp = 2;
% edit end here ----------------------
Kpi = pid(kvp,kvi);
Kp = pid(kpp);

% advanced design
Knotch = tf(1);
Klpf = tf(1);
K = Kpi*Knotch*Klpf;
% equivalent pid
Kpid_e = (s+kpp)*(kvp*s+kvi)/s;

%% Robustness test frequency domain
% Position loop 
S = feedback(1,Gvel/s*Kpid_e);
SG = feedback(Gvel/s,Kpid_e);
figure;bodemag(S,tf(2,1)); legend('Sensitivity','Constraint');

figure; nyquist(Gvel/s*Kpid_e); 
xlim([-2,1]); ylim([-2,1]);

figure;margin(Gvel/s*Kpid_e); 

%% Performance test in time domain
t = 0:1e-3:50;
y = step(SG,t);

figure; plot(t,y); title('Step disturbance response');
xlim([0,2]);

%% Score output
fprintf('----------------\n')
fprintf('ROBUSTNESS SCORE\n')
fprintf('||S||_inf: %.2f [dB]\n',mag2db(norm(S,inf)))
[Gm,Pm,Wcg,Wcp] = margin(Gvel/s*Kpid_e);
fprintf('GM: %.2f [dB] at %.2f [Hz]\n',mag2db(Gm),Wcg/2/pi)
fprintf('PM: %.2f [deg] at %.2f [Hz]\n',Pm,Wcp/2/pi)
fprintf('----------------\n')

fprintf('PERFORMANCE SCORE\n')
fprintf('Disturbance response: %.4f (smaller better)\n',norm(y,2))
fprintf('----------------\n')
