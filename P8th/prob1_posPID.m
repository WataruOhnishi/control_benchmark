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
  0.000000000000000 + 0.000000000000000i;
 -0.131098843525143 + 1.868124153338216i;
 -0.131098843525143 - 1.868124153338216i;
 -0.022700067819691 + 0.559791345771639i;
 -0.022700067819691 - 0.559791345771639i;
 -0.005446668039055 + 0.191363603871908i;
 -0.005446668039055 - 0.191363603871908i;
 -0.002100774089528 + 0.000000000000000i;
 ];
G_k = 0.990921091595854;

G = zpk(z_G,p_G,G_k);
Gn2 = zpk([],[p_G(1);p_G(end)],G_k);
Gn2 = Gn2 * dcgain(minreal(G*s))/dcgain(minreal(Gn2*s));

figure;bode(G,Gn2);

%% Controller design
% edit from here ---------------------
% [kp,ki,kd,tau] = designpid(Gn2,4*2*pi)
kp = 25; 
ki = 40;
kd = 4;
tau = 0.01;
K = pid(kp,ki,kd,tau);

% advanced design
Knotch = tf(1);
Klpf = tf(1);
K = K*Knotch*Klpf;
% edit end here ----------------------

% Process sensitivity
SG = feedback(G,K);
% input disturbance to output

%% Robustness test frequency domain
% Sensitivity function
S = feedback(1,G*K);
figure;bodemag(S,tf(2)); legend('Sensitivity','Constraint');

figure; nyquist(G*K); 
xlim([-2,1]); ylim([-2,1]);

figure;margin(G*K); 

%% Performance test in time domain
t = 0:1e-3:50;
y = step(SG,t);

figure; plot(t,y); title('Step disturbance response');
xlim([0,2]);

%% Score output
fprintf('----------------\n')
fprintf('ROBUSTNESS SCORE\n')
fprintf('||S||_inf: %.2f [dB]\n',mag2db(norm(S,inf)))
[Gm,Pm,Wcg,Wcp] = margin(G*K);
fprintf('GM: %.2f [dB] at %.2f [Hz]\n',mag2db(Gm),Wcg/2/pi)
fprintf('PM: %.2f [deg] at %.2f [Hz]\n',Pm,Wcp/2/pi)
fprintf('----------------\n')

fprintf('PERFORMANCE SCORE\n')
fprintf('Disturbance response: %.4f (smaller better)\n',norm(y,2))
fprintf('----------------\n')
