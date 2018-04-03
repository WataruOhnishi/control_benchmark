function Pc = func_plant_stage1(l) % l: height of the measurement point
% Fujimoto, H., Sakata, K., & Saiki, K. (2010). Application of Perfect Tracking Control to Large-Scale High-Precision Stage. In 5th IFAC Symposium on Mechatronic Systems (pp. 188–193).
% Hara, A., Saiki, K., Sakata, K., & Fujimoto, H. (2008). Basic examination on simultaneous optimization of mechanism and control for high precision single axis stage and experimental verification. In 34th Annual Conference of IEEE Industrial Electronics (pp. 2509–2514).

s = tf('s');
M = 7.7;
m = 5.3;
J = 0.015;
C = 24; 
k_theta = 1700;
mu_theta = 0.20;
L = 0.092;
g = 9.8;

a4 = M*m*L^2 + M*J + m*J;
a3 = M*mu_theta + m*mu_theta + (m*L^2 + J)*C;
a2 = M*k_theta + m*k_theta - M*m*g*L - m^2*g*L + mu_theta*C;
a1 = (k_theta - m*g*L)*C;
b2 = m*L^2 + J - m*L*l;
b1 = mu_theta;
b0 = k_theta - m*g*L;

Pc = (b2*s^2 + b1*s + b0)/(a4*s^4 + a3*s^3 + a2*s^2 + a1*s);

end
