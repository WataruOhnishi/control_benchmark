function sys_c = func_plant_stage1(l) % l: height of the measurement point

% Fujimoto, H., Sakata, K., & Saiki, K. (2010). Application of Perfect Tracking Control to Large-Scale High-Precision Stage. In 5th IFAC Symposium on Mechatronic Systems (pp. 188–193).
% Hara, A., Saiki, K., Sakata, K., & Fujimoto, H. (2008). Basic examination on simultaneous optimization of mechanism and control for high precision single axis stage and experimental verification. In 34th Annual Conference of IEEE Industrial Electronics (pp. 2509–2514).

syms s_syms
syms M m J mu_theta C k_theta 
syms L g l_sym

a4 = M*m*L^2 + M*J + m*J;
a3 = M*mu_theta + m*mu_theta + (m*L^2 + J)*C;
a2 = M*k_theta + m*k_theta - M*m*g*L - m^2*g*L + mu_theta*C;
a1 = (k_theta - m*g*L)*C;
b2 = m*L^2 + J - m*L*l_sym;
b1 = mu_theta;
b0 = k_theta - m*g*L;

Pn = (b2*s_syms^2 + b1*s_syms + b0)/(a4*s_syms^4 + a3*s_syms^3 + a2*s_syms^2 + a1*s_syms);
Pn_subs = subs(Pn,{M,m,J,C,k_theta,mu_theta,L,g},...
    {7.7,5.3,0.015,24,1700,0.20,0.092,9.8});
% Dr. Sakata thesis p.107
Pn_subs = subs(Pn_subs,l_sym,l); 

Pn_c = sym2tf(Pn_subs);

sys_c = Pn_c;

