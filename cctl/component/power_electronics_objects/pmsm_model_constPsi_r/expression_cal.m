clc;
clear;
close all

syms Rs Ld Lq id iq pd_dd pd_dq pd_qq pd_qd

I = [id; iq];
R = diag([Rs, Rs]);
L = diag([Ld, Lq]);
L_cross = [id*pd_dd, id*pd_dq; ...
        iq*pd_qd, iq*pd_qq];

A = L+L_cross;
invA = inv(A);
% unwrapped invA
% [(Lq + iq*pd_qq)/(Ld*Lq + Ld*iq*pd_qq + Lq*id*pd_dd + id*iq*pd_dd*pd_qq - id*iq*pd_dq*pd_qd),     -(id*pd_dq)/(Ld*Lq + Ld*iq*pd_qq + Lq*id*pd_dd + id*iq*pd_dd*pd_qq - id*iq*pd_dq*pd_qd)]
% [    -(iq*pd_qd)/(Ld*Lq + Ld*iq*pd_qq + Lq*id*pd_dd + id*iq*pd_dd*pd_qq - id*iq*pd_dq*pd_qd), (Ld + id*pd_dd)/(Ld*Lq + Ld*iq*pd_qq + Lq*id*pd_dd + id*iq*pd_dd*pd_qq - id*iq*pd_dq*pd_qd)]

denominator = (Ld*Lq + Ld*iq*pd_qq + Lq*id*pd_dd + id*iq*pd_dd*pd_qq - id*iq*pd_dq*pd_qd);
numerator = invA .* denominator;
% explicit invA = nuemerator ./ denominator
%
% [Lq + iq*pd_qq,     -id*pd_dq]
% [    -iq*pd_qd, Ld + id*pd_dd]
% -------------------------------
% (Ld*Lq + Ld*iq*pd_qq + Lq*id*pd_dd + id*iq*pd_dd*pd_qq - id*iq*pd_dq*pd_qd)
% dLd / did * dLq / diq  - dLd / diq * dLq / did
% in in fact, according to the exact form of the last 2 terms, if we have
% pdLd/pdid * pdLq/pdiq approximate to pdLd/pdiq * pdLq/pdid, we will have
%
% [Lq + iq*pd_qq,     -id*pd_dq]
% [    -iq*pd_qd, Ld + id*pd_dd]
% -------------------------------
% Ld*Lq + Ld*iq*pd_qq + Lq*id*pd_dd
%
% which means current decoupling from the perpendicular inductance  
syms Ud Uq Psir omega kd kq pd_Ld_theta pd_Lq_theta
Cd = id*Rs + 0 + id*omega*pd_Ld_theta + kd*omega*Lq*iq;
Cq = iq*Rs     + iq*omega*pd_Lq_theta + kq*omega*(Psir + Ld*id);
K = [0, kd; kq, 0];
Ur = R * I;
Psi = L * I + [Psir; 0];
Er = omega .* K * Psi; 
Et = omega .* diag([pd_Ld_theta, pd_Lq_theta]) * I;
H = [Ud; Uq] - (Ur + Et + Er);

pIdq = A \ H;

% pIdq =
% (id*pd_dq*(-Hq))/(denominator) - ((Lq + iq*pd_qq)*(-Hd))/(denominator)
% (iq*pd_qd*(-Hd))/(denominator) - ((Ld + id*pd_dd)*(-Hq))/(denominator)
% Ld*Ud - Ld*Rs*id + Ud*id*pd_dd + Uq*id*pd_dq - Rs*id^2*pd_dd - id^2*omega*pd_dd*pd_Ld_theta - Ld*id*omega*pd_Ld_theta - Rs*id*iq*pd_dq - Ld*Lq*iq*kd*omega - Psir*id*kq*omega*pd_dq - id*iq*omega*pd_dq*pd_Lq_theta - Ld*id^2*kq*omega*pd_dq - Lq*id*iq*kd*omega*pd_dd
% Lq*Uq - Lq*Rs*iq + Ud*iq*pd_qd + Uq*iq*pd_qq - Rs*iq^2*pd_qq - iq^2*omega*pd_qq*pd_Lq_theta - Lq*Psir*kq*omega - Rs*id*iq*pd_qd - Lq*iq*omega*pd_Lq_theta - Ld*Lq*id*kq*omega - Psir*iq*kq*omega*pd_qq - id*iq*omega*pd_qd*pd_Ld_theta - Lq*iq^2*kd*omega*pd_qd - Ld*id*iq*kq*omega*pd_qq
