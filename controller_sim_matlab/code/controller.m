function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M

%% define Gain
gain_pos = [1, 1, 1];
gain_vel = [1, 1, 5];
gain_angle = [1, 1, 1];     % phi theta psi
gain_omiga = [1, 1, 1];

%% calculate F
a_out = zeros(3);
for i = 1 : 3
    a_out(i) = gain_vel(i) * (s_des(i+3)-s(i+3)) + gain_pos(i) * (s_des(i)-s(i));
end

F = m * (g + a_out(3));

%% calculate M
Rot = QuatToRot([s(7), s(8), s(9), s(10)]');
[phi, theta, psi] = RotToRPY_ZXY(Rot);  % true angle

phi_des = 1 / g * (a_out(1)*sin(psi) - a_out(2)*cos(psi));
theta_des = 1 / g * (a_out(1)*cos(psi) + a_out(2)*sin(psi));
psi_des = psi;
wx_dot = gain_angle(1) * (phi_des - phi) + gain_omiga(1) * (0 - s(11));
wy_dot = gain_angle(2) * (theta_des - theta) + gain_omiga(2) * (0 - s(12));
wz_dot = gain_angle(3) * (psi_des - psi) + gain_omiga(3) * (0 - s(13));

M(1) = I(1, 1) * wx_dot + s(12) * s(13) * (I(3, 3) - I(2, 2));
M(2) = I(2, 2) * wy_dot + s(13) * s(11) * (I(1, 1) - I(3, 3));
M(3) = I(3, 3) * wz_dot + s(11) * s(12) * (I(2, 2) - I(1, 1));

%M = [0.0, 0.0, 0.0]';

disp("des x: " + s_des(1));
disp("des y: " + s_des(2));
disp("des z: " + s_des(3));
disp("true x: " + s(1));
disp("true y: " + s(2));
disp("true z: " + s(3));
% Rot_des = QuatToRot([s_des(7), s_des(8), s_des(9), s_des(10)]');
% [phi_des, theta_des, yaw_des] = RotToRPY_ZXY(Rot_des);



end
