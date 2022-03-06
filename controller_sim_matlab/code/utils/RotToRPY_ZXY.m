function [phi,theta,psi] = RotToRPY_ZXY(R)

% 
% R = [ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), 
%     cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta)]
% 
% [                                 -cos(phi)*sin(psi),
%     cos(phi)*cos(psi),             sin(phi)]
% 
% [ cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), 
%     sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),  cos(phi)*cos(theta)]

phi = asin(R(2,3));
%phi = asin(R(3,2));
psi = atan2(-R(2,1)/cos(phi),R(2,2)/cos(phi));
%psi = atan2(-R(1,2)/cos(phi),R(2,2)/cos(phi));
theta = atan2(-R(1,3)/cos(phi),R(3,3)/cos(phi));
