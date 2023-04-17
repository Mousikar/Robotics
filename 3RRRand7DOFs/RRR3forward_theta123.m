function phi = RRR3forward_theta123(theta)
% 3RRR并联机器人正解
global theta_1
theta_1=theta(1);
global theta_2
theta_2=theta(2);
global theta_3
theta_3=theta(3);
% %% 结构参数
% gamma=50/180*pi;
% beta=50/180*pi;
% alpha_1=pi/2;
% alpha_2=pi/2;
% eta=[0 2/3*pi 4/3*pi].';
%% 
% syms phi_1 phi_2 phi_3
% A11=(sin(alpha_1)*(cos(eta(1))*cos(theta_1) +cos(gamma)*sin(eta(1))*sin(theta_1)) + cos(alpha_1)*sin(eta(1))*sin(gamma));
% A12=(sin(alpha_1)*(cos(theta_1)*sin(eta(1)) -cos(eta(1))*cos(gamma)*sin(theta_1)) - cos(alpha_1)*cos(eta(1))*sin(gamma));
% A13=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_1));
% A=(sin(beta)*(-sin(eta(1))*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta(1))*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta)*cos(phi_1)*sin(phi_2))*A11 - (sin(beta)*(cos(eta(1))*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta(1))*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta)*sin(phi_1)*sin(phi_2))*A12 - A13*(cos(beta)*cos(phi_2) + cos(eta(1))*sin(beta)*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta(1))*sin(phi_2))-cos(alpha_2);
% A21=(sin(alpha_1)*(cos(eta(2))*cos(theta_2) + cos(gamma)*sin(eta(2))*sin(theta_2)) + cos(alpha_1)*sin(eta(2))*sin(gamma));
% A22=(sin(alpha_1)*(cos(theta_2)*sin(eta(2)) -cos(eta(2))*cos(gamma)*sin(theta_2)) - cos(alpha_1)*cos(eta(2))*sin(gamma));
% A23=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_2));
% B=(sin(beta)*(-sin(eta(2))*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta(2))*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta)*cos(phi_1)*sin(phi_2))*A21 - (sin(beta)*(cos(eta(2))*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta(2))*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta)*sin(phi_1)*sin(phi_2))*A22 - A23*(cos(beta)*cos(phi_2) + cos(eta(2))*sin(beta)*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta(2))*sin(phi_2))-cos(alpha_2);
% A31=(sin(alpha_1)*(cos(eta(3))*cos(theta_3) + cos(gamma)*sin(eta(3))*sin(theta_3)) + cos(alpha_1)*sin(eta(3))*sin(gamma));
% A32=(sin(alpha_1)*(cos(theta_3)*sin(eta(3)) -cos(eta(3))*cos(gamma)*sin(theta_3)) - cos(alpha_1)*cos(eta(3))*sin(gamma));
% A33=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_3));
% C=(sin(beta)*(-sin(eta(3))*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta(3))*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta)*cos(phi_1)*sin(phi_2))*A31 - (sin(beta)*(cos(eta(3))*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta(3))*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta)*sin(phi_1)*sin(phi_2))*A32 - A33*(cos(beta)*cos(phi_2) + cos(eta(3))*sin(beta)*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta(3))*sin(phi_2))-cos(alpha_2);
% 
% [phi1,phi2,phi3]=solve(A,B,C,phi_1,phi_2,phi_3);
phi=fsolve(@RRR3theta3myfun,[0 0 0].');
end

