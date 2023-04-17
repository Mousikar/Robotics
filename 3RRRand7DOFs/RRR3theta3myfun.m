function F = RRR3theta3myfun(phi,err)
%
global theta_1
global theta_2
global theta_3
% theta_1=1;
% theta_2=2;
% theta_3=3;

%% 结构参数
    if nargin==1
        err=[0 0 0 0 0 0 0 0 0 0 0 0].';
    end
[gamma,beta,alpha_1,alpha_2,eta,~,~,~] = paraconfig(err);
%%
phi_1=phi(1);
phi_2=phi(2);
phi_3=phi(3);
A11=(sin(alpha_1(1))*(cos(eta(1))*cos(theta_1) +cos(gamma(1))*sin(eta(1))*sin(theta_1)) + cos(alpha_1(1))*sin(eta(1))*sin(gamma(1)));
A12=(sin(alpha_1(1))*(cos(theta_1)*sin(eta(1)) -cos(eta(1))*cos(gamma(1))*sin(theta_1)) - cos(alpha_1(1))*cos(eta(1))*sin(gamma(1)));
A13=(cos(alpha_1(1))*cos(gamma(1)) - sin(alpha_1(1))*sin(gamma(1))*sin(theta_1));
A=(sin(beta(1))*(-sin(eta(1))*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta(1))*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta(1))*cos(phi_1)*sin(phi_2))*A11 - (sin(beta(1))*(cos(eta(1))*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta(1))*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta(1))*sin(phi_1)*sin(phi_2))*A12 - A13*(cos(beta(1))*cos(phi_2) + cos(eta(1))*sin(beta(1))*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta(1))*sin(eta(1))*sin(phi_2))-cos(alpha_2(1));
A21=(sin(alpha_1(2))*(cos(eta(2))*cos(theta_2) + cos(gamma(2))*sin(eta(2))*sin(theta_2)) + cos(alpha_1(2))*sin(eta(2))*sin(gamma(2)));
A22=(sin(alpha_1(2))*(cos(theta_2)*sin(eta(2)) -cos(eta(2))*cos(gamma(2))*sin(theta_2)) - cos(alpha_1(2))*cos(eta(2))*sin(gamma(2)));
A23=(cos(alpha_1(2))*cos(gamma(2)) - sin(alpha_1(2))*sin(gamma(2))*sin(theta_2));
B=(sin(beta(2))*(-sin(eta(2))*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta(2))*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta(2))*cos(phi_1)*sin(phi_2))*A21 - (sin(beta(2))*(cos(eta(2))*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta(2))*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta(2))*sin(phi_1)*sin(phi_2))*A22 - A23*(cos(beta(2))*cos(phi_2) + cos(eta(2))*sin(beta(2))*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta(2))*sin(eta(2))*sin(phi_2))-cos(alpha_2(2));
A31=(sin(alpha_1(3))*(cos(eta(3))*cos(theta_3) + cos(gamma(3))*sin(eta(3))*sin(theta_3)) + cos(alpha_1(3))*sin(eta(3))*sin(gamma(3)));
A32=(sin(alpha_1(3))*(cos(theta_3)*sin(eta(3)) -cos(eta(3))*cos(gamma(3))*sin(theta_3)) - cos(alpha_1(3))*cos(eta(3))*sin(gamma(3)));
A33=(cos(alpha_1(3))*cos(gamma(3)) - sin(alpha_1(3))*sin(gamma(3))*sin(theta_3));
C=(sin(beta(3))*(-sin(eta(3))*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta(3))*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta(3))*cos(phi_1)*sin(phi_2))*A31 - (sin(beta(3))*(cos(eta(3))*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta(3))*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta(3))*sin(phi_1)*sin(phi_2))*A32 - A33*(cos(beta(3))*cos(phi_2) + cos(eta(3))*sin(beta(3))*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta(3))*sin(eta(3))*sin(phi_2))-cos(alpha_2(3));

F = [
    A;
    B;
    C;
];

end

