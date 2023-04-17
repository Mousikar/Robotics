function phi = RRR3forward_theta123fail(theta_1,theta_2,theta_3)
% 3RRR并联机器人正解,封闭解
% %% 结构参数
[gamma,beta,alpha_1,alpha_2,eta,~,~,~] = paraconfig();
%% 
syms phi_1 phi_3
A11=(sin(alpha_1)*(cos(eta(1))*cos(theta_1) +cos(gamma)*sin(eta(1))*sin(theta_1)) + cos(alpha_1)*sin(eta(1))*sin(gamma));
A12=(sin(alpha_1)*(cos(theta_1)*sin(eta(1)) -cos(eta(1))*cos(gamma)*sin(theta_1)) - cos(alpha_1)*cos(eta(1))*sin(gamma));
A13=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_1));
UA=(sin(beta)*(-sin(eta(1))*( - cos(phi_1)*cos(phi_3)) + cos(eta(1))*(cos(phi_1)*sin(phi_3))) )*A11 ...
    - (sin(beta)*(cos(eta(1))*( - sin(phi_1)*sin(phi_3)) - sin(eta(1))*(cos(phi_3)*sin(phi_1))) )*A12 ...
    - A13*(cos(beta));
VA=(- cos(beta)*cos(phi_1))*A11 ...
    - (cos(beta)*sin(phi_1))*A12 ...
    - A13*(cos(eta(1))*sin(beta)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta(1)));
WA=(sin(beta)*(-sin(eta(1))*(sin(phi_1)*sin(phi_3) ) + cos(eta(1))*(cos(phi_3)*sin(phi_1) )))*A11 ...
    - (sin(beta)*(cos(eta(1))*(cos(phi_1)*cos(phi_3) ) - sin(eta(1))*(cos(phi_1)*sin(phi_3) )) )*A12 ...
    -cos(alpha_2);
phi2=[atan((VA+sqrt(VA^2+UA^2-WA^2))/(WA+UA));atan((VA-sqrt(VA^2+UA^2-WA^2))/(WA+UA))];

A21=(sin(alpha_1)*(cos(eta(2))*cos(theta_2) + cos(gamma)*sin(eta(2))*sin(theta_2)) + cos(alpha_1)*sin(eta(2))*sin(gamma));
A22=(sin(alpha_1)*(cos(theta_2)*sin(eta(2)) -cos(eta(2))*cos(gamma)*sin(theta_2)) - cos(alpha_1)*cos(eta(2))*sin(gamma));
A23=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_2));

UB=(sin(beta)*(-sin(eta(2))*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi)*cos(phi_3)) + cos(eta(2))*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi)*sin(phi_3))) - cos(beta)*cos(phi_1)*sin(phi))*A21 ...
    - (sin(beta)*(cos(eta(2))*(cos(phi_1)*cos(phi_3) - cos(phi)*sin(phi_1)*sin(phi_3)) - sin(eta(2))*(cos(phi_1)*sin(phi_3) + cos(phi)*cos(phi_3)*sin(phi_1))) + cos(beta)*sin(phi_1)*sin(phi))*A22 ...
    - A23*(cos(beta)*cos(phi) + cos(eta(2))*sin(beta)*sin(phi)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta(2))*sin(phi))-cos(alpha_2);
VB=
WB=

phi_3=

A31=(sin(alpha_1)*(cos(eta(3))*cos(theta_3) + cos(gamma)*sin(eta(3))*sin(theta_3)) + cos(alpha_1)*sin(eta(3))*sin(gamma));
A32=(sin(alpha_1)*(cos(theta_3)*sin(eta(3)) -cos(eta(3))*cos(gamma)*sin(theta_3)) - cos(alpha_1)*cos(eta(3))*sin(gamma));
A33=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_3));

UC=(sin(beta)*(-sin(eta(3))*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi)*cos(phi_3)) + cos(eta(3))*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi)*sin(phi_3))) - cos(beta)*cos(phi_1)*sin(phi))*A31 ...
    - (sin(beta)*(cos(eta(3))*(cos(phi_1)*cos(phi_3) - cos(phi)*sin(phi_1)*sin(phi_3)) - sin(eta(3))*(cos(phi_1)*sin(phi_3) + cos(phi)*cos(phi_3)*sin(phi_1))) + cos(beta)*sin(phi_1)*sin(phi))*A32 ...
    - A33*(cos(beta)*cos(phi) + cos(eta(3))*sin(beta)*sin(phi)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta(3))*sin(phi))-cos(alpha_2);

VC=
WC=

phi1=

phi1=eval(phi1);
phi2=eval(phi2);
phi3=eval(phi3);
phi=[phi1,phi2,phi3];%R2*3
end

