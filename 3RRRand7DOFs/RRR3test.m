%% % 测试反解程序的正确性
a=linspace(0,pi/9,100);
b=linspace(0,pi/6,100);
c=linspace(0,pi/11,100);
theta_history=zeros(3,100);
for i=1:100
    phi=[a(i),b(i),c(i)];
    [theta,psi] = RRR3inverse(phi);
    theta_history(:,i)=theta;
    pause(0.1);
    clf;
end
%% 求解正解草稿
% syms eta_i gamma alpha_1 theta_i phi_1 phi_2 phi_3 beta
% Q_u=rotz(pi/2+eta_i)*roty(pi-gamma)*floor(rotz(pi/2));
% Q_u=simplify(Q_u);
% u_0=[0 0 1].';
% u_i=Q_u*u_0;
% u_i=simplify(u_i);
% Q_v=Q_u*rotz(theta_i)*roty(alpha_1);
% Q_v=simplify(Q_v);
% v_0=[0 0 1].';
% v_i=Q_v*v_0;
% v_i=simplify(v_i);
% Q=rotz(phi_1)*roty(phi_2)*rotz(phi_3);
% w_0=[0 0 1].';
% Q_w=Q*rotz(pi/2+eta_i)*roty(beta)*floor(rotz(-pi/2));
% w_i=Q_w*w_0;
% w_i=simplify(w_i);
% COSalpha2=v_i.'*w_i;
% simplify(COSalpha2)
% collect(COSalpha2)
% 
% syms eta_1 theta_1 alpha_2
% syms eta_2 theta_2
% syms eta_3 theta_3
% syms A11 A12 A13
% syms A21 A22 A23
% syms A31 A32 A33
% % A11=(sin(alpha_1)*(cos(eta_1)*cos(theta_1) +
% % cos(gamma)*sin(eta_1)*sin(theta_1)) + cos(alpha_1)*sin(eta_1)*sin(gamma));
% % A12=(sin(alpha_1)*(cos(theta_1)*sin(eta_1) -
% % cos(eta_1)*cos(gamma)*sin(theta_1)) - cos(alpha_1)*cos(eta_1)*sin(gamma));
% % A13=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_1));
% A=(sin(beta)*(-sin(eta_1)*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta_1)*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta)*cos(phi_1)*sin(phi_2))*A11 - (sin(beta)*(cos(eta_1)*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta_1)*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta)*sin(phi_1)*sin(phi_2))*A12 - A13*(cos(beta)*cos(phi_2) + cos(eta_1)*sin(beta)*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta_1)*sin(phi_2))-cos(alpha_2);
% % A21=(sin(alpha_1)*(cos(eta_2)*cos(theta_2) + cos(gamma)*sin(eta_2)*sin(theta_2)) + cos(alpha_1)*sin(eta_2)*sin(gamma))
% % A22=(sin(alpha_1)*(cos(theta_2)*sin(eta_2) -
% % cos(eta_2)*cos(gamma)*sin(theta_2)) - cos(alpha_1)*cos(eta_2)*sin(gamma));
% % A23=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_2));
% B=(sin(beta)*(-sin(eta_2)*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta_2)*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta)*cos(phi_1)*sin(phi_2))*A21 - (sin(beta)*(cos(eta_2)*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta_2)*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta)*sin(phi_1)*sin(phi_2))*A22 - A23*(cos(beta)*cos(phi_2) + cos(eta_2)*sin(beta)*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta_2)*sin(phi_2))-cos(alpha_2);
% % A31=(sin(alpha_1)*(cos(eta_3)*cos(theta_3) + cos(gamma)*sin(eta_3)*sin(theta_3)) + cos(alpha_1)*sin(eta_3)*sin(gamma))
% % A32=(sin(alpha_1)*(cos(theta_3)*sin(eta_3) -
% % cos(eta_3)*cos(gamma)*sin(theta_3)) - cos(alpha_1)*cos(eta_3)*sin(gamma));
% % A33=(cos(alpha_1)*cos(gamma) - sin(alpha_1)*sin(gamma)*sin(theta_3));
% C=(sin(beta)*(-sin(eta_3)*(sin(phi_1)*sin(phi_3) - cos(phi_1)*cos(phi_2)*cos(phi_3)) + cos(eta_3)*(cos(phi_3)*sin(phi_1) + cos(phi_1)*cos(phi_2)*sin(phi_3))) - cos(beta)*cos(phi_1)*sin(phi_2))*A31 - (sin(beta)*(cos(eta_3)*(cos(phi_1)*cos(phi_3) - cos(phi_2)*sin(phi_1)*sin(phi_3)) - sin(eta_3)*(cos(phi_1)*sin(phi_3) + cos(phi_2)*cos(phi_3)*sin(phi_1))) + cos(beta)*sin(phi_1)*sin(phi_2))*A32 - A33*(cos(beta)*cos(phi_2) + cos(eta_3)*sin(beta)*sin(phi_2)*sin(phi_3) + cos(phi_3)*sin(beta)*sin(eta_3)*sin(phi_2))-cos(alpha_2);
% 
% [phi_1,phi_2,phi_3]=solve(A,B,C,phi_1,phi_2,phi_3)
% 
% 
% % syms x y
% % A=2*x+y-9;
% % B=5*x+9*y-89;
% % [x,y]=solve(A,B,x,y)

%% 验证正解程序
clc
theta_1=pi/6;
theta_2=pi/7;
theta_3=pi/11;
phi = RRR3forward_theta123(theta_1,theta_2,theta_3)
[theta1,psi] = RRR3inverse(phi)
theta_1,theta_2,theta_3
%% 先得到反解，再得到正解
clc
phi=[
pi/21;
pi/11;
pi/11;];
R=[    
    0.8227    0.5552    0.1224
   -0.5676    0.8140    0.1230
   -0.0314   -0.1707    0.9848];%rotz(phi(1))*roty(phi(2))*rotz(phi(3))
[theta,psi] = RRR3inversePro(R)
phi = RRR3forward_theta123(theta(1),theta(2),theta(3))
Q=rotz(phi(1))*roty(phi(2))*rotz(phi(3))
%% 验证正解
clear
global theta_1;
global theta_2;
global theta_3;
a=linspace(0,pi/9,100);
b=linspace(0,pi/6,100);
c=linspace(0,pi/11,100);
for i=1:100
    theta_1=a(i);
    theta_2=b(i);
    theta_3=c(i);
    phi=RRR3forward_theta123(theta_1,theta_2,theta_3);
    [theta,psi] = RRR3inverse(phi);    
    if theta(1)-theta_1<=0.01
        disp('right')
    else
        disp('wrong')
    end
    pause(0.2);
    clf;
end
%% 验证指数积正解
clear
a=linspace(0,pi/9,100);
b=linspace(0,pi/6,100);
c=linspace(0,pi/11,100);
MM=cell(100,1);
for i=1:100
    theta=a(i);
    phi=b(i);
    xi=c(i);
    [M,p,u,v,w] = RRR3expprodPro(theta,phi,xi);
    MM{i}=M;
    [alpha,beta,gamma]=r2zyz(M);
    modr=[alpha,beta,gamma].';
    logi=M-rotz(modr(1))*roty(modr(2))*rotz(modr(3))<=0.0001;
    if sum(sum(logi))~=9
        disp('wrong')
    end
    [theta,psi] = RRR3inverse(modr);    
    pause(0.2);
    clf;
end
%% 验证指数积反解
clear
a=linspace(0,pi/9,100);
b=linspace(0,pi/6,100);
c=linspace(0,pi/11,100);

theta_history=zeros(3,100);
psi_history=zeros(3,100);
M_hist=cell(100,1);
for i=1:100
    phi=[a(i),b(i),c(i)];    
    T=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
    [theta,phi,xi] = RRR3expprodinvPro(T);
    theta_history(:,i)=theta;
    psi_history(:,i)=phi;
    
    [M,p,u,v,w] = RRR3expprod(theta(1),phi(1),xi(1));
    M_hist{i}=M;
    [alpha,beta,gamma]=r2zyz(M);
    modr=[alpha,beta,gamma].';
    logi=M-rotz(modr(1))*roty(modr(2))*rotz(modr(3))<=0.0001;
    if sum(sum(logi))~=9
        disp('wrong')
    else
        disp('right')
    end
    [theta,psi] = RRR3inverse(modr);  
    
    pause(0.2);
    clf;
end
%% 验证升级版正解
    global theta
    count=0;
for i=1:100
    theta=theta_history(:,i);
    [R,psi] = RRR3expprodPro(theta);
    if (norm(psi-psi_history(:,i))<=0.001)&&(norm(R-M_hist{i})<=0.001)
        disp('right')
        count=count+1;
    end
end
%% 先正解过去再逆解回来，画出关节角度，角速度，角加速度
