% RRR3errormodel
%% SPM误差模型
clear
syms gamma_1 gamma_2 gamma_3 alpha_11 alpha_12 alpha_13 beta_1 beta_2 beta_3 eta_1 eta_2 eta_3
syms psi_1 psi_2 psi_3 phi_1 phi_2 phi_3 
eta=[eta_1 eta_2 eta_3];
alpha_1=[alpha_11 alpha_12 alpha_13];
beta=[beta_1 beta_2 beta_3];
gamma=[gamma_1 gamma_2 gamma_3];
theta1=[0 0 0].';
%% 求初始向量
% u0向量
Q_u=cell(1,3);
for i=1:3
    Q_u{i}=rotz(pi/2+eta(i))*roty(pi-gamma(i))*floor(rotz(pi/2));
    u_0=[0 0 1].';
    tem=Q_u{i};
    u0(:,i)=tem(:,3);
end
% v0向量：中间旋转接头
Q_v=cell(1,3);
for i=1:3
    Q_v{i}=Q_u{i}*rotz(theta1(i))*roty(alpha_1(i));
    v_0=[0 0 1].';
    tem=Q_v{i};
    v0(:,i)=tem*v_0;
end
% w0向量
Q_w=cell(1,3);
for i=1:3
    Q_w{i}=rotz(pi/2+eta(i))*roty(beta(i))*floor(rotz(-pi/2));
    w_0=[0 0 1].';
    tem=Q_w{i};
    w0(:,i)=tem*w_0;
end
u0=simplify(u0);
v0=simplify(v0);
w0=simplify(w0);
%% 表示闭合方程
% 关节
syms theta_1 theta_2 theta_3
%% ----------------------闭合方程1----------------------
v1=expprod(u0(:,1),theta_1)*v0(:,1);
v1=simplify(v1);
v1=simplify(v1)
Q=rotz(phi_1)*roty(phi_2)*rotz(phi_3);
Q_w1=Q*rotz(pi/2+eta(1))*roty(beta(1))*floor(rotz(-pi/2));
w1=Q_w1(:,3);
w1=simplify(w1);
w1=simplify(w1)
v1Tw1=v1.'*w1;
v1Tw1=simplify(v1Tw1)
%% 求闭合方程1对l的导数
df1dl1=diff(v1Tw1,gamma_1);     df1dl1=simplify(df1dl1)
df1dl2=diff(v1Tw1,gamma_2);     df1dl2=simplify(df1dl2)
df1dl3=diff(v1Tw1,gamma_3);     df1dl3=simplify(df1dl3)
df1dl4=diff(v1Tw1,alpha_11);    df1dl4=simplify(df1dl4)
df1dl5=diff(v1Tw1,alpha_12);    df1dl5=simplify(df1dl5)
df1dl6=diff(v1Tw1,alpha_13);    df1dl6=simplify(df1dl6)
df1dl7=diff(v1Tw1,beta_1);      df1dl7=simplify(df1dl7)
df1dl8=diff(v1Tw1,beta_2);      df1dl8=simplify(df1dl8)
df1dl9=diff(v1Tw1,beta_3);      df1dl9=simplify(df1dl9)
df1dl10=diff(v1Tw1,eta_1);      df1dl10=simplify(df1dl10)
df1dl11=diff(v1Tw1,eta_2);      df1dl11=simplify(df1dl11)
df1dl12=diff(v1Tw1,eta_3);      df1dl12=simplify(df1dl12)
%% f1 x
df1dx1=diff(v1Tw1,phi_1);   df1dx1=simplify(df1dx1)
df1dx2=diff(v1Tw1,phi_2);   df1dx2=simplify(df1dx2)
df1dx3=diff(v1Tw1,phi_3);   df1dx3=simplify(df1dx3)
%% ----------------------闭合方程2----------------------
v2=expprod(u0(:,2),theta_2)*v0(:,2);
v2=simplify(v2);
Q_w2=Q*rotz(pi/2+eta(2))*roty(beta(2))*floor(rotz(-pi/2));
w2=Q_w2(:,3);
w2=simplify(w2);
w2=simplify(w2)
v2Tw2=v2.'*w2;
v2Tw2=simplify(v2Tw2)
%% 求闭合方程2对l的导数
df2dl1=diff(v2Tw2,gamma_1);     df2dl1=simplify(df2dl1)
df2dl2=diff(v2Tw2,gamma_2);      df2dl2=simplify(df2dl2)
df2dl3=diff(v2Tw2,gamma_3);    df2dl3=simplify(df2dl3)
df2dl4=diff(v2Tw2,alpha_11);    df2dl4=simplify(df2dl4)
df2dl5=diff(v2Tw2,alpha_12);    df2dl5=simplify(df2dl5)
df2dl6=diff(v2Tw2,alpha_13);    df2dl6=simplify(df2dl6)
df2dl7=diff(v2Tw2,beta_1);      df2dl7=simplify(df2dl7)
df2dl8=diff(v2Tw2,beta_2);      df2dl8=simplify(df2dl8)
df2dl9=diff(v2Tw2,beta_3);      df2dl9=simplify(df2dl9)
df2dl10=diff(v2Tw2,eta_1);      df2dl10=simplify(df2dl10)
df2dl11=diff(v2Tw2,eta_2);      df2dl11=simplify(df2dl11)
df2dl12=diff(v2Tw2,eta_3);      df2dl12=simplify(df2dl12)
%% f2 x
df2dx1=diff(v2Tw2,phi_1);   df2dx1=simplify(df2dx1)
df2dx2=diff(v2Tw2,phi_2);   df2dx2=simplify(df2dx2)
df2dx3=diff(v2Tw2,phi_3);   df2dx3=simplify(df2dx3)
%% ----------------------闭合方程3----------------------
v3=expprod(u0(:,3),theta_3)*v0(:,3);
v3=simplify(v3);
Q_w3=Q*rotz(pi/2+eta(3))*roty(beta(3))*floor(rotz(-pi/2));
w3=Q_w3(:,3);
w3=simplify(w3);
w3=simplify(w3)
v3Tw3=v3.'*w3;
v3Tw3=simplify(v3Tw3)
%% 求闭合方程3对l的导数
df3dl1=diff(v3Tw3,gamma_1);     df3dl1=simplify(df3dl1)
df3dl2=diff(v3Tw3,gamma_2);     df3dl2=simplify(df3dl2)
df3dl3=diff(v3Tw3,gamma_3);     df3dl3=simplify(df3dl3)
df3dl4=diff(v3Tw3,alpha_11);    df3dl4=simplify(df3dl4)
df3dl5=diff(v3Tw3,alpha_12);    df3dl5=simplify(df3dl5)
df3dl6=diff(v3Tw3,alpha_13);    df3dl6=simplify(df3dl6)
df3dl7=diff(v3Tw3,beta_1);      df3dl7=simplify(df3dl7)
df3dl8=diff(v3Tw3,beta_2);      df3dl8=simplify(df3dl8)
df3dl9=diff(v3Tw3,beta_3);      df3dl9=simplify(df3dl9)
df3dl10=diff(v3Tw3,eta_1);      df3dl10=simplify(df3dl10)
df3dl11=diff(v3Tw3,eta_2);      df3dl11=simplify(df3dl11)
df3dl12=diff(v3Tw3,eta_3);      df3dl12=simplify(df3dl12)
%% f1 x
df3dx1=diff(v3Tw3,phi_1);   df3dx1=simplify(df3dx1)
df3dx2=diff(v3Tw3,phi_2);   df3dx2=simplify(df3dx2)
df3dx3=diff(v3Tw3,phi_3);   df3dx3=simplify(df3dx3)





