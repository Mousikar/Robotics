function [T,p,u,v,w] = RRR3expprod(theta,phi,xi,err)
% 指数积正解解法Exponential product
% 输入：一条支链的三个关节角theta,phi,xi
% 输出：末端向量p
% 三个转动关节向量uvw
%% 结构参数和初值
% u_i =[
%     0.0000
%     0.7660
%    -0.6428];
% v_i =[
%    -1.0000
%     0.0000
%    -0.0000];
% w_i =[
%    -0.0000
%     0.7660
%     0.6428];
%% 结构参数改变，gamma=60°，beta=45°
% u_i =[
%     0.0000
%     0.8660
%    -0.5000];
% v_i =[
%    -1.0000
%     0.0000
%    -0.0000];
% w_i =[
%    -0.0000
%     0.7071
%     0.7071];
%% 结构参数
    if nargin==3
        err=[0 0 0 0 0 0 0 0 0 0 0 0].';
    end
[gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig(err);
theta1=[0 0 0].';
%% u向量
Q_u=cell(1,3);
u=ones(3);
for i=1:3
    Q_u{i}=rotz(pi/2+eta(i))*roty(pi-gamma(i))*floor(rotz(pi/2));
    u_0=[0 0 1].';
    u(:,i)=Q_u{i}*u_0;
end
%% v向量：中间旋转接头
Q_v=cell(1,3);
v=ones(3);
for i=1:3
    Q_v{i}=Q_u{i}*rotz(theta1(i))*roty(alpha_1(i));
    v_0=[0 0 1].';
    v(:,i)=Q_v{i}*v_0;
end
%% w向量
Q_w=cell(1,3);
w=ones(3);
for i=1:3
    Q_w{i}=rotz(pi/2+eta(i))*roty(beta(i))*floor(rotz(-pi/2));
    w_0=[0 0 1].';
    w(:,i)=Q_w{i}*w_0;
end
u_i=u(:,1);
v_i=v(:,1);
w_i=w(:,1);
p_0=[0;0;1];
M=[
    1 0 0
    0 1 0
    0 0 1
    ];
%% 求末端向量p
p=expprod(u_i,theta)*expprod(v_i,phi)*expprod(w_i,xi)*p_0;
%% 求末端位姿T
T=expprod(u_i,theta)*expprod(v_i,phi)*expprod(w_i,xi)*M;
%% 求三个转动关节向量uvw
u=u_i;
v=expprod(u_i,theta)*v_i;
w=expprod(u_i,theta)*expprod(v_i,phi)*w_i;
end

