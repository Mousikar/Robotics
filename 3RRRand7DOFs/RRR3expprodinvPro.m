function [theta,phi,xi] = RRR3expprodinvPro(T,err)
% 指数积反解解法Exponential product
% 输入：末端位姿T
% 输出：三条支链的三个关节角theta（3×1）,phi（3×1）,xi（3×1）
% 三个转动关节向量uvw
%% 初始化
theta=zeros(3,2);
phi=zeros(3,2);
xi=zeros(3,2);
%% 结构参数
    if nargin==1
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
%% 计算反解
for i=1:3
    u_i=u(:,i);
    v_i=v(:,i);
    w_i=w(:,i);
    %% 求向量w末端C点，C点在末端坐标系y轴上
    pc=w_i;
    %% pc为子问题2的p，T*pc是子问题2的q
    p=pc;
    q=T*pc;
    for j=1:2
        [theta(i,j),phi(i,j)] = subproblem2(p,q,u_i,v_i,[0 0 0].',j*2-3);
        %% 最后一个关节角为子问题1
        exp3=expprod(-v_i,phi(i,j))*expprod(-u_i,theta(i,j))*T;
        p1=[0 0 1].'+w_i;
        q1=exp3*p1;
        xi(i,j)=subproblem1(p1,q1,w_i);
    end
end
end

