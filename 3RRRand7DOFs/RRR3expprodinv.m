function [theta,phi,xi] = RRR3expprodinv(T,err)
% 指数积反解解法Exponential product
% 输入：末端位姿T
% 输出：一条支链的三个关节角theta,phi,xi
% 三个转动关节向量uvw
%% 结构参数和初值
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
u_i=u(:,1);
v_i=v(:,1);
w_i=w(:,1);
%% 求向量w末端C点，C点在末端坐标系y轴上
pc=w_i;
%% pc为子问题2的p，T*pc是子问题2的q
p=pc;
q=T*pc;
[theta,phi] = subproblem2(p,q,u_i,v_i);
exp3=expprod(-v_i,phi)*expprod(-u_i,theta)*T;
p1=[0 0 1].'+w_i;
q1=exp3*p1;
xi=subproblem1(p1,q1,w_i);
end

