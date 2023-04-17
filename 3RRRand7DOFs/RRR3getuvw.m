function [u,v,w] = RRR3getuvw(Q,err)
%% RRR3getuvw 3RRR并联机器人反解程序加获取uvw向量
% 输入：
% Q：动平台位姿
% 输出：u,v,w
% theta：定平台关节角
%% 结构参数
    if nargin==1
        err=[0 0 0 0 0 0 0 0 0 0 0 0].';
    end
[gamma,beta,alpha_1,alpha_2,eta,~,~,~] = paraconfig(err);
%% 反解程序
% w向量
% 当移动平台MP到达初始位置时，
% w_i：先绕p轴转(pi/2+eta_i),再绕自身y轴转beta
% 再绕自身z轴转(-pi/2)，到达基座旋转关节初始位置
% eta_i=(i-1)*2/3*pi,i=1,2,3
Q_w=cell(1,3);
w=ones(3);
w_0=[0 0 1].';
p_0=[0 0 1].';
p=Q*p_0;
theta1=ones(2,3);
theta=ones(3,1);
for i=1:3
    % w向量
    % 当移动平台MP到达初始位置时，
    % w_i：先绕p轴转(pi/2+eta_i),再绕自身y轴转beta
    % 再绕自身z轴转(-pi/2)，到达基座旋转关节初始位置
    % eta_i=(i-1)*2/3*pi,i=1,2,3
    Q_w{i}=Q*rotz(pi/2+eta(i))*roty(beta(i))*floor(rotz(-pi/2));
    w(:,i)=Q_w{i}*w_0;
    % 反解theta
    % 求U,V,W
    w_x=w(1,i);
    w_y=w(2,i);
    w_z=w(3,i);
    U=-sin(alpha_1(i))*sin(eta(i))*w_y-sin(alpha_1(i))*cos(eta(i))*w_x;
    V=sin(alpha_1(i))*cos(gamma(i))*cos(eta(i))*w_y-sin(alpha_1(i))*cos(gamma(i))*sin(eta(i))*w_x+sin(alpha_1(i))*sin(gamma(i))*w_z;
    W=w_y*cos(alpha_1(i))*sin(gamma(i))*cos(eta(i))-w_x*cos(alpha_1(i))*sin(eta(i))*sin(gamma(i))-w_z*cos(alpha_1(i))*cos(gamma(i))-cos(alpha_2(i));
    % 求A,B,C
    A=W-U;
    B=2*V;
    C=W+U;
    sol=[-B+sqrt(B^2-4*A*C);-B-sqrt(B^2-4*A*C)];
    theta1(:,i)=2*atan2(sol,2*A);
    theta(i)=theta1(1,i);
end
%% u向量
% u_i：先绕z轴转(pi/2+eta),再绕自身y轴转(pi-gamma(i))
% 再绕自身z轴转(pi/2)，到达基座旋转关节初始位置
% eta=(i-1)*2/3*pi,i=1,2,3
Q_u=cell(1,3);
u=ones(3);
for i=1:3
    Q_u{i}=rotz(pi/2+eta(i))*roty(pi-gamma(i))*floor(rotz(pi/2));
    u_0=[0 0 1].';
    u(:,i)=Q_u{i}*u_0;
end
%% v向量：中间旋转接头
% 基座旋转初始关节u_i绕自身z轴旋转theta,再绕y轴旋转结构参数alpha_1(i),从而得到v_i
% v_i:u_i绕基座关节坐标系z轴旋转theta,再绕自身坐标系y轴转alpha_1(i)
% Q_v=Q_u*rotz(theta+pi/2)*roty(alpha_1);%为什么要加一个pi/2？？？
Q_v=cell(1,3);
v=ones(3);
for i=1:3
    Q_v{i}=Q_u{i}*rotz(theta(i))*roty(alpha_1(i));
    v_0=[0 0 1].';
    v(:,i)=Q_v{i}*v_0;
end
end

