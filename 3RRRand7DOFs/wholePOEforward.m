function T = wholePOEforward(q,err)
%wholePOEforward 整体机器人指数积正解
%   输入：7个关节角
%   输出：末端执行器位姿
%% 参数
%     q=[0 0 0 0 0 0 0].';
%     q=[0 0 0 pi/2 0 0 0].';
    if nargin==1
        err=[0 0 0 0 0 0 0 0 0 0 0 0].';
    end
[gamma,beta,alpha_1,~,eta,a_3,a_4,d_7] = paraconfig(err);
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
%% 各个关节轴线的线矢量
r=[
    0   0   0   0       0           0           0
    0   0   0   0       0           0           0
    0   0   0   a_3     a_3+a_4     a_3+a_4     a_3+a_4
    ];
mu=[0 1 0].';
L=zeros(6,7);
L(:,1)=[cross(r(:,1),u_i); u_i];
L(:,2)=[cross(r(:,2),v_i); v_i];
L(:,3)=[cross(r(:,3),w_i); w_i];
L(:,4)=[cross(r(:,4),mu);  mu];
L(:,5)=[cross(r(:,5),u_i); u_i];
L(:,6)=[cross(r(:,6),v_i); v_i];
L(:,7)=[cross(r(:,7),w_i); w_i];

%% 初始位姿
M=[
    1 0 0 0
    0 1 0 0 
    0 0 1 a_3+a_4+d_7
    0 0 0 1
    ];
%% 计算各个矩阵指数
expLtheta=cell(7,1);
T71=eye(4);
for i=7:-1:1
    expLtheta{i} = POE(L(:,i),q(i));
    T71=expLtheta{i}*T71;
end
T=T71*M;
end

