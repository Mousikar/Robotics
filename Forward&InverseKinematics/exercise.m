%% 机器人学代码练习
% 2021年3月8日 创建
% MousikaR

%% 普通求逆和构造求逆的时间比较
% 普通求逆
theta=-pi/2+pi*rand(3,1)
R=rotx(theta(1))*roty(theta(2))*rotz(theta(3))
T=diag([0,0,0,1]);
T(1:3,1:3)=R;
T(1:3,4)=1000*rand(3,1)
tic
Tinv=inv(T)
toc
% 结构求逆
theta=-pi/2+pi*rand(3,1)
R=rotx(theta(1))*roty(theta(2))*rotz(theta(3))
T=diag([0,0,0,1]);
T(1:3,1:3)=R;
T(1:3,4)=1000*rand(3,1)
tic
Tinv=diag([0,0,0,1]);
Tinv(1:3,1:3)=R';
T(1:3,4)=-R'*T(1:3,4);
toc

%% MATLAB符号推理
syms alpha beta gamma
Rz=[
    cos(alpha) -sin(alpha) 0;
    sin(alpha) cos(alpha) 0;
    0 0 1];
Ry=[
    cos(beta) 0 sin(beta);
    0 1 0;
    -sin(beta) 0 cos(beta)];
Rx=[
    1 0 0;
    0 cos(gamma) -sin(gamma);
    0 sin(gamma) cos(gamma)];
Rxyz=Rz*Ry*Rx

%% RPY反解
theta=-pi/2+pi*rand(3,1);
R=rotx(theta(1))*roty(theta(2))*rotz(theta(3))
[a,b,c]=RPYReverse(R)

%% 倾斜圆的坐标表示
% 圆参数方程中的theta,圆的半径R
% 圆心坐标(x_Bo,y_Bo,z_Bo)'
% 符号化
syms theta R x_Bo y_Bo z_Bo a b c

% 圆的法向矢量(a b c)'，坐标系{B}的z轴
z_B=[a b c].';
% 坐标系{B}的x轴
x_B=[0 c -b].';
% 坐标系{B}的y轴
y_B=cross(x_B,z_B)%;
% 变成单位向量
x_B=x_B/norm(x_B)%;
y_B=y_B/norm(y_B)
z_B=z_B/norm(z_B)%;

p_B=[R*cos(theta) R*sin(theta) 0 1].'%;
T_B_A=[
    0 0 0 x_Bo
    0 0 0 y_Bo
    0 0 0 z_Bo
    0 0 0 1
];
% 坐标系{A}的x，y，z轴
x_A=[1 0 0].'%;
y_A=[0 1 0].'%;
z_A=[0 0 1].'%;
% 旋转矩阵
x_B_A=[x_A'*x_B y_A'*x_B z_A'*x_B].'%;
y_B_A=[x_A'*y_B y_A'*y_B z_A'*y_B].'%;
z_B_A=[x_A'*z_B y_A'*z_B z_A'*z_B].'%;
R_B_A=[x_B_A y_B_A z_B_A];
T_B_A(1:3,1:3)=R_B_A%;
p_A=T_B_A*p_B

% 
syms alpha beta
rotx(beta)*roty(alpha)
rotz(alpha)*rotx(beta)

alpha=30;
beta=45;
R_B_A= rotz(alpha,'deg')*rotx(beta,'deg');
P_B=[3 1 5]';
P_A=R_B_A*P_B

syms f1 f2 f3 theta_1 theta_2 alpha_0 alpha_1 d_1 d_2

p_4_0=[f1 f2 f3 1].'

T_2_1=[
    cos(theta_2) -sin(theta_2) 0 alpha_1
    sin(theta_2)*cos(alpha_1) cos(theta_2)*cos(alpha_1) -sin(alpha_1) -d_2*sin(alpha_1)
    sin(theta_2)*sin(alpha_1) cos(theta_2)*sin(alpha_1) cos(alpha_1) d_2*cos(alpha_1)
    0 0 0 1
    ]

T_1_0=[
    cos(theta_1) -sin(theta_1) 0 alpha_0
    sin(theta_1)*cos(alpha_0) cos(theta_1)*cos(alpha_0) -sin(alpha_0) -d_1*sin(alpha_0)
    sin(theta_1)*sin(alpha_0) cos(theta_1)*sin(alpha_0) cos(alpha_0) d_1*cos(alpha_0)
    0 0 0 1
    ]
T_1_0*T_2_1*p_4_0

%% 作业2021年3月25日
% 测试一
syms alpha beta
rotx(beta)*roty(alpha)

alpha=30/180*pi;beta=45/180*pi;
R=rotz(alpha)*rotx(beta)
p=[3;1;5];
R*p

beta=atan2(0.8192,-0.5736)/pi*180
cosd(125)

T=diag([1,1,1,1]);
T(1:3,4)=[-3 -5 0]'
p=[4 2 -1 1]'
T*p
p=[1 -3 -1 1]'
T(1:3,4)=[0 -4 4]'
R=[
    cosd(-45) -sind(-45) 0 0
    sind(-45) cosd(-45) 0 0
    0 0 1 0
    0 0 0 1
]
T*R*p

%% 求解tan(theta_3/2)的一元二次方程
syms r theta_3 a_1 a_2 a_3 alpha_2 alpha_3 d_2 d_3 d_4 u
f1=a_3*cos(theta_3)+d_4*sin(alpha_3)*sin(theta_3)+a_2;
f2=a_3*cos(alpha_2)*sin(theta_3)-d_4*sin(alpha_3)*cos(alpha_2)*cos(theta_3)-d_4*sin(alpha_2)*cos(alpha_3)-d_3*sin(alpha_2);
f3=a_3*sin(alpha_2)*sin(theta_3)-d_4*sin(alpha_3)*sin(alpha_2)*cos(theta_3)+d_4*cos(alpha_2)*cos(alpha_3)+d_3*cos(alpha_2);
ans=f1.^2+f2.^2+f3.^2+a_1.^2+d_2.^2+2*d_2*f3
ans=expand(ans)
ans=simplify(ans)
ans=a_1^2 + a_2^2 + a_3^2 + d_2^2 + d_3^2 + d_4^2 + 2*d_2*d_3*cos(alpha_2) + 2*d_3*d_4*cos(alpha_3) ...
    + 2*a_2*a_3*(1-u^2)/(1+u^2) ...
    + 2*d_2*d_4*cos(alpha_2)*cos(alpha_3) ...
    + 2*a_3*d_2*sin(alpha_2)*(2*u)/(1+u^2) ...
    + 2*a_2*d_4*sin(alpha_3)*(2*u)/(1+u^2) ...
    - 2*d_2*d_4*sin(alpha_2)*sin(alpha_3)*(1-u^2)/(1+u^2)

f=ans-r;

result=solve(f==0,u)

theta_3result=2*atan2(result,1)

r= 200^2+ 100^2+ 100^2;
a_1=0;
a_2=431.8;
a_3=20.32;
alpha_2=0;
alpha_3=-pi/2;
d_2=149.09;
d_3=0;
d_4=433.07;

theta_3 =[-2*atan((2*a_3*d_2*sin(alpha_2) - sqrt(- a_1^4 - 2*a_1^2*a_2^2 - 2*a_1^2*a_3^2 - 2*a_1^2*d_2^2 - 4*a_1^2*d_2*d_3*cos(alpha_2) - 4*a_1^2*d_2*d_4*cos(alpha_2)*cos(alpha_3) - 2*a_1^2*d_3^2 - 4*a_1^2*d_3*d_4*cos(alpha_3) - 2*a_1^2*d_4^2 + 2*a_1^2*r - a_2^4 + 2*a_2^2*a_3^2 - 2*a_2^2*d_2^2 - 4*a_2^2*d_2*d_3*cos(alpha_2) - 4*a_2^2*d_2*d_4*cos(alpha_2)*cos(alpha_3) - 2*a_2^2*d_3^2 - 4*a_2^2*d_3*d_4*cos(alpha_3) + 4*a_2^2*d_4^2*sin(alpha_3)^2 - 2*a_2^2*d_4^2 + 2*a_2^2*r - a_3^4 + 4*a_3^2*d_2^2*sin(alpha_2)^2 - 2*a_3^2*d_2^2 - 4*a_3^2*d_2*d_3*cos(alpha_2) - 4*a_3^2*d_2*d_4*cos(alpha_2)*cos(alpha_3) - 2*a_3^2*d_3^2 - 4*a_3^2*d_3*d_4*cos(alpha_3) - 2*a_3^2*d_4^2 + 2*a_3^2*r - d_2^4 - 4*d_2^3*d_3*cos(alpha_2) - 4*d_2^3*d_4*cos(alpha_2)*cos(alpha_3) - 4*d_2^2*d_3^2*cos(alpha_2)^2 - 2*d_2^2*d_3^2 - 8*d_2^2*d_3*d_4*cos(alpha_2)^2*cos(alpha_3) - 4*d_2^2*d_3*d_4*cos(alpha_3) - 4*d_2^2*d_4^2*cos(alpha_2)^2*cos(alpha_3)^2 + 4*d_2^2*d_4^2*sin(alpha_2)^2*sin(alpha_3)^2 - 2*d_2^2*d_4^2 + 2*d_2^2*r - 4*d_2*d_3^3*cos(alpha_2) - 12*d_2*d_3^2*d_4*cos(alpha_2)*cos(alpha_3) - 8*d_2*d_3*d_4^2*cos(alpha_2)*cos(alpha_3)^2 - 4*d_2*d_3*d_4^2*cos(alpha_2) + 4*d_2*d_3*r*cos(alpha_2) - 4*d_2*d_4^3*cos(alpha_2)*cos(alpha_3) + 4*d_2*d_4*r*cos(alpha_2)*cos(alpha_3) - d_3^4 - 4*d_3^3*d_4*cos(alpha_3) - 4*d_3^2*d_4^2*cos(alpha_3)^2 - 2*d_3^2*d_4^2 + 2*d_3^2*r - 4*d_3*d_4^3*cos(alpha_3) + 4*d_3*d_4*r*cos(alpha_3) - d_4^4 + 2*d_4^2*r - r^2) + 2*a_2*d_4*sin(alpha_3))/(a_1^2 - 2*a_2*a_3 - r + a_2^2 + a_3^2 + d_2^2 + d_3^2 + d_4^2 + 2*d_2*d_3*cos(alpha_2) + 2*d_3*d_4*cos(alpha_3) + 2*d_2*d_4*cos(alpha_2)*cos(alpha_3) + 2*d_2*d_4*sin(alpha_2)*sin(alpha_3))); -2*atan((sqrt(- a_1^4 - 2*a_1^2*a_2^2 - 2*a_1^2*a_3^2 - 2*a_1^2*d_2^2 - 4*a_1^2*d_2*d_3*cos(alpha_2) - 4*a_1^2*d_2*d_4*cos(alpha_2)*cos(alpha_3) - 2*a_1^2*d_3^2 - 4*a_1^2*d_3*d_4*cos(alpha_3) - 2*a_1^2*d_4^2 + 2*a_1^2*r - a_2^4 + 2*a_2^2*a_3^2 - 2*a_2^2*d_2^2 - 4*a_2^2*d_2*d_3*cos(alpha_2) - 4*a_2^2*d_2*d_4*cos(alpha_2)*cos(alpha_3) - 2*a_2^2*d_3^2 - 4*a_2^2*d_3*d_4*cos(alpha_3) + 4*a_2^2*d_4^2*sin(alpha_3)^2 - 2*a_2^2*d_4^2 + 2*a_2^2*r - a_3^4 + 4*a_3^2*d_2^2*sin(alpha_2)^2 - 2*a_3^2*d_2^2 - 4*a_3^2*d_2*d_3*cos(alpha_2) - 4*a_3^2*d_2*d_4*cos(alpha_2)*cos(alpha_3) - 2*a_3^2*d_3^2 - 4*a_3^2*d_3*d_4*cos(alpha_3) - 2*a_3^2*d_4^2 + 2*a_3^2*r - d_2^4 - 4*d_2^3*d_3*cos(alpha_2) - 4*d_2^3*d_4*cos(alpha_2)*cos(alpha_3) - 4*d_2^2*d_3^2*cos(alpha_2)^2 - 2*d_2^2*d_3^2 - 8*d_2^2*d_3*d_4*cos(alpha_2)^2*cos(alpha_3) - 4*d_2^2*d_3*d_4*cos(alpha_3) - 4*d_2^2*d_4^2*cos(alpha_2)^2*cos(alpha_3)^2 + 4*d_2^2*d_4^2*sin(alpha_2)^2*sin(alpha_3)^2 - 2*d_2^2*d_4^2 + 2*d_2^2*r - 4*d_2*d_3^3*cos(alpha_2) - 12*d_2*d_3^2*d_4*cos(alpha_2)*cos(alpha_3) - 8*d_2*d_3*d_4^2*cos(alpha_2)*cos(alpha_3)^2 - 4*d_2*d_3*d_4^2*cos(alpha_2) + 4*d_2*d_3*r*cos(alpha_2) - 4*d_2*d_4^3*cos(alpha_2)*cos(alpha_3) + 4*d_2*d_4*r*cos(alpha_2)*cos(alpha_3) - d_3^4 - 4*d_3^3*d_4*cos(alpha_3) - 4*d_3^2*d_4^2*cos(alpha_3)^2 - 2*d_3^2*d_4^2 + 2*d_3^2*r - 4*d_3*d_4^3*cos(alpha_3) + 4*d_3*d_4*r*cos(alpha_3) - d_4^4 + 2*d_4^2*r - r^2) + 2*a_3*d_2*sin(alpha_2) + 2*a_2*d_4*sin(alpha_3))/(a_1^2 - 2*a_2*a_3 - r + a_2^2 + a_3^2 + d_2^2 + d_3^2 + d_4^2 + 2*d_2*d_3*cos(alpha_2) + 2*d_3*d_4*cos(alpha_3) + 2*d_2*d_4*cos(alpha_2)*cos(alpha_3) + 2*d_2*d_4*sin(alpha_2)*sin(alpha_3)))]
theta_3=theta_3(1);
z=100;
f1=a_3*cos(theta_3)+d_4*sin(alpha_3)*sin(theta_3)+a_2;
f2=a_3*cos(alpha_2)*sin(theta_3)-d_4*sin(alpha_3)*cos(alpha_2)*cos(theta_3)-d_4*sin(alpha_2)*cos(alpha_3)-d_3*sin(alpha_2);
f3=a_3*sin(alpha_2)*sin(theta_3)-d_4*sin(alpha_3)*sin(alpha_2).*cos(theta_3)+d_4*cos(alpha_2)*cos(alpha_3)+d_3*cos(alpha_2);
k1=f1;
k2=-f2;
k3=f1.^2+f2.^2+f3.^2+a_1.^2+d_2.^2+2*d_2.*f3;
alpha_1=-pi/2;
k4=f3*cos(alpha_1)+d_2*cos(alpha_1);

theta_2=[2*atan((sqrt(k1.^2*sin(alpha_1)^2 + k2^2*sin(alpha_1)^2 - k4.^2 + 2.*k4.*z - z^2) + k2.*sin(alpha_1))./(z - k4 + k1.*sin(alpha_1))); -2*atan((sqrt(k1.^2*sin(alpha_1)^2 + k2.^2*sin(alpha_1)^2 - k4.^2 + 2.*k4*z - z^2) - k2.*sin(alpha_1))./(z - k4 + k1.*sin(alpha_1)))]

%% 求解theta_2
clear;
syms z k1 k2 u alpha_1 k4
f=(k1*(1-u^2)/(1+u^2)+k2*2*u/(1+u^2))*sin(alpha_1)+k4-z
result=solve(f==0,u)
theta_2result=2*atan2(result,1)

%% 求解theta_1
clear
syms theta_2 alpha_1 f1 f2 a_1 d_2 f3 in1 u g1 g2
%g1=cos(theta_2).*f1-sin(theta_2).*f2+a_1
%g2=sin(theta_2).*cos(alpha_1).*f1-cos(theta_2).*cos(alpha_1).*f2-sin(alpha_1).*f3-d_2.*sin(alpha_1)
f=(1-u.^2)./(1+u.^2).*g1-2*u.^2./(1+u.^2).*g2-in1
f=simplify(f)
result=solve(f==0,u)
theta_1result=2*atan2(result,1)

%% 惯性张量
clear;
syms x y z
r=[x,y,z].';
skew(r)
-skew(r)*skew(r)

%% RPY角反解函数
function [alpha,beta,gamma]=RPYReverse(Rxyz)
alpha=atan2(Rxyz(2,1),Rxyz(1,1));
beta=atan2(-Rxyz(3,1),sqrt(Rxyz(1,1).^2+Rxyz(2,1).^2));
gamma=atan2(Rxyz(3,2),Rxyz(3,3));
end
