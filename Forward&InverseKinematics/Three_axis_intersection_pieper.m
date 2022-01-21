%% 三轴相交的PIEPER解法
% 输入：末端坐标和位姿
% 输出：六个关节角
function [theta] = Three_axis_intersection_pieper(in)
%% 准备工作
r=in(1).^2+in(2).^2+in(3).^2;
% 因为theta有8组解，因此返回值theta是一个元胞数组
theta=cell(2*8,1);
for i=1:8*2
    theta{i}=[0 0 0 0 0 0];
end
%% 参数
a_1=0;a_2=431.8;a_3=20.32;
alpha_1=-pi/2;alpha_2=0;alpha_3=-pi/2;
d_2=149.09;d_3=0;d_4=433.07;
%% 求解theta_3,推导过程见实时文档
% 如果a1等于0，sin(alpha1)≠0
a=2*a_2*a_3 - 2*d_2*d_4*sin(alpha_2)*sin(alpha_3);
b=2*a_3*d_2*sin(alpha_2)+ 2*a_2*d_4*sin(alpha_3);
c=r-(a_1^2 + a_2^2 + a_3^2 + d_2^2 + d_3^2 + d_4^2 + 2*d_2*d_3*cos(alpha_2) + 2*d_3*d_4*cos(alpha_3)+ 2*d_2*d_4*cos(alpha_2)*cos(alpha_3));
if a+c~=0
    theta_3=[2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));2*atan((b-sqrt(b^2+a^2-c^2))/(a+c))];
else
    theta_3=[pi;pi];
end
% theta的第1~4组第三个数字为theta_3(1)
theta{1}(3)=theta_3(1);theta{2}(3)=theta_3(1);theta{3}(3)=theta_3(1);theta{4}(3)=theta_3(1);
theta{5}(3)=theta_3(1);theta{6}(3)=theta_3(1);theta{7}(3)=theta_3(1);theta{8}(3)=theta_3(1);
% theta的第5~8组第三个数字为theta_3(2)
theta{9}(3)=theta_3(2);theta{10}(3)=theta_3(2);theta{11}(3)=theta_3(2);theta{12}(3)=theta_3(2);
theta{13}(3)=theta_3(2);theta{14}(3)=theta_3(2);theta{15}(3)=theta_3(2);theta{16}(3)=theta_3(2);
%% 求解theta_2,推导过程见实时文档
% 第一种情况
f1(1)=a_3*cos(theta_3(1))+d_4*sin(alpha_3)*sin(theta_3(1))+a_2;
f2(1)=a_3*cos(alpha_2)*sin(theta_3(1))-d_4*sin(alpha_3)*cos(alpha_2)*cos(theta_3(1))-d_4*sin(alpha_2)*cos(alpha_3)-d_3*sin(alpha_2);
f3(1)=a_3*sin(alpha_2)*sin(theta_3(1))-d_4*sin(alpha_3)*sin(alpha_2)*cos(theta_3(1))+d_4*cos(alpha_2)*cos(alpha_3)+d_3*cos(alpha_2);
k1(1)=f1(1);
k2(1)=-f2(1);
k3(1)=f1(1).^2+f2(1).^2+f3(1).^2+a_1.^2+d_2.^2+2*d_2*f3(1);
k4(1)=f3(1)*cos(alpha_1)+d_2*cos(alpha_1);
a=-k2(1)*sin(alpha_1);
b=k1(1)*sin(alpha_1);
c=in(3)-k4(1);
if a+c~=0
    theta_2(1:2)=[2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));2*atan((b-sqrt(b^2+a^2-c^2))/(a+c))];
else
    theta_2(1:2)=[pi;pi];
end
% 第二种情况
f1(2)=a_3*cos(theta_3(1))+d_4*sin(alpha_3)*sin(theta_3(1))+a_2;
f2(2)=a_3*cos(alpha_2)*sin(theta_3(1))-d_4*sin(alpha_3)*cos(alpha_2)*cos(theta_3(1))-d_4*sin(alpha_2)*cos(alpha_3)-d_3*sin(alpha_2);
f3(2)=a_3*sin(alpha_2)*sin(theta_3(1))-d_4*sin(alpha_3)*sin(alpha_2)*cos(theta_3(1))+d_4*cos(alpha_2)*cos(alpha_3)+d_3*cos(alpha_2);
k1(2)=f1(2);
k2(2)=-f2(2);
k3(1)=f1(2).^2+f2(2).^2+f3(2).^2+a_1.^2+d_2.^2+2*d_2*f3(2);
k4(2)=f3(2)*cos(alpha_1)+d_2*cos(alpha_1);
a=-k2(2)*sin(alpha_1);
b=k1(2)*sin(alpha_1);
c=in(3)-k4(2);
if a+c~=0
    theta_2(3:4)=[2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));2*atan((b-sqrt(b^2+a^2-c^2))/(a+c))];
else
    theta_2(3:4)=[pi;pi];
end
% theta2
theta{1}(2)=theta_2(1);theta{2}(2)=theta_2(1);theta{3}(2)=theta_2(1);theta{4}(2)=theta_2(1);
theta{5}(2)=theta_2(2);theta{6}(2)=theta_2(2);theta{7}(2)=theta_2(2);theta{8}(2)=theta_2(2);
theta{9}(2)=theta_2(3);theta{10}(2)=theta_2(3);theta{11}(2)=theta_2(3);theta{12}(2)=theta_2(3);
theta{13}(2)=theta_2(4);theta{14}(2)=theta_2(4);theta{15}(2)=theta_2(4);theta{16}(2)=theta_2(4);
%% 求解theta_1,推导过程见实时文档
% -------------------------------------------------------------------------------------------------------
f11=[f1(1);f1(1);f1(2);f1(2)];f22=[f2(1);f2(1);f2(2);f2(2)];f33=[f3(1);f3(1);f3(2);f3(2)];
g1=cos(theta_2).*f11-sin(theta_2).*f22+a_1;
g2=sin(theta_2).*cos(alpha_1).*f11-cos(theta_2).*cos(alpha_1).*f22-sin(alpha_1).*f33-d_2.*sin(alpha_1);
c=in(1);
% -------------------------------------------------------------------------------------------------------
a=g1(1);
b=-g2(1);
if a+c~=0
    theta_1(1:2)=[2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));2*atan((b-sqrt(b^2+a^2-c^2))/(a+c))];
else
    theta_1(1:2)=[pi;pi];
end

% -------------------------------------------------------------------------------------------------------
a=g1(2);
b=-g2(2);
if a+c~=0
    theta_1(3:4)=[2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));2*atan((b-sqrt(b^2+a^2-c^2))/(a+c))];
else
    theta_1(3:4)=[pi;pi];
end

% -------------------------------------------------------------------------------------------------------
a=g1(3);
b=-g2(3);
if a+c~=0
    theta_1(5:6)=[2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));2*atan((b-sqrt(b^2+a^2-c^2))/(a+c))];
else
    theta_1(5:6)=[pi;pi];
end

% -------------------------------------------------------------------------------------------------------
a=g1(4);
b=-g2(4);
if a+c~=0
    theta_1(7:8)=[2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));2*atan((b-sqrt(b^2+a^2-c^2))/(a+c))];
else
    theta_1(7:8)=[pi;pi];
end
% 赋值
theta{1}(1)=theta_1(1);theta{2}(1)=theta_1(1);theta{3}(1)=theta_1(2);theta{4}(1)=theta_1(2);
theta{5}(1)=theta_1(3);theta{6}(1)=theta_1(3);theta{7}(1)=theta_1(4);theta{8}(1)=theta_1(4);
theta{9}(1)=theta_1(5);theta{10}(1)=theta_1(5);theta{11}(1)=theta_1(6);theta{12}(1)=theta_1(6);
theta{13}(1)=theta_1(7);theta{14}(1)=theta_1(7);theta{15}(1)=theta_1(8);theta{16}(1)=theta_1(8);
%% 求解theta4/5/6
for i=1:16
    R_1_0=[cos(theta{i}(1)) -sin(theta{i}(1)) 0;
        sin(theta{i}(1)) cos(theta{i}(1)) 0;
        0 0 1
        ];
    R_2_1=[cos(theta{i}(2)) -sin(theta{i}(2)) 0;
        0 0 1;
        -sin(theta{i}(2)) -cos(theta{i}(2)) 0];
    R_3_2=[cos(theta{i}(3)) -sin(theta{i}(3)) 0;
        sin(theta{i}(3)) cos(theta{i}(3)) 0;
        0 0 1 ;
        ];
    R_3_0=R_1_0*R_2_1*R_3_2;
    R_6_0=rotz(in(4))*roty(in(5))*rotz(in(6));
    R_6_3=R_3_0\R_6_0;
    % 推导过程看唐瑞淳发的图片
    theta{i}(5)=acos(R_6_3(2,3));
    if sin(theta{i}(5))~=0
        theta{i}(4)=atan2(R_6_3(3,3),-R_6_3(1,3));
        theta{i}(6)=atan2(-R_6_3(2,2),R_6_3(2,1));
    else
        theta46=acos(R_6_3(1,1));
        theta{i}(4)=0;
        theta{i}(6)=theta46;
    end
%     cosbeta=sqrt(R_6_3(1,1)^2+R_6_3(2,1)^2);
%     if cosbeta~=0
%     theta{i}(5)=atan2(-R_6_3(3,1),sqrt(R_6_3(1,1)^2+R_6_3(2,1)^2));
%     theta{i}(4)=atan2(R_6_3(2,1),R_6_3(1,1));
%     theta{i}(6)=atan2(R_6_3(3,2),R_6_3(3,3));        
%     end
% -------------------------------------------------------------------------------------------------------
%     R_6_3=rotx(-pi/2)\R_6_3;
%     theta{i}(5)=atan2(sqrt(R_6_3(3,1)^2+R_6_3(3,2)^2),R_6_3(3,3));
%     theta{i}(4)=atan2(R_6_3(2,3),R_6_3(1,3))-pi;
%     theta{i}(6)=atan2(R_6_3(3,2),-R_6_3(3,1))-pi;

end

