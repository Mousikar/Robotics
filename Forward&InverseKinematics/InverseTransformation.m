function theta = InverseTransformation(in)
% Inverse transformation 反变换法
% 任萌 2021年3月31日
% 输入：末端坐标和位姿
% 输出：六个关节角
%% 参数，用的时候改参数，针对扭转角特定的情况（参见PUMA560参数表）
a_1=0;a_2=431.8;a_3=20.32;
alpha_1=-pi/2;alpha_2=0;alpha_3=-pi/2;
d_2=149.09;d_3=0;d_4=433.07;
%% 准备
% 因为theta有8组解，因此返回值theta是一个元胞数组
theta=cell(8,1);
for i=1:8
    theta{i}=[0 0 0 0 0 0];
end
R06=rotz(in(4))*roty(in(5))*rotz(in(6));
T06=diag([1 1 1 1]);
T06(1:3,1:3)=R06;
T06(1:3,4)=[in(1) in(2) in(3)];
k=(in(1)^2+in(2)^2+in(3)^2-a_2^2-a_3^2-d_2^2-d_4^2)/(2*a_2);
%% 求解theta_1角
theta_1=[atan2(in(2),in(1))-atan2(d_2,sqrt(in(1)^2+in(2)^2-d_2^2));
         atan2(in(2),in(1))-atan2(d_2,-sqrt(in(1)^2+in(2)^2-d_2^2))];
for i=1:4
    theta{i}(1)=theta_1(1);
end
for i=5:8
    theta{i}(1)=theta_1(2);
end
%% 求解theta_3角
theta_3=[atan2(a_3,d_4)-atan2(k,sqrt(a_3^2+d_4^2-k^2));
         atan2(a_3,d_4)-atan2(k,-sqrt(a_3^2+d_4^2-k^2));
    ];
theta{1}(3)=theta_3(1);theta{2}(3)=theta_3(1);theta{3}(3)=theta_3(2);theta{4}(3)=theta_3(2);
theta{5}(3)=theta_3(1);theta{6}(3)=theta_3(1);theta{7}(3)=theta_3(2);theta{8}(3)=theta_3(2);
%% 求解theta_2角
theta23=zeros(6,1);
for i=1:8
    theta23(i)=atan2((-a_3-a_2*cos(theta{i}(3)))*in(3)+(cos(theta{i}(1))*in(1)+sin(theta{i}(1))*in(2))*(a_2*sin(theta{i}(3))-d_4),(-d_4+a_2*sin(theta{i}(3)))*in(3)+(cos(theta{i}(1))*in(1)+sin(theta{i}(1))*in(2))*(a_2*cos(theta{i}(3))+a_3));
    theta{i}(2)=theta23(i)-theta{i}(3);
end
% %% 求解theta_4
% for i=1:8
%     theta{i}(4)=atan2(-T06(1,3)*sin(theta{i}(1))+T06(2,3)*cos(theta{i}(1)),-T06(1,3)*cos(theta{i}(1))*cos(theta23(i))-T06(2,3)*sin(theta{i}(1))*cos(theta23(i))+T06(3,3)*sin(theta23(i)));
% end
% %% 求解theta_5
% for i=1:8
%  theta{i}(5)=atan2(-T06(1,3)*(cos(theta{i}(1))*cos(theta23(i))*cos(theta{i}(4))+sin(theta{i}(1))*sin(theta{i}(4)))-T06(2,3)*(sin(theta{i}(1))*cos(theta23(i))*cos(theta{i}(4))-cos(theta{i}(1))*sin(theta{i}(4)))+T06(3,3)*sin(theta23(i))*cos(theta{i}(4)),-T06(1,3)*cos(theta{i}(1))*sin(theta23(i))-T06(2,3)*sin(theta{i}(1))*sin(theta23(i))-T06(3,3)*cos(theta23(i)));
% end
% %% 求解theta_6
% for i=1:8
%  theta{i}(6)=atan2(-T06(1,1)*(cos(theta{i}(1))*cos(theta23(i))*sin(theta{i}(4))-sin(theta{i}(1))*cos(theta{i}(4)))-T06(2,1)*(sin(theta{i}(1))*cos(theta23(i))*sin(theta{i}(4))+cos(theta{i}(1))*cos(theta{i}(4)))+T06(3,1)*sin(theta23(i))*sin(theta{i}(4)),T06(1,1)*((cos(theta{i}(1))*cos(theta23(i))*cos(theta{i}(4))+sin(theta{i}(1))*sin(theta{i}(4)))*cos(theta{i}(5))-cos(theta{i}(1))*sin(theta23(i))*sin(theta{i}(5)))+T06(2,1)*((sin(theta{i}(1))*cos(theta23(i))*cos(theta{i}(4))-cos(theta{i}(1))*sin(theta{i}(4)))*cos(theta{i}(5))-sin(theta{i}(1))*sin(theta23(i))*sin(theta{i}(5)))-T06(3,1)*(sin(theta23(i))*cos(theta{i}(4))*cos(theta{i}(5))+cos(theta23(i))*cos(theta{i}(5))));
% end

%% 欧拉角分解法
for i=1:8
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
end
%% 腕部翻转
for i=1:4
    theta{2*i}(4)=theta{2*i-1}(4)+pi;
    theta{2*i}(5)=-theta{2*i-1}(5);
    theta{2*i}(6)=theta{2*i-1}(6)+pi;
end
end

