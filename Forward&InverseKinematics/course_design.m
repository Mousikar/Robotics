%% 研讨课：PUMA560正反解验证过程
% 任萌 机械1807班 2021年3月31日
% 清空工作区
clear;
%% 建立PUMA560的改进DH模型
%       theta    d          a        alpha
L1=Link([0       0          0         0   ],'modified'); 
L2=Link([0       149.09     0        -pi/2],'modified');
L3=Link([0       0          431.8     0   ],'modified');
L4=Link([0       433.07     20.32    -pi/2],'modified');
L5=Link([0       0          0         pi/2],'modified');
L6=Link([0       0          0        -pi/2],'modified');

robot=SerialLink([L1,L2,L3,L4,L5,L6]);
robot.name='PUMA560';
robot.comment='RM';
puma=robot;
%% 正解
q=[pi/2 pi/3 -pi/2 pi/6 pi/4 pi/8];
disp('机器人工具箱自带的正解函数：')
puma.fkine(q)
disp('自己编写的正解函数求解结果：')
Kinematics(q)
%% 反解
%% 检验三轴相交PIEPER方法
% 给末端位置和相对于固定坐标系的zyz欧拉角
in=[-250 430 -240 pi/8 pi/4 pi/5];
theta=Three_axis_intersection_pieper(in);
disp('求出的六个关节角为：')
theta{1}
r=rotz(in(4))*roty(in(5))*rotz(in(6));
T=[
    1 0 0 in(1);
    0 1 0 in(2);
    0 0 1 in(3);
    0 0 0 1];
T(1:3,1:3)=r
disp('机器人工具箱的反解函数ikine求解得到的关节角：')
q=puma.ikine(T)
disp('机器人工具箱自带的正解fkine检验反解的正确性:')
puma.fkine(theta{1})
disp('通过自己写的正解检验反解的正确性:')
Kinematics(theta{1})

%% 检验反变换方法
in=[-200 300 -220 pi/3 pi/8 pi/6];
q=InverseTransformation(in);
% 检验
for i=1:8
    word=['反变换法的第',num2str(i),'组关节角:'];disp(word);
    q{i}
    disp('机器人工具箱自带的正解fkine检验其正确性:')
    puma.fkine(q{i})
end

r=rotz(in(4))*roty(in(5))*rotz(in(6));
T=[
    1 0 0 in(1);
    0 1 0 in(2);
    0 0 1 in(3);
    0 0 0 1];
disp('末端位姿：')
T(1:3,1:3)=r
disp('机器人工具箱的反解函数ikine求解得到的关节角：')
q=puma.ikine(T)
puma.fkine(q)


% mdl_puma560
% r=rotz(in(4))*roty(in(5))*rotz(in(6));
% T=[
%     1 0 0 in(1)/1000;
%     0 1 0 in(2)/1000;
%     0 0 1 in(3)/1000;
%     0 0 0 1];
% disp('末端位姿：')
% T(1:3,1:3)=r;
% disp('机器人工具箱的反解函数ikine求解得到的关节角：')
% q=p560.ikine6s(T)
% p560.fkine(q)

%% 欧拉角的转换
R_zyz = eul2r(15,30,60);  %这里输入是角度制
gamma = tr2eul(R_zyz); %这里输出是弧度制
%% 
mdl_puma560
r=rotz(in(4))*roty(in(5))*rotz(in(6));
T=[
    1 0 0 in(1)/1000;
    0 1 0 in(2)/1000;
    0 0 1 in(3)/1000;
    0 0 0 1];
T(1:3,1:3)=r;
q=p560.ikine6s(T)
p560.fkine(q)
%% 末端沿螺旋线运动
%螺旋线
plot(p560,[0 0 0 0 0 0]);

theta=-3*pi:0.1*pi:3*pi;
x=0.3*cos(theta);
y=0.3*sin(theta);
z=0.05*theta;

plot3(x,y,z);


for theta=-3*pi:0.2*pi:3*pi
    x=0.3*cos(theta);
    y=0.3*sin(theta);
    z=0.05*theta;
    
    targ_tran1=p560.base;
    targ_tran1(1:3,4)=[x y z]';
    targ_ang1=p560.ikine(targ_tran1);
    
    p560.plot(targ_ang1);
    
    targ_tran2=p560.base;
    targ_tran2(1:3,4)=[x y z]';
    targ_ang2=p560.ikine(targ_tran2);

    p560.plot(targ_ang2);
    
    pause(0.01);
end
%%
init_ang = [0 0 0 0 0 0];
targ_ang = [pi/4, -pi/3, pi/5, pi/2, -pi/4, pi/6];
step = 50;
[q,qd,qdd] = jtraj(init_ang,targ_ang,step);   %直接得到角度、角速度、角加速度的的序列

%动画显示
figure; 
p560.plot(q);

figure;
%显示位置、速度、加速度变化曲线
subplot(3, 1, 1);

plot(q);
title('关节角的角度');
grid on;

subplot(3, 1, 2);
i = 1:6;
plot(qd);
title('速度');
grid on;

subplot(3, 1, 3);
i = 1:6;
plot(qdd);
title('加速度');
grid on;

t =p560.fkine(q);%运动学正解
rpy=tr2rpy(t);  %t中提取位置（xyz）
figure;
plot2(rpy);
%% 
theta=-3*pi:0.1*pi:3*pi;
x=100*cos(theta)+300;
y=100*sin(theta)+300;
z=50*theta;
% plot3(x,y,z);
a_ang=[x(1) y(1) z(1) 0 0 0];
q=InverseTransformation(a_ang);
q=q{1};
puma.plot(q);
for i=2:length(theta)
    % 当前位置关节角转换为下一位置的关节角
    tar_ang=[x(i) y(i) z(i) 0 0 0];
    next_ang=comparetheta(q,tar_ang);
    q=next_ang;
    puma.plot(next_ang);
    pause(0.1);
    
end