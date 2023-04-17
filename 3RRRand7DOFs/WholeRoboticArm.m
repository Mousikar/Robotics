%% 建立7自由度模型，并联部分用三轴相交代替
%       theta    d          a        alpha
L1=Link([0       0          0        -pi/2],'modified'); 
L2=Link([0       0          0         pi/2],'modified');
L3=Link([0       0          0         pi/2],'modified');
L4=Link([0       0        255         0   ],'modified');
L5=Link([0     215          0        -pi/2],'modified');
L6=Link([0       0          0         pi/2],'modified');
L7=Link([0       100        0        -pi/2],'modified');

robot=SerialLink([L1,L2,L3,L4,L5,L6,L7]);
shoulder=SerialLink([L1,L2,L3]);
wrist=SerialLink([L5,L6,L7]);
robot.name='Bionic robotic arm';
robot.comment='RM';
RoboticArm=robot;
plot(RoboticArm,[0 0 0 0 0 0 0]);
%% 将3RRR并联机器人和三轴相交对应
% 输入：3RRR的输入变量
% 输出：parallel的输入变量
% 思路：把3RRR出入变量先转化为输出位姿，再用输出位姿求解三轴相交的输入，
% 也就是一个3RRR正解+一个三轴相交的反解
function theta=RRR2para(theta_1,theta_2,theta_3)
    phi = RRR3forward_theta123(theta_1,theta_2,theta_3);
    Q=rotz(phi(1))*roty(phi(2))*rotz(phi(3));
    T=diag([1 1 1 1]);
    T(1:3,1:3)=Q;
    % 输入：末端位姿
    % 输出：三个关节角
    for i=1:8
        theta{i}=[0 0 0 0 0 0];
    end
    
    
theta=
end