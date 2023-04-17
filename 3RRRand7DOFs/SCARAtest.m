%% 课本SCARA机器人空间雅可比矩阵
l1=100;
l2=120;
theta1=0;
theta2=0;
J_STs=[
    0   l1*cos(theta1)  l1*cos(theta1)+l2*cos(theta1+theta2)  0
    0   l1*sin(theta1)  l1*sin(theta1)+l2*sin(theta1+theta2)  0
    0   0               0                                     1
    0   0               0                                     0
    0   0               0                                     0
    1   1               1                                     0
];
dottheta=[0 0.1 0 0.1].';
V=J_STs*dottheta;
skV = skewV(V);
p=[0 l1+l2 0 1].';
vb=skV*p
Vb=V;Vb(1:3)=vb(1:3)