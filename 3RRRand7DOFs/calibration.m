% 运动参数标定parameter calibration
%% 观测真实误差下的六个不同的位姿 用正解测
clear;
err_real=[ 0 0.01 0 0 0 0 0.02 0 0.05 0 0 0].';%真实误差
delta_qreal=0;%真实误差
q=pi/180*[
-10	0	0
-10	-10	0
-10	-10	-10
-10	-10	5
-10	5	5
5	5	5
10	5	5
10	20	5
10	20	15
-3	20	15
-3	13	15
-3	13	28
15	13	28
15	-8	28
15	-8	35
0	0	0
-5	-8	-10
18	-8	-10

];
delta_q=0;
num=length(q);
R_real=cell(num,1);
R_cal=cell(num,1);
Jq=cell(num,1);
JL=cell(num,1);
err_cal=zeros(12,1);
alp=0.01*[652
832
1080
1190
910
-34
-168
-1000
-1250
-662
-482
-1075
-1789
-745
-1130
-181
665
-93

];
bet=0.01*[942
1140
501
1199
1302
82
-110
-24
714
1632
1548
2143
404
705
980
228
-125
-2284

];
gam=0.01*[497
-282
13
-780
580
30
4
1247
683
927
456
-666
-961
-2701
-3157
-67
-176
-943
];

% 循环
% for j=1:100
    %%
for i=1:num
    %%
    R_real{i}=rotz(alp(i))*roty(bet(i))*rotx(gam(i));
    [a, b, c]=r2zyz(R_real{i});phi_real(i,:)=[a b c].';%真实输出x‘
    % 关于关节角度的矩阵
    [R_cal{i},~,u,v,w] = RRR3expprodPro(q(i,:),err_cal);
    [a, b, c]=r2zyz(R_cal{i});phi_cal(i,:)=[a b c].';
    Jq{i} = RRR3Jacobian(u,v,w);
    % 关于结构参数的矩阵
    [gamma,beta,alpha_1,~,eta,~,~,~] = paraconfig(err_cal);
    JL{i} = JacobLPro(gamma,alpha_1,beta,eta,phi_cal,q(i,:));
    B(i*3-2:i*3,:)=[Jq{i} dphi2domega(phi_cal(i,:))*JL{i}];
    R_k=R_real{i}/R_cal{i};
    delta(1,i)=sign(R_k(3,2))*abs(R_k(3,2)-R_k(2,3))/2;
    delta(2,i)=sign(R_k(1,3))*abs(R_k(1,3)-R_k(3,1))/2;
    delta(3,i)=sign(R_k(2,1))*abs(R_k(2,1)-R_k(1,2))/2;
end
%%
b=delta(:);
xhat=inv(B'*B)*B'*b
delta_q=delta_q+xhat(1:3)
err_cal=err_cal+xhat(4:15)
% end