% 运动参数标定parameter calibration
%% 观测真实误差下的五个不同的位姿 用正解测
clear;
err_real=[ 0 0.01 0 0 0 0 0.02 0 0.05 0 0 0].';%真实误差
delta_qreal=0;%真实误差
q=-pi/8+pi/4*rand([20,3]);
delta_q=0;
num=length(q);
R_real=cell(num,1);
R_cal=cell(num,1);
Jq=cell(num,1);
JL=cell(num,1);
err_cal=zeros(12,1);
% 循环
% for j=1:100
    %%
for i=1:num
    %%
    [R_real{i},~,~,~,~] = RRR3expprodPro(q(i,:)+delta_qreal,err_real);
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
xhat=inv(B'*B)*B'*b;
delta_q=delta_q+xhat(1:3);
err_cal=err_cal+xhat(4:15);
% end