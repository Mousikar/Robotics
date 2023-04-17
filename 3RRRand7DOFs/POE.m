function expLtheta = POE(L,theta)
%POE 4*4指数积计算
%   输入：关节轴线矢量L=[r×w w].'
%   输出：指数积矩阵expLtheta
expLtheta=eye(4);
exp=expprod(L(4:6),theta);
expLtheta(1:3,1:3)=exp;
%% 两种不同方法求矩阵指数
expLtheta(1:3,4)=(eye(3)-exp)*cross(L(4:6),L(1:3))+L(4:6).'*L(1:3)*theta*L(4:6);
%% 第二种方法
% Gtheta=eye(3)*theta+(1-cos(theta)).*skew(L(4:6))+(theta-sin(theta)).*(skew(L(4:6))*skew(L(4:6)));
% expLtheta(1:3,4)=Gtheta*L(1:3);
end

