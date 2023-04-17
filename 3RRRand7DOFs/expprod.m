function R = expprod(omega,theta)
% 指数积计算
% 输入：单位向量omega，旋转角度theta
% 输出：旋转矩阵
% omega=omega/norm(omega);
R=eye(3)+sin(theta).*skew(omega)+(1-cos(theta)).*(skew(omega)*skew(omega));
end

