function Ad_V = Adjointmatrix(T)
%Adjointmatrix 此处显示有关此函数的摘要
%   此处显示详细说明
R=T(1:3,1:3);p=T(1:3,4);
Ad_V = [R skew(p)*R;
        zeros(3,3) R];
end

