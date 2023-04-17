function skewV = skewV(V)
%skewV 将线矢量转化为运动旋量
%   输入：线矢量，前三个是线速度，后三个元素是角速度
%   输出：运动旋量
skewV=zeros(4,4);
skewV(1:3,4) = V(1:3);
skewV(1:3,1:3) = skew(V(4:6));
end

