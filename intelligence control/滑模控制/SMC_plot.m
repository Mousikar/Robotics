close all;
e=out.e;
de=out.de;
c=pa.c;
% 画e-de图:包含s=0的理想情况（de=-ce）和实际的de
plot(e,-c*e,'k',e,de,'r:','LineWidth',2)
legend('s=0','s change')
xlabel('e')
ylabel('de')
title('Phase portrait')