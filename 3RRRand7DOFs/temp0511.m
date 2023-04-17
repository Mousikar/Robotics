joid=load('joid.mat');
plotdata=cell(6,1);
for i=1:6
    temp=joid.joid{i}.';
    vrds=abs(temp)./max(abs(temp));
    plotdata{i}=vrds.';
end
figure
plot(plotdata{1},'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
figure
plot(plotdata{2},'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
figure
plot(plotdata{3},'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
figure
plot(plotdata{4},'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
figure
plot(plotdata{5},'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
figure
plot(plotdata{6},'LineWidth',2)
legend('关节1','关节2','关节3','关节4','关节5','关节6','关节7' )
%%
pard=load('pard.mat');
plotdata=cell(6,1);
for i=1:6
%     temp=pard.pard{i}.';
%     vrds=abs(temp)./max(abs(temp));
%     plotdata{i}=vrds.';
    plotdata{i}=pard.pard{i};
end
figure
plot(plotdata{1},'LineWidth',2)
figure
plot(plotdata{2},'LineWidth',2)
figure
plot(plotdata{3},'LineWidth',2)
figure
plot(plotdata{4},'LineWidth',2)
figure
plot(plotdata{5},'LineWidth',2)
figure
plot(plotdata{6},'LineWidth',2)