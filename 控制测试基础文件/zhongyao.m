%工测实验代码
%% 试验一
%根据表中数据绘制电压重量变化曲线图，计算灵敏度 S1=ΔV/ΔM（输出电压变化量与质量变化量之比）
%质量（g）
m=[0	10	20	50	100	200	500];
%输出电压(mV)
V=[-130	-110	-80	0	130	420	1300];
plot(m,V);
hold on;
plot(m,V,'r.');
title('电压重量变化曲线图');
xlabel('质量(g)');
ylabel('输出电压(mV)');

difm=diff(m)
difV=diff(V)
S=difV./difm

%标准质量（g）	
stan=[20	50	100	200	300	400];
%称重结果（g）	
real=[20.952	51.194	101.48	201.33	301.37	401.34];
abs(stan-real)

figure;
x=[31.5 37.3 44.5 52.5];
y=[0.16021 0.13639 0.12428 0.1135];
plot(x,y);


%% 实验二，噪声测量
sound=xlsread("experiment.xlsx",'A3:A442');
figure;
plot(sound);

% n=880;
% %采样频率
% fs=20000;
% %频率范围
% f=[22.4, 28,35.5, 45, 56, 71, 90, 112, 140, 180, 224, 280, 355, 450, 560, 710, 900, 1120, 1400, 1800, 2240, 2800, 3550, 4500, 5600, 7100, 9000, 11200, 14000, 18000] ;
% %中心频率
% fm=[25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600,2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000] ;
% n3=length(fm) ;
% deltf=fs/n;
% % ff=0:fs/n:fs*(n-1)/n;
% ff=linspace(0,fs/2,n/2);
% yf=fft(sound);
% Pyy =2*sqrt(yf.* conj(yf))/n;
% Pyy(1)= Pyy(1)/n;
% figure;
% plot(ff,Pyy(1:n/2));%plot(ff, yf) ;
% title('高斯白噪声的功率谱');
% ylabel('功率/Pa' );
% xlabel('频率/Hz');

%% 生成高斯白噪声
n=8192;
T=0.2;
x=linspace(0,T,n);
y=randn(1,n) ;
y=y-mean(y) ;
y=y/std(y);
a=0;
b=sqrt(1) ;
y=a+b*y;
plot(y);
%% 采样频率
fs=51200;
s=xcorr(y,'unbiased');
x1=linspace(-T,T,2*n-1);
figure;
plot(x1,s);

%% 频率范围
f=[22.4, 28,35.5, 45, 56, 71, 90, 112, 140, 180, 224, 280, 355, 450, 560, 710, 900, 1120, 1400, 1800, 2240, 2800, 3550, 4500, 5600, 7100, 9000, 11200, 14000, 18000] ;
%中心频率
fm=[25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600,2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500, 16000] ;
n3=length(fm) ;
deltf=fs/n;
% ff=0:fs/n:fs*(n-1)/n;
ff=linspace(0,fs/2,n/2);
yf=fft(y);
Pyy =2*sqrt(yf.* conj(yf))/n;
Pyy(1)= Pyy(1)/2;
figure;
plot(ff,Pyy(1:n/2));%plot(ff, yf) ;
title('高斯白噪声的功率谱');
ylabel('功率/Pa' );
xlabel('频率/Hz');

% yn=yf.*yf;
yn =2*sqrt(yf.* conj(yf))/n;
yn(1)= yn(1)/2;
for i=1:n3
f1=f(i);
f2=f(i+1);
n1=floor(f1/deltf);
n2=floor(f2/deltf);
temp=0;
for j=n1:1:n2
temp1=yn(j)/20-5;
yn(j)=20*10^(temp1);
temp=temp+yn(j) ;
end
energy(i)=temp;
temp2=log10(energy(i)/(20*10^ (-5)));
ydb(i)=temp2*20/deltf;
end
figure;
bar(ydb) ;
title('1/3倍频程分析结果');
ylabel('分贝/dB' );
xlabel('频率/Hz') ;

%% 实验四，验证正确与否
clear;
% v0=0.079995*10^-3;
% v1=0.098768*10^-3;
% v2=0.096188*10^-3;
% v3=0.095596*10^-3;
% m=4*10^-3;
v0=0.206504*10^-3;
v1=0.386387*10^-3;
v2=0.134135*10^-3;
v3=0.416482*10^-3;
m=4.3*10^-3;
k_2=(v1^2+v2^2+v3^2-3*v0^2)/(3*m^2)
mx=(v1^2-v0^2)/(2*m*k_2)-1/2*m
my=(v2^2-v3^2)/(2*sqrt(3)*m*k_2)
m=sqrt(mx^2+my^2)
a=atand(my/mx)

N=1024;
T=1;
x=linspace(0,T,N);
y=rand(1,N);
plot(x,y);
s=xcorr(y);
x1=linspace(-T,T,2*N-1);
plot(x1,s);
s=xcorr(y,'unbiased');
x1=linspace(-T, T,2*N-1);
plot(x1,s);

N=1024; T=0.2;
x=linspace(0,T,N);
y1=4*sin(2*3.14*50*x);
subplot(4,1,1); plot(x,y1);
y2=randn(1,N);
subplot(4,1,2); plot(x,y2);
y3=y1+y2;
subplot(4,1,3); plot(x,y3);
s=xcorr(y3,'unbiased');
x1=linspace(-T,T,2*N-1);
subplot(4,1,4); plot(x1,s);
xlim([-0.1 0.1]);

N=1024; T=0.2;
x=linspace(0,T,N);
y1=4*sin(2*3.14*50*x);
figure;
plot(x,y1);
y2=randn(1,N);
figure;
plot(x,y2);

s=xcorr(y1,y2,'unbiased');
x1=linspace(-T,T,2*N-1);
figure; plot(s);

%% 任务二：产生一个100hz以下虚拟方波或者正弦波信号，
% 编写时域过零检测算法计算信号频率并显示出来。
Fs=44100; %采样频率
N=81920; %//采样长度
dt=1.0/5120.0; %//采样间隔
T=dt*N; %//采样总时间
x=linspace(0,T,N); %//采样时间点
y=sin(2*3.14*100*x); %//各时间点信号
plot(x,y); %//绘制波形曲线
audioplayer(x,Fs); %//播放信号声音
x2=square(2*pi*4*t,75);
% f=floor(100*rand(1));

Fs=100000; %采样频率
N=81920; %//采样长度
dt=1.0/Fs;%5120.0; %//采样间隔
T=dt*N; %//采样总时间
x=linspace(0,T,N); %//采样时间点
y=10*sin(2*3.14*261.62*x); %//各时间点信号
sound(y,Fs)

path="C:\Users\RM\Desktop\工程测试技术实验七\夜的钢琴曲二十三.wav";
% filename='琵琶行.mp3';
% filepath='E:\笔记\学习\工程测试基础\实验参考文件\任萌实验六\五\';
% path=[filepath,filename];
[soundy,soundFs] = audioread(path);
y=soundy(1:ceil(20/51*length(soundy)));
% sound(soundy,soundFs)
audiowrite('夜的钢琴曲二十三1.wav',y,soundFs)
sound(y,soundFs)

N=4410/2;
nfft = 2^nextpow2(N);

% A=fft(soundy,nfft);
Z=spectrogram(soundy(1:44100),2048,1024);
                Size=size(Z);
                P=20*log10(sqrt(Z.* conj(Z)));
                X=linspace(0,512/2, Size(1));
                Y=linspace(0,N/512, Size(2));
                

figure;
                mesh(X,Y,P'); 
                hold("on");
                view(15,70);

sf = 22050; n=16384; duration=(n+2)/sf; %采样
recorder = audiorecorder(sf,16,1); 
recordblocking(recorder, duration); 
y = getaudiodata(recorder); 
y=y(1:n); 
dt=1.0/sf; 
T =n*dt; 
x=[0:n-1]*dt; 
subplot(3,1,1); 
plot(x,y); 
% 定义三分之一倍频程的中心频率 
f = [1.00 1.25 1.60 2.00 2.50 3.15 4.00 5.00 6.300 8.00]; 
fc = [f,10*f,100*f,1000*f,10000*f]; 
oc6 = 2^(1/6); 
nc = length(fc); 
nfft = 2^nextpow2(n); 
a = fft(y,nfft); % FFT变换 
subplot(3,1,2); 
f1=linspace(0,sf/2,n/2); 
A1=abs(a)/(n/2); 
plot(f1,A1(1:n/2)); 
yc = zeros(1,nc); %计算倍频程谱 
for j = 1:nc 
    fl = fc(j)/oc6; % 下限频率 
    fu = fc(j)*oc6; % 上限频率 
    nl = round(fl*nfft/sf+1); % 下限频率序号
    nu = round(fu*nfft/sf+1); % 上限频率序号 
    
    if fu > sf/2 % 上限频率大于折叠频率 
        m = j-1; 
        break 
    end % 以每个中心频率段为通带进行累加
        b = zeros(1,nfft);
        b(nl:nu) = a(nl:nu);
        b(nfft-nu+1:nfft-nl+1) = a(nfft-nu+1:nfft-nl+1); 
        c = ifft(b,nfft); yc(j) = sqrt(var(real(b(1:n))));
end 
subplot(3,1,3); 
bar(fc(13:m),yc(13:m));

Fs = 5120.0;N=32768;
dt=1.0/Fs; df=(500.0/N);
T=dt*N;t=linspace(0,T,N);
for i = 1:8192
x(i) =sin(2*pi*200*i*dt);
x(i+8192) =sin(2*pi*400*i*dt);
x(i+16384) =sin(2*pi*600*i*dt);
x(i+24576) =sin(2*pi*800*i*dt);
end
Z=spectrogram(x,1024,512);
P=20*log10(sqrt(Z.* conj(Z)));
X=linspace(0,Fs/2, 513);
Y=linspace(0,dt*N, 63);
mesh(X,Y,P'); 
view(15,70);

%% 变频信号的fft观察
close all
clear,clc
%% 定义一个变频信号的参数
PIx2 = 2 * pi;
Fs = 1500;
T = 1/Fs;
LengthOfSignal = 3000;
NFFT = 2^nextpow2(LengthOfSignal); %要计算的点数
% NFFT = 128;
t = (0:LengthOfSignal-1)*T;
amp1 = 2;
amp2 = 2;
amp3 = 1.5;
offset = 2;
freq1 = 100;
freq2 = 150;
freq3 = 300;
signal = zeros(1,length(t));
 
%% 定义信号
for temp = 1:LengthOfSignal
    if(temp <= LengthOfSignal/4)
        signal(temp) =offset + amp1 * sin(PIx2 * freq1 * t(temp));
    elseif(temp <= LengthOfSignal/2)
        signal(temp) =offset -1*amp2 * sin(PIx2 * freq2 * t(temp));
    elseif(temp <= 3*LengthOfSignal/4)
        signal(temp) =offset -1*amp3 * sin(PIx2 * freq3 * t(temp));
    else
        signal(temp) =offset + amp1 * sin(PIx2 * freq1 * t(temp));
    end
end
 
%% 绘制图形
subplot(311);
plot(t, signal);
grid on
title('signal of different frequency');
xlabel('time');
ylabel('amp');
 
%% fft运算
fMax = NFFT/2 + 1;
signalFFT = abs(fft(signal,NFFT));
% signalFFTShow = 2 * abs(fft(signal(1:fMax),NFFT)/LengthOfSignal);
signalFFTShow = 2 * signalFFT / LengthOfSignal;
signalFFTShow(1) = signalFFTShow(1)/2;
f = Fs/2*linspace(0,1,fMax);
subplot(312);
plot(f,signalFFTShow(1:fMax));
grid on
title('fft signal');
xlabel('frequency');
ylabel('amp');
 
%% surf测试三维图(不合理x(j),y(i),z(i,j)对应)
 
subplot(313)

spectrogram(signal,128,120,128,1.5e3); %时频域图

%% 液压实验二
% 节流阀进油回路
p=[0.95 1.3 1.8 2.2 2.6 3.0 3.4 3.8]';%Mpa
F=12.56*10^-4.*p*10^6;%N
l=300*10^-3;%m
t=[8.86 9.16 11.39 11.88 14.02 17.38 21.49 36.63]';%s
v=l./t;
plot(v,p)

% 节流阀回油回路
p=[0.9 1 1.1 1.2 1.4 1.6]';%Mpa
F=12.56*10^-4.*p*10^6;%N
l=300*10^-3;%m
t=[13.81 14.38 15.39 15.82 16.20 19.74]';%s
v=l./t;
plot(v,p)

% 调速阀进油路回路
p=[0.73 1.2 1.4 1.8 2.2 2.6 3.0 3.4]';%Mpa
F=12.56*10^-4.*p*10^6;%N
l=300*10^-3;%m
t=[26.61 26.60 26.51 26.67 26.18 26.55 26.63 27.12]';%s
v=l./t;
plot(v,p)

%% 互换性实验2-1
y=[0 5.2 16.8 30.2 28.2 37.9 52.8 33.6 43.1]';
y1=y-5.2;
y1(1)=0

y2(1)=0;
for i=2:length(y)
    y2(i)=y1(i)+y2(i-1);
end
y2'

%% CAD技术练习2第3题
P00=[0,0,0];
P01=[20,0,20];
P02=[40,0,20];
P03=[60,0,0];
P04=[80,0,-10];

P10=[0,20,20];
P11=[20,20,40];
P12=[40,20,40];
P13=[60,20,20];
P14=[80,20,0];

P20=[0,40,20];
P21=[20,40,40];
P22=[40,40,40];
P23=[60,40,20];
P24=[80,40,0];

P30=[0,60,0];
P31=[20,60,20];
P32=[40,60,20];
P33=[60,60,0];
P34=[80,60,-10];

P40=[0,80,-20];
P41=[20,80,0];
P42=[40,80,0];
P43=[60,80,-20];
P44=[80,80,0];

p=[
P01
P02
P03
P04
P11 
P12
P13
P14
P21 
P22 
P23
P24
P31
P32
P33
P34
];
% plot3(p(:,1),p(:,2),p(:,3),'ro')

Mb=1/6*[
-1   3   -3  1
3   -6  3   0
-3  0   3   0
1   4   1   0
];
% syms u v
u=0.5;
v=1;
U=[u^3 u^2 u 1];
V=[v^3 v^2 v 1];
Nu=U*Mb
Nv=V*Mb
diff(Nv)
% Nu=[(1-u)^3 3*u*(1-u)^2 3*u^2*(1-u) u^3];
% Nv=[(1-v)^3 3*v*(1-v)^2 3*v^2*(1-v) v^3];
Px10=[
P10(1) P11(1) P12(1) P13(1)
P20(1) P21(1) P22(1) P23(1)
P30(1) P31(1) P32(1) P33(1) 
P40(1) P41(1) P42(1) P43(1) 
]
S10uvx=Nu*Px10*Nv.'
S10uvx=U*Mb*Px10*Mb.'*V.'
Px00=[
P00(1) P01(1) P02(1) P03(1)
P10(1) P11(1) P12(1) P13(1)
P20(1) P21(1) P22(1) P23(1)
P30(1) P31(1) P32(1) P33(1)    
];
S00uvx=Nu*Px00*Nv.'
% simplify(S00uvx)
Px01=[
P01(1) P02(1) P03(1) P04(1)
P11(1) P12(1) P13(1) P14(1)
P21(1) P22(1) P23(1) P24(1)
P31(1) P32(1) P33(1) P34(1)
]
S01uvx=Nu*Px01*Nv.'
Py01=[
P01(2) P02(2) P03(2) P04(2)
P11(2) P12(2) P13(2) P14(2)
P21(2) P22(2) P23(2) P24(2)
P31(2) P32(2) P33(2) P34(2)
]
S01uvy=Nu*Py01*Nv.'
Pz01=[
P01(3) P02(3) P03(3) P04(3)
P11(3) P12(3) P13(3) P14(3)
P21(3) P22(3) P23(3) P24(3)
P31(3) P32(3) P33(3) P34(3)
]
S01uvz=Nu*Pz01*Nv.'

Pd=[
-0.5*P02+0.5*P04
-0.5*P12+0.5*P14
-0.5*P22+0.5*P24
-0.5*P32+0.5*P34
];
Svd02=Nu*Pd
Svd02+[S01uvx S01uvy S01uvz]

%% 物理实验
3160-(3260^2*17.1/1000)/(130-3260*(-17.1/1000) )
t=[23.6 26.6 29.6 32.6 35.6 38.6 41.6 44.6 47.6 50.6];
t=[23.6 26.6 32.6 38.6 41.6 44.6 47.6 50.6];
T=t+273.15
x=1./T
Rt=[3160 2883 2627 2392 2182 1987 1815 1655 1511 1160];
Rt=[3160 2883 2392 1987 1815 1655 1511 1160];
y=log(Rt)
b=(mean(x.*y)-mean(x)*mean(y))/(mean(x.^2)-mean(x)^2)
a=mean(y)-b*mean(x)
exp(a)
