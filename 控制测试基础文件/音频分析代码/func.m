function [output]=func(~,~,app)
            global starttime;
            global endtime;
            global SoundY;
            global SoundFFT;
            global SoundFs;
            global N
            global fc
            global oc6
            global nc
            global value; 
            starttime=starttime+N;
            endtime=endtime+N;
            y1=SoundY(starttime:endtime);
%             sound(y1,SoundFs);
            tt=(starttime:endtime)/SoundFs;tt=tt';
            plot(app.TIMECH,tt,y1);drawnow;
            app.Average.Value=mean(y1); 
            app.RMS.Value=rms(y1);
            app.Peak.Value=max(y1);
            app.VPP.Value=peak2peak(y1);
            nfft = 2^nextpow2(N);
            SoundFFT=fft(y1,nfft);            
            ff=linspace(0,SoundFs/2,ceil(N/2));ff=ff';
            SoundA=2*abs(SoundFFT)/nfft;
            SoundA(1)=SoundA(1)/N;
            Power=SoundA.*SoundA;
            Power=20*log10(Power);
            switch value
                case('功率谱')
                    plot(app.FANALYSIS,log10(ff),Power(1:ceil(N/2)));
                case('幅频')
                    plot(app.FANALYSIS,log10(ff),SoundA(1:ceil(N/2)));
                case('相频')
                    ph = angle(SoundFFT)*180/pi;
                    plot(app.FANALYSIS,log10(ff),ph(1:ceil(N/2)));
                case('实频')
                    r=real(SoundFFT);
                    plot(app.FANALYSIS,log10(ff),r(1:ceil(N/2)));
                case('虛频')
                    i=imag(SoundFFT);
                    plot(app.FANALYSIS,log10(ff),i(1:ceil(N/2)));
            end
            drawnow;
            
             yc = zeros(1,nc); %计算倍频程谱 
                for j = 1:nc 
                    fl = fc(j)/oc6; % 下限频率 
                    fu = fc(j)*oc6; % 上限频率 
                    nl = round(fl*nfft/SoundFs+1); % 下限频率序号
                    nu = round(fu*nfft/SoundFs+1); % 上限频率序号 
                    
                    if fu > SoundFs/2 % 上限频率大于折叠频率 
                        m = j-1; 
                        break 
                    end 
                    % 以每个中心频率段为通带进行累加
                        b = zeros(1,nfft);
                        b(nl:nu) = SoundA(nl:nu);
                        b(nfft-nu+1:nfft-nl+1) = SoundA(nfft-nu+1:nfft-nl+1); 
                        yc(j) = sqrt(var(real(b(1:N))));
                end 
                bar(app.ONETHIRDANALYSIS,yc(13:m));drawnow;
                                
                Z=spectrogram(y1,2048,1024);
                Size=size(Z);
                P=20*log10(sqrt(Z.* conj(Z)));
                X=linspace(0,SoundFs/2, Size(1));X=X';
                Y=linspace(starttime/SoundFs,endtime/SoundFs, Size(2));Y=Y';
                surf(app.f_t_A,X,Y,P','FaceAlpha',0.3); drawnow;
                hold(app.f_t_A,"on");
        end