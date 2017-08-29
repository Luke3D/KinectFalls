% Resample to 30 Hz

%relative time Wearable
tw = (TStamp_QPC_35EE(:,1) - TStamp_QPC_35EE(1,1))/10^6 %[s]
tk = (KinectTimeBody(:,1) - KinectTimeBody(1,1))/10^7 % [s]

tw(end)
tk(end)
    
%Rescale to 0-1
% tw = linspace(0,1,length(TStamp_QPC_35EE(:,1)))
% tk = linspace(0,1,length(KinectTimeBody(:,1)))

%Resample to 100 Hz (evenly spaced samples)
tnew = linspace(0,tw(end),tw(end)*100);
[accX,accY,accZ]= AccRaw_2_Gforce_35EE(X_acc_raw_35EE,Y_acc_raw_35EE,Z_acc_raw_35EE);
accw = [accX, accY, accZ];
accw = interp1(tw,accw,tnew);
figure, plot(tnew,accw,'LineWidth',2)
figure, plot(tnew,sqrt(sum(accw.^2,2))','LineWidth',2)

%Freq spectrum
Fs = 100;
% T=1/Fs;
% L = size(accw,1);
% n = 2^nextpow2(L);
% Y = fft(accw,n);
% f = Fs*(0:(n/2))/n;
% P = abs(Y/n);
% figure, plot(f,P(1:n/2+1,:))

%High pass filter (cutoff 0.2 Hz)
fc = 0.2;
[b,a] = butter(2,fc/(Fs/2),'high');
% freqz(b,a)
accw_filt = filtfilt(b,a,accw);
 figure,plot(accw_filt)

%Resample acc data to 30 Hz
accw_filt_dec = [decimate(accw_filt(:,1),3),decimate(accw_filt(:,2),3),decimate(accw_filt(:,3),3)]
figure, plot(accw_filt_dec,'LineWidth',2), legend('X','Y','Z')

%total acc
figure, plot(sqrt(sum(accw_filt_dec.^2,2))','LineWidth',2)

%% compute derivative of skeleton position data
dt = 0.033;
tnew = linspace(0,tk(end),tk(end)/dt);
SklP = [jMatSklX jMatSklY jMatSklZ];
% SklP = SklP-mean(SklP,1);
SklP = interp1(tk,SklP,tnew); %resample
velk = diff(SklP,1)/dt;
acck = diff(SklP,2)/dt;
figure, subplot(311), plot(tnew,SklP), legend('X','Y','Z')
subplot(312),plot(tnew(2:end),velk), subplot(313), plot(tnew(3:end),acck)



%Freq spectrum
Fs = 1/dt;
L = size(SklP,1);
n = 2^nextpow2(L);
Y = fft(SklP,n);
f = Fs*(0:(n/2))/n;
P = abs(Y/n);
figure, plot(f,P(1:n/2+1,:))




