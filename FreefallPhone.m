close all
figure, subplot(211), plot(freefallaccacc(:,1),freefallaccacc(:,2:end))
hold on
subplot(212), plot(freefalllinacclac(:,1),freefalllinacclac(:,2:end))
legend('X','Y','Z')

%remove gravity from 1
Fs1 = length(freefallaccacc(:,1))/freefallaccacc(end,1)
Fs2 = length(freefalllinacclac(:,1))/freefalllinacclac(end,1)

acc = freefallaccacc(:,2:end);
lacc = freefalllinacclac(:,2:end);

Fsnew = 50;
tnew = linspace(0,freefallaccacc(end,1),freefallaccacc(end,1)*Fsnew);
accnew = interp1(freefallaccacc(:,1),acc,tnew);

d = designfilt('highpassfir','FilterOrder',2,'CutoffFrequency',fc/(Fsnew/2),'SampleRate',Fsnew);
fvtool(d)

fc = 1; [b,a] = butter(2,fc/(Fsnew/2),'high');
freqz(b,a)
accf = filtfilt(b,a,accnew);
figure, plot(accf), legend('X','Y','Z')


figure
plot(freefalllinacclac(:,1),freefalllinacclac(:,2:end))
legend('X','Y','Z')
