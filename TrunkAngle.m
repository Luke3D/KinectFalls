%the shoulder (A,21) or mid-spine (A,2) and base (B,1) spine joints in kinect space
A_k = reshape(jMatSkl(spinejoint,:,:), [3 size(jMatSkl,3)]);
B_k = reshape(jMatSkl(1,:,:), [3 size(jMatSkl,3)]);
% figure, hold on
% plot(A_k'), 
% title('Kinect (Skeleton) frame')
% plot(B_k','.')
% legend('X_A','Y_A','Z_A','X_B','Y_B','Z_B')

%find trunk angle phi
A0_k = reshape(jMatSkl(spinejoint,:,:)-jMatSkl(1,:,:), [3 size(jMatSkl,3)]);
B0_k = reshape(jMatSkl(1,:,:)-jMatSkl(1,:,:), [3 size(jMatSkl,3)]); %this is just 0
% figure, subplot(121)
% plot(A0_k'), legend('X','Y','Z')
% title('A_o')
% subplot(122)
% plot3(B0_k(1,:), B0_k(3,:), B0_k(2,:),'.b','markersize',25); view(90,0); axis equal;  hold on                  
% plot3(A0_k(1,:), A0_k(3,:), A0_k(2,:),'.r','markersize',25); 

%transform phi vector coordinates from kinect to body ref frame and filter
th = pi/2;
[Rx,Ry,Rz] = RotMat(th);
A0_BF = Ry*Rx*A0_k;
B0_BF = Ry*Rx*B0_k;
phi = atan2(A0_BF(2,:),A0_BF(1,:));
phi = sign(phi(1))*abs(phi);
phi = smooth(phi,20);

%find phi (trunk angle)
figure, plot(phi)
title('trunk angle in fixed (upright) body ref frame')

% p = polyfit([1:length(phinew)-1, length(phi)],phinew,2);
% phifit = polyval(p,1:length(phi));
% hold on, plot(phifit,'r')

%Transform hip or spine mid joint from kinect to body frame
%this joint represents the accelerometer position
%Filter joint position from kinect data (to remove noise at the end of the
%fall)
accjoint = 2; %spine mid (joint used to simulate accelerometer position)
% accjoint = 17 %hip right
Waistxyz_k = jMatSkl(accjoint,:,:);
Waistxyz_k = Waistxyz_k/1000; %from mm to meters
Waistxyz_k = reshape(Waistxyz_k,[3 size(Waistxyz_k,3)]); 
Nps = 10; %points to smooth
Waistxyz_k = [smooth(Waistxyz_k(1,:),Nps) smooth(Waistxyz_k(2,:),Nps) smooth(Waistxyz_k(3,:),Nps)]';

%transform coord of right hip joint () from kinect frame to body frame
Waistxyz_b=[];
[Rx,Ry,~] = RotMat(pi/2);
for i = 1:length(phi)
    [~,~,Rz] = RotMat(-phi(i)); %trunk angle rotation
    Rphi(:,:,i) = Rz;
    Waistxyz_b(:,i) = Rphi(:,:,i)*Ry*Rx*Waistxyz_k(:,i);
end
Waistxyz_b = Waistxyz_b';

figure
subplot(211), plot(Waistxyz_k'), hold on, title('kinect frame - hip position - filtered')
legend('X','Y','Z')
subplot(212), plot(Waistxyz_b), title('body frame - hip position - filtered')
legend('X','Y','Z')

%acc from waist accelerometer
tw = (TStamp_QPC_35EE(:,1) - TStamp_QPC_35EE(1,1))/10^6; %[s]
Fs = 100;
tnew = linspace(0,tw(end),tw(end)*100);
[accX,accY,accZ]= AccRaw_2_Gforce_35EE(X_acc_raw_35EE,Y_acc_raw_35EE,Z_acc_raw_35EE);
accTrue = [accX, accY, accZ];
accTrue = interp1(tw,accTrue,tnew);
%Resample acc data to 30 Hz
accTrue = [decimate(accTrue(:,1),3),decimate(accTrue(:,2),3),decimate(accTrue(:,3),3)];
Fs = 100/3;
%High pass filter (cutoff 0.2 Hz)
fc = 0.2; [b,a] = butter(2,fc/(Fs/2),'high');
accTrue = filtfilt(b,a,accTrue);
figure('Name',Folder(7:end)), hold on, 
subplot(121), plot(accTrue,'LineWidth',2), legend('X','Y','Z'), title('Accelerometer- Low pass filtered')

%acc from Kinect in body frame
Tsk = 1/30;
%filter position data on each axis
% fc = 10; [b,a] = butter(2,fc/(Fs/2),'low');
% Waistxyz_b = [filtfilt(b,a,Waistxyz_b(1,:))' filtfilt(b,a,Waistxyz_b(2,:))' filtfilt(b,a,Waistxyz_b(3,:))']; 
vbody = [diff(Waistxyz_b(:,1)), diff(Waistxyz_b(:,2)), diff(Waistxyz_b(:,3))]./Tsk;
%filter velocity
vbody = [smooth(vbody(:,1)) smooth(vbody(:,2)) smooth(vbody(:,3))]; 
% vbody = [filtfilt(b,a,vbody(:,1)) filtfilt(b,a,vbody(:,2)) filtfilt(b,a,vbody(:,3))]; 

accbody = diff(vbody)./Tsk;
accbody = [smooth(accbody(:,1)) smooth(accbody(:,2)) smooth(accbody(:,3))]; 

subplot(122), plot(accbody./9.81,'.-','LineWidth',2), legend('X','Y','Z'), title('Body acc')

   
    