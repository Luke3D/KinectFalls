%Place this code in the same folder of the database (Data1,Data2,...Data11)
clear all;
% close all;
clc;

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % Choose:
startFoldIdx = 1;   % Index of the first test folder to load (1->'Data1')
stopFoldIdx = 2;    % Index of the last test folder to load (11->'Data11')
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%The script loads:
% - PckVal_*:       packet from wearable device
% - TStamp_QPC_*:   timestamp assigned by the PC to the packet, when it arrives to the PC [us] 
% - TStamp_Int_*:   internal timestamp of the wearable device [ms]
% - *_acc_raw_*:      acceleration data
% - M:              depth frame
% - KinectTime:     timestamps assigned by the PC to the frame, when it is available to the PC [100ns]/[us]
% - jMatDep:        skeleton joints in depth space
% - jMatSkl:        skeleton joints in skeleton space
% - KinectTimeBody: timestamps assigned by the PC to the skeleton, when it is available to the PC [100ns]/[us]

%     JointType_SpineBase = 0,
%     JointType_SpineMid = 1,
%     JointType_Neck = 2,
%     JointType_Head = 3,
%     JointType_ShoulderLeft = 4,
%     JointType_ElbowLeft = 5,
%     JointType_WristLeft = 6,
%     JointType_HandLeft = 7,
%     JointType_ShoulderRight = 8,
%     JointType_ElbowRight = 9,
%     JointType_WristRight = 10,
%     JointType_HandRight = 11,
%     JointType_HipLeft = 12,
%     JointType_KneeLeft = 13,
%     JointType_AnkleLeft = 14,
%     JointType_FootLeft = 15,
%     JointType_HipRight = 16,
%     JointType_KneeRight = 17,
%     JointType_AnkleRight = 18,
%     JointType_FootRight = 19,
%     JointType_SpineShoulder = 20,
%     JointType_HandTipLeft = 21,
%     JointType_ThumbLeft = 22,
%     JointType_HandTipRight = 23,
%     JointType_ThumbRight = 24,



ADLFolderName = {'sit','grasp','walk','lay'};
FallFolderName = {'front','back','side','EndUpSit'};
%*****Wearable device*****
device1 = '35EE'; %Number of device1
device2 = '36F9'; %Number of device2

%*******Kinect - V2*******
rowPixel = 424; %[pixel] number of row
columnPixel = 512;  %[pixel] number of column
frameIdx = 1; %depth frame number

% Load selected folders
for idx_folder = startFoldIdx:stopFoldIdx
    for groupName = {'ADL' 'Fall'}
        if strcmp(groupName,'ADL')
            subfolder = ADLFolderName;
        else
            subfolder = FallFolderName;
        end
        for name_Subfolder = subfolder
            for idx_test = 1:3
                Folder = strcat('Data',num2str(idx_folder),'/',cell2mat(groupName),'/',cell2mat(name_Subfolder),'/',num2str(idx_test)); %Folder where are stored the data
                disp(Folder)        
                
                %only load one trial
                if strcmp(Folder,'Data2/Fall/front/1')
                    
                %*************************
                %Load wearable device data
                %*************************
                %**35EE**
                PckVal_35EE = loadPackets(device1,Folder);
                PckValSize_35EE = size(PckVal_35EE);
                TStamp_QPC_35EE = csvread(strcat(Folder,'/Time/TimeStamps',device1,'.csv'));
                TStamp_Int_35EE = (PckVal_35EE(:,3)+256.*PckVal_35EE(:,4)+256.*256.*PckVal_35EE(:,5));
                %accelerometer values
                X_acc_raw_35EE = PckVal_35EE(:,8)+256*PckVal_35EE(:,9);
                Y_acc_raw_35EE = PckVal_35EE(:,10)+256*PckVal_35EE(:,11);
                Z_acc_raw_35EE = PckVal_35EE(:,12)+256*PckVal_35EE(:,13);
                %36F9
                PckVal_36F9 = loadPackets(device2,Folder);
                PckValSize_36F9 = size(PckVal_36F9);
                TStamp_QPC_36F9 = csvread(strcat(Folder,'/Time/TimeStamps',device2,'.csv'));
                TStamp_Int_36F9 = (PckVal_36F9(:,3)+256.*PckVal_36F9(:,4)+256.*256.*PckVal_36F9(:,5));
                %accelerometer values
                X_acc_raw_36F9 = PckVal_36F9(:,8)+256*PckVal_36F9(:,9);
                Y_acc_raw_36F9 = PckVal_36F9(:,10)+256*PckVal_36F9(:,11);
                Z_acc_raw_36F9 = PckVal_36F9(:,12)+256*PckVal_36F9(:,13);
                
                %*************************
                %****Load depth frame*****
                %*************************               
                nframes = length(dir(strcat(Folder,'/Depth/*.bin')));
                M = zeros(rowPixel, columnPixel,nframes);
                for frameIdx = 1:nframes
                    fid = fopen(strcat(Folder,'/Depth/Filedepth_',num2str(frameIdx-1),'.bin'));
                    arrayFrame = fread(fid,'uint16');
                    fclose(fid);
                    for r=1:rowPixel
                        M(r,:,frameIdx) = arrayFrame((r-1)*columnPixel+1:r*columnPixel);
                    end
                end
                                
                %Load time information
                KinectTime = csvread(strcat(Folder,'/Time/DepthTime.csv'));
                
                %*************************
                %******Load skeleton******
                %*************************
                fileNameSk1DS = strcat(Folder,'/Body','/Fileskeleton.csv'); %joint in the depth frame
                fileNameSk1SS = strcat(Folder,'/Body','/FileskeletonSkSpace.csv'); %joint in 3D space
                Sk1SkDepth = csvread(fileNameSk1DS);
                Sk1SkSpace = csvread(fileNameSk1SS);
                %Find player number
                for idx_player = 1:6
                    if Sk1SkDepth(25*(idx_player-1)+1,1) ~= 0
                        break;
                    end
                end
                %Find row index of the specific player in Sk1SkDepth
                row_idx = find(Sk1SkDepth(:,5) == idx_player-1);
                Sk1SkDepth = Sk1SkDepth(row_idx,:);
                NumFrameSkelDepth = fix(length(Sk1SkDepth(:,1))/25);
                Sk1SkSpace = Sk1SkSpace(row_idx,:);
                NumFrameSkelSpace = fix(length(Sk1SkSpace(:,1))/25);
                %restore 25 joints in groups 
                jMatDep = zeros(25, 3, NumFrameSkelDepth);
                jMatSkl = zeros(25, 3, NumFrameSkelSpace);
                for n = 1:NumFrameSkelDepth
                    jMatDep(:,:,n) = Sk1SkDepth(((n-1)*25+1):n*25,1:3);
                end
                for n = 1:NumFrameSkelSpace
                    jMatSkl(:,:,n) = Sk1SkSpace(((n-1)*25+1):n*25,1:3);
                end
                %Load time information
                KinectTimeBody = csvread(strcat(Folder,'/Time/BodyTime.csv'));
                
                % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
                % %accjoint = Hip Right Joint (17) or SpineMid (2)
                % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
                accjoint = 2; %reference joint for accelerometer position
                jMatSklX = reshape(jMatSkl(accjoint,1,:),[size(jMatSkl,3),1]);
                jMatSklY = reshape(jMatSkl(accjoint,2,:),[size(jMatSkl,3),1]);
                jMatSklZ = reshape(jMatSkl(accjoint,3,:),[size(jMatSkl,3),1]);
                jMatDepX = reshape(jMatDep(accjoint,1,:),[size(jMatDep,3),1]);
                jMatDepY = reshape(jMatDep(accjoint,2,:),[size(jMatDep,3),1]);
                jMatDepZ = reshape(jMatDep(accjoint,3,:),[size(jMatDep,3),1]);
                
                %Shoulder joint position (skeleton base)
                jSpineShX = reshape(jMatSkl(1,1,:),[size(jMatSkl,3),1]);
                jSpineShY = reshape(jMatSkl(1,2,:),[size(jMatSkl,3),1]);
                jSpineShZ = reshape(jMatSkl(1,3,:),[size(jMatSkl,3),1]);
                
                %Spinebase joint position (skeleton base)
                jSpineBX = reshape(jMatSkl(21,1,:),[size(jMatSkl,3),1]);
                jSpineBY = reshape(jMatSkl(21,2,:),[size(jMatSkl,3),1]);
                jSpineBZ = reshape(jMatSkl(21,3,:),[size(jMatSkl,3),1]);

                
                
                % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
                % Plot raw Acceleration data of each test   
                [NormX,NormY,NormZ]= AccRaw_2_Gforce_35EE(X_acc_raw_35EE,Y_acc_raw_35EE,Z_acc_raw_35EE);
                figure('Name',Folder(7:end))
                subplot 221; title('35EE'); hold on;
                plot(NormX,'b'); plot(NormY,'r'); plot(NormZ,'k');
                legend('X','Y','Z')
                subplot 222; title('36F9'); hold on;
                plot(X_acc_raw_36F9,'b'); plot(Y_acc_raw_36F9,'r'); plot(Z_acc_raw_36F9,'k');
                subplot 223; title('Skeleton space'); hold on;
                plot(jMatSklX,'b'); plot(jMatSklY,'r'); plot(jMatSklZ,'k'); 
                legend('X','Y','Z')
%                 plot(diff(diff(jMatSklX)),'b'); plot(diff(diff(jMatSklY)),'r'); plot(diff(diff(jMatSklZ)),'k');
                subplot 224; title('Depth space'); hold on;
                plot(jMatDepX,'b'); plot(jMatDepY,'r'); plot(jMatDepZ,'k');
%            
                %skeleton joints centered on base spine joint
                offset = jMatSkl(1,:,:);
                jMatSkl_o = jMatSkl - offset;  
                [Rx,Ry,~] = RotMat(pi/2);

                % Plot Depth and Skeleton data of each test
%                 close all
                figure;      
                subplot(131),
                subplot(132), axis manual, xlim([-800 400]),ylim([-1500 1500]),zlim([2000 6000])
                for t = 50:size(jMatSkl,3)
                    subplot 131;
                    imagesc(M(:,:,t)); title('depth frame');
                    
                    subplot 132, %cla(gca);
                    %plot skeleton in skeleton space (kinect space) 
%                     plot3(jMatSkl(:,1,t),jMatSkl(:,3,t),jMatSkl(:,2,t),'.r','markersize',20); view(90,0); axis equal;

                    %plot skeleton in skeleton space centered on base spine
                    plot3(jMatSkl(:,1,t)-jMatSkl(1,1,t), jMatSkl(:,3,t)-jMatSkl(1,3,t) ,jMatSkl(:,2,t)-jMatSkl(1,2,t),'.r','markersize',20); view(90,0); axis equal;                                                            
                    title(num2str(t)),xlabel('z'),ylabel('x')
                    hold off                    

%                     %title('skeleton in skeleton space');
                    subplot 133;hold on
% %                     xlim([-800 1200]), ylim([-1200 1200])
%                     plot3(jMatDep(:,1,t),jMatDep(:,3,t),jMatDep(:,2,t),'.b','markersize',20); view(0,0); axis equal; set(gca,'ZDir','Reverse');
                    %title('skeleton in depth space');
%                     pA_kinect = jMatSkl_o(21,:,t)'; 
%                     pA_BF = Ry*Rx*pA_kinect;    %spine shoulder in body fixed coord
%                     plot3(jMatSkl(1,1,t)-jMatSkl(1,1,t), jMatSkl(1,3,t)-jMatSkl(1,3,t) ,jMatSkl(1,2,t)-jMatSkl(1,2,t),'.b','markersize',25); view(90,0); axis equal;                    
                    
                    %show trunk trajectory (from base spine (1) to either spine mid (2) or shoulder (21) 
                    plot3(jMatSkl(1,1,t)-jMatSkl(1,1,t), jMatSkl(1,3,t)-jMatSkl(1,3,t) ,jMatSkl(1,2,t)-jMatSkl(1,2,t),'.b','markersize',25); view(90,0); axis equal;  %SPINE BASE                  
                    spinejoint = 2; %Use 21 for spine shoulder
                    plot3(jMatSkl(spinejoint,1,t)-jMatSkl(1,1,t), jMatSkl(spinejoint,3,t)-jMatSkl(1,3,t) ,jMatSkl(spinejoint,2,t)-jMatSkl(1,2,t),'.r','markersize',25); 
                    
                    hold off
                    xlabel('x'), ylabel('z'), zlabel('y'), %set(gca,'ZDir','Reverse');

                    drawnow
%                     w = waitforbuttonpress;
%                     pause(0.0001)
                end
                % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
                
                TrunkAngle
                end
            end
        end
    end
end


