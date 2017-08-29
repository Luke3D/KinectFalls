function [Rx,Ry,Rz] = RotMat(th)
%coord transformation
Rx = [1 0 0; 0 cos(th) -sin(th);0 sin(th) cos(th)]; %Rx(90)
Ry = [cos(th) 0 sin(th);0 1 0;-sin(th) 0 cos(th)]; %Ry(90)
Rz = [cos(th) -sin(th) 0;sin(th) cos(th) 0;0 0 1];

%Example :point in accelerometer space
% th = pi/2;
% pk = [0 0 1]'; %point in kinect space
% pa = Ry*Rx*pk;


