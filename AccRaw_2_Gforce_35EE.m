function [NormX,NormY,NormZ]= AccRaw_2_Gforce_35EE(X_acc,Y_acc,Z_acc)

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% Function to convert raw acceleration data to g-scale
% input:
%   - X_acc: x-Acceleration data (raw from 35EE)
%   - Y_acc: y-Acceleration data (raw from 35EE)
%   - Z_acc: z-Acceleration data (raw from 35EE)
% output:
%   - NormX: x-Acceleration data (g-scale)
%   - NormY: y-Acceleration data (g-scale)
%   - NormZ: z-Acceleration data (g-scale)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%Convert from raw measurement to normalized measurement
%Acceleromeater measures from -4g to 4g, sensitivity (300 mV/g)
%Acc range [0 4096] 
%Zero-g voltage = Vdd/2 = 3.3/2 V = 1.65 V (2047)
%+1g --> ~2419 (> Vdd/2)
%+0g --> ~2047 (Vdd/2)
%-1g --> ~1674 (< Vdd/2)
%normalized measurement: 
%1)remove the Zero-g voltage (-2047), so that the Acc value are centered in 0
%2)From the dogital value to g (/372)
%[NormX NormY NormZ] are in g
NormX = (X_acc-2047+84)/372;   % The coefficient 84 derives from the calibration of accelerometer
NormY = (Y_acc-2047)/372;
NormZ = (Z_acc-2047+12)/372;

%Remove saturation
NormX(NormX >  4) = 4;
NormX(NormX < -4) = -4;

NormY(NormY >  4) = 4;
NormY(NormY < -4) = -4;

NormZ(NormZ >  4) = 4;
NormZ(NormZ < -4) = -4;