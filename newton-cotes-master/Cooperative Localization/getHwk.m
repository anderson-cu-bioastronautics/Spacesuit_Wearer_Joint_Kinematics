function [ Hwk ] = getHwk( x, u, L )
%Hw Linearized continuous-time input matrix
%   Detailed explanation goes here

% Unpack state and input
thg = x(3);
tha = x(6);
vg = u(1);
phig = u(2);

% Compute Hw
Hwk = [cos(thg) 0 0 0;
    sin(thg) 0 0 0;
    1/L*tan(phig) vg/L*sec(phig)^2 0 0;
    0 0 cos(tha) 0;
    0 0 sin(tha) 0;
    0 0 0 1];

end

