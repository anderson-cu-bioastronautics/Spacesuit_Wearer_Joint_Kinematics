function [ Gxk ] = getGxk( x )
%Gx Linearized discrete-time observation model
%   Detailed explanation goes here

% Compute range
rho = norm(x(1:2) - x(4:5));

% Compute Gx
Gxk = [(x(1) - x(4))/rho (x(2) - x(5))/rho 0 (x(4) - x(1))/rho (x(5) - x(2))/rho 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0];

end

