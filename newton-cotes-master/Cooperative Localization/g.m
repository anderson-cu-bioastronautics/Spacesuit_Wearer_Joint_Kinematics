function [ y ] = gcoop( x )
%gcoop Continuous-time nonlinear observation model for coop localization
%problem
%   Detailed explanation goes here
y = [norm(x(1:2) - x(4:5));
    x(4);
    x(5)];

end

