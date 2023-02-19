function [ Hwksimp ] = getHwksimp( q, om, T )
%getHwk2 Get linearized discrete-time input matrix
%   Detailed explanation goes here

dhdom = getHwk(q, om, T);

Hwksimp = [1/3*dhdom 4/3*dhdom 1/3*dhdom; zeros(4, 9)];

end