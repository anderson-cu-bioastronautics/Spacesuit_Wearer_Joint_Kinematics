function [ Hxk ] = getHxksimp( om, T )
%getHxk Get linearized discrete-time state transition matrix
%   Detailed explanation goes here

Hxk = [zeros(4) getHxk(om, 2*T); eye(4) zeros(4)];

end

