function [ Hxk ] = getHxk( om, T )
%getHxk Get linearized discrete-time state transition matrix
%   Detailed explanation goes here

if norm(om) ~= 0
    dhdq = cos(norm(om)*T/2)*eye(4) + [-crossm(om) om; -om' 0]/norm(om)*sin(norm(om)*T/2);
else
    dhdq = eye(4);
end

Hxk = dhdq;

end

