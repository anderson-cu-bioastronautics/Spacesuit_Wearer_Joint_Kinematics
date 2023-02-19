function [ C ] = q2C( q )
%q2C Given attitude quaternion, convert to right-handed orthogonal rotation
%matrix
%   Detailed explanation goes here
C = (1 - 2*q(1:3)'*q(1:3))*eye(3) + 2*q(1:3)*q(1:3)' - 2*q(4)*crossm(q(1:3));

end

