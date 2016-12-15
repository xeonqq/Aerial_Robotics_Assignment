function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% FILL IN YOUR CODE HERE
kp = 42;
kd = 8;
persistent vel_des_prev;

if isempty(vel_des_prev)
	vel_des_prev = 0;
end


dt = 0.01;

e = s_des - s;

% e(1) error in z
% e(2) error in v_z
vel_des = s_des(2);

ff = (vel_des - vel_des_prev)/dt;

u = params.mass * (ff + kp*e(1) + kd*e(2) + params.gravity);

vel_des_prev = vel_des;

end

