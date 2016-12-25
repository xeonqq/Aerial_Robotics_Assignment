function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% FILL IN YOUR CODE HERE
err_pos = des_state.pos - state.pos;
err_vel = des_state.vel - state.vel;

kvz = 10;
kpz = 42;

kvy = 10;
kpy = 25;

kvphi = 6.27*1/ 8 * 40;
kpphi = 1*0.8 * 500;

phi_ddot_des = 0;
phi_dot_des = 0;


u1 = params.mass * (params.gravity + des_state.acc(2) + err_pos(2) * kpz + err_vel(2)*kvz);

%dt = 0.01;

phi_des = -(des_state.acc(1) + kpy * err_pos(1) + err_vel(1)*kvy)/params.gravity;

%phi_des = -pi/6;
err_phi = phi_des - state.rot;
err_phi_dot = phi_dot_des - state.omega;


u2 = params.Ixx * (phi_ddot_des + kpphi* err_phi + kvphi*err_phi_dot);




end

