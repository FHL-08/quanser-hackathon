clc;
clear;
close all

import casadi.*

% number of joint of the n-link arm
n_joints = 4;

% Symbolic variables for the robot's state
q = SX.sym('q', [n_joints, 1]);       % Joint positions

%% Robot Link Parameters
l_1 = 0.14;
l_2 = 0.35;
l_3 = 0.05;
lambda_2 = norm([l_2; l_3]);
l_4 = 0.25;
l_5 = 0.15;
lambda_3 = l_4 + l_5;
beta = deg2rad(8.13);

%% Robot Forward Kinematics (Denavit-Hartenberg Parameters)
% Standard DH parameters [d, theta_offset, a, alpha]
% Defined for the Quanser QArm robot from the datasheet

dh_table = [l_1,           0,        0, -pi/2;     % Bto1
              0, (beta-pi/2), lambda_2,     0;     % 1to2
              0,       -beta,        0, -pi/2;     % 2to3
            l_4,           0,        0,     0;     % 3to4
            l_5,           0,        0,     0      % 4toEE
           ];

% Initial Homogenous Transformation Matrix
H = eye(4);

%% Recursive Homogenous Transformation
for i = 1:n_joints+1
    % Extract DH parameters for the current link
    d = dh_table(i, 1);
    theta_offset = dh_table(i, 2);
    a = dh_table(i, 3);
    alpha = dh_table(i, 4);

    % Homogeneous transformation matrix using the standard DH convention
    if i <= n_joints
        theta = q(i) + theta_offset;
    else
        theta = theta_offset;
    end
    H_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                    0,             sin(alpha),             cos(alpha),            d;
                    0,                      0,                      0,            1;
          ];
    H = H*H_i; % frame i to frame 0
end

% Angle-Axis Orientation
R_ee = H(1:3, 1:3);
end_effector_rotation_func = Function('end_effector_rotation_func', {q}, {R_ee});

% End Effector's position in frame 0
world_frame_EE_position = H(1:3, 4);

% End Effector's orientation using Euler angles in frame 0
roll  = atan2(R_ee(3,2), R_ee(3,3));
pitch = asin(-R_ee(3,1));
yaw   = atan2(R_ee(2,1), R_ee(1,1));
world_frame_EE_orientation = [roll; pitch; yaw];

cartesian_state = [world_frame_EE_position; world_frame_EE_orientation];
cartesian_state_func = Function('cartesian_state_func', {q}, {cartesian_state});
% Example usage: cart_state = full(cartesian_state_func([0.2; 0.1; 0.05; 0]);

%% Inverse Kinematics
p_EE = SX.sym('p_EE', [3, 1]);
IK = SX.zeros(n_joints, 1);

IK(1) = atan2(p_EE(2), p_EE(1));

p_2_to_EE = p_EE - [0; 0; l_1];
lambda_4 = norm(p_2_to_EE);
delta_now = acos((lambda_2^2 + lambda_3^2 - lambda_4^2)/(2*lambda_2*lambda_3));
IK(3) = pi/2 + (beta - delta_now);

D = norm(p_2_to_EE(1:2));
r = norm(p_2_to_EE(:));
psi = atan2(p_2_to_EE(3), D);
phi = asin(lambda_3*sin(delta_now)/r);
IK(2) = (pi/2 - beta) - (phi + psi);

IK(4) = -pi/2;

IK_func = Function('IK_func', {p_EE}, {IK});
% Example usage: joint_angles = full(IK_func([0.22; 0.13; 0.07]);

% Can ignore this function. It just checks whether the end-effector position is reachable.
function IK = inverse_kinematics(p_EE)
    EE_position = p_EE(1:3);
    IK_temp = full(IK_func(EE_position));
    joint_UB = deg2rad([170; 85; 75; 160]);
    joint_LB = deg2rad(-[170; 85; 75; 160]);
    all_in_bounds = ((IK_temp >= joint_LB) & (IK_temp <= joint_UB));
    reachable = (norm(EE_position - [l_1; 0; 0]) <= 0.7536);
    if (all_in_bounds && reachable)
        IK = IK_temp;
        disp("End-Effector Position is reachable.\n");
    else
        IK = zeros(4, 1);
        disp("End-Effector Position is NOT reachable.\n");
    end
end