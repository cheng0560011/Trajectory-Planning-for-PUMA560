function T = forward_Kinematics_T( joint_variables )
% This function is used for PUMA560 with known Kinematic table
% This function take an array of six joint variables in rads and will
% return the x, y, z, phy, theta, psi Cartesian Points for the end-effector
% of this robot manipulator
% This function is constructed with the same process as which is used in
% project 1 of Robotics

% This function use the same code as robotic project 1
% last modified by YuTung, Cheng
% last modified Jun 14th, 2018

% joint_variables is an array for joint angles of each joint

% the Kinematic table is fixed for the puma560
KTable = [ 0      0      -pi/2    joint_variables(1,1);
           0      0.432   0       joint_variables(2,1);
           0.149  -0.02   pi/2    joint_variables(3,1);
           0.433  0       -pi/2   joint_variables(4,1);
           0      0       pi/2    joint_variables(5,1);
           0      0       0       joint_variables(6,1);];
       
A = zeros(4,4,6); % Ai matrix for each axis
                  % Ai all shown in D-H model
T = zeros(4,4);
%==============Calculation of Transfer function by Kinematic Table============%

% KTable
% [ d_i, a_i, alpha_i, theta_i]

for i = 1:1:6
    A(:,:,i) = [cos(KTable(i,4))   -sin(KTable(i,4))*cos(KTable(i,3))    sin(KTable(i,4))*sin(KTable(i,3))  KTable(i,2)*cos(KTable(i,4));
                sin(KTable(i,4))    cos(KTable(i,4))*cos(KTable(i,3))   -cos(KTable(i,4))*sin(KTable(i,3))  KTable(i,2)*sin(KTable(i,4));
                0                   sin(KTable(i,3))                     cos(KTable(i,3))                   KTable(i,1);
                0                   0                                    0                                  1;];
            
    for j = 1:1:i
        if j == 1
            T(:,:,i) = A(:,:,1);            
        end
        
        if j>1;
            T(:,:,i) = T(:,:,i)*A(:,:,j);
        end
    end
end

T = T(:,:,6);  

end

