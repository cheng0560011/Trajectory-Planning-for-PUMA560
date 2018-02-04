% This function is used for PUMA560 with known Kinematic table
% This function take an array of six joint variables in rads and will
% return the x, y, z, phy, theta, psi Cartesian Points for the end-effector
% of this robot manipulator
% This function is constructed with the same process as which is used in
% project 1 of Robotics

% This function use the same code as robotic project 1
% last modified by YuTung, Cheng
% last modified Jun 14th, 2018

function cartesianVector = foward_Kinematics(joint_variables)

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


T6 = zeros(4,4);
T6 = T(:,:,6);    % T6 is the D-H model result of the end-effector

% assign values for x, y, z, for end-effector
x = T6(1,4);
y = T6(2,4);
z = T6(3,4);

%==============Calculate the Euler Angle and check for degeneration============%

EulerAngle = zeros(1,3);
if T6(abs(T6(1,3)) == 0 && abs(T6(2,3) == 0))
    % for degenerate condition
    % degenerate happens when sin_theta = 0, which means theta = 0 or theta
    % = 180
    
    if T6(3,3)>0
        EulerAngle(1,2) = 0; % theta = 0
    else
        EulerAngle(1,2) = 180; % theta = 180
    end
    
    % in degenerate condition we set psi as 0
    EulerAngle(1,3) = 0;
    
    % use atan2 to solve phy
    if ((T6(2,1)==0) && (T6(1,1)==0))
        EulerAngle(1,1) = 0;
    else
        EulerAngle(1,1) = atan2(T6(2,1),T6(1,1));
    end
    
    %for i = 1:1:3
        %EulerAngle(2,i) = EulerAngle(1,i);    
    %end
    
else
        % not degenerate
            
        EulerAngle(1,1) = atan2(T6(2,3),T6(1,3));  % atan2(Y,X)
    
        if (cos(EulerAngle(1,1))*T6(1,3)+sin(EulerAngle(1,1))*T6(2,3))*T6(3,3) == 0
            EulerAngle(1,2) = 0;     
        else
            EulerAngle(1,2) = atan2((cos(EulerAngle(1,1))*T6(1,3)+sin(EulerAngle(1,1))*T6(2,3)),T6(3,3));
        end
    
        if (-sin(EulerAngle(1,1))*T6(1,1)+cos(EulerAngle(1,1))*T6(2,1))*( -sin(EulerAngle(1,1)*T6(1,2))+cos(EulerAngle(1,1))*T6(2,2) ) == 0
            EulerAngle(1,3) = 0;

        else
            EulerAngle(1,3) = atan2(  (-sin(EulerAngle(1,1))*T6(1,1)+cos(EulerAngle(1,1))*T6(2,1)) , ( -sin(EulerAngle(1,1)*T6(1,2))+cos(EulerAngle(1,1))*T6(2,2) )  );
        
        end
    
end

phy = EulerAngle(1,1);
theta = EulerAngle(1,2);
psi = EulerAngle(1,3);

cartesianVector = [x, y, z, phy, theta, psi];