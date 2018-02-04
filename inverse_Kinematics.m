% This is a function of inverse Kinematics
% This function will convert a cartesian position of end effector into a
% set of joint variable of PUMA560 with a known and fixed Kinematic Table

% This is as the same code with the robotic project 1
% last modified by YuTung, Cheng
% last modified Jun 14th, 2018

function joint_variables = inverse_Kinematics(cartesian_position_DH)

% ===========Construction of the Kinematic Table=================%
% Kinematic Table for Puma560
%                  d      a       alpha    theta
KTable_PUMA560 = [ 0      0      -pi/2    0;
                   0      0.432   0       0;
                   0.149  -0.02   pi/2    0;
                   0.433  0       -pi/2   0;
                   0      0       pi/2    0;
                   0      0       0       0;];
d3 = 0.149;
d4 = 0.433;
a2 = 0.432;
a3 = -0.02;


% =============Calculate A1 to A6 of PUMA560=============%

A =zeros(4,4,6);
for i = 1:1:6
    A(:,:,i) = [cos(KTable_PUMA560(i,4))   -sin(KTable_PUMA560(i,4))*cos(KTable_PUMA560(i,3))    sin(KTable_PUMA560(i,4))*sin(KTable_PUMA560(i,3))  KTable_PUMA560(i,2)*cos(KTable_PUMA560(i,4));
                sin(KTable_PUMA560(i,4))    cos(KTable_PUMA560(i,4))*cos(KTable_PUMA560(i,3))   -cos(KTable_PUMA560(i,4))*sin(KTable_PUMA560(i,3))  KTable_PUMA560(i,2)*sin(KTable_PUMA560(i,4));
                0                           sin(KTable_PUMA560(i,3))                             cos(KTable_PUMA560(i,3))                           KTable_PUMA560(i,1);
                0                           0                                                    0                                                  1;];
            
    for j = 1:1:i
        if j == 1
            T(:,:,i) = A(:,:,1);            
        end
        
        if j>1;
            T(:,:,i) = T(:,:,i)*A(:,:,j);
        end
    end
end
               
               
               
% ===============User input Cartesian Point==================%
% T6 = [1 0 0 0.412;
%       0 1 0 0.149;
%       0 0 1 0.433;
%       0 0 0     1;]
  
T6 = cartesian_position_DH;

px = T6(1,4);
py = T6(2,4);
pz = T6(3,4);
 
% T6_ref4 = A(:,:,5)*A(:,:,6);
% T6_ref3 = A(:,:,4)*A(:,:,5)*A(:,:,6);

% ================ Slove theta1 to theta3 ==================== %
% theta1 to theta3 is about the point of end effector of the robot

% to solve two configurations of theta1
% known d3 = 0.149 previously
theta1_1 = (atan2(T6(2,4),T6(1,4))-atan2(d3,(T6(1,4)^2+T6(2,4)^2-d3^2)^(1/2)));
theta1_2 = (atan2(T6(2,4),T6(1,4))-atan2(d3,-(T6(1,4)^2+T6(2,4)^2-d3^2)^(1/2)));

% to solve two configurations of theta3
% px=0.412; % testing
% py=0.149; % testing
% pz=0.433; % testing
d3 = 0.149;
d4 = 0.433;
a2 = 0.432;
a3 = -0.02;
M = (px^2 + py^2 + pz^2 - a2^2 - a3^2 - d3^2 - d4^2) / (2*a2);
inner_sqrt = sqrt(a3^2+d4^2-M^2);
theta3_1 = atan2( M, inner_sqrt)- atan2(a3,d4) ;
theta3_2 = atan2( M, (-1)*inner_sqrt) - atan2(a3,d4);

% to solve four configurations of theta2
theta23_1 = atan2( d4+a2*sin(theta3_1) , a3 + a2*cos(theta3_1) ) - atan2 (pz, cos(theta1_1)*px + sin(theta1_1)*py );
theta2_1 = theta23_1 - theta3_1;

theta23_2 = atan2( d4+a2*sin(theta3_2) , a3 + a2*cos(theta3_2) ) - atan2 (pz, cos(theta1_1)*px + sin(theta1_1)*py );
theta2_2 = theta23_2 - theta3_2;

theta23_3 = atan2( d4+a2*sin(theta3_1) , a3 + a2*cos(theta3_1) ) - atan2 (pz, cos(theta1_2)*px + sin(theta1_2)*py );
theta2_3 = theta23_3 - theta3_1;

theta23_4 = atan2( d4+a2*sin(theta3_2) , a3 + a2*cos(theta3_2) ) - atan2 (pz, cos(theta1_2)*px + sin(theta1_2)*py );
theta2_4 = theta23_4 - theta3_2;

% ================== Solve theta 4 to theta 6 ,for the combinations of the theta1 to theta3 =========================%
% ============ Calculate dynamics for each combination of theta1,2,3 ====%
% when solving theta 4,5,6, all combinations of theta 1,2,3 should be apply to A1, A2, A3
% 
A11 = [cos(theta1_1)   -sin(theta1_1)*cos(KTable_PUMA560(1,3))    sin(theta1_1)*sin(KTable_PUMA560(1,3))  KTable_PUMA560(1,2)*cos(theta1_1);
       sin(theta1_1)    cos(theta1_1)*cos(KTable_PUMA560(1,3))   -cos(theta1_1)*sin(KTable_PUMA560(1,3))  KTable_PUMA560(1,2)*sin(theta1_1);
           0            sin(KTable_PUMA560(1,3))                  cos(KTable_PUMA560(1,3))                KTable_PUMA560(1,1);
           0                   0                                    0                                     1;];
A12 = [cos(theta1_2)   -sin(theta1_2)*cos(KTable_PUMA560(1,3))    sin(theta1_2)*sin(KTable_PUMA560(1,3))  KTable_PUMA560(1,2)*cos(theta1_2);
       sin(theta1_2)    cos(theta1_2)*cos(KTable_PUMA560(1,3))   -cos(theta1_2)*sin(KTable_PUMA560(1,3))  KTable_PUMA560(1,2)*sin(theta1_2);
           0            sin(KTable_PUMA560(1,3))                  cos(KTable_PUMA560(1,3))                KTable_PUMA560(1,1);
           0                   0                                    0                                     1;];  
A21 = [cos(theta2_1)   -sin(theta2_1)*cos(KTable_PUMA560(2,3))    sin(theta2_1)*sin(KTable_PUMA560(2,3))  KTable_PUMA560(2,2)*cos(theta2_1);
       sin(theta2_1)    cos(theta2_1)*cos(KTable_PUMA560(2,3))   -cos(theta2_1)*sin(KTable_PUMA560(2,3))  KTable_PUMA560(2,2)*sin(theta2_1);
           0            sin(KTable_PUMA560(2,3))                  cos(KTable_PUMA560(2,3))                KTable_PUMA560(2,1);
           0                   0                                    0                                     1;];     
A22 = [cos(theta2_2)   -sin(theta2_2)*cos(KTable_PUMA560(2,3))    sin(theta2_2)*sin(KTable_PUMA560(2,3))  KTable_PUMA560(2,2)*cos(theta2_2);
       sin(theta2_2)    cos(theta2_2)*cos(KTable_PUMA560(2,3))   -cos(theta2_2)*sin(KTable_PUMA560(2,3))  KTable_PUMA560(2,2)*sin(theta2_2);
           0            sin(KTable_PUMA560(2,3))                  cos(KTable_PUMA560(2,3))                KTable_PUMA560(2,1);
           0                   0                                    0                                     1;];     
A23 = [cos(theta2_3)   -sin(theta2_3)*cos(KTable_PUMA560(2,3))    sin(theta2_3)*sin(KTable_PUMA560(2,3))  KTable_PUMA560(2,2)*cos(theta2_3);
       sin(theta2_3)    cos(theta2_3)*cos(KTable_PUMA560(2,3))   -cos(theta2_3)*sin(KTable_PUMA560(2,3))  KTable_PUMA560(2,2)*sin(theta2_3);
           0            sin(KTable_PUMA560(2,3))                  cos(KTable_PUMA560(2,3))                KTable_PUMA560(2,1);
           0                   0                                    0                                     1;];            
A24 = [cos(theta2_4)   -sin(theta2_4)*cos(KTable_PUMA560(2,3))    sin(theta2_4)*sin(KTable_PUMA560(2,3))  KTable_PUMA560(2,2)*cos(theta2_4);
       sin(theta2_4)    cos(theta2_4)*cos(KTable_PUMA560(2,3))   -cos(theta2_4)*sin(KTable_PUMA560(2,3))  KTable_PUMA560(2,2)*sin(theta2_4);
           0            sin(KTable_PUMA560(2,3))                  cos(KTable_PUMA560(2,3))                KTable_PUMA560(2,1);
           0                   0                                    0                                     1;];          
A31 = [cos(theta3_1)   -sin(theta3_1)*cos(KTable_PUMA560(3,3))    sin(theta3_1)*sin(KTable_PUMA560(3,3))  KTable_PUMA560(3,2)*cos(theta3_1);
       sin(theta3_1)    cos(theta3_1)*cos(KTable_PUMA560(3,3))   -cos(theta3_1)*sin(KTable_PUMA560(3,3))  KTable_PUMA560(3,2)*sin(theta3_1);
           0            sin(KTable_PUMA560(3,3))                  cos(KTable_PUMA560(3,3))                KTable_PUMA560(3,1);
           0                   0                                    0                                     1;];          
A32 = [cos(theta3_2)   -sin(theta3_2)*cos(KTable_PUMA560(3,3))    sin(theta3_2)*sin(KTable_PUMA560(3,3))  KTable_PUMA560(3,2)*cos(theta3_2);
       sin(theta3_2)    cos(theta3_2)*cos(KTable_PUMA560(3,3))   -cos(theta3_2)*sin(KTable_PUMA560(3,3))  KTable_PUMA560(3,2)*sin(theta3_2);
           0            sin(KTable_PUMA560(3,3))                  cos(KTable_PUMA560(3,3))                KTable_PUMA560(3,1);
           0                   0                                    0                                     1;];          


% =============Solving the wrist joint of joint 4,5,6=================%
% Reference of solving Euler Angles
%
EulerAngle_Invk = zeros(8,3);
T6_ref3_array=zeros(4,4,4);
T6_ref3_array(:,:,1) = inv(A31)*inv(A21)*inv(A11)*T6;
T6_ref3_array(:,:,2) = inv(A32)*inv(A22)*inv(A11)*T6;
T6_ref3_array(:,:,3) = inv(A31)*inv(A23)*inv(A12)*T6;
T6_ref3_array(:,:,4) = inv(A32)*inv(A24)*inv(A12)*T6;

j = 1;
for i = 1:1:4
    T6_ref3 = T6_ref3_array(:,:,i);
    
    if abs(T6_ref3(3,3)) == 1
    % degenerate when cos of 1
    % theta 5 == 0 makes degenerate
    %set(handles.text_status,'String','degenerate');
    
        if T6_ref3(3,3)>0
            EulerAngle_Invk(j,2) = 0;
        else
            EulerAngle_Invk(j,2) = pi;
        end
    
    % in degenerate condition we set psi as 0
        EulerAngle_Invk(j,3) = 0;
    
    % use atan2 to solve phy
%         if ((T6_ref3(2,1)==0) && (T6_ref3(3,1)==0))
%             EulerAngle_Invk(j,1) = 0;
%         else
%             EulerAngle_Invk(j,1) = atan2(T6_ref3(2,1),T6_ref3(3,1));
%         end

        EulerAngle_Invk(j,1) = atan2(T6_ref3(1,2),T6_ref3(1,1));

        EulerAngle_Invk(j+1,:) = EulerAngle_Invk(j,:);
        EulerAngle_Invk(j+1,1) = EulerAngle_Invk(j+1,1) + pi;
        
    else
    % not degenerate
    %set(handles.text_status,'String','not degenerate');
    
        % column 1 of EulerAngle is phy
        EulerAngle_Invk(j,1) = atan2(T6_ref3(2,3),T6_ref3(1,3));
        EulerAngle_Invk(j+1,1) = atan2(T6_ref3(2,3),T6_ref3(1,3))+pi;
 
        % cloumn 2 of EulerAngle is theta
        EulerAngle_Invk(j,2) = atan2((cos(EulerAngle_Invk(j,1))*T6_ref3(1,3)+sin(EulerAngle_Invk(j,1))*T6_ref3(2,3)),T6_ref3(3,3));
        EulerAngle_Invk(j+1,2) = atan2(cos(EulerAngle_Invk(j+1,1))*T6_ref3(1,3)+sin(EulerAngle_Invk(j+1,1))*T6_ref3(2,3),T6_ref3(3,3));
        
        % cloumn 3 of EulerAngle psi
        %EulerAngle_Invk(j,3) = atan2(  (-sin(EulerAngle_Invk(j,1))*T6_ref3(1,1) + cos(EulerAngle_Invk(j,1))*T6_ref3(2,1)) ,  -sin(EulerAngle_Invk(j,1)*T6_ref3(1,2)+cos(EulerAngle_Invk(j,1))*T6_ref3(2,2) )  );
        %EulerAngle_Invk(j+1,3) = atan2(  -sin(EulerAngle_Invk(j+1,1))*T6_ref3(1,1) + cos(EulerAngle_Invk(j+1,1))*T6_ref3(2,1) , -sin(EulerAngle_Invk(j+1,1))*T6_ref3(1,2)+cos(EulerAngle_Invk(j+1,1))*T6_ref3(2,2)  );
        EulerAngle_Invk(j,3) = atan2( -sin(EulerAngle_Invk(j,1))*T6_ref3(1,1)+cos(EulerAngle_Invk(j,1))*T6_ref3(2,1) , -sin(EulerAngle_Invk(j,1))*T6_ref3(1,2)+cos(EulerAngle_Invk(j,1))*T6_ref3(2,2));
        EulerAngle_Invk(j+1,3) = atan2( -sin(EulerAngle_Invk(j+1,1))*T6_ref3(1,1)+cos(EulerAngle_Invk(j+1,1))*T6_ref3(2,1) , -sin(EulerAngle_Invk(j+1,1))*T6_ref3(1,2)+cos(EulerAngle_Invk(j+1,1))*T6_ref3(2,2));        
    end
    
    j = j+2;
    
end   
       
% ===============output and show result of Inverse Kinematics======== %           

Result = zeros(8,6);

Result(1,:) = [ theta1_1, theta2_1, theta3_1, EulerAngle_Invk(1,1), EulerAngle_Invk(1,2), EulerAngle_Invk(1,3) ];
Result(2,:) = [ theta1_1, theta2_1, theta3_1, EulerAngle_Invk(2,1), EulerAngle_Invk(2,2), EulerAngle_Invk(2,3) ];
Result(3,:) = [ theta1_1, theta2_2, theta3_2, EulerAngle_Invk(3,1), EulerAngle_Invk(3,2), EulerAngle_Invk(3,3) ];
Result(4,:) = [ theta1_1, theta2_2, theta3_2, EulerAngle_Invk(4,1), EulerAngle_Invk(4,2), EulerAngle_Invk(4,3) ];
Result(5,:) = [ theta1_2, theta2_3, theta3_1, EulerAngle_Invk(5,1), EulerAngle_Invk(5,2), EulerAngle_Invk(5,3) ];
Result(6,:) = [ theta1_2, theta2_3, theta3_1, EulerAngle_Invk(6,1), EulerAngle_Invk(6,2), EulerAngle_Invk(6,3) ];
Result(7,:) = [ theta1_2, theta2_4, theta3_2, EulerAngle_Invk(7,1), EulerAngle_Invk(7,2), EulerAngle_Invk(7,3) ];
Result(8,:) = [ theta1_2, theta2_4, theta3_2, EulerAngle_Invk(8,1), EulerAngle_Invk(8,2), EulerAngle_Invk(8,3) ];

% Result is the joint variables shown in rads

% to normalize the angle and let it between 0 to 360
% in rads its 0 to 2*pi

for i = 1:1:8
    for j = 1:1:6
        if Result(i,j) > 2*pi
            Result(i,j) = rem(i,2*pi);
        end
        if Result(i,j) > pi
            Result(i,j) = Result(i,j)-2*pi;
        end
        
    end
end
            

% ===============check if the result is out of ragne================== %

OutofRange = zeros(8,6);

% before checking for available, changed it from rad to degree
Result = Result*(180/pi);
for i = 1:1:8
    
    if abs(Result(i,1))>160
        OutofRange(i,1) = 1;
    else
        OutofRange(i,1) = 0;
    end 

    if abs(Result(i,2))>125
        OutofRange(i,2) = 1;
    else
        OutofRange(i,2) = 0;
    end     

    if abs(Result(i,3))>135
        OutofRange(i,3) = 1;
    else
        OutofRange(i,3) = 0;
    end 
    if abs(Result(i,4))>140
        OutofRange(i,4) = 1;
    else
        OutofRange(i,4) = 0;
    end 

    if abs(Result(i,5))>100
        OutofRange(i,5) = 1;
    else
        OutofRange(i,5) = 0;
    end 

    if abs(Result(i,6))>260
        OutofRange(i,6) = 1;
    else
        OutofRange(i,1) = 0;
    end 
    
end


%===================== Show Result on GUI ==================%
showResult = zeros(16,6);
for i = 1:1:8
    showResult(2*i-1,:) = Result(i,:);
    showResult(2*i,:) = OutofRange(i,:);
end

% ===================Choose a set as result ================ %
%joint_variables = Result

% if there is a set for all not out of range, show it as result
% if more than one set of result, this program shows the first one
% if there is no set for all not out of range, show it as [-1, -1, -1, -1, -1, -1]

% initialize th joint_variables as [-1 -1 -1 -1 -1 -1]
% return [-1, -1, -1, -1, -1, -1 ] to shows this cartesian is not available
% for this robot manipulator
joint_variables = [-1 -1 -1 -1 -1 -1];

% if there is legal joint_variables it will re
for i = 1:1:8
    outofRange = sum(OutofRange(i,:),2);
    if(sum(OutofRange(i,:),2)) == 0
        % this set of answer is legal
        joint_variables = Result(i,:);
        break;
    end
end




