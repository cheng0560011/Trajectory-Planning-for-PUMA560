
% this function is used to convert matrix of DH model (4x4 matrix for n, o, a, p of each column) into a vector of
% the following variables cartesianVector = [x, y, z, phy, theta, psi]
% last modified by YuTung, Cheng
% last modified Jun 14th, 2018

function cartesianVector = DHconvert2cartesian(T)
%==============Calculate the Euler Angle and check for degeneration============%
cartesianVector = zeros(1,6);

cartesianVector(1,1) = T(1,4); % x
cartesianVector(1,2) = T(2,4); % y
cartesianVector(1,3) = T(3,4); % z

% for the three Euler Angle of matrix T
if abs(T(1,3)) == 0 && abs(T(2,3) == 0)
    % for degenerate condition
    % degenerate happens when sin_theta = 0, which means theta = 0 
    % or theta = 180
    
    if T(3,3)>0
        cartesianVector(1,5) = 0; % theta = 0
    else
        cartesianVector(1,5) = pi; % theta = 180
    end
    
    % in degenerate condition we set psi as 0
    cartesianVector(1,6) = 0;
    
    % solve phy
    cartesianVector(1,4) = atan2((-1)*T(1,2),T(2,2));
        
else
        % not degenerate
        % atan2(Y,X)
        cartesianVector(1,4) = atan2(T(2,3),T(1,3)); % phy  
        cartesianVector(1,5) = atan2( (cos(cartesianVector(1,4)*T(1,3)) + sin(cartesianVector(1,4))*T(2,3)) , T(3,3) ); % theta
        cartesianVector(1,6) = atan2( -sin(cartesianVector(1,4))*T(1,1)+cos(cartesianVector(1,4))*T(2,1) , -sin(cartesianVector(1,4))*T(1,2)+cos(cartesianVector(1,4))*T(2,2) ); %psi
    
end