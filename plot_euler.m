% this function is for plotting the Euler angle for a input point with
% direction
% input should be a T matirx of [n, o, a, p] from forward kinematics
% Last modified by YuTung, Cheng
% Last modified Jun 14th, 2018

function plot_euler(T)

    % for T = [n, o, a, p]
    % plot euler angle for the point

    position = zeros(3,1);
    position = [T(1,4); T(2,4); T(3,4)];
    
    euler_n = position + 0.1*T(1:3,1);
    euler_o = position + 0.1*T(1:3,2);    
    euler_a = position + 0.1*T(1:3,3);
    
    plot3([position(1);euler_n(1)],[position(2);euler_n(2)],[position(3);euler_n(3)],'r');hold on;
    plot3([position(1);euler_o(1)],[position(2);euler_o(2)],[position(3);euler_o(3)],'g');hold on;
    plot3([position(1);euler_a(1)],[position(2);euler_a(2)],[position(3);euler_a(3)],'b');       
end

