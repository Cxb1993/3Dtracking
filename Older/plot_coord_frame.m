function [ ] = plot_coord_frame( H )
%PLOT_COORD_FRAME Summary of this function goes here
%   Make sure hold on has been called before you do this. 
    
    % trans = 4x1
    trans = H * [0 0 0 1]';
    % x, y, and z of the origin
    x_o = trans(1);
    y_o = trans(2);
    z_o = trans(3);
    
    x_1 = H * [1 0 0 1]'; 
    y_1 = H * [0 1 0 1]'; 
    z_1 = H * [0 0 1 1]';
    
    % Plot axes for the origin
    plot3([x_o x_1(1)],[y_o x_1(2)], [z_o x_1(3)], 'blue');
    plot3([x_o y_1(1)],[y_o y_1(2)], [z_o y_1(3)], 'green');
    plot3([x_o z_1(1)],[y_o z_1(2)], [z_o z_1(3)], 'red');
    
    
    % Plot axes for the origin
    %plot3([0 1],[0 0], [0 0], 'blue');
    %plot3([0 0],[0 1], [0 0], 'green');
    %plot3([0 0],[0 0], [0 1], 'red');


end

