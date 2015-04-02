run('get_measurement_model');
% This puts the following variables in scope:
%   symbolics = (sym)[ f, phi, theta, psi, pose_{x, y, z}, off_{x, y, z}
%   measurement_model = (sym) measurement model of...system. 
%               TODO pose vs. cad points? dimensions?
%   jacobian_of_mm = (sym) 
%               TODO dimension?
%   f = (sym) 1x1 focal length of camera




% Create an object that moves through space. 
%initial_pose = [0, 0, 0, 0, 0, 0];
%pose_after_10_seconds = [3, 3, 3, pi/2, 0, 0];
num_time_steps = 10;

pose_array = [linspace(0, 3, num_time_steps),
                linspace(0, 3, num_time_steps)
                linspace(0, 3, num_time_steps)
                linspace(0, pi/2, num_time_steps)
                linspace(0, 0, num_time_steps)
                linspace(0, 0, num_time_steps)]';
            
            
            