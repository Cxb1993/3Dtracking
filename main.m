run('get_measurement_model');
% This puts the following variables in scope:
%   symbolics = (sym)[ f, phi, theta, psi, pose_{x, y, z}, off_{x, y, z}
%   measurement_model = (sym) measurement model of...system. 
%               TODO pose vs. cad points? dimensions?
%   jacobian_of_mm = (sym) 
%               TODO dimension?
%   f = (sym) 1x1 focal length of camera
% 
% Currently, the jacobian step still relies on this.

% Create an object that moves through space. 
%initial_pose = [0, 0, 0, 0, 0, 0];
%pose_after_10_seconds = [3, 3, 3, pi/2, 0, 0];
num_time_steps = 10;

% Note that initial pose cannot be zero, since
%  then a projection with a focal length
%  of two does not make sense. 
pose_array = [linspace(0, 3, num_time_steps);
                linspace(0, 0, num_time_steps);
                linspace(2, 4, num_time_steps);
                linspace(0, 0, num_time_steps);
                linspace(0, 0, num_time_steps);
                linspace(0, 0, num_time_steps)];
            

% Initialize Secret State; we will use this to get measurements
s = SampledObject;
s.pose = pose_array(:, 1);
s.cad_points = [ 0, 0, 1; 
                 0, 1, 0;
                 1, 0, 0;
                 2, 2, 2]';

fh = figure(1);

s.plot_transform_axes(zeros(4, 4), fh);
s.plot_axes(fh);
s.plot_cad_points(fh);

% Set up follower pose
follower = SampledObject;
follower.pose = pose_array(:, 2);
follower.cad_points = s.cad_points;

% Loop through the poses, solve for the transforms between them
for i = 1:size(pose_array, 2)
    % Update the secret pose, and get the projected points
    s.pose = pose_array(:, i);
    new_measurement = s.project(2);
   
    % Get the projected pose of the follower pose.
    old_measurement = follower.project(2);
    X1_tall = old_measurement.';
    X2_tall = new_measurement.';

    % Find the difference between the associated measurements
    X_diff = X2_tall - X1_tall;
    X_diff_tall = reshape(X_diff, numel(X_diff), 1);

    % Sub into Jacobian...for each one!
    tall_jacob = [];
    for i = 1:size(X1_tall, 1)
        tall_jacob = [tall_jacob; eval_proj_point(jacobian_of_measurement_model, ...
            symbolics, 2, follower.pose', follower.cad_points(:, i)')];
    end

    % Find the pseudoinverse of the jacobian
    % Matrix is tall and skinny; use the Moore-Penrose Inverse
    % J_# = (J.' * J)^-1 * J.'
    pinv_jacob = pinv(tall_jacob);%inv(tall_jacob.' * tall_jacob) * tall_jacob.';

    best_fit_pose_delta = pinv_jacob * X_diff_tall;

    % Update the follower
    follower.pose = follower.pose + best_fit_pose_delta;
    
end


