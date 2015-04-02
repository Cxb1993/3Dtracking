% Given a set of associated features, determine the change in pose. 
run('get_measurement_model');

% Initial Pose
pose = [0, 0, 0, 3, 3, 3];

% Cad Model Points
cad_points = [-1, -1, -1; ...
                0, 0, 0; ...
                1, -1, 1];


% Our secret delta
secret_pose_delta = [0, 0, 0, 0, 0, 0.1];

% Show these points
hold on;
% Plot axes for the origin
plot_coord_frame(eye(4));
% 3D
for i = 1:size(cad_points, 1)
    ith = cad_points(i, :);
    p1 = eval_proj_point(p_c, symbolics, 1, pose, ith);
    scatter3(p1(1), p1(2), p1(3), 'blue');
end
for i = 1:size(cad_points, 1)
    ith = cad_points(i, :);
    p1 = eval_proj_point(p_c, symbolics, 1, pose+secret_pose_delta, ith);
    scatter3(p1(1), p1(2), p1(3), 'green');
end


% Projected Points of the cad model; initial measurement
X1 = zeros(size(cad_points, 1), size(cad_points, 2));
for i=1:size(X1, 1)
   X1(i, :) = eval_proj_point(measurement_model, symbolics, 1, ...
       pose, cad_points(i, :));
    
end
X1(:, 3) = [];

% Projected points after perturbation; second measurement
X2 = zeros(size(cad_points, 1), size(cad_points, 2));
for i=1:size(X2, 1)
   X2(i, :) = eval_proj_point(measurement_model, symbolics, 1, ...
       pose+secret_pose_delta, cad_points(i, :));
   % Chop off third row
     
end
X2(:, 3) = [];

% Show 2D points
for i = 1:size(X1, 1)

end
% /show

% X1 and X2 are associated points. 

% Add gaussian noise?
% TODO one day....

% Now we have to find the pose delta that minimizes the 
%  error between the first observation and the second 
%  observation. 


% Combine measurements into tall-vector. 
X1_tall = X1.';
X2_tall = X2.';

% Find the difference between the associated measurements
X_diff = X2_tall - X1_tall;
X_diff_tall = reshape(X_diff, numel(X_diff), 1);

% Sub into Jacobian...for each one!
tall_jacob = [];
for i = 1:size(X1, 1)
    tall_jacob = [tall_jacob; eval_proj_point(jacobian_of_measurement_model, symbolics, 1, pose, cad_points(i, :))];
end

% Find the pseudoinverse of the jacobian
% Matrix is tall and skinny; use the Moore-Penrose Inverse
% J_# = (J.' * J)^-1 * J.'
pinv_jacob = pinv(tall_jacob);%inv(tall_jacob.' * tall_jacob) * tall_jacob.';


% Iterate this many times
best_fit = pinv_jacob * X_diff_tall






