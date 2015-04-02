run('get_measurement_model');

%% Do plotting to verify results
xlabel('X axis (m)');
ylabel('Y axis (m)');
zlabel('Z axis (m)');
hold on;

% Plot axes for the origin
plot_coord_frame(eye(4));

% Experimental Constants
focal = 2; % focal length

% First point : Apply jacobian to this!
this_cad_offset = [0, 1, 0];
this_cad_pose = [pi/4, 0, 0, 5, 5, 2];
proj1 = eval_proj_point(measurement_model, symbolics, focal, this_cad_pose, this_cad_offset);
scatter3(proj1(1), proj1(2), proj1(3), 'red');

% Second point: Check jacobian against this.
cad_offset_2 = [0, 1, 0];
cad_pose_2 = [pi/4, 0, 0, 6, 5, 2];
proj2 = eval_proj_point(measurement_model, symbolics, focal, cad_pose_2, cad_offset_2);
scatter3(proj2(1), proj2(2), proj2(3), 'red');

% Plot the cad model frame axes
cad_frame = eval_transform(R_from_b_to_c, symbolics, this_cad_pose);
plot_coord_frame(cad_frame);

% Plot the first CAD model frame point
p1 = eval_proj_point(p_c, symbolics, 1, this_cad_pose, this_cad_offset);
scatter3(p1(1), p1(2), p1(3), 'green');
plot3([0,p1(1)], [0, p1(2)], [0, p1(3)]);

% Plot the second CAD model frame point
p2 = eval_proj_point(p_c, symbolics, 1, cad_pose_2, cad_offset_2);
scatter3(p2(1), p2(2), p2(3), 'green');
plot3([0,p2(1)], [0, p2(2)], [0, p2(3)]);


% Now, use the jacobian to apply a small offset

jmm2 = eval_proj_point(jmm, symbolics, focal, this_cad_pose, this_cad_offset);

offset = [1, 0, 0, 0, 0, 0].';
projected = proj1 + jmm*offset;
scatter3(projected(1), projected(2), projected(3), 'blue');