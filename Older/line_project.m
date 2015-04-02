%% Analytical Measurement Model of a CAD Model Point on a 2D Plane
% Determine an analytical solution to the projection of a 3D point
% on a CAD model onto a plane. 
% 
% Inspired by centroidMeasurementModel.m, by George Kantor
% 
% Originally, a script to check roll-pitch yaw convention and 
% compute berm centroid measurement model
% 


%% 
% We will 

% Get the measurement model
%% 
    % We will 
    % Note that: (phi=roll), (theta=pitch), (psii=yaw)

    % pose of model; these are known values
    syms theta phi psii
    syms pose_x pose_y pose_z

    % f is the focal length of the camera
    syms f

    % points in model frame; these are known values
    syms off_x off_y off_z


    p = [off_x; off_y; off_z];

    % define single axis rotation about phi, theta, psii
    Rx = [1  0   0
          0 cos(phi) -sin(phi)
          0 sin(phi) cos(phi)];
    Ry = [cos(theta)  0 sin(theta)
           0  1  0
         -sin(theta)  0 cos(theta)];
    Rz = [cos(psii) -sin(psii)  0
          sin(psii)  cos(psii)  0
           0   0  1];

    d_b_to_c = [pose_x; pose_y; pose_z];
    % assume RPY means roll then pitch then yaw relative to fixed frame
    %  ==> get overall rotation by multiplying successively on left
    R_from_b_to_c = Rz * Ry * Rx;

    H_cad = eye(4);
    H_cad(1:3, 1:3) = R_from_b_to_c;
    H_cad(1:3, 4) = this_cad_pose(1:3)';

    
    p_c = R_from_b_to_c*p + d_b_to_c;

    % Camera Matrix
    % Assuming a pinhole model for a camera, with a focal length f
    p_cx = p_c(1);
    p_cy = p_c(2); 
    p_cz = p_c(3);

    % Get the final measurement model
    measurement_model = f .* [ p_cx; p_cy; p_cz];
    measurement_model = measurement_model ./ p_cz;

    % Print out full jacobian
    jacobian_mm = ...
        jacobian(measurement_model, [theta, phi, psii, pose_x, pose_y, pose_z]);


%% Do plotting to verify results

% First point
this_f = 2;
this_cad_offset = [0, 1, 0];
this_cad_pose = [5, 5, 2, pi/4, 0, 0];
p1 = subs(measurement_model, f, this_f);
p1 = subs(p1, {pose_x, pose_y, pose_z, phi, theta, psi}, this_cad_pose);
p1 = subs(p1, {off_x, off_y, off_z}, this_cad_offset);
hold on;

% Second point
f_2 = 2;
cad_offset_2 = [0, 1, 0];
cad_pose_2 = [6, 5, 2, pi/4, 0, 0];
p2 = subs(measurement_model, f, f_2);
p2 = subs(p2, {pose_x, pose_y, pose_z, phi, theta, psi}, cad_pose_2);
p2 = subs(p2, {off_x, off_y, off_z}, cad_offset_2);
hold on;


% Plot axes for the origin
plot_coord_frame(eye(4));

% Plot the cad model frame axes
cad_frame = eye(4);
cad_frame(1:3, 1:3) = eval(subs(R_from_b_to_c, {phi, theta, psii}, this_cad_pose(4:6)));
cad_frame(1:3, 4) = this_cad_pose(1:3)';
plot_coord_frame(cad_frame);

% Plot the cad model frame point
pc2 = subs(p_c, {pose_x, pose_y, pose_z, phi, theta, psii}, this_cad_pose);
pc2 = subs(pc2, {off_x, off_y, off_z}, this_cad_offset);
scatter3(pc2(1), pc2(2), pc2(3), 'green');
plot3([0,pc2(1)], [0, pc2(2)], [0, pc2(3)]);

p1 = eval(p1);
scatter3(p1(1), p1(2), p1(3), 'green');

% Plot the second cad model frame point
pc2 = subs(p_c, {pose_x, pose_y, pose_z, phi, theta, psii}, cad_pose_2);
pc2 = subs(pc2, {off_x, off_y, off_z}, cad_ofset_2);
scatter3(pc2(1), pc2(2), pc2(3), 'green');
plot3([0,pc2(1)], [0, pc2(2)], [0, pc2(3)]);

p2 = eval(p2);
scatter3(p2(1), p2(2), p2(3), 'green');

% Draw a line between them!
plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)]);
% Now, use the jacobian to apply a small offset
jmm = subs(jacobian_of_measurement_model, f, this_f);
jmm = subs(jmm, {pose_x, pose_y, pose_z, phi, theta, psii}, this_cad_pose);
jmm = subs(jmm, {off_x, off_y, off_z}, this_cad_offset);

offset = [1, 0, 0, 0, 0, 0].';
projected = p1 + jmm*offset
scatter3(projected(1), projected(2), projected(3), 'blue');
hold off;