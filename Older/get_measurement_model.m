function [ measurement_model, H_cad, jacobian_mm ] = get_measurement_model() 
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

end