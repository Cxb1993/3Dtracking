function [ error ] = jacobian_error(ah, cad_pose, cad_offset, diff_pose)
%JACOBIAN_ERROR Summary of this function goes here
%   cad_pose = 
%   cad_offset = 
%   diff_pose =
    run('get_measurement_model');

    %% Do plotting to verify results

    hold on;

    % Plot axes for the origin
    if(ah ~= 0) plot_coord_frame(eye(4)); end;

    % Experimental Constants
    focal = 1; % focal length

    % First point : Apply jacobian to this!
    proj1 = eval_proj_point(measurement_model, symbolics, focal, cad_pose, cad_offset);
    if(ah ~= 0) scatter3(proj1(1), proj1(2), proj1(3), 'red'); end;

    % Second point: Check jacobian against this.
    proj2 = eval_proj_point(measurement_model, symbolics, focal, cad_pose + diff_pose, cad_offset);
    if(ah ~= 0) scatter3(proj2(1), proj2(2), proj2(3), 'red'); end;

    % Plot the cad model frame axes
    cad_frame = eval_transform(R_from_b_to_c, symbolics, cad_pose);
    if(ah ~= 0) plot_coord_frame(cad_frame); end;

    % Plot the update cad frame aces
    cad_frame_updated = eval_transform(R_from_b_to_c, symbolics, cad_pose + diff_pose)
    if(ah ~= 0) plot_coord_frame(cad_frame_updated); end;
    
    % Plot the first CAD model frame point
    p1 = eval_proj_point(p_c, symbolics, 1, cad_pose, cad_offset);
    if(ah ~= 0) scatter3(p1(1), p1(2), p1(3), 'magenta'); end;
    if(ah ~= 0) plot3([0,p1(1)], [0, p1(2)], [0, p1(3)]); end;

    % Plot the second CAD model frame point
    p2 = eval_proj_point(p_c, symbolics, 1, cad_pose + diff_pose, cad_offset);
    if(ah ~= 0) scatter3(p2(1), p2(2), p2(3), 'green'); end;
    if(ah ~= 0) plot3([0,p2(1)], [0, p2(2)], [0, p2(3)]); end;


    % Now, use the jacobian to apply a small offset
    jmm = eval_proj_point(jacobian_of_measurement_model, symbolics, ...
        focal, cad_pose, cad_offset);

    projected = proj1 + jmm*diff_pose';
    if(ah ~= 0) scatter3(projected(1), projected(2), projected(3), 'blue'); end;

    error = sum(abs(projected-proj2));
    
    %if(ah ~= 0) pause(0.01); end;
    
end