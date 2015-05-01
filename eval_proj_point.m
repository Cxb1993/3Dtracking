function [ proj_point ] = eval_proj_point( mm, symbolics, f, cad_pose, cad_offset)
%EVAL_PROJ_POINT Project a CAD model+point into the image plane.
%   symbolics = [ f, phi, theta, psi, pose_{x, y, z}, off_{x, y, z}

    % TODO add tests for the right dimensionality of cad_pose/offset
    %if(size(arr, 2) ~= 2)
    %    error('Arr must be an Nx2 array');
    %end
    
    s_mm = subs(mm, symbolics(1), f);
    s_mm = subs(s_mm, symbolics(2:7), cad_pose);
    s_mm = subs(s_mm, symbolics(8:10), cad_offset);
    proj_point = eval(s_mm);

end


