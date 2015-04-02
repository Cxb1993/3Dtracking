function [ H ] = eval_transform(sym_rot, symbolics, pose)
%PROJECT_POINT Summary of this function goes here
%   symbolics = [ f, phi, theta, psi, pose_{x, y, z}, off_{x, y, z}
    H = eye(4);
    H(1:3, 1:3) = eval(subs(sym_rot, symbolics(2:4), pose(1:3)));
    H(1:3, 4) = pose(4:6)';
end

