% Determine the error between the jacobian estimation and the reprojection
% Let's vary in x
%range = -0.1:0.001:0.1;
%range = -1:0.1:1;
range = -.3:0.005:0.3;

some_pose = [0, 0, 0, 3, 3, 3];
cad_offset = [1, 0, 0];

figure_handle = figure();
subplot(1, 2, 1);
axis_handle = gca;
axis vis3d

hold off;
xlabel('X axis (m)');
ylabel('Y axis (m)');
zlabel('Z axis (m)');

measurements = zeros(size(range));
for i=1:size(range, 2)
    
    % phi theta psi x, y, z
    diff_pose = [0, 0, 0, 0, 0, range(i)];
   
    measurements(i) = jacobian_error(axis_handle, some_pose, cad_offset, diff_pose);
      
end

subplot(1, 2, 2);
plot(range, measurements, 'b.');
title('Error (Reprojection vs. Jacobian)');
xlabel('Change in Z');
ylabel('Error (in euclidean distance)');