function [f, g] = my_optimization_fun(SampledObj, guess_pose_delta, ...
    nn_measurements, intrinsicParams, jacobian_of_mm, symbolics)
% Pose is a 6x1, xyzypr
% pose_delta is a 6x1, xyzypr
% measurements are the measurements, not yet matched. columns are
%       measurements. 
% objective function to minimize, e.g. the error99

   % Try this guess_pose_delta
   SampledObj.pose = SampledObj.pose + guess_pose_delta;

   believed_measurement = SampledObj.get_cad_points_in_image_frame(intrinsicParams);
   %new_measurement = [tm.getHomCoords(), tl.getHomCoords(), ...
   %                     tr.getHomCoords(), bl.getHomCoords(), ...
   %                     br.getHomCoords()];
   new_measurement = [];
   for i = 1:size(nn_measurements, 2)
       new_measurement = [new_measurement, nn_measurements(i).getHomCoords()];
   end
   [distance_for_row, I] = pdist2(believed_measurement', new_measurement', 'euclidean', 'Smallest', 1);
   
   
   if(numel(unique(I)) ~= numel(I))
        f = 100000000;
        return;
   end
   matched_new_measurements = believed_measurement(:, I);

   
   
   % Find the difference between the associated measurements  
   % Don't include Z. 
   % And match up the corresponding measurements.
   %X1_tall = believed_measurement(1:2, :).';
   X1_tall = matched_new_measurements(1:2, :).'
   X2_tall = new_measurement(1:2, :).'
   
   % Find the difference between the associated measurements
   X_diff = (X2_tall - X1_tall).^2;
   
   
   % Return this as an error metric
   f = sum(sum(X_diff));
   
   
   % Find the gradient
   
   % Sub into Jacobian...for each one!
   focal = 0.0018; % 1 / 553;  % focal length
   tall_jacob = [];
   for i = 1:size(X1_tall, 1)
       tall_jacob = [tall_jacob; eval_proj_point(jacobian_of_mm, ...
           symbolics, focal, SampledObj.pose', SampledObj.cad_points(:, i)')];
   end
    
   X_diff_tall = reshape(X_diff, numel(X_diff), 1);
   best_fit_pose_delta = tall_jacob \ X_diff_tall;
   
   
    if nargout > 1
        % how to change the objective scalar with the input
        % take current error vector, multiply it by jacob, 
        %   reorganize 
        
        g = sum(tall_jacob, 2).';
        g = [ g(4:6); flipud(g(1:3))];
end