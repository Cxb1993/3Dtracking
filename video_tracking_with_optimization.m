% Perform tracking of an object in the video
addpath('vision', path);

% Prepare the tracker. 
run('get_measurement_model');
S = SampledObject;
% In meters, of the given target
% Recall this is xyz, ypr
S.pose = [ 0.035, -0.01, 0.35, 0, 0, pi ]';
% Demo: S.pose = [ 0.03, 0, 0.35, 0, 0, pi/6];
% these are presumably in meters
S.cad_points = [0.0,    0.085, 0;
                0.085,  0.085, 0;
                -0.085, 0.085, 0;
                -0.085, -0.085, 0;
                0.085, -0.085, 0;
                ]';

%world_frame_points = S.project(0.0018);

% Load camera stuff
load('vision/cameraParams', 'cameraParams');
%% TODO these instrinsics are made up to let me use weird image size.
%intrinsicParams = cameraParams.IntrinsicMatrix.';
intrinsicParams = [553.9621,    0, 853 / 2
                        0,   552.4798, 480 / 2;
                        0,        0,    1.0000];
%intrinsicParams = [553.9621,    0,  319.7582
%                        0,   552.4798, 180.8905;
%                        0,        0,    1.0000];
                    
% Prepare video frame stuff
vr = VideoReader('vision/tracking_video.avi');

vidWidth = vr.Width;
vidHeight = vr.Height;

goalWidth = 853; % 1920/2.25 = 853.333 (kinda like 640)
goalHeight = 1080/2.25;% 480

sampled_mov = struct('cdata',zeros(goalHeight,goalWidth, 3, 'uint8'), ...
            'colormap', []);
centroids = [];


% Initialize trackers
%tl = NNTracker(621, 384);
%tm = NNTracker(483, 391);
%tr = NNTracker(345, 392);
%bl = NNTracker(619, 119);
%br = NNTracker(345, 120);
tl = NNTracker(345, 90);
tm = NNTracker(482, 92);
tr = NNTracker(618, 97);
bl = NNTracker(353, 363);
br = NNTracker(613, 361);


k = 1;
j = 1;

% Read the first frame, and initialize the trackers
while hasFrame(vr) && k < 160
   sampled_mov(k).cdata = flipud(imresize(readFrame(vr), [goalHeight goalWidth]));
   % Note that we have to flip the image about the Y, so that
   %  we can have Y increase as we go up. 
   %display_image = sampled_mov(k).cdata;
   %work_image = sampled_mov(k).cdata;
   
   [BW, maskRGBImage] = pink_dot_mask(sampled_mov(k).cdata);
   
   hold on;
   subplot(1, 2, 1);
   image(sampled_mov(k).cdata);
   
   subplot(1, 2, 2);
   image(maskRGBImage);
   
   % Get only the five "true" ones.
   regions = regionprops(BW);
   big_enough = regions([regions.Area] > 250);
   if(size(big_enough, 1) == 5)
       centroids(:, :, j) = reshape([big_enough.Centroid], [2, 5])';
       j = j + 1;
       
       
   end
   
   % Plot each of the 3D points as we see them
   S.draw_on_image(intrinsicParams);
   % Update the tracked positions
   tl_ind = tl.findNearest(centroids(:, :, j-1));
   tl.addMeasurement(centroids(tl_ind, :, j-1));
   tl.draw('b');   

   tm_ind = tm.findNearest(centroids(:, :, j-1));
   tm.addMeasurement(centroids(tm_ind, :, j-1));
   tm.draw('r');
   
   tr_ind = tr.findNearest(centroids(:, :, j-1));
   tr.addMeasurement(centroids(tr_ind, :, j-1));
   tr.draw('g');
   
   bl_ind = bl.findNearest(centroids(:, :, j-1));
   bl.addMeasurement(centroids(bl_ind, :, j-1));
   bl.draw('c');
   
   br_ind = br.findNearest(centroids(:, :, j-1));
   br.addMeasurement(centroids(br_ind, :, j-1));
   br.draw('m');
   
   % Print out parameters
   image_frame = tl.getHomCoords()
   camera_frame = pinv(intrinsicParams) * tl.getHomCoords() / 1000
   divided_frame = camera_frame ./ (camera_frame(3))
   
   % TODO there is a bug where one centroid jumps from one to the other
   %  at some point early in the computation.
   % Update the pose
   believed_measurement = S.get_cad_points_in_image_frame(intrinsicParams)
   %new_measurement = [tm.getHomCoords(), tl.getHomCoords(), ...
   %                     tr.getHomCoords(), bl.getHomCoords(), ...
   %                     br.getHomCoords()];
   new_measurement = [  tm.getHomCoords(), tr.getHomCoords(), ...
                        tl.getHomCoords(), bl.getHomCoords(), ...
                        br.getHomCoords()]
  
   [distance_for_row, I] = pdist2(believed_measurement', new_measurement', 'euclidean', 'Smallest', 1);
   assert(numel(unique(I)) == numel(I));
   matched_new_measurements = believed_measurement(:, I)
   
   
   % Find the difference between the associated measurements  
   % Don't include Z. 
   % And match up the corresponding measurements.
   %X1_tall = believed_measurement(1:2, :).';
   X1_tall = matched_new_measurements(1:2, :).';
   X2_tall = new_measurement(1:2, :).';
   
   
   
   % Find the difference between the associated measurements
   X_diff = X2_tall - X1_tall;
   X_diff_tall = reshape(X_diff, numel(X_diff), 1);
   
   % Sub into Jacobian...for each one!
    f = 0.0018; % 1 / 553;  % focal length
    tall_jacob = [];
    for i = 1:size(X1_tall, 1)
        tall_jacob = [tall_jacob; eval_proj_point(jacobian_of_measurement_model, ...
            symbolics, f, S.pose', S.cad_points(:, i)')];
    end
    
    % Find the pseudoinverse of the jacobian
    % Matrix is tall and skinny; use the Moor-Penrose Inverse
    % J_# = (J.' * J)^-1 * J.';
    %pinv_jacob = pinv(tall_jacob); % inv(tall_jacob.' * tall_jacob) * tall_jacob
    
    %best_fit_pose_delta = tall_jacob \ X_diff_tall;
    best_fit_pose_delta = fminunc(@(guess_delta) my_optimization_fun(S, guess_delta,[tm, tr, ...
                        tl, bl, br], intrinsicParams, jacobian_of_measurement_model, symbolics), [0, 0, 0, 0, 0, 0]'); 
    % TODO confirm that this makes sense:
    %    jacobian(measurement_model, [phi, theta, psi, pose_x, pose_y, pose_z]);
    % phi = roll, theta=pitch, psi=yaw
    S.pose = S.pose + best_fit_pose_delta;
    
    
    k = k + 1
   
    pause(1);
   
end

%hf = figure;
%set(hf, 'position', [150 150 goalWidth goalHeight]);

%movie(hf,sampled_mov,1,vr.FrameRate);