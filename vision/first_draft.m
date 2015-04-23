% Read the frames of the video

vr = VideoReader('tracking_video.avi');

vidWidth = vr.Width;
vidHeight = vr.Height;

goalWidth = 640;
goalHeight = 480;

sampled_mov = struct('cdata',zeros(goalHeight,goalWidth, 3, 'uint8'), ...
            'colormap', []);
centroids = [];


% Initialize trackers
tl = NNTracker(264, 391);
tm = NNTracker(362, 389);
tr = NNTracker(465, 385);
bl = NNTracker(264, 118);
br = NNTracker(461, 120);

k = 1;
j = 1;

% Read the first frame, and initialize the trackers
while hasFrame(vr) && k < 200
   sampled_mov(k).cdata = imresize(readFrame(vr), [goalHeight goalWidth]);
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
   % Plot each of the centroids
   %plot(centroids(1, 1, j-1), centroids(1, 2, j-1), 's', 'MarkerFaceColor', 'm', 'LineWidth', 10);
   %plot(centroids(2, 1, j-1), centroids(2, 2, j-1), 's', 'MarkerFaceColor', 'm', 'LineWidth', 10);
   %plot(centroids(3, 1, j-1), centroids(3, 2, j-1), 's', 'MarkerFaceColor', 'm', 'LineWidth', 10);
   %plot(centroids(4, 1, j-1), centroids(4, 2, j-1), 's', 'MarkerFaceColor', 'm', 'LineWidth', 10);
   %plot(centroids(5, 1, j-1), centroids(5, 2, j-1), 's', 'MarkerFaceColor', 'm', 'LineWidth', 10);
   
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
   
   %sampled_mov(k).cdata = maskRGBImage;
   k = k + 1
   
   pause(0.01);
   
end

%hf = figure;
%set(hf, 'position', [150 150 goalWidth goalHeight]);

%movie(hf,sampled_mov,1,vr.FrameRate);