% Read the frames of the video

vr = VideoReader('tracking_video.avi');

vidWidth = vr.Width;
vidHeight = vr.Height;

goalWidth = 640;
goalHeight = 480;

sampled_mov = struct('cdata',zeros(goalHeight,goalWidth, 3, 'uint8'), ...
            'colormap', []);
centroids = [];

fh = figure(1);

k = 1;
j = 1;
while hasFrame(vr)
   sampled_mov(k).cdata = imresize(readFrame(vr), [goalHeight goalWidth]);
   [BW, maskRGBImage] = pink_dot_mask(sampled_mov(k).cdata);
   
   hold on;
   image(maskRGBImage)
   
   % Get only the five "true" ones.
   regions = regionprops(BW);
   big_enough = regions([regions.Area] > 300);
   if(size(big_enough, 1) == 5)
       centroids(:, :, j) = reshape([big_enough.Centroid], [2, 5])';
       j = j + 1;
       
       
       % Plot each of the centroids
       plot(centroids(1, 1, j-1), centroids(1, 2, j-1), 's', 'MarkerFaceColor', 'm', 'LineWidth', 10);
   end
   
   sampled_mov(k).cdata = maskRGBImage;
   k = k + 1
   
   pause(0.25);
   
end

hf = figure;
set(hf, 'position', [150 150 goalWidth goalHeight]);

movie(hf,sampled_mov,1,vr.FrameRate);