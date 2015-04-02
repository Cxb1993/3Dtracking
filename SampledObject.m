classdef SampledObject
    %UNTITLED3 Made of a pose and CAD model points
    %   Detailed explanation goes here
    
    properties
        
        pose
        % pose = 6x1 (x; y; z; rx; ry; rz)
        % x, y, z are in meters
        % rx, ry, and rz are rotation about the x, y, and z axis (rads)
        % rz = yaw, ry = pitch, rx = roll
        
        cad_points
        % cad_points = 3xN 
        % each column is a point
        % rows are [x, y, z] in the Cad model frame
        
        mm
        % measurement model
        
        symbolics
        % the symbolics we need currently...
        % these should go away.
    end
    
    methods
        
        function H = get_homogeneous(obj) 
            H = zeros(4, 4);
            
            % Set R
            rx = obj.pose(4);
            ry = obj.pose(5);
            rz = obj.pose(6);
            
            Rx = [1  0  0;
                    0 cos(rx) -sin(rx);
                    0 sin(rx) cos(rx)];
                
            Ry = [cos(ry)  0  sin(ry);
                    0  1  0;
                    -sin(ry) 0 cos(ry)];
            
            Rz = [cos(rz) -sin(rz) 0;
                    sin(rz) cos(rz) 0;
                    0 0 1];
            
            H(1:3, 1:3) = Rz * Ry * Rx;
            
            % Set t
            H(1:3, 4) = obj.pose(1:3);
            
            % Set last element
            H(4, 4) = 1;
        end
        
        % Using the current pose, get the cad points in the 
        %   world frame.
        function points = get_cad_points_in_wframe(obj)
            rx = obj.pose(4);
            ry = obj.pose(5);
            rz = obj.pose(6);
            
            Rx = [1  0  0;
                    0 cos(rx) -sin(rx);
                    0 sin(rx) cos(rx)];
                
            Ry = [cos(ry)  0  sin(ry);
                    0  1  0;
                    -sin(ry) 0 cos(ry)];
            
            Rz = [cos(rz) -sin(rz) 0;
                    sin(rz) cos(rz) 0;
                    0 0 1];
            
            R = Rz * Ry * Rx;
            
            points_world_frame = zeros(size(obj.cad_points));
            for i=1:size(obj.cad_points, 1)
                points_world_frame(:, i) = obj.pose(1:3);
                points_world_frame(:, i) = points_world_frame(:,i) + R * obj.cad_points(:, i);
            end
            points = points_world_frame;
        end          
            
        % Projects a point to the origin, at a focal length f_l
        %   point = 3x1 (x, y, z)
        %   focal_length = 1x1 (the focal length)
        function p = project_point(obj, point, focal_length_along_z)
            % TODO stop assuming the camera is projecting
            %   on to the Z plane
            f = focal_length_along_z;
            p(1, :) = f * point(1) / point(3);
            p(2, :) = f * point(2) / point(3);
            p(3, :) = f * point(3) / point(3);
        end
              
        % projected_cad_points = 2xN point projections
        %  returns projected points in the world frame.
        function projected_cad_points = project(obj, focal_length)
            % TODO convert this into a math operation, not
            %     a symbolid operation
            
            proj_points = obj.get_cad_points_in_wframe();
            for i=1:size(proj_points, 1)
              proj_points(:, i) = obj.project_point(proj_points(:, i), ...
                    focal_length);
            end
            projected_cad_points = proj_points(1:2, :);
        end
        
        % this plots the object pose in the world frame.
        function [] = plot_axes(obj, fh)
            H = obj.get_homogeneous();
            hold on;
            axis vis3d;
            plot_coord_frame(fh, H);
            hold off;
        end
        
        function [] = plot_transform_axes(~, H, fh)
            hold on;
            axis vis3d;
            plot_coord_frame(fh, H);
            hold off;
        end
        
        % this plots the cad points in the world frame.
        function [] = plot_cad_points(obj, fh)
            cad_points_world_frame = obj.get_cad_points_in_wframe();
            hold on;
            axis vis3d;
            for i = 1:size(obj.cad_points, 1)
                ith = cad_points_world_frame(:, i);
                scatter3(fh.CurrentAxes, ith(1,:), ith(2,:), ith(3,:), 'blue');
            end
            hold off;
        end
    end
    
end

