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
        
        % Using the current pose, get the cad points in the 
        %   world frame.
        function points = get_cad_points_in_wframe(obj)
            pose = obj.pose;
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
                    sin(r) cos(rz) 0;
                    0 0 1];
            
            R = Rz * Ry * Rx;
            
            points_world_frame = zeros(size(obj.cad_points));
            for i=1:size(obj.cad_points, 1)
                points_world_frame(1, i) = obj.pose(1) + R * obj.cad_points(1, i);
                points_world_frame(2, i) = obj.pose(2) + R * obj.cad_points(2, i);
                points_world_frame(3, i) = obj.pose(3) + R * obj.cad_points(3, i);
            end
        end
        
        % Projects a point to the origin, at a focal length f_l
        %   point = 3x1 (x, y, z)
        %   focal_length = 1x1 (the focal length)
        function p = project_point(point, focal_length_along_z)
            % TODO stop assuming the camera is projecting
            %   on to the Z plane
            f = focal_length_along_z;
            p(1) = f * point(1) / point(3);
            p(2) = f * point(2) / point(3);
            P(3) = f * point(3) / point(3);
        end
        
        
        function ps = project_points(points, focal_length_along_z)
            % TODO stop assuming the camera is projecting
            %   on to the Z plane
            f = focal_length_along_z;
            p(1) = f * point(1) / point(3);
            p(2) = f * point(2) / point(3);
            P(3) = f * point(3) / point(3);           
        end
        
        % perturbation is a pose delta.
        % perturbed... = 
        function perturbed_projected_cad_points = ...
                project(points, focal_length)
            % TODO convert this into a math operation, not
            %     a symbolid operation
            proj_points = zeros(size(obj.cad_points));
            for i=1:size(proj_points, 1)
                % Add perturbation into the pose!
              % out?
            end
            perturbed_projected_cad_points = proj_points;
        end
        
        % this plots the object pose in the world frame.
        function () = plot_axes(fh)
            %plot_coord_frame(
        end
        
        % this plots the cad points in the world frame.
        function () = plot_cad_points(fh)
            % TODO plot this.
        end
    end
    
end

