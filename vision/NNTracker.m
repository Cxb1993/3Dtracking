classdef NNTracker < handle
    %NNTRACKER Nearest Neighbor Tracker of a single point
    %   Detailed explanation goes here
    
    properties
        row;
        col;
    end
    
    methods
        function obj = NNTracker(row, col)
            if nargin == 2
                if isnumeric(row) && isnumeric(col)
                    obj.row = row;
                    obj.col = col;
                else
                    error('Value must be numberic')
                end
            else
                error('Must have 2 arguments')
            end
        end
        
        % Row comes first.
        function [] = addMeasurement(obj, rc)
            if(size(rc, 2) ~= 2 || size(rc, 1) ~= 1)
                error('Arr must be an Nx2 array');
            end
            obj.row = rc(1);
            obj.col = rc(2);
        end
        
        % Returns the rowid of the nearest neighbor. 
        % Arr = Nx2, with first col being x, second col being y
        function rowid = findNearest(obj, arr)
            if(size(arr, 2) ~= 2)
                error('Arr must be an Nx2 array');
            end
            
            rep_current_measurement = repmat([obj.row, obj.col], size(arr, 1), 1);
            diffs = arr - rep_current_measurement;
            
            % Use l1 distance because it's good enough and I don't want
            %  to find the function for L2
            dist = abs(sum(diffs, 2));
            [~, I] = min(dist);
            rowid = I;
        end
        
        function [] = draw(obj, color)
            rectangle('Position', [obj.row-5, obj.col-5, 10, 10], 'LineWidth',2,...
            'EdgeColor', color);
        end
    end
    
end

