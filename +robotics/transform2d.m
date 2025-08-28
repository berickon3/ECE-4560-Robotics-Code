classdef transform2d
    %TRANSFORM This class defines a transform in 2d
    %   Detailed explanation goes here
    
    properties
        displacement
        rotation
        points
    end
    
    methods
        function obj = transform2d(displacement,theta)
            obj.displacement = displacement;
            obj.rotation = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        end
        
        function points = pointOperation(obj,points)
            points = obj.displacement + obj.rotation*points;
        end
    end
end

