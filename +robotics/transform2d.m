classdef transform2d
    %TRANSFORM This class defines a transform in 2d
    
    properties
        displacement
        rotation
        points
        theta
    end
    
    methods
        function obj = transform2d(displacement,theta)
            obj.displacement = displacement;
            obj.theta = theta;
            obj = obj.updateRotMat();
            obj.points = NaN([2,1]);
        end

        function obj = compose(obj,transform2)
            obj.updateRotMat();
            transform2.updateRotMat();
            overallDisplacement = obj.displacement + obj.rotation*transform2.displacement;
            overallRotation = obj.rotation * transform2.rotation;
            overallTheta = atan2(overallRotation(2,1),overallRotation(1,1));
            obj = robotics.transform2d(overallDisplacement,overallTheta);
        end
        
        function points = pointOperation(obj,points)
            obj.updateRotMat();
            points = obj.displacement + obj.rotation*points;
        end
        
        function obj = inverse(obj)
            obj.displacement = -transpose(obj.rotation)*obj.displacement;
            obj.theta = -obj.theta;
            obj.updateRotMat;
        end
        
        function obj = updateRotMat(obj)
            obj.rotation = [cos(obj.theta), -sin(obj.theta); sin(obj.theta), cos(obj.theta)];
        end
    end
end

