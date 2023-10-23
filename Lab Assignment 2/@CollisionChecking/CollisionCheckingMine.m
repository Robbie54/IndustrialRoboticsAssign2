classdef CollisionChecking < handle
    %Class for ensuring collisions do not occur
    %in the brief it says we make and control the object so the easiest
    %would be a sphere just sphere centre and radius if in that volume is a
    %collision

    properties
        Property1
            

    end

    methods
        function self = CollisionChecking
            %object is preset
            % self.getObject()
            self.collisionCheck();
            %self.IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound);
            
        end
    end


    methods (Static)
        function obj = getObject()
            %for active joint tracking we could have a distance that scans
            %in front of the direction the robot is traveling 
            
            [v,f,fn] = RectangularPrism([2,-1.1,-1], [3,1.1,1]); %rectangle size etc
            obj = [v,f,fn];
        end
        function collisionBool = collisionCheck(robot, qMatrix)
            [vertex,faces,faceNormals] = getObject();
            returnOnceFound = true; %can make this a passed parameter from main 
            collisionBool = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound);
        end
          
    end
end
