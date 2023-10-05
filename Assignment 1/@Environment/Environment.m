classdef Environment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here

    properties
    end

    methods 
        function self = Environment()
            self.createEnvironment();
        end
    end

    methods (Static)
        function createEnvironment()
            hold on;

            view([3 -2 1.5]);
            axis([-2.5, 2.5, -2.5, 2.5, 0, 3])

            surf([-2,-2;2,2],[-2,2;-2,2],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg') ,'FaceColor','texturemap');

            surf([-2,-2;-2,-2],[-2,2;-2,2], [3,3;0,0],'CData',imread('brickWall.jpg'),'FaceColor','texturemap');
            surf([-2,-2;2,2],[2,2;2,2], [3,0;3,0],'CData',imread('brickWall.jpg'),'FaceColor','texturemap');
            PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0,0,00]);
            PlaceObject('emergencyStopButton.ply', [2,0,1]);
            PlaceObject('fireExtinguisher.ply', [1.5,-1.8,0]);
            PlaceObject('barrier1.5x0.2x1m.ply', [-2,-2.3,0]);
            PlaceObject('barrier1.5x0.2x1m.ply', [0,-2.3,0]);
             PlaceObject('barrier1.5x0.2x1m.ply', [2,-2.3,0]);
              
        end
    end
end