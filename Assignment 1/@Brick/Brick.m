classdef Brick < handle
    %BRICKS Summary of this class goes here
    %   Detailed explanation goes here

    %     properties (Constant)
    %         MatrixSize = [9,3];
    %     end
    %
    %     properties (Access = private)
    %         bMatrix;
    %     end

    methods
        function self = Brick()
            self.PlaceBricks();
            self.placeABrick();
            
        end
    end

    methods (Static)
        function bPoses = PlaceBricks()
            % bMatrix = zeros(9,3);
            %Assigning brick locations to the matrix
            bMatrix = [0.4, 0.4, 1;0.3, 0.4, 1;
            0.2, 0.2, 1;
            0.1, 0.2, 1;
            0, 0.4, 1;
            -0.1, 0.4, 1;
            -0.2, 0.4, 1;
            -0.3, 0.4, 1;
           -0.4, 0.4, 1];


            hold on;
            
            %placing all brick objects
            b1 = PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(1,:));
            b2 = PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(2,:));
            b3 = PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(3,:));
             b4 = PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(4,:));
            b5= PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(5,:));
           b6 = PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(6,:));
            b7 = PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(7,:));
            b8 = PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(8,:));
            b9 = PlaceObject('HalfSizedRedGreenBrick.ply', bMatrix(9,:));

            bPoses = [b1; b2;b3;b4;b5;b6;b7;b8;b9];
        end
           %function to place a single brick 
        function placeABrick(brickLocation)
            PlaceObject('HalfSizedRedGreenBrick.ply', brickLocation);
        end


       

       
        

    end
end