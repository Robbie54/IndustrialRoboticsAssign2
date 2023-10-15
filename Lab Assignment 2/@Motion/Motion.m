classdef Motion < handle

    properties
    end

    methods
        function self = Motion
            self.UR3Motion()
            self.DrawbotMotion()
        end
    end

    methods (Static)
        function UR3Motion()
            %clear workspace, command window and figures
            % clear all;
            % close all;
            % clf;

            %initialise robot and attach suction cup to the end effector
            % hold on;
            r1 = UR3()
            r1.model;
            %q = zeros(1,7);
            %b = SuctionCup();
            %b.suctionModel{1}.base = r.model.fkine(r.model.getpos());
            %pause;
            
            axis equal;


            %define Matrices that hold the locations of the intial and
            %final Paper Stack

            %Define Number of Paper sheets to be manipulated
            paperNo = 5;

            %initial paper stack
            initialPaperMatrix = zeros(paperNo,3);
            %initialPaperMatrix(1,:) = ....
            initialPaperMatrix(1,:) = [0.2, -0.315, 0.7];
            initialPaperMatrix(2,:) = [0.2, -0.315, 0.71];
            initialPaperMatrix(3,:) = [0.2, -0.315, 0.72];
            initialPaperMatrix(4,:) = [0.2, -0.315, 0.73];
            initialPaperMatrix(5,:) = [0.2, -0.315, 0.74];

            %drawing board
            drawingBoardMatrix = zeroes(1,3);
            drawingBoardMatrix(1,:) = [-0.2,0.315,0.7]

            %final paper stack
            finalPaperMatrix = zeros(paperNo,3);
            %finalPaperMatrix(1,:) = ....
            finalPaperMatrix(1,:) = [-0.2, 0.315, 0.7];
            finalPaperMatrix(2,:) = [-0.2, 0.315, 0.71];
            finalPaperMatrix(3,:) = [-0.2, 0.315, 0.72];
            finalPaperMatrix(4,:) = [-0.2, 0.315, 0.73];
            finalPaperMatrix(5,:) = [-0.2, 0.315, 0.74];

            %create matrix that will store unique identifer to the paper so
            %they can be called in the future
            paperUniqueID = cell(paperNo, 1);

            % Create and store the paper identifier values via the loop.
            % spawn in brick at 0,0,0. translate by the initial brick
            % location matrix
            for paperIndex = 1:paperNo
                paperUniqueID{paperIndex} = PlaceObject('papersheet_industrial_new.ply'); %brickMatrix(brickIndex, :)
                vertices = get(paperUniqueID{paperIndex}, 'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(initialPaperMatrix(paperIndex,:))';
                set(paperUniqueID{paperIndex},'Vertices',transformedVertices(:,1:3));
            end

            %Create a nested for loop to iterate through the robot arms
            %path and update the animation

            %For each brick 1-9:
            for paperIndex = 1:paperNo
                placeDrawingBoard(paperIndex);

                %execute drawing motion.

                placeFinalStack(paperIndex);

            
            end

        end

        function DrawbotMotion()
            % clear all;
            % close all;
            % clf;

            %initialise robot and attach suction cup to the end effector
            r2 = Drawbot()
            r2.model;
            %q = zeros(1,7);
            count = 50;

            % axis equal;

            %Define Number of Paper sheets to be manipulated
            ColourPoints = 9;

            %initial paper locations
            Drawing = zeros(ColourPoints,3);
            %initialPaperMatrix(1,:) = ....
            Drawing(1,:) = [-0.2, 0.4, 0.73];
            Drawing(2,:) = [-0.2, 0.3, 0.73];
            Drawing(3,:) = [-0.2, 0.2, 0.73];
            Drawing(4,:) = [-0.2, 0.1, 0.73];
            Drawing(5,:) = [-0.2, 0, 0.73];
            Drawing(6,:) = [-0.2, -0.1, 0.73];
            Drawing(7,:) = [-0.2, -0.2, 0.73];
            Drawing(8,:) = [-0.2, -0.3, 0.73];
            Drawing(9,:) = [-0.2, -0.4, 0.73];
            Drawing(10,:) = [-0.2, 0, 0.73];

            rsp = r2.model.getpos;
            r2p1 = r2.model.ikcon(transl(Drawing(1,:)));
            r2p2 = r2.model.ikcon(transl(Drawing(2,:)));
            r2p3 = r2.model.ikcon(transl(Drawing(3,:)));
            r2p4 = r2.model.ikcon(transl(Drawing(4,:)));
            r2p5 = r2.model.ikcon(transl(Drawing(5,:)));
            r2p6 = r2.model.ikcon(transl(Drawing(6,:)));
            r2p7 = r2.model.ikcon(transl(Drawing(7,:)));
            r2p8 = r2.model.ikcon(transl(Drawing(8,:)));
            r2p9 = r2.model.ikcon(transl(Drawing(9,:)));
            r2p10 = r2.model.ikcon(transl(Drawing(10,:)));


            %% Pathing and Animation
            % Pathing from Start to Brick 1 Position
            qPath1 = jtraj(rsp,r2p1,count);
            for i = 1:length(qPath1)
                r2.model.animate(qPath1(i,:));
                drawnow();
            end

            qPath2 = jtraj(r2p1,r2p2,count);
            for i = 1:length(qPath2)
                r2.model.animate(qPath2(i,:));
                drawnow();
            end

            qPath3 = jtraj(r2p2,r2p3,count);
            for i = 1:length(qPath3)
                r2.model.animate(qPath3(i,:));
                drawnow();
            end

            qPath4 = jtraj(r2p3,r2p4,count);
            for i = 1:length(qPath4)
                r2.model.animate(qPath4(i,:));
                drawnow();
            end

            qPath5 = jtraj(r2p4,r2p5,count);
            for i = 1:length(qPath5)
                r2.model.animate(qPath5(i,:));
                drawnow();
            end

            qPath6 = jtraj(r2p5,r2p6,count);
            for i = 1:length(qPath6)
                r2.model.animate(qPath6(i,:));
                drawnow();
            end

            qPath7 = jtraj(r2p6,r2p7,count);
            for i = 1:length(qPath7)
                r2.model.animate(qPath7(i,:));
                drawnow();
            end

            qPath8 = jtraj(r2p7,r2p8,count);
            for i = 1:length(qPath8)
                r2.model.animate(qPath8(i,:));
                drawnow();
            end

            qPath9 = jtraj(r2p8,r2p9,count);
            for i = 1:length(qPath9)
                r2.model.animate(qPath9(i,:));
                drawnow();
            end

            qPath10 = jtraj(r2p9,r2p10,count);
            for i = 1:length(qPath10)
                r2.model.animate(qPath10(i,:));
                drawnow();
            end

            qPath11 = jtraj(r2p10,rsp,count);
            for i = 1:length(qPath11)
                r2.model.animate(qPath11(i,:));
                drawnow();
            end
        end
        function placeDrawingBoard(a)
    
            initialPosition = r1.model.getpos()
            currentPaper = initialPaperMatrix(a, :);
            drawingBoard = drawingBoardMatrix(1,:)
        
            %set count variable to dictate how many joint angle paths
            %will be created to model the Path.
            count = 100;

            %get the robots current arm location
            robotLocation = r1.model.getpos();

            %calculate the joint angles necessary ot traverse to the
            %chosen paper location. paper is rotated so the Z-axis is
            %facing down here so that the robot grabs the paper from
            %the top.
            currentPaperPath = r1.model.ikcon(transl(currentPaper)*troty(pi));

            %computes a joint space trajectory that inrerpolates
            %between the robots position and chosen brick location.
            currentQPath = jtraj(robotLocation, currentPaperPath, count);

            %for each joint step
            for i = 1:size(currentQPath, 1)
                %animate the robots arm
                r1.model.animate(currentQPath(i, :));
                drawnow();
            end

            %% move to final Paper stack location
            %calculate the joint angles necessary to traverse to the
            %paper location. paper is rotated so the Z-axis is
            %facing down here so that the robot places the paper from
            %the top.
            finalPaperPath = r1.model.ikcon(transl(drawingBoard)*troty(pi));

            %computes a joint space trajectory that inrerpolates
            %between the robots position and final paper location.
            finalQPath = jtraj(currentPaperPath, finalPaperPath, count);

            %for each joint step
            for i = 1:size(finalQPath, 1)

                %animate arm
                r1.model.animate(finalQPath(i, :));

                %calculate the end effector transform of the robot arm
                endEffectorTransform = r1.model.fkine(r1.model.getpos()).T;

                %update paper vertices to the new coordinates of the
                %robots end effector
                transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)]*endEffectorTransform';

                %update the paper vertices with the transformed
                %vertices
                set(paperUniqueID{a}, 'Vertices', transformedBrickVertices(:, 1:3));
                drawnow();
            end

            %%move robot back to initial position
            count = 100;
            robotLocation = r1.model.getpos();
            QPath = jtraj(robotLocation, initialPosition, count);
            for i = 1:size(QPath, 1)
                r1.model.animate(QPath(i, :));
                drawnow();
            end

            
        end
        function placeFinalStack(a)
    
            initialPosition = r1.model.getpos()
            drawingBoard = drawingBoardMatrix(1, :);
            finalPaper = finalPaperMatrix(a, :);

        
            %set count variable to dictate how many joint angle paths
            %will be created to model the Path.
            count = 100;

            %get the robots current arm location
            robotLocation = r1.model.getpos();

            %calculate the joint angles necessary ot traverse to the
            %chosen paper location. paper is rotated so the Z-axis is
            %facing down here so that the robot grabs the paper from
            %the top.
            currentPaperPath = r1.model.ikcon(transl(drawingBoard)*troty(pi));

            %computes a joint space trajectory that inrerpolates
            %between the robots position and chosen brick location.
            currentQPath = jtraj(robotLocation, currentPaperPath, count);

            %for each joint step
            for i = 1:size(currentQPath, 1)
                %animate the robots arm
                r1.model.animate(currentQPath(i, :));
                drawnow();
            end

            %% move to final Paper stack location
            %calculate the joint angles necessary to traverse to the
            %paper location. paper is rotated so the Z-axis is
            %facing down here so that the robot places the paper from
            %the top.
            finalPaperPath = r1.model.ikcon(transl(finalPaper)*troty(pi));

            %computes a joint space trajectory that inrerpolates
            %between the robots position and final paper location.
            finalQPath = jtraj(currentPaperPath, finalPaperPath, count);

            %for each joint step
            for i = 1:size(finalQPath, 1)

                %animate arm
                r1.model.animate(finalQPath(i, :));

                %calculate the end effector transform of the robot arm
                endEffectorTransform = r1.model.fkine(r1.model.getpos()).T;

                %update paper vertices to the new coordinates of the
                %robots end effector
                transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)]*endEffectorTransform';

                %update the paper vertices with the transformed
                %vertices
                set(paperUniqueID{a}, 'Vertices', transformedBrickVertices(:, 1:3));
                drawnow();
            end

            %%move robot back to initial position
            count = 100;
            robotLocation = r1.model.getpos();
            QPath = jtraj(robotLocation, initialPosition, count);
            for i = 1:size(QPath, 1)
                r1.model.animate(QPath(i, :));
                drawnow();
            end
        end
    
    end
end