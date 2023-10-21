classdef Motion < handle

    properties (Constant)
        paperNo = 5;
        ColourPoints = 9;
        initialPaper = Motion.initialPaperMatrix();
        finalPaper = Motion.finalPaperMatrix();
        boardLocation = Motion.drawingBoardMatrix();
        HomePosition = Motion.HomePositionMatrix();
        penDrawing = Motion.penDrawingMatrix();
    end
    properties
        % r1 = UR3();
        % r2 = Drawbot();
    end

    methods
        function self = Motion
            self.RobotMotion()
        end
    end

    methods (Static)
        function RobotMotion()
            %clear workspace, command window and figures
            % clear all;
            % close all;
            % clf;

            %initialise robot and attach suction cup to the end effector
            hold on;
            r1 = UR3();
            r2 = Drawbot();
            r1.model;
            r2.model;
            %q = zeros(1,7);
            % b = SuctionCup();
            % b.suctionModel{1}.base = r1.model.fkine(r1.model.getpos());
            %pause;

            axis equal;

            %initialise constant properties into function
            paperNo = Motion.paperNo;
            ColourPoints = Motion.ColourPoints;
            initialPaper = Motion.initialPaper;
            finalPaper = Motion.finalPaper;
            boardLocation = Motion.boardLocation;
            HomePosition = Motion.HomePosition;

            %create matrix that will store unique identifer to the paper so
            %they can be called in the future
            paperUniqueID = cell(paperNo, 1);

            % Create and store the paper identifier values via the loop.
            % spawn in brick at 0,0,0. translate by the initial brick
            % location matrix
            for paperIndex = 1:paperNo
                paperUniqueID{paperIndex} = PlaceObject('papersheet_industrial_new.ply'); %brickMatrix(brickIndex, :)
                vertices = get(paperUniqueID{paperIndex}, 'Vertices');
                transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(initialPaper(paperIndex,:))';
                set(paperUniqueID{paperIndex},'Vertices',transformedVertices(:,1:3));
            end

            %Create a nested for loop to iterate through the robot arms
            %path and update the animation

            %For each brick 1-9:
            for paperIndex = 1:paperNo

                %finding matrix positions necessary
                initialLocation = initialPaper(paperIndex,:);
                middleLocation = boardLocation;
                finalLocation = finalPaper(paperIndex,:);

                %moving paper from initial stack to drawing board
                placePaper(paperIndex, initialLocation, middleLocation, HomePosition);

                %draw on page 
                draw();

                %moving paper from drawing board to final stack
                placePaper(paperIndex, middleLocation, finalLocation, HomePosition);
            end

            function placePaper(paperIndex,initialLocation,finalLocation,HomePosition)

                initialPosition = r1.model.getpos();

                %set count variable to dictate how many joint angle paths
                %will be created to model the Path.
                count = 100;

                %get the robots current arm location
                robotLocation = r1.model.getpos();

                %calculate the joint angles necessary to traverse to the
                %chosen paper location. paper is rotated so the Z-axis is
                %facing down here so that the robot grabs the paper from
                %the top.
                currentPaperPath = r1.model.ikcon(transl(initialLocation)*troty(pi));

                %computes a joint space trajectory that inrerpolates
                %between the robots position and chosen brick location.
                currentQPath = jtraj(robotLocation, currentPaperPath, count);

                %for each joint step
                for j = 1:size(currentQPath, 1)
                    %animate the robots arm
                    r1.model.animate(currentQPath(j, :));
                    drawnow();
                end

                %% move to final Paper stack location
                %calculate the joint angles necessary to traverse to the
                %paper location. paper is rotated so the Z-axis is
                %facing down here so that the robot places the paper from
                %the top.
                finalPaperPath = r1.model.ikcon(transl(finalLocation)*troty(pi));

                %computes a joint space trajectory that inrerpolates
                %between the robots position and final paper location.
                finalQPath = jtraj(currentPaperPath, finalPaperPath, count);

                %for each joint step
                for h = 1:size(finalQPath, 1)

                    %animate arm
                    r1.model.animate(finalQPath(h, :));

                    %calculate the end effector transform of the robot arm
                    endEffectorTransform = r1.model.fkine(r1.model.getpos()).T;

                    %update paper vertices to the new coordinates of the
                    %robots end effector
                    transformedBrickVertices = [vertices, ones(size(vertices, 1), 1)]*endEffectorTransform';

                    %update the paper vertices with the transformed
                    %vertices
                    set(paperUniqueID{paperIndex}, 'Vertices', transformedBrickVertices(:, 1:3));
                    drawnow();
                end

                %%move robot back to initial position
                HomePosition = r1.model.ikcon(transl([0,0.5,1.1]));
                count = 100;
                robotLocation = r1.model.getpos();
                QPath = jtraj(robotLocation, HomePosition, count);
                for h = 1:size(QPath, 1)
                    r1.model.animate(QPath(h, :));
                    drawnow();
                end
            end
            function draw()
                
                count = 50;
                penDrawing = Motion.penDrawing;
                rsp = r2.model.getpos;
                rspFinal = rsp;
                r2p1 = r2.model.ikcon(transl(penDrawing(1,:)));
                r2p2 = r2.model.ikcon(transl(penDrawing(2,:)));
                r2p3 = r2.model.ikcon(transl(penDrawing(3,:)));
                r2p4 = r2.model.ikcon(transl(penDrawing(4,:)));
                r2p5 = r2.model.ikcon(transl(penDrawing(5,:)));
                r2p6 = r2.model.ikcon(transl(penDrawing(6,:)));
                r2p7 = r2.model.ikcon(transl(penDrawing(7,:)));
                r2p8 = r2.model.ikcon(transl(penDrawing(8,:)));
                r2p9 = r2.model.ikcon(transl(penDrawing(9,:)));
                r2p10 = r2.model.ikcon(transl(penDrawing(10,:)));

                rspArray = [rsp,r2p1,r2p2,r2p3,r2p4,r2p5,r2p6,r2p7,r2p8,r2p9,r2p10,rspFinal]; 


                for i =1:numel(rspArray)
                    if i == 12
                        break;
                    else 
                        qPath = jtraj(rspArray(i),rspArray(i+1),count);
                        for j = 1:length(qPath)
                            r2.model.animate(qPath(j,:));
                            drawnow();
                        end
                    end
                end

            end
        end

        function initialPaper = initialPaperMatrix()
            %initial paper stack
            paperNo = Motion.paperNo;

            initialPaperMatrix = zeros(paperNo,3);
            %initialPaperMatrix(1,:) = ....
            initialPaperMatrix(1,:) = [0.2, -0.315, 0.7];
            initialPaperMatrix(2,:) = [0.2, -0.315, 0.71];
            initialPaperMatrix(3,:) = [0.2, -0.315, 0.72];
            initialPaperMatrix(4,:) = [0.2, -0.315, 0.73];
            initialPaperMatrix(5,:) = [0.2, -0.315, 0.74];
            initialPaper = initialPaperMatrix;
        end

        function finalPaper = finalPaperMatrix()
            %final paper stack
            paperNo = Motion.paperNo;

            finalPaperMatrix = zeros(paperNo,3);
            %finalPaperMatrix(1,:) = ....
            finalPaperMatrix(1,:) = [-0.2, 0.315, 0.7];
            finalPaperMatrix(2,:) = [-0.2, 0.315, 0.71];
            finalPaperMatrix(3,:) = [-0.2, 0.315, 0.72];
            finalPaperMatrix(4,:) = [-0.2, 0.315, 0.73];
            finalPaperMatrix(5,:) = [-0.2, 0.315, 0.74];
            finalPaper = finalPaperMatrix;
        end

        function boardLocation = drawingBoardMatrix()
            %drawing board
            drawingBoardMatrix = zeros(1,3);
            drawingBoardMatrix(1,:) = [-0.2,0.315,0.7];
            boardLocation = drawingBoardMatrix;
        end

        function HomePosition = HomePositionMatrix()
            %Home Position
            HomePositionMatrix = zeros(1,3);
            HomePositionMatrix(1,:) = [0,0.5,1.1];
            HomePosition = HomePositionMatrix;
        end

        function penDrawing = penDrawingMatrix()

            ColourPoints = Motion.ColourPoints;

            penDrawingMatrix = zeros(ColourPoints,3);
            penDrawingMatrix(1,:) = [-0.2, 0.4, 0.73];
            penDrawingMatrix(2,:) = [-0.2, 0.3, 0.73];
            penDrawingMatrix(3,:) = [-0.2, 0.2, 0.73];
            penDrawingMatrix(4,:) = [-0.2, 0.1, 0.73];
            penDrawingMatrix(5,:) = [-0.2, 0, 0.73];
            penDrawingMatrix(6,:) = [-0.2, -0.1, 0.73];
            penDrawingMatrix(7,:) = [-0.2, -0.2, 0.73];
            penDrawingMatrix(8,:) = [-0.2, -0.3, 0.73];
            penDrawingMatrix(9,:) = [-0.2, -0.4, 0.73];
            penDrawingMatrix(10,:) = [-0.2, 0, 0.73];
            penDrawing = penDrawingMatrix;
        end


    end
end