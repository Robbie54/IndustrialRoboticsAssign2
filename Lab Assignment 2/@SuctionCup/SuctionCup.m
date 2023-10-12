classdef SuctionCup < handle
    %ROBOTCOWS A class that creates a herd of robot cows
	%   The cows can be moved around randomly. It is then possible to query
    %   the current location (base) of the cows.    
    
    %#ok<*TRYNC>    

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 3;
    end
    
    properties
        %> Number of cows
        cupCount = 1;
        
        %> A cell structure of \c cupCount cow models
        suctionModel;
        
        %> workszone in meters
        workszone = [2,2];        
        
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = SuctionCup(cupCount)
            if 0 < nargin
                self.cupCount = cupCount;
            end
            
            self.workspaceDimensions = [-self.workszone(1)/2, self.workszone(1)/2 ...
                                       ,-self.workszone(2)/2, self.workszone(2)/2 ...
                                       ,0,self.maxHeight];

            % Create the required number of cows
            for i = 1:self.cupCount
                self.suctionModel{i} = self.GetsuctionModel(['cow',num2str(i)]);
                % Random spawn
                basePose = SE3(SE2((2 * rand()-1) * self.workszone(1)/2 ...
                                         , (2 * rand()-1) * self.workszone(2)/2 ...
                                         , (2 * rand()-1) * 2 * pi));
                self.suctionModel{i}.base = basePose;
                
                 % Plot 3D model
                plot3d(self.suctionModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            axis equal
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
        end
        
        function delete(self)
            for index = 1:self.cupCount
                handles = findobj('Tag', self.suctionModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end       
        
        %% PlotSingleRandomStep
        % Move each of the cows forward and rotate some rotate value around
        % the z axis
        function PlotSingleRandomStep(self)
            for brickIndex = 1:self.cupCount
                % Move Forward
                self.suctionModel{brickIndex}.base = self.suctionModel{brickIndex}.base * SE3(SE2(0.2, 0, 0));
                animate(self.suctionModel{brickIndex},0);
                
                % Turn randomly
                % Save base as a temp variable
                tempBase = self.suctionModel{brickIndex}.base.T;
                rotBase = tempBase(1:3, 1:3);
                posBase = tempBase(1:3, 4);
                newRotBase = rotBase * rotz((rand-0.5) * 30 * pi/180);
                newBase = [newRotBase posBase ; zeros(1,3) 1];
                           
                % Update base pose
                self.suctionModel{brickIndex}.base = newBase;
                animate(self.suctionModel{brickIndex},0);                

                % If outside workspace rotate back around
                % Get base as temp
                tempBase = self.suctionModel{brickIndex}.base.T;
                
                if tempBase(1,4) < self.workspaceDimensions(1) ...
                || self.workspaceDimensions(2) < tempBase(1,4) ...
                || tempBase(2,4) < self.workspaceDimensions(3) ...
                || self.workspaceDimensions(4) < tempBase(2,4)
                    self.suctionModel{brickIndex}.base = self.suctionModel{brickIndex}.base * SE3(SE2(-0.2, 0, 0)) * SE3(SE2(0, 0, pi));
                end
            end
            % Do the drawing once for each interation for speed
            drawnow();
        end    
        
        %% TestPlotManyStep
        % Go through and plot many random walk steps
        function TestPlotManyStep(self,numSteps,delay)
            if nargin < 3
                delay = 0;
                if nargin < 2
                    numSteps = 200;
                end
            end
            for i = 1:numSteps
                self.PlotSingleRandomStep();
                pause(delay);
            end
        end
    end
    
    methods (Static)
        %% GetsuctionModel
        function model = GetsuctionModel(name)
            if nargin < 1
                name = 'Brick';
            end
            [faceData,vertexData] = plyread(['suction_cup_ur3e.ply'],'tri');
            link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end