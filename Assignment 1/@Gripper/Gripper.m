classdef Gripper < RobotBaseClass
    properties(Access = public)
            
            plyFileNameStem = 'gripper';
          
            defaultBaseTr = eye(4);
    %         initalQ = [0, 0, -pi/2, 0, -pi/2, 0 ,0];
            defaultWorkspace = [-0.5 0.5 -0.5 0.5 -0.01 0.01];
            
        end
    

        methods
%% Constructor
function self = Gripper(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = [1,0,0,0.4; 0,1,0,0; 0,0,1,1; 0,0,0,1];  %set position of gripper
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.model.tool = self.toolTr;
%             self.homeQ = self.initalQ;
            self.workspace = self.defaultWorkspace;
            self. PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
           
            link(1) = Link('d',0,'a',0,'alpha',0,'qlim',0,'offset',0);
            link(2) = Link('d',0,'a',0,'alpha',deg2rad(90),'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',0,'alpha',deg2rad(-90),'qlim',deg2rad([-360 360]), 'offset',0);

            self.model = SerialLink(link,'name',self.name);
        end   
        
    end
end
  
