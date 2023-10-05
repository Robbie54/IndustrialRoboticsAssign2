classdef LinearUR3 < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'LinearUR3';
        defaultBaseTr = eye(4);
%         initalQ = [0, 0, -pi/2, 0, -pi/2, 0 ,0];
        defaultWorkspace = [-0.5 0.5 -0.5 0.5 -0.01 0.01];
        
    end
    
    methods
%% Constructor
function self = LinearUR3(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = [1,0,0,0.4; 0,1,0,0; 0,0,1,1; 0,0,0,1];  
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
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            link(2) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-90 90]), 'offset',0);
            link(4) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-170 170]), 'offset', 0);
            link(5) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            link(1).qlim = [-0.8 -0.01];

            link(3).offset = -pi/2;
            link(5).offset = -pi/2;
             
            self.model = SerialLink(link,'name',self.name);
        end   
        
    end
end