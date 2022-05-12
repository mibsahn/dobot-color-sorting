classdef Dobot < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-0.5 0.5 -0.5 0.5 -0.3 1.5];
        
        %> Flag to indicate if gripper is used
        useGripper = false;
    end
    
    methods%% Class for UR3 robot simulation
        function self = Dobot(useGripper)
            if nargin < 1
                useGripper = false;
            end
            self.useGripper = useGripper;
            
            %> Define the boundaries of the workspace
            
            
            % robot =
            self.GetDobotRobot();
            % robot =
            self.PlotAndColourRobot();%robot,workspace);
        end
        
        %% GetDobotRobot
        % Given a name (optional), create and return a Dobot robot model
        function GetDobotRobot(self)
            
            pause(0.001);
            name = ['Dobot_M_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            L1 = Link('d',0.137,'a',0,'alpha',-pi/2,'offset',0,'qlim', deg2rad([-135 135]));
            L2 = Link('d',0,'a',0.1393,'alpha',0,'offset',-pi/2, 'qlim', deg2rad([5 80]));
            L3 = Link('d',0,'a',0.16193,'alpha',0,'offset',0, 'qlim', deg2rad([15 170])); %pi/2 –q2 actual offset
            L4 = Link('d',0,'a',0.0597,'alpha',pi/2,'offset',-pi/2, 'qlim', [-pi/2 pi/2]);
            %             L5 = Link('d',0,'a',0,'alpha',0,'offset',0, 'qlim', deg2rad([-85 85]));
            
            self.model = SerialLink([L1 L2 L3 L4],'name',name);
            
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [faceData, vertexData, plyData{linkIndex+1} ] = plyread(['dobot_Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
            
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
    end
end