classdef DobotClass < handle
    properties
        model;
        simulation;
        eStop = false;
        sensorP;
        toolOffset = transl(0.06,0,0.065);
        workspace = [-0.5 0.5 -0.5 0.5 -0.7814 0.5];
        qNeutral = [0,deg2rad(45),deg2rad(90),deg2rad(45),0];
        qSimulation = [0,deg2rad(45),deg2rad(90),deg2rad(45)];
        draw = 0;
    end
    methods
        %% Constructor
        function self = DobotClass()
            location = transl(0,0,0);
            CreateDobot(self,location);
        end
        %% E-stop function
        function Stopcheck(self)
            while(self.eStop == true)
                disp('E-stop pressed');
                pause(0.05);
            end
        end
        %% Creating the Dobot both model with attachment and simulation
        function CreateDobot(self, location)
            
            pause(0.001);
            
            name = ['Dobot',datestr(now,'yyyymmddTHHMMSSFFF')];
            L1 = Link('d',0.137,'a',0,'alpha',-pi/2,'offset',0,'qlim', deg2rad([-135 135]));
            L2 = Link('d',0,'a',0.1393,'alpha',0,'offset',-pi/2, 'qlim', deg2rad([5 80]));
            L3 = Link('d',0,'a',0.16193,'alpha',0,'offset',0, 'qlim', deg2rad([15 170])); %pi/2 –q2 actual offset
            L4 = Link('d',0,'a',0.061,'alpha',pi/2,'offset',-pi/2, 'qlim', [-pi/2 pi/2]);
            L5 = Link('d',0,'a',0,'alpha',0,'offset',0, 'qlim', deg2rad([-85 85]));
            self.model = SerialLink([L1 L2 L3 L4 L5], 'name', name);
            
            pause(0.001);
            
            name = ['DobotMovementSimulation'];
            L1 = Link('d',0.137,'a',0,'alpha',-pi/2,'offset',0,'qlim', deg2rad([-135 135]));
            L2 = Link('d',0,'a',0.1393,'alpha',0,'offset',-pi/2, 'qlim', deg2rad([5 80]));
            L3 = Link('d',0,'a',0.16193,'alpha',0,'offset',0, 'qlim', deg2rad([15 170])); %pi/2 –q2 actual offset
            L4 = Link('d',0,'a',0.061,'alpha',pi/2,'offset',-pi/2, 'qlim', [-pi/2 pi/2]);
            self.simulation = SerialLink([L1 L2 L3 L4], 'name', name);
            
            self.model.base = location;
            self.simulation.base = location;
        end
        %% Plot Stick figure of the robot
        function PlotRobot(self,location)
            self.model.base = location;
            self.model.plot(self.qNeutral);
        end
        %% Plot 3D model with attachment
        function PlotModel3d(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['dobot_Link',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            self.model.plot3d(self.qNeutral,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
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
        %% Plot 3D simulation Dobot - no attachments
        function PlotSimulation3d(self)
            for linkIndex = 0:self.simulation.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['dobot_Link',num2str(linkIndex),'.ply'],'tri');
                self.simulation.faces{linkIndex + 1} = faceData;
                self.simulation.points{linkIndex + 1} = vertexData;
            end
            self.simulation.plot3d(self.qSimulation,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.simulation.delay = 0;
            for linkIndex = 0:self.simulation.n
                handles = findobj('Tag', self.simulation.name);
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
        %% Animate qMatrix
        function animate(self, qMatrix, payload)
            for i = 1:size(qMatrix,1)
                self.model.animate(qMatrix(i,:));
                if (nargin > 2)
                    effectorPos = self.model.fkine(qMatrix(i,:));
                    payload.MoveObj([(effectorPos(1:3,4)'+[0.02 0 -0.0954]) 0 0 0]);
                end
                drawnow();
                pause(0.02)
            end
        end
        %% RMRC
        function rmrc(self, steps, deltaTime, obj)
        %RMRC Move robot in RMRC
        %   Generate trajectory and animate robot motion
            
            minMani = 0.1;
            wayPoints = 2;
            wayPointRMRC = [0.3    0.0   (0.15+0.0754);
                            0.3    0.0   (0.02+0.0754)];
            
            qMat = zeros(steps,5);
            trans = zeros(3,steps);
            rot = zeros(3,steps);
            
            s = lspb(0,1,steps);                                    % Trapezoidal trajectory scalar
            for i = 1:wayPoints-1
                for j = 1:steps
                    trans(1,j,i) = (1-s(j))*wayPointRMRC(i,1) + s(j)*wayPointRMRC(i+1,1);            % Points in x
                    trans(2,j,i) = (1-s(j))*wayPointRMRC(i,2) + s(j)*wayPointRMRC(i+1,2);            % Points in y
                    trans(3,j,i) = (1-s(j))*wayPointRMRC(i,3) + s(j)*wayPointRMRC(i+1,3);            % Points in z
                    rot(:,j,i) = zeros(3,1);                                       % Yaw angle
                end
                startPos = makehgtform('translate', trans(:,1,i));
                qMat(1,:,i) = self.model.ikcon(startPos, [0 pi/3 -pi/3 0 0]);
            end

            for i = 1:wayPoints-1
                text_h = text(-0.5, 0.5, 0.45, ['RMRC Mode Juice Extraction'], 'FontSize', 12, 'Color', [.6 .2 .6]);
                for j = 1:steps-1
                    if self.sensors_sim(qMat(j,:,i))
                        disp('Obstacle detected, halt operation!')
                        return
                    end
                    T = self.model.fkine(qMat(j,:,i));               % Get forward transformation at current joint state
                    deltaTrans = trans(:,j+1,i) - T(1:3,4);         % Get position error from next waypoint
                    Rd = rpy2r(rot(1,i+1),rot(2,i+1),rot(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                    Rdot = (1/deltaTime)*(Rd - Ra);                                                % Calculate rotation matrix error
                    S = Rdot*Ra';                                                           % Skew symmetric!
                    veloRot = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                    veloTrans = deltaTrans / deltaTime;             % Calculate velocity at discrete time step
                    xdot = [veloTrans; veloRot];                          % Calculate end-effector velocity to reach next waypoint
                    J = self.model.jacob0(qMat(j,:,i));         % Jacobian at current pose
                    % Check Manipulability
                    m = sqrt(det(J*J'));
                    if m < minMani
                        lambda = 0.01;      %(1 - m(i)/minMani)*5E-2;
                    else
                        lambda = 0;
                    end
                    invJ = inv(J'*J + lambda*eye(5))*J';                                   % DLS Inverse
                    qdot(j,:,i) = (invJ * xdot)';                             % Singularity avoidance with DLS
            %         for j = 1:6                                                             % Loop through joints 1 to 6
            %             if qMatrix(i,j) + deltaT*qdot(i,j) < p560.qlim(j,1)                     % If next joint angle is lower than joint limit...
            %                 qdot(i,j) = 0; % Stop the motor
            %             elseif qMatrix(i,j) + deltaT*qdot(i,j) > p560.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            %                 qdot(i,j) = 0; % Stop the motor
            %             end
            %         end
                    qMat(j+1,:,i) = qMat(j,:,i) + deltaTime*qdot(j,:,i);        % Update next joint state
                    self.model.animate(qMat(j,:,i));
                    obj.MoveObj([(T(1:3,4)'+[0.02 0 -0.0954]) 0 0 0]);
                    drawnow();
                    pause(0.02);
                end
                delete(text_h);
            end
            obj.MoveObj([0.3 0 -0.1 0 0 0]);
            pause(0.2);
            obj.MoveObj([0.3 0 -0.16 0 0 0]);
        end
        %% Sensors sim
        function bool = sensors_sim(self,q)
            tf = self.model.fkine(q);
            eefP = tf(1:3,4)';
            eefP(3) = eefP(3) - 0.0754;
            if norm(eefP-self.sensorP) < 0.1
                bool = 1;
            else
                bool = 0;
            end
        end
        %% Calculate Trajectory
%         function angleMatrix = CalculateTrajectory(self,initial_q,final_q,steps)
%             scalar = lspb(0,1,steps);
%             angleMatrix = nan(steps,self.model.n);
%             for i = 1:steps
%                 angleMatrix(i,:) = (1-scalar(i))*initial_q + scalar(i)*final_q;
%             end
%         end
%         %% Move Robot
%         function GoTo(self,x,y,steps,boolean)
%             
%             if(self.draw == 0)
%                 z = 0.05;
%             else
%                 z = 0;
%             end
%             
%             location = transl(x,y,z);
%             angle = atan(location(2,4) / location(1,4));
%             xOffset = cos(angle) * self.toolOffset(1,4);
%             yOffset = sin(angle) * self.toolOffset(1,4);
%             location = location * transl(xOffset,yOffset,self.toolOffset(3,4));
%             robotJoints = self.model.getpos();
%             newJoints = self.model.ikcon(location);
%             jointMatrix = self.CalculateTrajectory(robotJoints, newJoints, steps);
%             
%             for i = 1:steps
%                 self.Stopcheck();
%                 jointMatrix(i,5) = 0;
%                 self.model.animate(jointMatrix(i,:));
%                 if(boolean)
%                     self.DrawingSpace();
%                 end
%                 pause(0.02);
%                 self.Stopcheck();
%             end
%         end
%         %% Dobot Requirement
%         function requirement(self)
%             load dobot_q
%             for i = 1:338
%                 q1REAL = dobot_q(i,1);
%                 q2REAL = dobot_q(i,2);
%                 q3REAL = dobot_q(i,3);
%                 q4REAL = pi - (q2REAL + q3REAL);
%                 
%                 q3MODEL = pi/2 - q2REAL + q3REAL;
%                 q4MODEL = pi - (q2REAL + q3MODEL);
%                 
%                 qMove = [q1REAL,q2REAL,q3MODEL,q4MODEL];
%                 self.simulation.animate(qMove);
%                 drawnow();
%                 pause(0.05);
%             end
%         end
%         %% Dobot move both real & simulation
%         function moveBothJoints(self, q1, q2, q3)
%             if(self.eStop == false)
%                 q1REAL = q1;
%                 q2REAL = q2;
%                 q3REAL = q3;
%                 q4REAL = pi - (q2REAL + q3REAL);
%                 
%                 q3MODEL = pi/2 - q2REAL + q3REAL;
%                 q4MODEL = pi - (q2REAL + q3MODEL);
%                 q5MODEL = 0;
%                 
%                 qMoveMODEL = [q1REAL,q2REAL,q3MODEL,q4MODEL, q5MODEL];
%                 qMoveREAL = [q1REAL, q2REAL, q3REAL, q4REAL];                
%                 
%                 self.model.animate(qMoveMODEL);
%                 drawnow();
%             end
%         end
%         
%         %% Dobot move both real & simulation
%         function moveBothLocation(self, x,y,z)
%             if(self.eStop == false)
%                 location = transl(x,y,z);
%                 robotJoints = self.model.getpos();
%                 newJoints = self.model.ikcon(location);
%                 
%                 cartsvc_ = rossvcclient('/dobot_magician/PTP/set_cartesian_pos');
%                 cartmsg_.TargetPoints=[x,y,z,0];
%                 cartmsg_ = rosmessage(cartsvc_);
%                 cartmsg_.TargetPoints=[x,y,z,0];
%                 cartsvc_.call(cartmsg_)
%                 
%                 self.model.animate(newJoints);
%                 drawnow();
%             end
%         end
%         %% draw box
%         function drawBox(self)
%             for i = -0.2:0.005:0.2
%                 point = transl(0.1,i,0);
%                 point = point * self.toolOffset;
%                 newJoints = self.model.ikcon(point);
%                 self.model.animate(newJoints);
%                 self.DrawingSpace()
%                 pause(0.02);
%             end
%             for i = 0.1:0.005:0.2
%                 point = transl(i,0.2,0);
%                 point = point * self.toolOffset;
%                 newJoints = self.model.ikcon(point);
%                 self.model.animate(newJoints);
%                 self.DrawingSpace()
%                 pause(0.02);
%             end
%             
%             for i = 0.2:-0.005:-0.2
%                 point = transl(0.2,i,0);
%                 point = point * self.toolOffset;
%                 newJoints = self.model.ikcon(point);
%                 self.model.animate(newJoints);
%                 self.DrawingSpace()
%                 pause(0.02);
%             end
%             
%             for i = 0.2:-0.005:0.1
%                 point = transl(i,-0.2,0);
%                 point = point * self.toolOffset;
%                 newJoints = self.model.ikcon(point);
%                 self.model.animate(newJoints);
%                 self.DrawingSpace()
%                 pause(0.02);
%             end
%         end
%         %% lift & lower
%         function lift(self,boolean)
%             if(boolean == true)
%                 movement = transl(0,0,0.05);
%                 self.draw = 0;
%             end
%             if(boolean == false)
%                 movement = transl(0,0,-0.05);
%                 self.draw = 1;
%             end
%             jointAngles = self.model.getpos();
%             endEffector = self.model.fkine(jointAngles);
%             endEffector = endEffector * movement;
%             NewjointAngles = self.model.ikcon(endEffector);
%             jointMatrix = self.CalculateTrajectory(jointAngles, NewjointAngles, 75);
%             for i = 1:75
%                 jointMatrix(i,5) = 0;
%                 self.model.animate(jointMatrix(i,:));
%                 pause(0.02);
%                 self.Stopcheck()
%             end
%         end
%         %%
%         function    GotoREAL(self,x,y)
%             cartsvc_ = rossvcclient('/dobot_magician/PTP/set_cartesian_pos');
%             cartmsg_ = rosmessage(cartsvc_);
%             if(self.draw == true)
%                 cartmsg_.TargetPoints=[x,y,-0.08,0];
%             else
%                 cartmsg_.TargetPoints=[x,y,-0.05,0];
%             end
%             cartsvc_.call(cartmsg_)
%             
%         end
%         
%         %%
%         function    GotoREALXYZ(self,x,y,z)
%             cartsvc_ = rossvcclient('/dobot_magician/PTP/set_cartesian_pos');
%             cartmsg_ = rosmessage(cartsvc_);
%                 cartmsg_.TargetPoints=[x,y,z,0];
%             cartsvc_.call(cartmsg_)           
%         end
%         
%         
%         %% draw animation
%         function DrawingSpace(self)
%             
%             blastStartTr = self.model.fkine(self.model.getpos()) * transl(0,0,-0.04);
%             blastStartPnt = blastStartTr(1:3,4)';
%             
%             blastEndTr = self.model.fkine(self.model.getpos()) * transl(0,0,-0.08);
%             blastEndPnt = blastEndTr(1:3,4)';
%             blastPlot_h = plot3([blastStartPnt(1),blastEndPnt(1)],[blastStartPnt(2),blastEndPnt(2)],[blastStartPnt(3),blastEndPnt(3)],'r');
%             axis equal;
%             
%             planeXntersect = 0.01;
%             planeBounds = [0.15,0.62,-0.22,0.25,0,0];
%             [X,Y] = meshgrid(planeBounds(1):0.01:planeBounds(2),planeBounds(3):0.01:planeBounds(4));
%             Z = repmat(planeXntersect,size(Y,1),size(Y,2));
%             surf(X,Y,Z);
%             
%             planePnt = [0,0,0];
%             planeNormal = [0,0,1];
%             
%             [intersectionPoints,check] = self.LinePlaneIntersection(planeNormal,planePnt,blastStartPnt,blastEndPnt);
%             if check == 1
%                 intersectionPointPlot_h = plot3(intersectionPoints(:,1),intersectionPoints(:,2),intersectionPoints(:,3),'g*');
%             end
%             
%         end
%         %% LinePlaneIntersection
%         function [intersectionPoint,check] = LinePlaneIntersection(self,planeNormal,pointOnPlane,point1OnLine,point2OnLine)
%             
%             intersectionPoint = [0 0 0];
%             u = point2OnLine - point1OnLine;
%             w = point1OnLine - pointOnPlane;
%             D = dot(planeNormal,u);
%             N = -dot(planeNormal,w);
%             check = 0; %#ok<NASGU>
%             if abs(D) < 10^-7        % The segment is parallel to plane
%                 if N == 0           % The segment lies in plane
%                     check = 2;
%                     return
%                 else
%                     check = 0;       %no intersection
%                     return
%                 end
%             end
%             
%             %compute the intersection parameter
%             sI = N / D;
%             intersectionPoint = point1OnLine + sI.*u;
%             
%             if (sI < 0 || sI > 1)
%                 check= 3;          %The intersection point  lies outside the segment, so there is no intersection
%             else
%                 check=1;
%             end
%         end
    end
end
