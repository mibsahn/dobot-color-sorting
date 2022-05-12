%% LAB ASSIGNMENT 2
clear;  clc;

%% INITIALISATION

q0 = zeros(1,5);
q1 = [-pi/2 pi/2 0 0 0];
dobot = Dobot;
dobot.model.plotopt = {'noname', 'noshadow'}; %'nojoints', , 'nowrist'
view(3);
axis tight;
dobot.model.plot([-pi/2 pi/3 -pi/3 0 ]);
hold on
%% GUI

%% SIMULATION

%% Visual servoing

%% Colision avoidance

%% Animation

% Resolved Motion Rate Control (RMRC)

% Trajectory planning

totalTime = 10;
deltaTime = 0.4;
steps = totalTime/deltaTime;
delta = 2*pi/steps;
minMani = 0.1;

qdot = zeros(steps,4);          % Array for joint velocities
qMat = zeros(steps,4,3);       % Array for joint state
wayPoints = 2;
wayPointRMRC = [0.3    0.0   (0.15+0.0754); ...
                0.3    0.0   (0.02+0.0754)];
wayPointMat = [0    -0.3     0;
               0.3   0    0.15;
               0.3   0.0  (0.02+0.0754);
               0    -0.27   0;
               0.03 -0.27   0];
trans = zeros(3,steps);
rot = zeros(3,steps);
RMRC(dobot, steps, deltaTime);
Trapezoidal(dobot,[0 -0.3 0; 0.3 0 0.15],steps);

s = lspb(0,1,steps);                                    % Trapezoidal trajectory scalar
for i = 1:wayPoints-1
    for j = 1:steps
        trans(1,j,i) = (1-s(j))*wayPointRMRC(i,1) + s(j)*wayPointRMRC(i+1,1);             % Points in x
        trans(2,j,i) = (1-s(j))*wayPointRMRC(i,2) + s(j)*wayPointRMRC(i+1,2);            % Points in y
        trans(3,j,i) = (1-s(j))*wayPointRMRC(i,3) + s(j)*wayPointRMRC(i+1,3);                % Points in z
        rot(3,j,i) = 0;                                       % Yaw angle
        rot(2,j,i) = 0;
        rot(1,j,i) = 0;
    end
    startPos = makehgtform('translate', trans(:,1,i));
    qMat(1,:,i) = dobot.model.ikcon(startPos, [0 pi/6 -pi/3 0]);
end


%%
for i = 1:wayPoints-1
    text_h = text(-0.5, 0, 0.5, ['Waypoint ' num2str(i)], 'FontSize', 10, 'Color', [.6 .2 .6]);
    for j = 1:steps-1
        T = dobot.model.fkine(qMat(j,:,i));               % Get forward transformation at current joint state
        deltaTrans = trans(:,j+1,i) - T(1:3,4);         % Get position error from next waypoint
        Rd = rpy2r(rot(1,i+1),rot(2,i+1),rot(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaTime)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        veloRot = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        veloTrans = deltaTrans / deltaTime;             % Calculate velocity at discrete time step
        xdot = [veloTrans; veloRot];                          % Calculate end-effector velocity to reach next waypoint
        J = dobot.model.jacob0(qMat(j,:,i));         % Jacobian at current pose
        % Check Manipulability
        m = sqrt(det(J*J'));
        if m < minMani
            lambda = 0.01;      %(1 - m(i)/minMani)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda*eye(4))*J';                                   % DLS Inverse
        qdot(j,:,i) = (invJ * xdot)';                             % Singularity avoidance with DLS
%         for j = 1:6                                                             % Loop through joints 1 to 6
%             if qMatrix(i,j) + deltaT*qdot(i,j) < p560.qlim(j,1)                     % If next joint angle is lower than joint limit...
%                 qdot(i,j) = 0; % Stop the motor
%             elseif qMatrix(i,j) + deltaT*qdot(i,j) > p560.qlim(j,2)                 % If next joint angle is greater than joint limit ...
%                 qdot(i,j) = 0; % Stop the motor
%             end
%         end
        qMat(j+1,:,i) = qMat(j,:,i) + deltaTime*qdot(j,:,i);        % Update next joint state
        dobot.model.animate(qMat(j,:,i));
        drawnow();
        pause(0.5);
    end
    delete(text_h);
end

%%
% Trapezoidal(dobot,wayPointMat(3:4,:),steps);
% Trapezoidal(dobot,[wayPointMat(4,:);wayPointMat(2,:)],steps);
% Trapezoidal(dobot,[wayPointMat(3,:);wayPointMat(5,:)],steps);
% Trapezoidal(dobot,[wayPointMat(5,:);wayPointMat(2,:)],steps);

%%
sugarCane = PlaceObj('SugarCaneR.ply');
TrapezoidalObject(dobot,[wayPointMat(3,:);wayPointMat(5,:)],steps, sugarCane);
% PlaceObject('SugarCaneY.ply');
%% PLOTTING
% figure(1)
% plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
% p560.plot(qMatrix,'trail','r-')
%
% for i = 1:6
%     figure(2)
%     subplot(3,2,i)
%     plot(qMatrix(:,i),'k','LineWidth',1)
%     title(['Joint ', num2str(i)])
%     ylabel('Angle (rad)')
%     refline(0,p560.qlim(i,1));
%     refline(0,p560.qlim(i,2));
%
%     figure(3)
%     subplot(3,2,i)
%     plot(qdot(:,i),'k','LineWidth',1)
%     title(['Joint ',num2str(i)]);
%     ylabel('Velocity (rad/s)')
%     refline(0,0)
% end
%
% figure(4)
% subplot(2,1,1)
% plot(positionError'*1000,'LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Position Error (mm)')
% legend('X-Axis','Y-Axis','Z-Axis')
%
% subplot(2,1,2)
% plot(angleError','LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Angle Error (rad)')
% legend('Roll','Pitch','Yaw')
% figure(5)
% plot(m,'k','LineWidth',1)
% refline(0,epsilon)
% title('Manipulability')
