%% LAB ASSIGNMENT 2
clear;  clc;

%% INITIALISATION

q0 = zeros(1,5);
q1 = [-pi/2 pi/2 0 0 0];
dobot = DobotClass;
dobot.PlotModel3d;
view(3);
axis tight;
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

offsetZ = 0.0754;
offsetX = 0.02;
wayPointMat = [0,    -0.3,     offsetZ;
               0.3,   0,       (0.15+offsetZ);
               0.3,   0.0,     (0.02+offsetZ);
               0,    -0.27,    offsetZ;
               0.03, -0.27,    offsetZ];

% Place object (sugarcanes) in environment)
hold on
sugarCane(1) = PlaceObj('SugarCaneR.ply');
sugarCane(1).MoveObj([ [0 -0.3 -0.02] zeros(1,3)]);
sugarCane(2) = PlaceObj('SugarCaneY.ply');
sugarCane(2).MoveObj([ [0 -0.27 -0.02] zeros(1,3)]);
sugarCane(3) = PlaceObj('SugarCaneY.ply');
sugarCane(3).MoveObj([ [0.03 -0.27 -0.02] zeros(1,3)]);

% v = VideoWriter('Dobot.avi');
% open(v);

PlotEnvironment('Title.jpg',[-0.7 0.7; -0.7 0.7],[0.7 0.7; 0.7 0.7],[0.5 0.5; -0.22 -0.22]);
PlotEnvironment('_0.jpg',[-0.7 -0.7; 0.7 0.7],[-0.7 0.7; -0.7 0.7],[-0.22 -0.22; -0.22 -0.22]);
PlaceObject('Table.ply',[0 0 0]);
PlaceObject('SmallTable.ply',[0.02 -0.27 -0.1]);
PlaceObject('SmallTable.ply',[0.31 0.16 -0.1]);
PlaceObject('Box.ply',[0.02 -0.27 -0.1]);
PlaceObject('Box.ply',[0.30 0 -0.22]);
PlaceObject('SugarCaneJuiceExtractor.ply', [0.31 0 0]);
axis tight
camlight;
pause(10);

TrapezoidalObject(dobot,[wayPointMat(1,:);wayPointMat(2,:)],steps, sugarCane(1));
RMRC(dobot, steps, deltaTime, sugarCane(1));
Trapezoidal(dobot,wayPointMat(3:4,:),steps);
TrapezoidalObject(dobot,[wayPointMat(4,:);wayPointMat(2,:)],steps, sugarCane(2));
RMRC(dobot, steps, deltaTime, sugarCane(2));
Trapezoidal(dobot,[wayPointMat(3,:);wayPointMat(5,:)],steps);
TrapezoidalObject(dobot,[wayPointMat(5,:);wayPointMat(2,:)],steps, sugarCane(3));
RMRC(dobot, steps, deltaTime, sugarCane(3));
Trapezoidal(dobot,[wayPointMat(3,:);wayPointMat(2,:)],steps);

% frame = getframe(gcf);
% writeVideo(v,frame);
% 
% close(v);

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
