%% LAB ASSIGNMENT 2
clear; close; clf; clc;
addpath(genpath('./Addition'));
%% INITIALISATION

eStop = false;
q0 = zeros(1,5);
dobot = DobotClass;
% dobot.model.plot(q0);
dobot.PlotModel3d;
view(3)
axis tight
hold on

% Initialise working environment of Dobot
sugarCane = PlotEnvironment;
axis tight
camlight;
pause(4);
%% GUI
g = gui();
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
                0.03, -0.27,    offsetZ;
                0,    -0.3,     0.2+offsetZ;
                0.3,   0,       0.2+offsetZ;
                0     -0.3      (0.15+offsetZ)];

% Place obstacle in environment
hold on
%%
% RMRC halt on obstacle detected
qMat = dobot.trapezoidal([wayPointMat(1,:);wayPointMat(6,:)],steps);
qMat = [qMat;dobot.trapezoidal([wayPointMat(6,:);wayPointMat(7,:)],steps)];
qMat = [qMat;dobot.trapezoidal([wayPointMat(7,:);wayPointMat(2,:)],steps)];

% dobot.animate(qMat,sugarCane(1));
dobot.trapezoidal([wayPointMat(2,:);wayPointMat(1,:)],steps, true); % Get to pick-up position
disp('Moving first sugar cane.');
dobot.trapezoidal([wayPointMat(1,:);wayPointMat(8,:)],steps, true, sugarCane(1));   % Pick up first cane
dobot.trapezoidal([wayPointMat(8,:);wayPointMat(2,:)],steps, true, sugarCane(1));
dobot.rmrc(steps, deltaTime, sugarCane(1));

%%
dobot.trapezoidal(wayPointMat(3:4,:),steps,true);
disp('Moving second sugar cane.');
dobot.trapezoidal([wayPointMat(4,:);wayPointMat(8,:)],steps, true, sugarCane(2));   % Pick up second cane
dobot.trapezoidal([wayPointMat(8,:);wayPointMat(2,:)],steps, true, sugarCane(2));
dobot.rmrc(steps, deltaTime, sugarCane(2));

plotOpts.plotFaces = true;
for i = 1:10
    try delete(jug.faces); end;
    [vertex,faces,faceNormals,jug] = RectangularPrism([(-0.3+0.05*i),-0.2,0],[(-0.4+0.05*i),-0.1,0.15],plotOpts);
    pause(0.02);
end

qMtx = dobot.trapezoidal([wayPointMat(3,:);wayPointMat(5,:)],steps); % Secure planned joint state of robot

for i = 1: size(qMtx,1)
    [intersectP, result(i)] = IsCollision(dobot.model,qMtx(i,:),faces,vertex,faceNormals,false);
    if result(i)
        break;
    end
end

qMat = [qMat;qMtx(1:(i-3),:)];
for j = 1:i-3
    dobot.model.animate(qMtx(j,:));
    pause(0.02)
end

T = dobot.model.fkine(qMat(j,:));
dobot.trapezoidal([T(1:2,4)' 0.02+offsetZ; T(1:2,4)' 0.12+offsetZ],steps,true);
dobot.trapezoidal([T(1:2,4)' 0.12+offsetZ; wayPointMat(5,1:2) 0.12+offsetZ],steps,true);
dobot.trapezoidal([wayPointMat(5,1:2) 0.12+offsetZ; wayPointMat(5,:)],steps,true);

for i = 1:10
    try delete(jug.faces); end;
    [vertex,faces,faceNormals,jug] = RectangularPrism([(0.2+0.05*i),-0.2,0],[(0.1+0.05*i),-0.1,0.15],plotOpts);
    pause(0.02);
end

disp('Moving third sugar cane.');
dobot.trapezoidal([wayPointMat(5,:);wayPointMat(8,:)],steps, true, sugarCane(3));
dobot.trapezoidal([wayPointMat(8,:);wayPointMat(2,:)],steps, true, sugarCane(3));
dobot.rmrc(steps, deltaTime, sugarCane(3));
dobot.trapezoidal([wayPointMat(3,:);wayPointMat(2,:)],steps);
disp('Your sugar cane juice is ready! Please enjoy!');
disp('Lab Assignment 2 ends.');

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function [intersectP, result] = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
            if (check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:)))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end
    end
end
end
%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is
% inside (result == 1) or
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end
%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end