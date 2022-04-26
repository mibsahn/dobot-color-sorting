%% LAB ASSIGNMENT 2
clear;  clc;
%% INITIALISATION

%% GUI

%% SIMULATION

%% Colision avoidance

%% Animation

% Resolved Motion Rate Control (RMRC)

% Trajectory planning

totalTime = 10;
deltaTime = 0.1;
steps = totalTime/deltaTime;
delta = 2*pi/steps;
minMani = 0.1;
posStart = [];
posEnd = [];
initialPos = [];

trans = zeros(3,steps);
rot = zeros(1,steps);

s = lspb(0,1,steps);                                    % Trapezoidal trajectory scalar
for i=1:steps
    trans(1,i) = (1-s(i))*0.35 + s(i)*0.35;             % Points in x
    trans(2,i) = (1-s(i))*-0.55 + s(i)*0.55;            % Points in y
    trans(3,i) = 0.5 + 0.2*sin(i*delta);                % Points in z
    rot(1,i) = 0;                                       % Yaw angle
end


q0 = zeros(1,6);
q = ikcon(initialPos, q0);



for i = 1:steps-1
    % Calculate velocity at discrete time step
    
    xdot = [];
    
    % Jacobian
    
    J = [];
    
    % Manipulability
    
    
    m = sqrt(det(J*J'));
    if m < minMani
        qdot = inv(J'*J + 0.01*eye(2))*J'*xdot;                             % Singularity avoidance with DLS
  
    % Solve velocitities via RMRC
    else
        qdot = inv(J) * xdot;                                               % Solve velocitities via RMRC
    end
    error(:,i) = xdot - J*qdot;
    
    % Animation
    
    % Update next joint state
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot';
end
