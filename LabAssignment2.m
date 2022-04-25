%% LAB ASSIGNMENT 2
clear;  clc;
%% INITIALISATION

%% GUI

%% SIMULATION

%% Colision avoidance

%% Animation

% Resolved Motion Rate Control (RMRC)

% Trajectory planning

steps = 20;
minMani = 0.1;
posStart = [];
posEnd = [];
initialPos = [];
q = ikcon(initialPos);

for i = 1:steps
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
