%% LAB ASSIGNMENT 2
clear;  clc;

%% INITIALISATION

q0 = zeros(1,4);
dobot = Dobot;
dobot.model.plotopt = {'noname', 'noshadow'}; %'nojoints', , 'nowrist'
dobot.model.plot(q0);

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
posObjStart = [0.0 0.2 0.0];
posObjEnd = [0.2 0.0 0.0];
initialPos = q0;
qMatrix = zeros(steps,4);       % Array for joint anglesR
qdot = zeros(steps,4);          % Array for joint velocities

trans = zeros(3,steps);
rot = zeros(1,steps);

s = lspb(0,1,steps);                                    % Trapezoidal trajectory scalar
for i=1:steps
    trans(1,i) = (1-s(i))*0.35 + s(i)*0.35;             % Points in x
    trans(2,i) = (1-s(i))*-0.55 + s(i)*0.55;            % Points in y
    trans(3,i) = 0.5 + 0.2*sin(i*delta);                % Points in z
    rot(1,i) = 0;                                       % Yaw angle
end



% q = ikcon(initialPos, q0);



for i = 1:steps-1
    % Calculate velocity at discrete time step
    
    xdot = [];
    
    % Jacobian
    
    J = dobot.model.jacob0(qMatrix);
    
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

%% PLOTTING
figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
p560.plot(qMatrix,'trail','r-')

for i = 1:6
    figure(2)
    subplot(3,2,i)
    plot(qMatrix(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    ylabel('Angle (rad)')
    refline(0,p560.qlim(i,1));
    refline(0,p560.qlim(i,2));
    
    figure(3)
    subplot(3,2,i)
    plot(qdot(:,i),'k','LineWidth',1)
    title(['Joint ',num2str(i)]);
    ylabel('Velocity (rad/s)')
    refline(0,0)
end

figure(4)
subplot(2,1,1)
plot(positionError'*1000,'LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Position Error (mm)')
legend('X-Axis','Y-Axis','Z-Axis')

subplot(2,1,2)
plot(angleError','LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Angle Error (rad)')
legend('Roll','Pitch','Yaw')
figure(5)
plot(m,'k','LineWidth',1)
refline(0,epsilon)
title('Manipulability')
