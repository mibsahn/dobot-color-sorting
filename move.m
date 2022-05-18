%% Move the robot with qMat extracted from simulation
for i = 1:size(qMat,1)
    %%
    fileID = fopen('eStop.txt','r');
    A = fscanf(fileID,'%s');
    try
        eStop = (A == 'eStop');
    catch
    end
    if eStop
        send(safetyStatePublisher,safetyStateMsg);
        return
    end
    %%
    msg = endEffectorPoseSubscriber.LatestMessage;
    pos = [msg.Pose.Position.X,
          msg.Pose.Position.Y,
          msg.Pose.Position.Z];
    %%
    if (abs(pos(1)) < 0.05) & (abs(pos(2)+0.3)<0.1) & (abs(pos(3))<0.1)
        if ~state
            state = 1
            robotSuck(state)
        end
    end
    if (abs(pos(1)-0.3) < 0.05) & (abs(pos(2))<0.1) & (abs(pos(3))<0.03)
        if state
            state = 0
            robotSuck(state)
            pos
        end
    end

    robotMove(qMat(i,:))
    pause(0.05)
end
%% The pose messages sending ends before robot finish moving so we have this loop running to send eStop messages
%% and/or tool messages (suction on/off)
while true
    fileID = fopen('eStop.txt','r');safetyStateMsg
    A = fscanf(fileID,'%s');
    try
        eStop = (A == 'eStop')
    catch
    end
    if eStop
        send(safetyStatePublisher,safetyStateMsg);
        break
    end
    msg = endEffectorPoseSubscriber.LatestMessage;
    pos = [msg.Pose.Position.X,
          msg.Pose.Position.Y,
          msg.Pose.Position.Z];
    if (abs(pos(1)) < 0.05) & (abs(pos(2)+0.3)<0.1) & (abs(pos(3))<0.1)
        if ~state
            state = 1
            robotSuck(state)
        end
    end
    if (abs(pos(1)-0.3) < 0.05) & (abs(pos(2))<0.1) & (abs(pos(3))<0.05)
        if state
            state = 0
            robotSuck(state)
            pos
        end
    end
end