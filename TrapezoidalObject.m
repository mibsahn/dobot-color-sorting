function TrapezoidalObject(robot, path, steps, payload)

% Trapezoidal Return joint state
% Use ikcon to calculate required joint state of robot

pos1 = makehgtform('translate', path(1,:));
pos2 = makehgtform('translate', path(2,:));

qStart = robot.model.ikcon(pos1, [-pi/2 pi/3 pi/3 0 0]);
qEnd = robot.model.ikcon(pos2, [0 pi/3 pi/3 0 0]);

s = lspb(0,1,steps);

for i = 1:steps
    qMat(i,:) = (1-s(i))*qStart + s(i)*qEnd;
    robot.model.animate(qMat(i,:));
    effectorPos = robot.model.fkine(qMat(i,:));
    payload.MoveObj([(effectorPos(1:3,4)'+[0.02 0 -0.0954]) 0 0 0]);
    drawnow();
    pause(0.1);
end

end
