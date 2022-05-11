function Trapezoidal(robot, path, steps, payload)

% Trapezoidal Return joint state
% Use ikcon to calculate required joint state of robot

pos1 = makehgtform('translate', path(1,:));
pos2 = makehgtform('translate', path(2,:));

qStart = robot.model.ikcon(pos1);
qEnd = robot.model.ikcon(pos2, qStart);

s = lspb(0,1,steps);


    for i = 1:steps
        qMat(i,:) = (1-s(i))*qStart + s(i)*qEnd;
        robot.model.animate(qMat(i,:));
        drawnow();
        pause(0.5);
    end



end
