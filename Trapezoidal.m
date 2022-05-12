function Trapezoidal(robot, path, steps)

% Trapezoidal Return joint state
% Use ikcon to calculate required joint state of robot

pos1 = makehgtform('translate', path(1,:));
pos2 = makehgtform('translate', path(2,:));

qStart = robot.model.ikcon(pos1, [0 pi/3 pi/3 0 0]);
qEnd = robot.model.ikcon(pos2, [-pi/2 pi/3 pi/3 0 0]);

s = lspb(0,1,steps);


    for i = 1:steps
        qMat(i,:) = (1-s(i))*qStart + s(i)*qEnd;
        robot.model.animate(qMat(i,:));
        drawnow();
        pause(0.1);
    end



end
