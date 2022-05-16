function qMat = trapezoidal(robot, path, steps, animate, payload)
    pos1 = makehgtform('translate', path(1,:));
    pos2 = makehgtform('translate', path(2,:));
    
    qStart = robot.model.ikcon(pos1, [0 pi/3 pi/3 0 0]);
    qEnd = robot.model.ikcon(pos2, [-pi/2 pi/3 pi/3 0 0]);
    
    s = lspb(0,1,steps);

    for i = 1:steps
        qMat(i,:) = (1-s(i))*qStart + s(i)*qEnd;
        if (nargin < 4)
            continue
        elseif (animate)
            robot.model.animate(qMat(i,:));
            if (nargin == 5)
                effectorPos = robot.model.fkine(qMat(i,:));
                payload.MoveObj([(effectorPos(1:3,4)'+[0.02 0 -0.0954]) 0 0 0]);
            end
            drawnow();
            pause(0.02)
        end
    end
end
