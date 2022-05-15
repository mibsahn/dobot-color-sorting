        %% RMRC
        function rmrc(dobot, steps, deltaTime, obj)
        %RMRC Move robot in RMRC
        %   Generate trajectory and animate robot motion
        
        minMani = 0.1;
        wayPoints = 2;
        wayPointRMRC = [0.3    0.0   (0.15+0.0754);
                        0.3    0.0   (0.02+0.0754)];
        
        qMat = zeros(steps,5);
        trans = zeros(3,steps);
        rot = zeros(3,steps);
        
        s = lspb(0,1,steps);                                    % Trapezoidal trajectory scalar
        for i = 1:wayPoints-1
            for j = 1:steps
                trans(1,j,i) = (1-s(j))*wayPointRMRC(i,1) + s(j)*wayPointRMRC(i+1,1);            % Points in x
                trans(2,j,i) = (1-s(j))*wayPointRMRC(i,2) + s(j)*wayPointRMRC(i+1,2);            % Points in y
                trans(3,j,i) = (1-s(j))*wayPointRMRC(i,3) + s(j)*wayPointRMRC(i+1,3);            % Points in z
                rot(:,j,i) = zeros(3,1);                                       % Yaw angle
            end
            startPos = makehgtform('translate', trans(:,1,i));
            qMat(1,:,i) = dobot.model.ikcon(startPos, [0 pi/3 -pi/3 0 0]);
        end
        
        
        %%
        for i = 1:wayPoints-1
            text_h = text(-0.5, 0.5, 0.45, ['RMRC Mode Juice Extraction'], 'FontSize', 12, 'Color', [.6 .2 .6]);
            for j = 1:steps-1
        
                T = dobot.model.fkine(qMat(j,:,i));               % Get forward transformation at current joint state
                deltaTrans = trans(:,j+1,i) - T(1:3,4);         % Get position error from next waypoint
                Rd = rpy2r(rot(1,i+1),rot(2,i+1),rot(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaTime)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                veloRot = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                veloTrans = deltaTrans / deltaTime;             % Calculate velocity at discrete time step
                xdot = [veloTrans; veloRot];                          % Calculate end-effector velocity to reach next waypoint
                J = dobot.model.jacob0(qMat(j,:,i));         % Jacobian at current pose
                % Check Manipulability
                m = sqrt(det(J*J'));
                if m < minMani
                    lambda = 0.01;      %(1 - m(i)/minMani)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda*eye(5))*J';                                   % DLS Inverse
                qdot(j,:,i) = (invJ * xdot)';                             % Singularity avoidance with DLS
        %         for j = 1:6                                                             % Loop through joints 1 to 6
        %             if qMatrix(i,j) + deltaT*qdot(i,j) < p560.qlim(j,1)                     % If next joint angle is lower than joint limit...
        %                 qdot(i,j) = 0; % Stop the motor
        %             elseif qMatrix(i,j) + deltaT*qdot(i,j) > p560.qlim(j,2)                 % If next joint angle is greater than joint limit ...
        %                 qdot(i,j) = 0; % Stop the motor
        %             end
        %         end
                qMat(j+1,:,i) = qMat(j,:,i) + deltaTime*qdot(j,:,i);        % Update next joint state
                dobot.model.animate(qMat(j,:,i));
                obj.MoveObj([(T(1:3,4)'+[0.02 0 -0.0954]) 0 0 0]);
                drawnow();
                pause(0.1);
            end
            delete(text_h);
        end
        obj.MoveObj([0.3 0 -0.1 0 0 0]);
        pause(0.2);
        obj.MoveObj([0.3 0 -0.16 0 0 0]);
        end

