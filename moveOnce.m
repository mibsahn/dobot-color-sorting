[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");

q = parseQ(qMat(1,:));
trajectoryPoint.Positions = q(1:4);
targetJointTrajMsg.Points = trajectoryPoint;
send(targetJointTrajPub,targetJointTrajMsg);
pause(0.05)
state = 0
robotSuck(0)
safetyStateMsg.Data = 3;

%%
[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 4;
send(safetyStatePublisher,safetyStateMsg);