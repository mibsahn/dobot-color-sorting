function robotMove(qMat)
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");

        q = parseQ(qMat);
        trajectoryPoint.Positions = q(1:4);
        targetJointTrajMsg.Points = trajectoryPoint;
        send(targetJointTrajPub,targetJointTrajMsg);
end