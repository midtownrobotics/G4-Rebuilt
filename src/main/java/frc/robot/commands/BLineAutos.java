package frc.robot.commands;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.drive.Drive;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.Waypoint;

public class BLineAutos {
    final FollowPath.Builder pathBuilder;
    final Drive m_drivetrain;
    final RobotCommands m_robotCommands;

    public BLineAutos(Drive drivetrain, RobotCommands robotCommands){
        m_drivetrain = drivetrain;
        m_robotCommands = robotCommands;
        pathBuilder = new FollowPath.Builder(
                drivetrain,
                drivetrain::getPose,
                drivetrain::getChassisSpeeds,
                drivetrain::runVelocity,
                new PIDController(2.0, 0.0, 0.0),
                new PIDController(1.0, 0.0, 0.0),
                new PIDController(5, 0.0, 0.0)).withDefaultShouldFlip()
                .withTRatioBasedTranslationHandoffs(true);
    }

    public Command Madtown(Boolean left){
        Path madtown = new Path("madtown");

        if(left){
            madtown.mirror();
        }

        Command madtwonPath = pathBuilder
            .withPoseReset(m_drivetrain::resetPose)
            .build(madtown);
        
        Command command = Commands.sequence(
            madtwonPath,
            m_robotCommands.shoot(),
            Commands.waitSeconds(5)
        );
        return command;
    }

    public Command GoTo(Pose2d pose){
        Path path = new Path(
            List.of(
                new Waypoint(pose)
            )
        );

        System.out.println(pose.toString());

        return pathBuilder.build(path);
    }

    public Command GoTo(String varName, boolean mirrored) {
        Pose2d pose = AutoHelper.poseFromChoreoVar(varName);
        if(mirrored){
            double fieldWidth = FieldConstants.kFieldWidth.baseUnitMagnitude();
            double mirroredY = pose.getY();
            mirroredY = fieldWidth - mirroredY;
            Rotation2d mirroredRotation = pose.getRotation().unaryMinus();
            return GoTo(new Pose2d(pose.getX(), mirroredY, mirroredRotation));
        }
        return GoTo(pose);
    }
}
