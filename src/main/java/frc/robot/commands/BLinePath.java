package frc.robot.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Drivetrain;

public class BLinePath extends SequentialCommandGroup {

    private Drivetrain drivetrain = Drivetrain.getInstance();

    public BLinePath(String pathName, boolean resetPosition) {

        if (resetPosition) {
            drivetrain.blineBuilder.withPoseReset(drivetrain::resetOdometry);
        } else
        {
            drivetrain.blineBuilder.withPoseReset(pose->{});
        }

        addCommands(drivetrain.runOnce(() -> drivetrain.setCommand("Traj-" + pathName)));

        Command swerveCommand = drivetrain.blineBuilder.build(new Path(pathName));
        addCommands(swerveCommand);
    }

}
