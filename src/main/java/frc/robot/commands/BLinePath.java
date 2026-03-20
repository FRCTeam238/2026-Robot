package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class BLinePath extends SequentialCommandGroup {

    private Drivetrain drivetrain = Drivetrain.getInstance();

    public BLinePath(String pathName, boolean rightSide, boolean resetPosition) {
        FollowPath.registerEventTrigger("deployIntake", new DeployIntake());
        FollowPath.registerEventTrigger("startIntake", new IntakeFuel());
        FollowPath.registerEventTrigger("stopIntake", new InstantCommand(() -> Intake.getInstance().stopRoller(), Intake.getInstance()));
        FollowPath.registerEventTrigger("spinUp", new SpinUp(50));
        if (resetPosition) {
            drivetrain.blineBuilder.withPoseReset(drivetrain::resetOdometry);
        } else
        {
            drivetrain.blineBuilder.withPoseReset(pose->{});
        }

        addCommands(drivetrain.runOnce(() -> drivetrain.setCommand("Traj-" + pathName)));
        drivetrain.blineBuilder.withShouldMirror(() -> rightSide);
        Command swerveCommand = drivetrain.blineBuilder.build(new Path(pathName));
        addCommands(swerveCommand);
    }

}
