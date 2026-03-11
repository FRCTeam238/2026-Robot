package frc.robot.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class ChoreoPath extends SequentialCommandGroup {

    private Drivetrain drivetrain = Drivetrain.getInstance();

    public ChoreoPath(String pathName, boolean resetPosition, int splitNum) {
        Trajectory<SwerveSample> trajectory = (Trajectory<SwerveSample>) Choreo.loadTrajectory(pathName).get().getSplit(splitNum >= 0 ? splitNum : 0).get();

        if (resetPosition) {

            Command resetPos = drivetrain.runOnce(() -> 
                    drivetrain.resetOdometry(
                        (Pose2d)trajectory.getInitialPose(DriverStation.getAlliance().get() == Alliance.Red).get()
                    ));
            addCommands(resetPos);
        }

        addCommands(drivetrain.runOnce(() -> drivetrain.setCommand("Traj-" + pathName)));

        Command swerveCommand = drivetrain.choreoCommand(trajectory, () -> {return DriverStation.getAlliance().get() == Alliance.Red;});
        addCommands(swerveCommand);


    }

}
