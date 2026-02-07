// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Feeder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Launch extends Command {

  /** Creates a new Launch. */
  public Launch() {
    addRequirements(Launcher.getInstance(), Feeder.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Feeder.getInstance().setSpeed(FeederConstants.launchSpeed);
    Launcher.getInstance().setSpeed(LauncherConstants.launchSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Feeder.getInstance().stop();
    Launcher.getInstance().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
