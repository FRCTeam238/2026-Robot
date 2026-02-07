// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LauncherConstants.launchSpeed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUp extends Command {
  /** Creates a new SpinUp. */
  public SpinUp() {
    addRequirements(Launcher.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Launcher.getInstance().setSpeed(launchSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted)
    {
      Launcher.getInstance().stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Launcher.getInstance().atSpeed();
  }
}
