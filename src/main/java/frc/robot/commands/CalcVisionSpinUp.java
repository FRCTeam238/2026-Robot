// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalcVisionSpinUp extends Command {
  /** Creates a new SpinUp. */

  public CalcVisionSpinUp() {
    addRequirements(Launcher.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Launcher.getInstance().setCommand("CalcVisionSpinUp");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double flyWheelSpeed = Launcher.getInstance().calculateLaunchSpeed();
    Launcher.getInstance().setSpeed(flyWheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Launcher.getInstance().stop();
    }
    Launcher.getInstance().setCommand("");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Launcher.getInstance().atSpeed();
  }
}
