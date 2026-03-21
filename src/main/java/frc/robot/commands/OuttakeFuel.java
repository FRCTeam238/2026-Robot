// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FeederConstants.feederSpeed;
import static frc.robot.Constants.IntakeConstants.intakeDown;
import static frc.robot.Constants.IntakeConstants.intakeRollerVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OuttakeFuel extends Command {
  /** Creates a new Outtake. */
  public OuttakeFuel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake.getInstance(), Feeder.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.getInstance().setCommand("OuttakeFuel");
    Feeder.getInstance().setCommand("OuttakeFuel");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Intake.getInstance().tiltAtTarget()) {
      Intake.getInstance().runIntake(-intakeRollerVoltage);

      if (Intake.getInstance().getTargetPosition() == intakeDown) {
        Feeder.getInstance().outtakeLower(-feederSpeed);
      } else {
        Feeder.getInstance().stop();
      }
    } else {
      Intake.getInstance().stopRoller();
      Feeder.getInstance().stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.getInstance().stopRoller();
    Feeder.getInstance().stop();
    
    Intake.getInstance().setCommand("");
    Feeder.getInstance().setCommand("");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
