// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class timedIntake extends CommandBase {
  /** Creates a new timedIntake. */
  private final Intake intake;
  private final double secs;
  private Timer timer;
  /** Creates a new runIntake. */
  public timedIntake(Intake intakeSubsystem, double seconds) {
    addRequirements(intakeSubsystem);
    intake = intakeSubsystem;
    secs = seconds;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(-0.9);
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(secs);
  }
}
