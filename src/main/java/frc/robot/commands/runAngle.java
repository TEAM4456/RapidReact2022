// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class runAngle extends CommandBase {
  private final Arm arm;
  private final double percent;
  /** Creates a new runAngle. */
  public runAngle(Arm armSubsystem, double percentOutput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
    arm = armSubsystem;
    percent = percentOutput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.runAngle(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.runAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
