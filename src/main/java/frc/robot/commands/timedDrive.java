package frc.robot.commands;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class timedDrive extends CommandBase {
  /** Creates a new timedIntake. */
  private final Drive drive;
  private final double secs;
  private Timer timer;
  /** Creates a new runIntake. */
  public timedDrive(Drive driveSubsystem, double seconds) {
    addRequirements(driveSubsystem);
    drive = driveSubsystem;
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
    drive.leftDrive.set(ControlMode.PercentOutput, -0.5);
    drive.rightDrive.set(ControlMode.PercentOutput, -0.5);
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.leftDrive.set(ControlMode.PercentOutput, 0);
    drive.rightDrive.set(ControlMode.PercentOutput, 0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(secs);
  }
}