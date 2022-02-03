// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveWithArcadeCommand;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public final Drive drive = new Drive(RobotMap.leftMaster, RobotMap.rightMaster);

  public final static DifferentialDrive diffDrive = new DifferentialDrive(RobotMap.leftMaster, RobotMap.rightMaster);
  
  public final XboxController controller = new XboxController(0);
  //private final DriveWithArcadeCommand driveWithArcadeCommand = new DriveWithArcadeCommand(drive, controller);
  
  //JoystickButton aButton = new JoystickButton(controller, 1);

  //Ramsete Command Setup
  //DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltsSecondsPerMeter, Constants.kaVoltsSecondSquaredPerMeter), Constants.kTrackWidthMeters, 0);

  //TrajectoryConfig config = new TrajectoryConfig(Constants.maxVelocity, Constants.kMaxAccelerationMetersPerSecondSquared).
        //setKinematics(Constants.kTrackWidthMeters).addConstraint(autoVoltageConstraint);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindindigs
  //  configureButtonBindings();
    drive.setDefaultCommand(new RunCommand(() -> diffDrive.arcadeDrive(controller.getRawAxis(0), controller.getRawAxis(1), controller.getRightStickButtonPressed()), drive));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  //public void diffDrive(Double speed, double rotate) {
    //diffDrive.arcadeDrive(speed, rotate);
  //}

  /*public Trajectory getTrajectoryFromString(String path) {
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return trajectory;
    }
    catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
      return null;
    }
  }*/


  private void configureButtonBindings() {
    JoystickButton aButton = new JoystickButton(controller, 1);
      aButton.whileActiveContinuous(new DriveWithArcadeCommand(drive, controller));
  }

  
  public Command getAutonomousCommand() { 

    drive.resetOdometry(drive.getPose());
    drive.zeroHeading();
    
      

     var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltsSecondsPerMeter,
                                       Constants.kaVoltsSecondSquaredPerMeter), Constants.kTrackWidthMeters, 10);
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared).
        setKinematics(drive.getKinematics()).addConstraint(autoVoltageConstraint);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 0), new Translation2d(2, 0)), new Pose2d(3, 0, new Rotation2d((0))), config);
        
        RamseteCommand ramseteCommand = new RamseteCommand(
          traj, 
          drive::getPose, 
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
          new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltsSecondsPerMeter, Constants.kaVoltsSecondSquaredPerMeter), 
          Constants.kTrackWidthMeters,
          drive::getSpeeds, new PIDController(Constants.kPDriveVelocity, 0, 0), new PIDController(Constants.kPDriveVelocity, 0, 0), drive::tankDriveVolts, drive);

        SmartDashboard.putBoolean("Auto", ramseteCommand.isScheduled());
        
        return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
        //return m_autoCommand;
  }

  public DifferentialDrive getDrive() {
    return diffDrive;
  }
  
  

}
