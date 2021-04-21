// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BrushConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.BackwardABit;
import frc.robot.commands.BarrelAuto;
import frc.robot.commands.BounceAuto;
import frc.robot.commands.ControlFeeder;
import frc.robot.commands.ControlIntake;
import frc.robot.commands.ControlPneumatics;
import frc.robot.commands.Drive;
import frc.robot.commands.FowardABit;
import frc.robot.commands.ManualShooterControl;
import frc.robot.commands.ReverseBrush;
import frc.robot.commands.RunBrush;
import frc.robot.commands.SearchAuto;
import frc.robot.commands.SlalomAuto;
import frc.robot.subsystems.Brush;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Socket;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final Feeder m_feeder = new Feeder(FeederConstants.kFeederMotorID, FeederConstants.kFeederMotorReversed);
  public final Intake m_intake = new Intake(IntakeConstants.kIntakeMotorID, IntakeConstants.kIntakeMotorReversed);
  public final Brush m_brush = new Brush(BrushConstants.kBrushMotorID, BrushConstants.kBrushMotorReversed);
  public final Pneumatics m_pneumatics = new Pneumatics(PneumaticsConstants.kPCMID, PneumaticsConstants.kSolenoidChannel);
  public final Shooter m_shooter = new Shooter(ShooterConstants.kFlyLeftID, ShooterConstants.kFlyRightID, 
                                               ShooterConstants.kLeftServoID, ShooterConstants.kRightServoID,
                                               ShooterConstants.kTurretID, 
                                               ShooterConstants.kFlyLeftReversed, ShooterConstants.kFlyRightReversed, ShooterConstants.kTurretReversed,
                                               ShooterConstants.kFlyEncoderReversed);
  public final Socket m_socket = new Socket(5801, 4096);

  // Controller(s)
  public final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // Buttons
  public final JoystickButton m_intakeButton = new JoystickButton(m_driverController, OIConstants.kIntakeButtonID);
  public final JoystickButton m_ejectButton = new JoystickButton(m_driverController, OIConstants.kEjectButtonID);
  public final JoystickButton m_feedButton = new JoystickButton(m_driverController, OIConstants.kFeedButtonID);
  public final JoystickButton m_toggleIntake = new JoystickButton(m_driverController, OIConstants.kToggleIntakeButtonID);
  public final JoystickButton m_enableShooter = new JoystickButton(m_driverController, OIConstants.kEnableAutoAimButtonID);
  public final JoystickButton m_enableTracking = new JoystickButton(m_driverController, OIConstants.kEnableAutoTrackButtonID);
  public final JoystickButton m_reverseBrush = new JoystickButton(m_driverController, OIConstants.kReverseBrush);
  public final JoystickButton m_forwardABit = new JoystickButton(m_driverController, OIConstants.kForwardABit);
  public final JoystickButton m_backwardABit = new JoystickButton(m_driverController, OIConstants.kBackwardABit);

  // Auto chooser
  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, m_driverController));
    m_shooter.setDefaultCommand(new ManualShooterControl(m_shooter, m_driverController));
    // m_brush.setDefaultCommand(new RunBrush(m_brush, m_driverController));

    // Add autos
    chooser.addOption("Slalom", new SlalomAuto(m_driveSubsystem));
    chooser.addOption("Barrel", new BarrelAuto(m_driveSubsystem));
    chooser.addOption("Bounce", new BounceAuto(m_driveSubsystem));
    chooser.addOption("Search", new SearchAuto(m_driveSubsystem, m_pneumatics, m_intake, m_brush, m_socket));
    SmartDashboard.putData("Auto Mode", chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Intake buttons
    m_intakeButton.whenPressed(new ControlIntake(m_intake, IntakeConstants.kIntakePower));
    m_intakeButton.whenReleased(new ControlIntake(m_intake, IntakeConstants.kNeutralPower));
    // m_ejectButton.whenPressed(new ControlIntake(m_intake, IntakeConstants.kEjectPower));
    // m_ejectButton.whenReleased(new ControlIntake(m_intake, IntakeConstants.kNeutralPower));
    // Feeder buttons
    m_feedButton.whenPressed(new ControlFeeder(m_feeder, FeederConstants.kFeedPower));
    m_feedButton.whenReleased(new ControlFeeder(m_feeder, FeederConstants.kNeutralPower));

    m_toggleIntake.whenPressed(new ControlPneumatics(m_pneumatics));

    m_enableShooter.toggleWhenActive(new AutoAim(m_shooter, false));
    m_enableTracking.toggleWhenActive(new AutoAim(m_shooter, true));

    m_reverseBrush.whileHeld(new ReverseBrush(m_brush));

    m_forwardABit.whileHeld(new FowardABit(m_driveSubsystem));
    m_backwardABit.whileHeld(new BackwardABit(m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    Command m_autoCommand = chooser.getSelected();
    return m_autoCommand;

    // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(.29, 0.7), 
    //                 new Translation2d(1, 1),
    //                 new Translation2d(1.7, 0.7),
    //                 new Translation2d(2, 0),
    //                 new Translation2d(1.7, -0.7), 
    //                 new Translation2d(1, -1),
    //                 new Translation2d(.29, -0.7)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(0, 0, new Rotation2d(Math.PI)),
    //         config);

    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         List.of(new Pose2d(0.0, 0.0, new Rotation2d(0)),
    //                 new Pose2d(1.0, 1.0, new Rotation2d(0)),
    //                 new Pose2d(0.0, 2.0, new Rotation2d(0))),
    //         config);

    // Trajectory exampleTrajectory = 
    //   TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1.0, 0.0),
    //             new Translation2d(2.0, 0.0)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3.0, 0, new Rotation2d(0.0)),
    //     config);

    // Trajectory slalomTrajectory = 
    //   TrajectoryGenerator.generateTrajectory(
    //     List.of(new Pose2d(0.76, 0.0, new Rotation2d(0.0)),
    //             new Pose2d(1.52, -0.76, new Rotation2d(-Math.PI / 2.0)),
    //             new Pose2d(2.29, -1.52, new Rotation2d(0.0)),
    //             new Pose2d(5.33, -1.52, new Rotation2d(0.0)),
    //             new Pose2d(6.1, -0.76, new Rotation2d(Math.PI / 2.0)),
    //             new Pose2d(6.86, 0.0, new Rotation2d(0.0)),
    //             new Pose2d(7.62, -0.76, new Rotation2d(-Math.PI / 2.0)),
    //             new Pose2d(6.86, -1.52, new Rotation2d(Math.PI)),
    //             new Pose2d(6.1, -0.76, new Rotation2d(Math.PI / 2.0)),
    //             new Pose2d(5.33, 0.0, new Rotation2d(Math.PI)),
    //             new Pose2d(2.29, 0.0, new Rotation2d(Math.PI)),
    //             new Pose2d(1.52, -0.76, new Rotation2d(-Math.PI / 2.0)),
    //             new Pose2d(0.76, -1.52, new Rotation2d(Math.PI)),
    //             new Pose2d(0.0, -1.52, new Rotation2d(Math.PI))),
    //     config);

    //   Trajectory barrelTrajectory = 
    //   TrajectoryGenerator.generateTrajectory(
    //     List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    //             new Pose2d(3.05, 0.0, new Rotation2d(0.0)),
    //             new Pose2d(3.81, 0.76, new Rotation2d(Math.PI / 2.0)),
    //             new Pose2d(3.05, 1.52, new Rotation2d(Math.PI)),
    //             new Pose2d(2.29, 0.76, new Rotation2d(-Math.PI / 2.0)),
    //             new Pose2d(3.05, 0.0, new Rotation2d(0.0)),
    //             new Pose2d(5.33, 0.0, new Rotation2d(0.0)),
    //             new Pose2d(6.1, -0.76, new Rotation2d(-Math.PI / 2.0)),
    //             new Pose2d(5.33, -1.52, new Rotation2d(Math.PI)),
    //             new Pose2d(4.57, -0.76, new Rotation2d(Math.PI / 2.0)),
    //             new Pose2d(6.86, 1.52, new Rotation2d(0.0)),
    //             new Pose2d(7.62, 0.76, new Rotation2d(-Math.PI / 2.0)),
    //             new Pose2d(6.86, 0.0, new Rotation2d(Math.PI)),
    //             new Pose2d(0.0, 0.0, new Rotation2d(Math.PI))),
    //     config);

    // var thetaController =
    //     new ProfiledPIDController(
    //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand =
    //     new SwerveControllerCommand(
    //         barrelTrajectory,
    //         m_driveSubsystem::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_driveSubsystem::setModuleStates,
    //         m_driveSubsystem);

    // // Reset odometry to the starting pose of the trajectory.
    // m_driveSubsystem.resetGyro();
    // m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_driveSubsystem.setModuleStates(DriveSubsystem.calculate(0.0, 
    //                                                                                                        0.0, 
    //                                                                                                        0.0, 
    //                                                                                                        m_driveSubsystem.getHeading(), 
    //                                                                                                        DriveConstants.kWheelBase, 
    //                                                                                                        DriveConstants.kTrackWidth)));
  }
}
