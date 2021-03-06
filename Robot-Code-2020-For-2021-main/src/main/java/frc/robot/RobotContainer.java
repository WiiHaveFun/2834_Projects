/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.CenterAuto;
import frc.robot.commands.AimTurret;
import frc.robot.commands.Climb;
import frc.robot.commands.ControlFeeder;
import frc.robot.commands.ControlIntake;
import frc.robot.commands.ControlPneumatics;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.SpinCarousel;
import frc.robot.commands.Climb.Direction;
import frc.robot.commands.ControlPneumatics.Solenoid;
import frc.robot.subsystems.BallManager;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Socket;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain driveTrain = new DriveTrain();
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  public final BallManager ballManager = new BallManager();
  public final Feeder feeder = new Feeder();
  public final Pneumatics pneumatics = new Pneumatics();
  public final Climber climber = new Climber();
  public final Socket socket = new Socket(5801, 4096);

  // Controllers
  XboxController controller = new XboxController(0);
  Joystick buttonBox = new Joystick(1);

  // Buttons and Triggers
  JoystickButton intakeButton = new JoystickButton(buttonBox, Constants.intakeButton);
  JoystickButton outputButton = new JoystickButton(buttonBox, Constants.outputButton);
  JoystickButton driverIntake = new JoystickButton(controller, Constants.driverIntake);
  JoystickButton driverOutput = new JoystickButton(controller, Constants.driverOutput);
  JoystickButton aimBotButton = new  JoystickButton(buttonBox, Constants.aimBotButton);
  JoystickButton feedButton = new JoystickButton(buttonBox, Constants.feedButton);
  JoystickButton armButton = new JoystickButton(buttonBox, Constants.armButton);
  JoystickButton climbUpButton = new JoystickButton(buttonBox, Constants.climberUpButton);
  JoystickButton climbDownButton = new JoystickButton(buttonBox, Constants.climberDownButton);

  // Auto chooser
  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    shooter.setDefaultCommand(new ManualTurret(shooter, controller));
    ballManager.setDefaultCommand(new SpinCarousel(ballManager, feeder));

    // Add autos
    chooser.addOption("Center Auto", new CenterAuto(shooter, feeder, driveTrain, intake));
    SmartDashboard.putData("Auto Mode", chooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    intakeButton.whenPressed(new ControlIntake(intake, Constants.intakePower));
    intakeButton.whenReleased(new ControlIntake(intake, 0.0));

    driverIntake.whenPressed(new ControlIntake(intake, Constants.intakePower));
    driverIntake.whenReleased(new ControlIntake(intake, 0.0));

    outputButton.whenHeld(new ControlIntake(intake, Constants.outputPower));
    outputButton.whenReleased(new ControlIntake(intake, 0.0));

    driverOutput.whenHeld(new ControlIntake(intake, Constants.outputPower));
    driverOutput.whenReleased(new ControlIntake(intake, 0.0));

    aimBotButton.whenHeld(new AimTurret(shooter));
    
    feedButton.whenPressed(new ControlFeeder(feeder, Constants.feedPower));
    feedButton.whenReleased(new ControlFeeder(feeder, Constants.feedNeutralPower));

    armButton.whenPressed(new ControlPneumatics(pneumatics, Solenoid.ARM_SOLENOID, Value.kReverse));
    armButton.whenReleased(new ControlPneumatics(pneumatics, Solenoid.ARM_SOLENOID, Value.kForward));
    
    climbUpButton.whenHeld(new Climb(climber, pneumatics, Direction.UP, buttonBox));
    climbDownButton.whenHeld(new Climb(climber, pneumatics, Direction.DOWN, buttonBox));
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

    // // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(Constants.ksVolts,
    //           Constants.kvVoltSecondsPerMeter,
    //           Constants.kaVoltSecondsSquaredPerMeter),
    //           Constants.kDriveKinematics,
    //           10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
    //                          Constants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(Constants.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //   // Start at the origin facing the +X direction
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   // Pass through these two interior waypoints, making an 's' curve path
    //   List.of(
    //       new Translation2d(2, 0),
    //       new Translation2d(4, 0)
    //   ),
    //   // End 3 meters straight ahead of where we started, facing forward
    //   new Pose2d(5, 0, new Rotation2d(0)),
    //   // Pass config
    //   config
    //   );

    // RamseteCommand ramseteCommand = new RamseteCommand(
    //     exampleTrajectory,
    //     driveTrain::getPose,
    //     new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    //     new SimpleMotorFeedforward(Constants.ksVolts,
    //                                Constants.kvVoltSecondsPerMeter,
    //                                Constants.kaVoltSecondsSquaredPerMeter),
    //                                Constants.kDriveKinematics,
    //     driveTrain::getWheelSpeeds,
    //     new PIDController(Constants.kPDriveVel, 0, 0),
    //     new PIDController(Constants.kPDriveVel, 0, 0),
    //     // RamseteCommand passes volts to the callback
    //     driveTrain::tankDriveVolts,
    //     driveTrain
    // );

    // // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
}
