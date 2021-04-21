// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // Set ID and port number
  TalonSRX m_talon = new TalonSRX(1);
  XboxController m_controller = new XboxController(1);

  // Measure rates
  final double nominalTurnOutputPercent = 0.95;
  final int encoderTickAtNominal = 325;
    
  final double fGain = (nominalTurnOutputPercent * 1023) / encoderTickAtNominal;

  // Tune gains
  final double pGain = 0.5; //0.2
  final double error = 135;
  final double pTerm = (pGain * 1023) / error;
  final double dGain = 0*pTerm;

  // Set based on module
  final double ticksPerRevolution = 1024;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_talon.configFactoryDefault();

    // Set encoder type
    m_talon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 30);

    m_talon.configNeutralDeadband(0.001, 30);

    // Check before testing
    m_talon.setSensorPhase(false);
    m_talon.setInverted(false);
    
    m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    m_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
    
    m_talon.configNominalOutputForward(0, 30);
		m_talon.configNominalOutputReverse(0, 30);
		m_talon.configPeakOutputForward(1, 30);
    m_talon.configPeakOutputReverse(-1, 30);
    
    m_talon.selectProfileSlot(0, 0);
		m_talon.config_kF(0, fGain, 30);
		m_talon.config_kP(0, pTerm, 30);
		m_talon.config_kI(0, 0.0000, 30);
    m_talon.config_kD(0, dGain, 30);
    
    // Configure properly
    m_talon.configMotionCruiseVelocity(325, 30);
		m_talon.configMotionAcceleration(325, 30);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SmartDashboard.putNumber("kF", 0.0);
    SmartDashboard.putNumber("kP", 0.0);
    SmartDashboard.putNumber("kD", 0.0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(m_controller.getAButton()) {
      // Go to encoder zero
      m_talon.set(ControlMode.MotionMagic, 0);
    } else if(m_controller.getBButton()) {
      // 1/4 rotation
      m_talon.set(ControlMode.MotionMagic, ticksPerRevolution / 4);
    } else if(m_controller.getXButton()) {
      // 1/2 rotation
      m_talon.set(ControlMode.MotionMagic, ticksPerRevolution / 2);
    } else if(m_controller.getYButton()) {
      // 1 rotation
      m_talon.set(ControlMode.MotionMagic, ticksPerRevolution);
    } else if(m_controller.getBackButton()) {
      // Change kF, kP, and kD values for manual tuning
      double kF = SmartDashboard.getNumber("kF", 0.0);
      double kP = SmartDashboard.getNumber("kP", 0.0);
      double kD = SmartDashboard.getNumber("kD", 0.0);
      m_talon.config_kF(0, kF, 30);
      m_talon.config_kP(0, kP, 30);
      m_talon.config_kD(0, kD, 30);
    }

    // Add encoder telemetry and immediately push to the network table
    SmartDashboard.putNumber("encoder position", m_talon.getSelectedSensorPosition(0));
    SmartDashboard.putNumber(("encoder velocity"), m_talon.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("target", m_talon.getClosedLoopTarget(0));
    SmartDashboard.putNumber("error", m_talon.getClosedLoopError(0));
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
