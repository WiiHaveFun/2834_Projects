// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveTrain;

public class RotateToAngle extends CommandBase {
  DriveTrain driveTrain;
  double targetAngle;

  PIDController controller;


  /** Creates a new RotateToAngle. */
  public RotateToAngle(DriveTrain driveTrain, double targetAngle) {
    this.driveTrain = driveTrain;
    this.targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveTrain);

    controller = new PIDController(0.008, 0.0, 0.0001);
    controller.setTolerance(3.0);
    controller.enableContinuousInput(-180.0, 180.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.gyro.zeroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = MathUtil.clamp(controller.calculate(driveTrain.gyro.getYaw(), targetAngle), -1.0, 1.0);
    SmartDashboard.putNumber("power", power);
    driveTrain.setPower(power, -power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setPower(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(targetAngle - driveTrain.gyro.getYaw()) < 3.0) {
      return true;
    } else {
      return false;
    }
  }
}
