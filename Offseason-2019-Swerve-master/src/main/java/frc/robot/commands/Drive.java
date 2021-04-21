/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveModule;

public class Drive extends Command {
  public Drive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double FWD = Robot.m_oi.driver.getRawAxis(1);
    if(Math.abs(FWD) < 0.1) {
      FWD = 0;
    } else {
      if(FWD > 0) {
        FWD = Math.pow(FWD, 2);
      } else {
        FWD = -Math.pow(FWD, 2);
      }
    }
    double STR = Robot.m_oi.driver.getRawAxis(0);
    if(Math.abs(STR) < 0.1) {
      STR = 0;
    } else {
      if(STR > 0) {
        STR = Math.pow(STR, 2);
      } else {
        STR = -Math.pow(STR, 2);
      }
    }
    double RCW = Robot.m_oi.driver.getRawAxis(4);
    if(Math.abs(RCW) < 0.1) {
      RCW = 0;
    } else {
      if(RCW > 0) {
        RCW = Math.pow(RCW, 2);
      } else {
        RCW = -Math.pow(RCW, 2);
      }
    }

    double speedMultiplier = 0.95;
    double turnMultiplier = 0.95;

    double[][] vectors = SwerveModule.calculate(-FWD*speedMultiplier, 
    STR*speedMultiplier, 
    RCW*turnMultiplier, 
    Robot.drivetrain.gyro.getYaw(), 
    Robot.drivetrain.fr.getBaseLength(), 
    Robot.drivetrain.fr.getBaseWidth());


    Robot.drivetrain.controlModule(Robot.drivetrain.fr, vectors[0][0], vectors[0][1]);
    Robot.drivetrain.controlModule(Robot.drivetrain.fl, vectors[1][0], vectors[1][1]);
    Robot.drivetrain.controlModule(Robot.drivetrain.bl, vectors[2][0], vectors[2][1]);
    Robot.drivetrain.controlModule(Robot.drivetrain.br, vectors[3][0], vectors[3][1]);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
