package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Trajectories {
  /**
   * Creates a new class containing all of the trajectories
   */
  public Trajectories() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  TrajectoryConfig config =
    new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  TrajectoryConfig barrelConfig =
    new TrajectoryConfig(
      3.8,
      3.8)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  TrajectoryConfig bounceConfig =
    new TrajectoryConfig(
      3.9,
      3.9)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  TrajectoryConfig searchConfig =
    new TrajectoryConfig(
      3.9,
      7.8)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  TrajectoryConfig aBitConfig =
    new TrajectoryConfig(
      3.9,
      2.0)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
    
  public Trajectory slalomTrajectory = 
    TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
              new Pose2d(0.76, 0.0, new Rotation2d(0.0)),
              new Pose2d(1.52, -0.76, new Rotation2d(-Math.PI / 2.0)),
              new Pose2d(2.29, -1.52-0.38, new Rotation2d(0.0)),
              new Pose2d(5.33, -1.52-0.38, new Rotation2d(0.0)),
              new Pose2d(6.1+0.38, -0.76, new Rotation2d(Math.PI / 2.0)),
              new Pose2d(6.86+0.38, 0.0+0.38, new Rotation2d(0.0)),
              new Pose2d(7.62+0.38, -0.76, new Rotation2d(-Math.PI / 2.0)),
              new Pose2d(6.86+0.38, -1.52-0.38, new Rotation2d(Math.PI)),
              new Pose2d(6.1+0.38, -0.76, new Rotation2d(Math.PI / 2.0)),
              new Pose2d(5.33, 0.0+0.38, new Rotation2d(Math.PI)),
              new Pose2d(2.29, 0.0+0.38, new Rotation2d(Math.PI)),
              new Pose2d(1.52, -0.76, new Rotation2d(-Math.PI / 2.0)),
              // new Pose2d(0.76, -1.52, new Rotation2d(Math.PI)),
              new Pose2d(0.0, -1.52, new Rotation2d(Math.PI)),
              new Pose2d(-1.52-0.38, -1.52, new Rotation2d(Math.PI)),
              new Pose2d(-2.29-0.38, -2.29-0.76, new Rotation2d(-Math.PI / 2.0)),
              new Pose2d(0.0, -3.05-0.76, new Rotation2d(0.0))),
      config);

  public Trajectory barrelTrajectory = 
    TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
              new Pose2d(3.05, -0.38, new Rotation2d(0.0)),
              new Pose2d(3.81+0.38, 0.76, new Rotation2d(Math.PI / 2.0)),
              new Pose2d(3.05, 1.52+0.38, new Rotation2d(Math.PI)),
              new Pose2d(2.29-0.38, 0.76, new Rotation2d(-Math.PI / 2.0)),
              new Pose2d(3.05, -0.38, new Rotation2d(0.0)),
              new Pose2d(5.33, 0.38, new Rotation2d(0.0)),
              new Pose2d(6.1, -0.76, new Rotation2d(-Math.PI / 2.0)),
              new Pose2d(5.33, -1.52-0.38, new Rotation2d(Math.PI)),
              new Pose2d(4.57-0.38, -0.76, new Rotation2d(Math.PI / 2.0)),
              new Pose2d(6.86, 1.52+0.38, new Rotation2d(0.0)),
              new Pose2d(7.62+0.38, 0.76, new Rotation2d(-Math.PI / 2.0)),
              new Pose2d(6.86, 0.0, new Rotation2d(Math.PI)),
              // new Pose2d(5.33, 0.0, new Rotation2d(Math.PI)),
              new Pose2d(0.0, 0.0, new Rotation2d(Math.PI)),
              new Pose2d(-1.52-0.38, 0.0, new Rotation2d(Math.PI)),
              new Pose2d(-2.29-0.38, -0.76-0.38-0.38, new Rotation2d(-Math.PI / 2.0)),
              new Pose2d(0.0, -1.52-0.38-0.38, new Rotation2d(0.0))
              ),
      config);

  // TODO add offsets for the ends of each trajectory to account for drift in -x a +y.
  public Trajectory bounceTrajectory1 = 
    TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(Math.PI / 2.0)),
              new Pose2d(0.0, 0.76, new Rotation2d(Math.PI / 2.0)),
              new Pose2d(0.76, 1.52, new Rotation2d(0.0)),
              new Pose2d(1.52, 1.52, new Rotation2d(0.0))),
      bounceConfig);

  public Trajectory bounceTrajectory2 = 
    TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(Math.PI)),
              new Pose2d(-0.76, 0.0, new Rotation2d(Math.PI)),
              new Pose2d(-1.52, 0.38, new Rotation2d(3 * Math.PI / 4)),
              new Pose2d(-2.29, 0.76-0.19, new Rotation2d(Math.PI)),
              new Pose2d(-3.05, 1.52, new Rotation2d(Math.PI / 2)),
              new Pose2d(-2.29, 2.29, new Rotation2d(0)),
              new Pose2d(0.0+0.38, 2.29, new Rotation2d(0))),
      bounceConfig);

  public Trajectory bounceTrajectory3 = 
    TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(Math.PI)),
              new Pose2d(-2.29-0.38, 0.0-0.38-0.19, new Rotation2d(Math.PI)),
              new Pose2d(-3.05-0.38, 0.76-0.38, new Rotation2d(Math.PI / 2)),
              new Pose2d(-3.05-0.38, 1.52-0.19, new Rotation2d(Math.PI / 2)),
              new Pose2d(-2.29-0.38, 2.29-0.19, new Rotation2d(0.0)),
              new Pose2d(0.0, 2.29, new Rotation2d(0.0))),
      bounceConfig);

  public Trajectory bounceTrajectory4 = 
    TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(Math.PI)),
              new Pose2d(-0.76-0.19-0.08, 0.0, new Rotation2d(Math.PI)),
              new Pose2d(-1.52-0.19, 0.76, new Rotation2d(Math.PI / 2)),
              new Pose2d(-1.52-0.19, 2.29, new Rotation2d(Math.PI / 2)),
              new Pose2d(-0.76-0.19, 3.05, new Rotation2d(0.0))),
      bounceConfig);

  public Trajectory forwardABitTrajectory = 
    TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
              new Pose2d(3.05, 0.0, new Rotation2d(0.0))),
      aBitConfig);

  public Trajectory backwardABitTrajectory = 
    TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
              new Pose2d(3.05, 0.0, new Rotation2d(0.0))),
      aBitConfig);

  // public Trajectory backwardABitTrajectory = 
  //   TrajectoryGenerator.generateTrajectory(
  //     new Pose2d(0.0, 0.0, new Rotation2d(Math.PI)), 
  //     List.of(new Translation2d(-1.5, 0.0)), 
  //     new Pose2d(-3.05, 0.0, new Rotation2d(Math.PI)), 
  //     aBitConfig);

  public ProfiledPIDController thetaController =
      new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  public Trajectory variableTrajectory;

  /**
   * Creates the trajectory to pick up 3 balls
   * @param objects A list of positions of the ball
   * @return The trajectory to pick up 3 balls
   */
  public Trajectory createPickUpTrajectory(List<List<Double>> objects) {
    List<Double> ball1Translation = objects.get(0);
    List<Double> ball2Translation = objects.get(1);
    List<Double> ball3Translation = objects.get(2);
    
    Trajectory pickupTrajectory = TrajectoryGenerator.generateTrajectory(
      List.of(new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
              new Pose2d(ball1Translation.get(2) - 0.56, ball1Translation.get(0) + 0.19, new Rotation2d(0.0)),
              new Pose2d(ball2Translation.get(2) - 0.56, ball2Translation.get(0) + 0.19, new Rotation2d(0.0)),
              new Pose2d(ball3Translation.get(2) - 0.56, ball3Translation.get(0) + 0.19, new Rotation2d(0.0)),
              new Pose2d(10.0, ball3Translation.get(0) + 0.19, new Rotation2d(0.0))), // Change 6.0 to however many meters needed to clear the endzone.
      searchConfig);

    return pickupTrajectory;
  }
}
