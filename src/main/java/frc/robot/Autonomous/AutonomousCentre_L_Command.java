package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;

import frc.robot.commands.LauncherActivateCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.List;

public class AutonomousCentre_L_Command extends SequentialCommandGroup {


    public AutonomousCentre_L_Command(SwerveSubsystem swerveSubsystem, LauncherSubsystem launcherSubsystem) {
        // Asume que necesitas el subsistema de swerve para ejecutar la trayectoria

        // 1. Create trajectory configuration
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // 2-A. Generate trajectory
        Trajectory trajectoryA = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(0.3,0)),
            new Pose2d(0.6, 0, Rotation2d.fromDegrees(0)),
            trajectoryConfig);
        
        // 2-B. Generate trajectory
        Trajectory trajectoryB = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(0, -1.5),
                new Translation2d(1, -2),
                new Translation2d(2, -1)),
            new Pose2d(2.4, -1, Rotation2d.fromDegrees(90)),
            trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4-A. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommandA = new SwerveControllerCommand(
            trajectoryA,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModulesState,
            swerveSubsystem);

        // 4-B. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommandB = new SwerveControllerCommand(
            trajectoryB,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModulesState,
            swerveSubsystem);

        // 5. Add commands to the SequentialCommandGroup
        addCommands(

            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
            
            // Follow the trajectory A
            swerveControllerCommandA,

            // Launcher start
            new LauncherActivateCmd(launcherSubsystem, swerveSubsystem, LauncherConstants.kUpperRoller_Speed2),

            // Follow the trajectory B
            swerveControllerCommandB,

            // Reset odometry to trajectory's initial pose
            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectoryA.getInitialPose())),

            // Stop the swerve modules after trajectory is complete
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }
}