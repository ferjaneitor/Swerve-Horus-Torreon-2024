package frc.robot.Macros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ClawSubsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.LimeConstants;
import frc.robot.Constants.ClawConstants;

import frc.robot.commands.AimAmpCmd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.List;

public class AmpScoreMac extends SequentialCommandGroup {


    public AmpScoreMac(SwerveSubsystem swerveSubsystem, LauncherSubsystem launcherSubsystem, ClawSubsystem clawSubsystem, LimeLightSubsystem limeLightSubsystem) {
        // Asume que necesitas el subsistema de swerve para ejecutar la trayectoria

        // 1. Create trajectory configuration
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // 2.A Generate trajectory
        Trajectory AmptrajectoryA = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(-.4, 0)),
            new Pose2d(-0.8, 0, Rotation2d.fromDegrees(90)),
            trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4.A Construct command to follow trajectory
        SwerveControllerCommand AmpSwerveControllerCommandA = new SwerveControllerCommand(
            AmptrajectoryA,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModulesState,
            swerveSubsystem);

        // 5. Add commands to the SequentialCommandGroup
        addCommands(

            new AimAmpCmd(swerveSubsystem, limeLightSubsystem),

            // Follow the trajectory
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
            AmpSwerveControllerCommandA,

            new InstantCommand(() -> swerveSubsystem.stopModules()),
            new InstantCommand(() -> clawSubsystem.clawSetPower(ClawConstants.kClawOutFast_Speed)),

            // Reset odometry to trajectory's initial pose && Stop the swerve modules after trajectory is complete
            new InstantCommand(() -> swerveSubsystem.resetOdometry(AmptrajectoryA.getInitialPose())),
            new InstantCommand(() -> swerveSubsystem.stopModules()) 

            
        );
    }
}
