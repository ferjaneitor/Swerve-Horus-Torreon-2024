package frc.robot.Macros;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.AimSpeakerCmd;
import frc.robot.commands.LauncherActivateCmd;

public class SpeakerScoreMac extends SequentialCommandGroup {
    
    public SpeakerScoreMac(SwerveSubsystem swerveSubsystem, LauncherSubsystem launcherSubsystem, LimeLightSubsystem limeLightSubsystem) {

        addCommands(
            new AimSpeakerCmd(swerveSubsystem, limeLightSubsystem),

            new InstantCommand(() -> limeLightSubsystem.setLimeLed(2)),

            new LauncherActivateCmd(launcherSubsystem, swerveSubsystem, LauncherConstants.kUpperRoller_Speed2),

            new InstantCommand(() -> limeLightSubsystem.setLimeLed(0))
        );
    }
}
