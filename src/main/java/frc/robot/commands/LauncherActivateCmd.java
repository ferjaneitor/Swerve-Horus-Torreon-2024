package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.LauncherConstants;

public class LauncherActivateCmd extends Command {
    private final LauncherSubsystem launcherSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final double upperSpeed;
    private final Timer timer = new Timer();
    private int stage = 0;

    public LauncherActivateCmd(LauncherSubsystem launcherSubsystem_i, SwerveSubsystem swerveSubsystem_i, double upperSpeed) {
        this.launcherSubsystem = launcherSubsystem_i;
        this.swerveSubsystem = swerveSubsystem_i;
        this.upperSpeed = upperSpeed;
        addRequirements(launcherSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Launcher Cmd started!");
        swerveSubsystem.stopModules();
        stage = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        switch (stage) {
            case 0:
                launcherSubsystem.upper_Rollers(upperSpeed);// Revolucionar los rollers de arriba
                if (timer.hasElapsed(0.8)) {
                    launcherSubsystem.lower_Rollers(LauncherConstants.kLowerRoller_Speed);
                    stage++;
                    timer.reset();
                }
                break;
            case 1:
                if (timer.hasElapsed(1)) {
                    launcherSubsystem.stopRollers();
                    stage++;
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcherSubsystem.stopRollers();
        System.out.println("Launcher Cmd ended!");
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // El comando termina después de completar la última etapa
        return stage > 1;
    }
}
