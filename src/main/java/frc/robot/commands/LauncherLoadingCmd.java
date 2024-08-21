package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TejuinoBoard;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class LauncherLoadingCmd extends Command {
    private final LauncherSubsystem launcherSubsystem;
    private final double loadingSpeed;
    private final Timer timer = new Timer();
    private int stage = 0;

    public LauncherLoadingCmd(LauncherSubsystem launcherSubsystem_i, double loadingSpeed) {
        this.launcherSubsystem = launcherSubsystem_i;
        this.loadingSpeed = loadingSpeed;
        addRequirements(launcherSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Loading Cmd started!");
        stage = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        switch (stage) {
            case 0:
                launcherSubsystem.lower_Rollers(loadingSpeed); // Subir aro hasta perder traccion
                if (timer.hasElapsed(1.3)) {
                    launcherSubsystem.lower_Rollers(0);
                    stage++;
                    timer.reset();
                }
                break;
            case 1:
                if (timer.hasElapsed(0.5)) {
                    launcherSubsystem.upper_Rollers(-loadingSpeed); // Bajar el aro para ajustarlo a punto de disparo
                    launcherSubsystem.lower_Rollers(-loadingSpeed);
                    stage++;
                    timer.reset();
                }
                break;
            case 2:
                if (timer.hasElapsed(0.27)) {
                    launcherSubsystem.stopRollers();
                    stage++;
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcherSubsystem.stopRollers();
        System.out.println("Loading Cmd ended!");
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // El comando termina después de completar la última etapa
        return stage > 2;
    }
}
