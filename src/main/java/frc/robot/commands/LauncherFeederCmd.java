package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.TejuinoBoard;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;


public class LauncherFeederCmd extends Command { // This Command is a while-pressed Type
private final LauncherSubsystem launcherSubsystem;
private final LimeLightSubsystem limeLightSubsystem;
    private final double feederSpeed;
    private TejuinoBoard board = new TejuinoBoard();

    public LauncherFeederCmd(LauncherSubsystem launcherSubsystem_i, double feederSpeed, LimeLightSubsystem limeLightSubsystem_i) {
        this.launcherSubsystem = launcherSubsystem_i;
        this.limeLightSubsystem = limeLightSubsystem_i;
        this.feederSpeed = feederSpeed;
        addRequirements(launcherSubsystem);
    }


    @Override
    public void initialize() {
        System.out.println("Reload Cmd started!");
        limeLightSubsystem.setLimeLed(1);
    }

    @Override
    public void execute() { // Los rollers giran hacia adentro para absorber el aro
        
        board.init(1);
        board.all_led_control(1, 255, 125, 0);
        board.all_led_control(0, 255, 125, 0);

        launcherSubsystem.upper_Rollers(-feederSpeed);
        launcherSubsystem.lower_Rollers(-feederSpeed);
    }

    @Override 
    public void end(boolean interrupted) {
        launcherSubsystem.stopRollers();
        limeLightSubsystem.setLimeLed(0);
        System.out.println("Reload Cmd ended!");
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}