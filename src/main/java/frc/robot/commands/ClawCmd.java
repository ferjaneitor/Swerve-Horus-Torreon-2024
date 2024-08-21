package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TejuinoBoard;
import frc.robot.subsystems.ClawSubsystem;


public class ClawCmd extends Command { // This Command is a while-pressed Type
private final ClawSubsystem clawSubsystem;
    private final double speed;
    private TejuinoBoard board = new TejuinoBoard();

    public ClawCmd(ClawSubsystem clawSubsystem, double speed) {

        this.clawSubsystem = clawSubsystem;
        this.speed = speed;
        addRequirements(clawSubsystem);
    }


    @Override
    public void initialize() {
        System.out.println("ClawCmd started!");
    }

    @Override
    public void execute() { // Los rollers giran hacia adentro para absorber el aro
                
        clawSubsystem.clawSetPower(speed);
        board.init(1);
        board.all_led_control(0, 125, 255, 0);
        board.all_led_control(1, 125, 255, 0);

    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stopClaw();
        System.out.println("ClawCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}