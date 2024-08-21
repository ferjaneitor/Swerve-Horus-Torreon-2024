package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TejuinoBoard;

public class LEDSubsystem extends SubsystemBase{
    private final TejuinoBoard board = new TejuinoBoard();

    public LEDSubsystem() {
        board.init(board.TEJUINO_DEVICE_NUMBER_1);
        board.rainbow_effect(board.LED_STRIP_0); //Tejuino
        board.rainbow_effect(board.LED_STRIP_1);
        board.rainbow_effect(board.LED_STRIP_2);
    }
}
