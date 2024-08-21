package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import frc.robot.Constants.LauncherConstants;

/*import frc.robot.Constants.LauncherConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;*/

public class LauncherSubsystem extends SubsystemBase {

    private CANSparkMax upperRight_roller = new CANSparkMax(LauncherConstants.kUpperRightRollerMotorPort, MotorType.kBrushless);
    private CANSparkMax upperLeft_roller = new CANSparkMax(LauncherConstants.kUpperLeftRollerMotorPort, MotorType.kBrushless);
    private CANSparkMax lowerRight_roller = new CANSparkMax(LauncherConstants.kLowerRightRollerMotorPort, MotorType.kBrushless);
    private CANSparkMax lowerLeft_roller = new CANSparkMax(LauncherConstants.kLowerLeftRollerMotorPort, MotorType.kBrushless);

    RelativeEncoder upperRight_Encoder = upperRight_roller.getEncoder(Type.kHallSensor, 42);
    RelativeEncoder upperLeft_Encoder = upperLeft_roller.getEncoder(Type.kHallSensor, 42);
    RelativeEncoder lowerRight_Encoder = lowerRight_roller.getEncoder(Type.kHallSensor, 42);
    RelativeEncoder lowerLeft_Encoder = lowerLeft_roller.getEncoder(Type.kHallSensor, 42);

    public LauncherSubsystem() {
    }


    public void upper_Rollers(double speed) { // Set speed de UpperRollers
        upperRight_roller.set(-speed);
        upperLeft_roller.set(speed);
    }

    public void lower_Rollers(double speed) { // Set speed de LowerRollers
        lowerRight_roller.set(-speed);
        lowerLeft_roller.set(speed);
    }

    public void stopRollers() {
        upperRight_roller.set(0);
        upperLeft_roller.set(0);
        lowerRight_roller.set(0);
        lowerLeft_roller.set(0);
    }
}