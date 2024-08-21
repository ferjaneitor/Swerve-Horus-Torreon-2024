package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClawConstants;


public class ClawSubsystem extends SubsystemBase {

    private CANSparkMax claw_Motor = new CANSparkMax(ClawConstants.kClawMotorPort, MotorType.kBrushless);
    RelativeEncoder claw_Encoder = claw_Motor.getEncoder(Type.kHallSensor, 42);
   
    public ClawSubsystem() {
    }

    @Override
    public void periodic() { // FUncion periodica para publicar Dashboards
        SmartDashboard.putNumber("Upper Right Roller", claw_Encoder.getPosition());
        
    }

    public void clawSetPower(double speed) { // Set speed de UpperRollers
        claw_Motor.set(speed);
        
    }

    public void stopClaw() {
        claw_Motor.set(0);
        
    }
}