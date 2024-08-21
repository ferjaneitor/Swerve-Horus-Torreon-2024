package frc.robot.subsystems;

//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;



public class SwerveModules {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turnPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed; 
    private final double absoluteEncoderOffsetRad;
    private int absoluteEncoderID;

    public SwerveModules(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderID = absoluteEncoderID;
        absoluteEncoder = new CANcoder(absoluteEncoderID, "6348 Horus CANivore");

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); 
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad); 
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turnPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getAbsolutePosition().getValue();
        angle *= 2.0 * Math.PI;
        angle -= (absoluteEncoderOffsetRad * 2 * Math.PI);
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }    

    public void resetEncoders() {
        SmartDashboard.putNumber("Encoder: "+absoluteEncoderID+"", getAbsoluteEncoderRad());
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getTurningPosition()));

    }

    public void setDesiredState (SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }
    
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }    
    
        
}

