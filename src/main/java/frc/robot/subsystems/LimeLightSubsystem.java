package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeConstants;

public class LimeLightSubsystem extends SubsystemBase {
    private final NetworkTable limeLightTable;

    public LimeLightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeLightTable.getEntry("pipeline").setNumber(0); // Remove when its solved
    }
    
    @Override
    public void periodic() {        
        //post to smart dashboard periodically
        SmartDashboard.putNumber("Target ID", getID());
        SmartDashboard.putNumber("Target TX", getYaw());
        SmartDashboard.putNumber("Target TY", getTY());
        SmartDashboard.putNumber("Speaker Distance", getDistance(LimeConstants.kTargetSpeakertHeight));
        SmartDashboard.putNumber("Speaker Error", getDistanceError(LimeConstants.kTargetSpeakertHeight, LimeConstants.kDistanceToSpeaker));
        SmartDashboard.putNumber("Amp Distance", getDistance(LimeConstants.kTargetAmpHeight));
        SmartDashboard.putNumber("Amp Error", getDistanceError(LimeConstants.kTargetAmpHeight, LimeConstants.kDistanceToAmp));
        SmartDashboard.updateValues();
    }

    public double getYaw() {
        return limeLightTable.getEntry("tx").getDouble(0);
    }
    
    public double getTY() {
        return limeLightTable.getEntry("ty").getDouble(0);
    }
    
    public double getID() {
        return limeLightTable.getEntry("tid").getDouble(0);
    }

    public double getDistance(double ProvidedHeight) { // Calcula la distancia horizontal al Target
        
        double targetOffsetAngle_Vertical = getTY();
        
        double targetHeight = ProvidedHeight; // Ejemplo: altura del AprilTag
        double cameraHeight = 20; 
        double cameraAngle = 20.5;
        
        return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + targetOffsetAngle_Vertical));
    }

    public double getDistanceError(double height, double distance) { // Calcula la distancia al punto optimo de posicion
        return (getDistance(height) - distance);
    }

    public boolean isTargetAligned(double height, double distance, double dE, double gY) { // Finaliza el Comando cuando el robot esta alineado
        return  Math.abs(getDistanceError(height, distance)) < dE && Math.abs(getYaw()) < gY;
    }

    public void setPipeline(int pipeline) {
        limeLightTable.getEntry("pipeline").setNumber(pipeline);
    }

    public void setLimeLed(int mode) { // Setea el modo de las leds
        limeLightTable.getEntry("ledMode").setValue(mode);
        // 0 Solid
        // 1 Off
        // 2 Blink
    }

}
