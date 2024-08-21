package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.LimeConstants;
import frc.robot.subsystems.LimeLightSubsystem; // Asumiendo que tienes un subsistema para manejar la LimeLight

public class AimAmpCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final LimeLightSubsystem limeLightSubsystem;
    public static boolean AutoAimRunning;

    public AimAmpCmd(SwerveSubsystem swerveSubsystem, LimeLightSubsystem limeLightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limeLightSubsystem = limeLightSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        limeLightSubsystem.setPipeline(1); // Asumiendo que tienes un método para configurar la pipeline adecuada
        System.out.println("Amp Auto Aim Started ");
        AutoAimRunning = true;
    
    }

    @Override
    public void execute() {
        // Obtiene la desviación angular (yaw) y la distancia desde la LimeLight
        double yaw = limeLightSubsystem.getYaw()/3;
        double distance = limeLightSubsystem.getDistanceError(LimeConstants.kTargetAmpHeight, LimeConstants.kDistanceToAmp);

        // Aquí, puedes implementar tu lógica para ajustar los módulos Swerve
        swerveSubsystem.alignToTarget(yaw, distance);
    }

    @Override
    public boolean isFinished() {
        // Define una condición para finalizar el comando, por ejemplo, cuando el robot esté suficientemente alineado
        return limeLightSubsystem.isTargetAligned(LimeConstants.kTargetAmpHeight, LimeConstants.kDistanceToAmp, 1, 2);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            swerveSubsystem.stopModules();
        }
        System.out.println("Amp Auto Aim Finished ");
        AutoAimRunning = false;
    }
}
