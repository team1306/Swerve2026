package frc.robot.controls;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.Intake;

import java.util.Optional;
import java.util.function.Consumer;

public class ControllerMappings {
    
    private final Controls controls;
    
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final Intake intake;

    public ControllerMappings(Controls controls, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, CommandXboxController operatorController, Intake intake) {
        this.controls = controls;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.intake = intake;
    
    }

    public void bindDefaultControls() {
        clearAllPreviousControls();
        bindCommonControls();

        drivetrain.setDefaultCommand(controls.getDrivetrainFieldCentricCommand());

        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        intake.setDefaultCommand(intake.runPercent(() -> operatorController.getLeftTriggerAxis()));
       
    }

    public void bindSwerveTestingControls() {
        clearAllPreviousControls();
        bindCommonControls();

        drivetrain.setDefaultCommand(controls.getDrivetrainFieldCentricCommand());
        
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
                controls.point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        driverController.start().and(driverController.a()).onTrue(new InstantCommand(SignalLogger::start));
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driverController.start().and(driverController.b()).onTrue(new InstantCommand(SignalLogger::stop));

        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));
    }

    public void bindCommonControls() {
        RobotModeTriggers.disabled().whileTrue(controls.getDrivetrainIdleCommand());
    }

    public void clearAllPreviousControls(){
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        removeAndCancelDefaultCommand(drivetrain);
    }


    //TODO ============================== SHOULD BE MOVED TO BADGERUTILS ===============================
    public static void removeAndCancelDefaultCommand(Subsystem subsystem) {
        runIfNotNull(subsystem.getDefaultCommand(), (Command command) -> {
            subsystem.removeDefaultCommand();
            command.cancel();
        });
    }

    /**
     * Run consumer if object is not null, else do nothing
     *
     * @param <T> type of object
     * @param object input object
     * @param objectConsumer consumer to apply to object
     *
     * @return returns optional input object
     */
    public static <T> Optional<T> runIfNotNull(T object, Consumer<T> objectConsumer) {
        if (object != null) {
            objectConsumer.accept(object);
        }
        return Optional.ofNullable(object);
    }
}
