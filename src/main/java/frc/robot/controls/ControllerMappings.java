package frc.robot.controls;

import badgerutils.triggers.RobotTriggers;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;
import java.util.function.Consumer;

public class ControllerMappings {
    
    private final Controls controls;
    
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    public ControllerMappings(Controls controls, CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, CommandXboxController operatorController) {
        this.controls = controls;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.operatorController = operatorController;
    }

    public void bindDefaultControls(EventLoop eventLoop) {
        bindCommonControls(eventLoop);

        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.a().whileTrue(drivetrain.applyRequest(() -> controls.brake));
    }

    public void startDefaultControls() {
        clearAllPrevious();
        startCommonControls();
    }

    public void bindSwerveTestingControls(EventLoop eventLoop) {
        bindCommonControls(eventLoop);

        driverController.a().whileTrue(drivetrain.applyRequest(() -> controls.brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
                controls.point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        driverController.start().and(driverController.a()).onTrue(new InstantCommand(SignalLogger::start));
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driverController.start().and(driverController.b()).onTrue(new InstantCommand(SignalLogger::stop));

        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public void startSwerveTestingControls(){
        clearAllPrevious();
        startCommonControls();
    }

    public void bindCommonControls(EventLoop eventLoop) {
        RobotTriggers.disabled(eventLoop).whileTrue(drivetrain.applyRequest(() -> controls.idle).ignoringDisable(true));
    }
    
    public void startCommonControls(){
        Command fieldCentricCommand = controls.getDrivetrainFieldCentricCommand();
        drivetrain.setDefaultCommand(
                fieldCentricCommand == null ? new InstantCommand() : fieldCentricCommand
        );
    }

    public void clearAllPrevious(){
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
