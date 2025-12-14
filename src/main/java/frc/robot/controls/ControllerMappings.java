package frc.robot.controls;

import badgerutils.triggers.RobotTriggers;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    public void bindDefaultControls() {
        EventLoop eventLoop = new EventLoop();
        clearAllPreviousControls();
        bindCommonControls(eventLoop);

        drivetrain.setDefaultCommand(controls.getDrivetrainFieldCentricCommand());

        driverController.leftBumper(eventLoop).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.a(eventLoop).whileTrue(drivetrain.applyRequest(() -> controls.brake));

        CommandScheduler.getInstance().setActiveButtonLoop(eventLoop);
    }

    public void bindSwerveTestingControls() {
        EventLoop eventLoop = new EventLoop();
        clearAllPreviousControls();
        bindCommonControls(eventLoop);

        drivetrain.setDefaultCommand(controls.getDrivetrainFieldCentricCommand());
        
        driverController.a(eventLoop).whileTrue(drivetrain.applyRequest(() -> controls.brake));
        driverController.b(eventLoop).whileTrue(drivetrain.applyRequest(() ->
                controls.point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        driverController.start(eventLoop).and(driverController.a(eventLoop)).onTrue(new InstantCommand(SignalLogger::start));
        driverController.back(eventLoop).and(driverController.y(eventLoop)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back(eventLoop).and(driverController.x(eventLoop)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start(eventLoop).and(driverController.y(eventLoop)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start(eventLoop).and(driverController.x(eventLoop)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driverController.start(eventLoop).and(driverController.b(eventLoop)).onTrue(new InstantCommand(SignalLogger::stop));

        driverController.leftBumper(eventLoop).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        CommandScheduler.getInstance().setActiveButtonLoop(eventLoop);
    }

    public void bindCommonControls(EventLoop eventLoop) {
        RobotTriggers.disabled(eventLoop).whileTrue(drivetrain.applyRequest(() -> controls.idle).ignoringDisable(true));
    }

    public void clearAllPreviousControls(){
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
