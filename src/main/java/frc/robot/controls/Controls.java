package frc.robot.controls;

import badgerlog.BadgerLog;
import badgerutils.statemachine.Edges;
import badgerutils.statemachine.StateMachine;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.EnumMap;

import static frc.robot.subsystems.CommandSwerveDrivetrain.MaxAngularRate;
import static frc.robot.subsystems.CommandSwerveDrivetrain.MaxSpeed;

public class Controls {
    
    //============================== SWERVE REQUESTS ===============================
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    
    //============================== SUBSYSTEMS ====================================
    private final CommandSwerveDrivetrain drivetrain;

    
    //============================== CONTROLLERS ===================================
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;


    //============================== CONTROLLER STATE ==============================

    private final StateMachine<ControllerBindings> stateMachine;
    private final EnumMap<ControllerBindings, EventLoop> stateToLoop = new EnumMap<>(ControllerBindings.class);
    private final ControllerMappings mappings;
    
    public Controls(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        DriverStation.silenceJoystickConnectionWarning(true);
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        
        this.mappings = new ControllerMappings(this, drivetrain, driverController, operatorController);
        
        for(var state : ControllerBindings.values()) {
            stateToLoop.put(state, new EventLoop());
        }

        mappings.bindDefaultControls(stateToLoop.get(ControllerBindings.DEFAULT));
        mappings.bindSwerveTestingControls(stateToLoop.get(ControllerBindings.SWERVE_TESTING));
        
        Edges<ControllerBindings> edges = new Edges<>();
        edges.anyToState(ControllerBindings.DEFAULT, transition -> mappings.startDefaultControls());
        edges.anyToState(ControllerBindings.SWERVE_TESTING, transition -> mappings.startSwerveTestingControls());
        
        edges.anyToAny(transition -> CommandScheduler.getInstance().setActiveButtonLoop(stateToLoop.get(transition.nextState())));
        
        stateMachine = new StateMachine<>(ControllerBindings.DEFAULT, edges);
        BadgerLog.createSelectorFromEnum(
                "Controls/Controller Mode", 
                ControllerBindings.class, 
                ControllerBindings.DEFAULT, 
                value -> stateMachine.tryChangeState((ControllerBindings) value));
    }
    
    public Command getDrivetrainFieldCentricCommand() {
        return drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    public Command getDrivetrainIdleCommand() {
        final var idle = new SwerveRequest.Idle();
        return drivetrain.applyRequest(() -> idle).ignoringDisable(true);
    }

    public enum ControllerBindings {
        SWERVE_TESTING,
        DEFAULT
    }
}
