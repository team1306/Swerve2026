package frc.robot.autos;

import badgerlog.BadgerLog;
import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;
import badgerlog.annotations.Key;
import badgerlog.annotations.UnitConversion;
import badgerutils.triggers.AllianceTriggers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.Seconds;

/**
 * Adds a chooser to NetworkTables under "Autos/autoChooser" that contains a list of all the autos present in the project as well as a "none" option (no auto run). Defaults to "none"
 * Also adds a NetworkTables entry of a wait time in seconds before the selected auto runs.
 * For validation before the auto runs, a NetworkTables button is created that resets the odometry to the starting position of the auto. It is purely for pre-auto validation, as paths reset odometry on autonomous start anyway.
 */
public class Autos {
    
    private final CommandSwerveDrivetrain drivetrain;
    
    @Entry(EntryType.SENDABLE)
    private SendableChooser<String> autoChooser = new SendableChooser<>();
    
    @Entry(EntryType.SUBSCRIBER)
    @Key("Wait Time Seconds")
    @UnitConversion("seconds")
    private Time autoWaitTime = Seconds.of(0);
    
    public Autos(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        autoChooser.setDefaultOption("None", "");

        for (String auto : AutoBuilder.getAllAutoNames()){
            autoChooser.addOption(auto, auto);
        }
        
        BadgerLog.createAutoResettingButton("Autos/Reset Odometry", CommandScheduler.getInstance().getDefaultButtonLoop())
                .onTrue(new InstantCommand(this::resetAutoOdometry).ignoringDisable(true));
        
        bindNamedCommands();
    }
    
    public Command createAutoCommand(){
        if(autoChooser.getSelected().isEmpty()){
            return new InstantCommand();
        }
        
        return new WaitCommand(autoWaitTime).andThen(new PathPlannerAuto(autoChooser.getSelected()));
    }

    private void resetAutoOdometry() {
        if(!DriverStation.isDisabled() || autoChooser.getSelected().isEmpty()) return;
        
        Pose2d startingPosition = new PathPlannerAuto(autoChooser.getSelected()).getStartingPose();
        
        drivetrain.resetPose(AllianceTriggers.isBlueAlliance() ? startingPosition : FlippingUtil.flipFieldPose(startingPosition));
    }
    
    private void bindNamedCommands(){
//        NamedCommands.registerCommand();
    }
}
