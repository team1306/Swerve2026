package frc.robot.autos;

import badgerlog.BadgerLog;
import badgerlog.annotations.Entry;
import badgerlog.annotations.EntryType;
import badgerlog.annotations.Key;
import badgerutils.triggers.AllianceTriggers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Autos {
    
    private final CommandSwerveDrivetrain drivetrain;
    
    @Entry(EntryType.SENDABLE)
    private SendableChooser<String> autoChooser = new SendableChooser<>();
    
    @Entry(EntryType.SUBSCRIBER)
    @Key("Wait Time")
    private double autoWaitTime = 0;
    
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
        return new WaitCommand(autoWaitTime).andThen(new PathPlannerAuto(autoChooser.getSelected()));
    }

    private void resetAutoOdometry() {
        if(!DriverStation.isDisabled()) return;

        Pose2d startingPosition = new PathPlannerAuto(autoChooser.getSelected()).getStartingPose();
        
        drivetrain.resetPose(AllianceTriggers.isBlueAlliance() ? startingPosition : FlippingUtil.flipFieldPose(startingPosition));
    }
    
    private void bindNamedCommands(){
//        NamedCommands.registerCommand();
    }
}
