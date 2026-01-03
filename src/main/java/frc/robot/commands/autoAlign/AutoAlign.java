package frc.robot.commands.autoAlign;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign {

    private CommandSwerveDrivetrain drivetrain;
    private PathPlannerPath path;

    public AutoAlign(CommandSwerveDrivetrain drivetrain, Pose2d pose) {
        this.drivetrain = drivetrain;

        this.path = drivetrain.createPathToPose(pose);
    }

    
}
