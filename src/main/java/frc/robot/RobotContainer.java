// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.CommandSwerveDrivetrain.MaxSpeed;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.autos.Autos;
import frc.robot.controls.Controls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Controls controls;
    
    public final Autos autos;
    
    public RobotContainer() {
        controls = new Controls(drivetrain);
        autos = new Autos(drivetrain);
        
        configureBindings();
        
        SignalLogger.enableAutoLogging(false);
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autos.createAutoCommand();
    }
}