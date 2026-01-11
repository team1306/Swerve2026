// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.CommandSwerveDrivetrain.MaxSpeed;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.autos.Autos;
import frc.robot.controls.Controls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Controls controls;
    
    public final Autos autos;

    private final Camera frontLeftCamera = 
    new Camera(drivetrain, drivetrain::addVisionMeasurement, "frontLeft", 
    new Transform3d(0.307325, 0.307325, 0.215781, 
    new Rotation3d(0, Units.degreesToRadians(-45),  Units.degreesToRadians(45)))); 

    private final Camera frontRightCamera = 
    new Camera(drivetrain, drivetrain::addVisionMeasurement, "frontRight", 
    new Transform3d(0.307325, -0.307325, 0.215781, 
    new Rotation3d(0, Units.degreesToRadians(-45),  Units.degreesToRadians(-45))));

    private final Camera backLeftCamera = 
    new Camera(drivetrain, drivetrain::addVisionMeasurement, "backLeft", 
    new Transform3d(-0.307325, 0.307325, 0.215781, 
    new Rotation3d(0, Units.degreesToRadians(-45),  Units.degreesToRadians(135)))); 

    private final Camera backRightCamera = 
    new Camera(drivetrain, drivetrain::addVisionMeasurement, "backRight", 
    new Transform3d(-0.307325, -0.307325, 0.215781, 
    new Rotation3d(0, Units.degreesToRadians(-45),  Units.degreesToRadians(-135)))); 
   
    public RobotContainer() {
        controls = new Controls(drivetrain);
        autos = new Autos(drivetrain);
        
        configureBindings();
        
        SignalLogger.enableAutoLogging(true);
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Approach Processer");
        //return autos.createAutoCommand();
    }
}