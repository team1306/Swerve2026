// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * This intake implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(32);
  private final StatusSignal<Voltage> intakeVoltage = intakeMotor.getMotorVoltage();
  private final StatusSignal<Current> intakeSupplyCurrent = intakeMotor.getSupplyCurrent();
  private final StatusSignal<Current> intakeStatorCurrent = intakeMotor.getStatorCurrent();
  private final StatusSignal<Temperature> intakeTemperature = intakeMotor.getDeviceTemp();
  private final StatusSignal<AngularVelocity> intakeVelocity = intakeMotor.getVelocity();


  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeMotor.getConfigurator().apply(config, 0.25);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakeVelocity, intakeSupplyCurrent, intakeStatorCurrent, intakeTemperature,intakeVelocity);
    intakeMotor.optimizeBusUtilization();
  }


@Override
public void updateInputs(IntakeIOInputs inputs){
    
    BaseStatusSignal.refreshAll(
        intakeVoltage, 
        intakeSupplyCurrent, 
        intakeStatorCurrent,
        intakeTemperature, 
        intakeVelocity
        );

    inputs.intakeVoltage = intakeVoltage.getValueAsDouble();
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValueAsDouble();
    inputs.intakeStatorCurrent = intakeStatorCurrent.getValueAsDouble();
    inputs.intakeTemperature = intakeTemperature.getValueAsDouble();
    inputs.intakeVelocity = intakeVelocity.getValueAsDouble();
}
@Override
public void setIntakeVoltage(double volts) {
intakeMotor.setVoltage(volts);
}
}


