package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOPhoenix6 implements IntakeIO{
    private final TalonFX intakeMotor;

   
    
    private final StatusSignal<Voltage> intakeVoltage;
    private final StatusSignal<Current> intakeSupplyCurrent;
    private final StatusSignal<Current> intakeStatorCurrent;
    private final StatusSignal<Temperature> intakeTemperature;
    private final StatusSignal<AngularVelocity> intakeVelocity;

    public IntakeIOPhoenix6(){
        intakeMotor = new TalonFX(100);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        configuration.CurrentLimits.SupplyCurrentLimit = 40;
        configuration.CurrentLimits.StatorCurrentLimit = 60;

        intakeMotor.getConfigurator().apply(configuration);

        intakeVoltage = intakeMotor.getMotorVoltage();
        intakeSupplyCurrent = intakeMotor.getSupplyCurrent();
        intakeStatorCurrent = intakeMotor.getStatorCurrent();
        intakeTemperature = intakeMotor.getDeviceTemp();
        intakeVelocity = intakeMotor.getVelocity();
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
