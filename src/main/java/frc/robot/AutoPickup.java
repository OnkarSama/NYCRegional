// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoPickup {

  // private Robot m_robot;

  private CANSparkMax outtake;
  private CANSparkMax outtake2;
  private CANSparkMax uptake;
  private CANSparkMax intakeMotor;
  private CANSparkMax intakeMotor2;

  private DifferentialDrive robotDrive;

  private DoubleSolenoid dropIntake1;
  private DoubleSolenoid dropIntake2;

  private double startTime;

  /** Creates a new SimpleAuto. */
  public AutoPickup(double startTimelocal, CANSparkMax outtake, CANSparkMax outtake2, CANSparkMax uptake,
      DifferentialDrive robotDrive, DoubleSolenoid dropIntake1, DoubleSolenoid dropIntake2, CANSparkMax intakeMotor,
      CANSparkMax intakeMotor2) {

    this.outtake = outtake;
    this.outtake2 = outtake2;
    this.uptake = uptake;
    this.intakeMotor = intakeMotor;
    this.intakeMotor2 = intakeMotor2;

    this.robotDrive = robotDrive;

    this.dropIntake1 = dropIntake1;
    this.dropIntake2 = dropIntake2;

    this.startTime = startTimelocal;

  }

  // Called when the command is initially scheduled.

  public void init() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void periodic() {

    double time = Timer.getFPGATimestamp();

    if (time - startTime < 1) {

      dropIntake1.set(Value.kForward);
      dropIntake2.set(Value.kForward);

      this.robotDrive.arcadeDrive(0, 0);

      intakeMotor.set(.5);
      intakeMotor.set(.5);

    } else if (time - startTime < 1.5) {

      dropIntake1.set(Value.kForward);
      dropIntake2.set(Value.kForward);

      intakeMotor.set(.5);
      intakeMotor.set(.5);

      this.robotDrive.arcadeDrive(.5, 0);

    } else if (time - startTime > 3 && time - startTime < 8) {

      dropIntake1.set(Value.kReverse);
      dropIntake2.set(Value.kReverse);

      intakeMotor.set(0);
      intakeMotor.set(0);

      this.robotDrive.arcadeDrive(0, 0);

      this.outtake.set(1);
      ;
      this.outtake2.set(1);

      this.uptake.set(1);

    } else {

      dropIntake1.set(Value.kReverse);
      dropIntake2.set(Value.kReverse);

      intakeMotor.set(0);
      intakeMotor.set(0);
      this.robotDrive.arcadeDrive(0, 0);

      this.outtake2.set(0);
      this.outtake.set(0);

      this.uptake.set(0);
    }

  }

}
