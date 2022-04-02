// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoHigh {

  // private Robot m_robot;

  private CANSparkMax outtake;
  private CANSparkMax outtake2;
  private CANSparkMax uptake;

  private DifferentialDrive robotDrive;

  private double startTime;

  /** Creates a new SimpleAuto. */
  public AutoHigh(double startTimelocal, CANSparkMax outtake, CANSparkMax outtake2, CANSparkMax uptake,
      DifferentialDrive robotDrive) {

    this.outtake = outtake;
    this.outtake2 = outtake2;
    this.uptake = uptake;

    this.robotDrive = robotDrive;

    this.startTime = startTimelocal;

  }

  // Called when the command is initially scheduled.

  public void init() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void periodic() {

    double time = Timer.getFPGATimestamp();

    if (time - startTime < 1.5) {

      // this.leftSpeedGroup.set(.2);
      // this.rightSpeedGroup.set(.2);
      this.robotDrive.arcadeDrive(.5, 0);
    } else if (time - startTime > 3 && time - startTime < 8) {

      this.robotDrive.arcadeDrive(0, 0);

      this.outtake.set(1);
      ;
      this.outtake2.set(1);

      this.uptake.set(1);

    } else {
      this.robotDrive.arcadeDrive(0, 0);

      this.outtake2.set(0);
      this.outtake.set(0);

      this.uptake.set(0);
    }

  }

}
