/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class TeleOp extends Command {

  private boolean zeroLock = false;
  private boolean fieldCentric = false;

  public TeleOp() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.swerveDriveSubsystem);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.swerveDriveSubsystem.zeroModules(); //will uncomment when we add zero-ing sensors
    Robot.navXSubsystem.zero();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double y = -Robot.driveJoy.getRawAxis(1);
    double x = Robot.driveJoy.getRawAxis(0);
    double forward = Robot.driveJoy.getRawAxis(2);
    double backward = Robot.driveJoy.getRawAxis(3);

    double turnX = Robot.driveJoy.getRawAxis(4);
    if (Math.abs(turnX) < 0.01) turnX = 0;

    if (Robot.driveJoy.getRawButton(8)) {
      if (!zeroLock) {
        fieldCentric = !fieldCentric;
        zeroLock = true;
      }
    } else {
      zeroLock = false;
    }

    SmartDashboard.putNumber("Swerve LeftFront Ticks", Robot.swerveDriveSubsystem.LeftFront.getTurnTicks());
    SmartDashboard.putNumber("Swerve RightRear Ticks", Robot.swerveDriveSubsystem.RightRear.getTurnTicks());
    SmartDashboard.putNumber("Swerve LeftFront Absolute", Robot.swerveDriveSubsystem.LeftFront.getAbsoluteTurnTicks());
    SmartDashboard.putNumber("Swerve RightRear Absolute", Robot.swerveDriveSubsystem.RightRear.getAbsoluteTurnTicks());
    SmartDashboard.putBoolean("Left Hand Visible?", Robot.leapTable.getEntry("LeftVisible").getBoolean(false));
    SmartDashboard.putBoolean("Right Hand Visible?", Robot.leapTable.getEntry("RightVisible").getBoolean(false));
    SmartDashboard.putNumber("Left Hand X", Robot.leapTable.getEntry("LeftPalmPositionX").getDouble(0));
    SmartDashboard.putNumber("Right Hand X", Robot.leapTable.getEntry("RightPalmPositionX").getDouble(0));
    SmartDashboard.putNumber("Detection Count", Robot.tensorTable.getEntry("Detection Count").getDouble(0));
    SmartDashboard.putNumber("Main Center X", Robot.tensorTable.getEntry("Detection Center X").getDoubleArray(new double[]{0})[0]);
    SmartDashboard.putNumber("Main Center Y", Robot.tensorTable.getEntry("Detection Center Y").getDoubleArray(new double[]{0})[0]);
    
    Robot.swerveDriveSubsystem.swerveDrive((forward-backward), Robot.swerveDriveSubsystem.joystickAngle(x, y), turnX, fieldCentric);
  
    // double leftHand = 0;
    // double rightHand = 0;

    // if (Robot.leapTable.getEntry("LeftVisible").getBoolean(false) && Robot.leapTable.getEntry("LeftPinchDistance").getDouble(100) < 15)
    //   leftHand = Robot.leapTable.getEntry("LeftPalmPositionZ").getDouble(0)/300;
    // if (Robot.leapTable.getEntry("RightVisible").getBoolean(false) && Robot.leapTable.getEntry("RightPinchDistance").getDouble(100) < 15)
    //   rightHand = Robot.leapTable.getEntry("RightPalmPositionZ").getDouble(0)/300;

    // Robot.swerveDriveSubsystem.TankDrive(leftHand, rightHand);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
