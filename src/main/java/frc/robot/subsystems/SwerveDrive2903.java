package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class SwerveDrive2903 extends Subsystem {

SwerveModule2903 LeftFront;
// SwerveModule2903 LeftRear;
// SwerveModule2903 RightFront;
SwerveModule2903 RightRear;

double deadzone = 0.01;
  
int targetAngle = 0;

  public void init() {
    LeftFront = new SwerveModule2903(RobotMap.LeftFrontForward, RobotMap.LeftFrontTurn, RobotMap.LeftFrontLimit);
    // LeftRear = new SwerveModule2903(RobotMap.TBD, RobotMap.TBD);
    // RightFront = new SwerveModule2903(RobotMap.TBD, RobotMap.TBD);
    RightRear = new SwerveModule2903(RobotMap.RightRearForward, RobotMap.RightRearTurn, RobotMap.RightRearLimit);
    SmartDashboard.putNumber("Swerve LeftFront Amperage", 0);
    SmartDashboard.putNumber("Swerve RightRear Amperage", 0);
  }

  public void zeroModules() {
    LeftFront.zeroTurnMotor();
    RightRear.zeroTurnMotor();
  }

  public void goToZero() {
    LeftFront.setTurnDegrees(0);
    RightRear.setTurnDegrees(0);
  }

  public int joystickAngle(double x, double y) {
    int angle = -1;
    if (Math.abs(x) > deadzone || Math.abs(y) > deadzone) {
      angle = (int)Math.toDegrees(Math.atan2(x, y));
      if (angle < 0)
        angle += 360;
      targetAngle = angle;
    }
    return angle;
  }

  public double joystickMag(double x, double y) {
    return Math.sqrt(x*x+y*y);
  }

  public void swerveDrive(double power, double angle, double turn) {
    // LeftRear.setTurnDegrees((int)(angle-(turn*-45)));
    if (angle != -1) {
      RightRear.setTurnDegrees((int)(angle-(turn*-135)));
      LeftFront.setTurnDegrees((int)(angle-(turn*45)));
    } else {
      RightRear.setTurnDegrees((int)(targetAngle-(turn*-135)));
      LeftFront.setTurnDegrees((int)(targetAngle-(turn*45)));
    }
    // RightFront.setTurnDegrees((int)(angle-(turn*135)));

    //LeftRear.setForward(power);
    if (deadzone < power)
      RightRear.setForward(power);
      LeftFront.setForward(power);
    // RightFront.setForward(power);
    SmartDashboard.putNumber("Swerve Forward Speed", power);
    SmartDashboard.putNumber("Swerve Current Angle", LeftFront.getTurnDegrees());
    SmartDashboard.putNumber("Swerve Target Angle", angle);
    SmartDashboard.putNumber("Swerve LeftFront Current", LeftFront.ticksToAngle(LeftFront.getAbsoluteTurnTicks())%360);
    SmartDashboard.putNumber("Swerve RightRear Current", RightRear.ticksToAngle(RightRear.getAbsoluteTurnTicks())%360);

    SmartDashboard.putBoolean("Swerve LeftFront Limit", LeftFront.getLimit());
    SmartDashboard.putBoolean("Swerve RightRear Limit", RightRear.getLimit());
    
    if (SmartDashboard.getNumber("Swerve LeftFront Amperage", 0) < LeftFront.TurnMotor.getOutputCurrent())
    SmartDashboard.putNumber("Swerve LeftFront Amperage", LeftFront.TurnMotor.getOutputCurrent());
    if (SmartDashboard.getNumber("Swerve RightRear Amperage", 0) < RightRear.TurnMotor.getOutputCurrent())
    SmartDashboard.putNumber("Swerve RightRear Amperage", RightRear.TurnMotor.getOutputCurrent());
  }

  @Override
  protected void initDefaultCommand() {

  }
}