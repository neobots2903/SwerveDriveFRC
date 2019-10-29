package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class SwerveDrive2903 extends Subsystem {

SwerveModule2903 LeftFront;
// SwerveModule2903 LeftRear;
// SwerveModule2903 RightFront;
//SwerveModule2903 RightRear;
  
int targetAngle = 0;

  public void init() {
    LeftFront = new SwerveModule2903(RobotMap.LeftFrontForward, RobotMap.LeftFrontTurn);
    //LeftFront.setZero(-3385);
    // LeftRear = new SwerveModule2903(RobotMap.TBD, RobotMap.TBD);
    // RightFront = new SwerveModule2903(RobotMap.TBD, RobotMap.TBD);
    //RightRear = new SwerveModule2903(RobotMap.RightRearForward, RobotMap.RightRearTurn);
    //RightRear.setZero(15275);
  }

  public void goToZero() {
    LeftFront.setTurnDegrees(0);
    //RightRear.setTurnDegrees(0);
  }

  public int joystickAngle(double x, double y) {
    int angle = -1;
    if (Math.abs(x) > 0.1 || Math.abs(y) > 0.1) {
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
    //RightRear.setTurnDegrees((int)(angle-(turn*-135)));
    LeftFront.setTurnDegrees((int)-(turn*45)+((angle > -1) ? (int)angle : targetAngle));
    // RightFront.setTurnDegrees((int)(angle-(turn*135)));

    // LeftRear.setForward(power);
    //RightRear.setForward(power);
    LeftFront.setForward(power);
    // RightFront.setForward(power);
    SmartDashboard.putNumber("Swerve Forward Speed", power);
    SmartDashboard.putNumber("Swerve Current Angle", LeftFront.getTurnDegrees());
    SmartDashboard.putNumber("Swerve Target Angle", angle);
    SmartDashboard.putNumber("Swerve LeftFront Current", LeftFront.getAbsoluteTurnTicks());
    //SmartDashboard.putNumber("Swerve RightRear Current", RightRear.getAbsoluteTurnTicks());
  }

  @Override
  protected void initDefaultCommand() {

  }
}