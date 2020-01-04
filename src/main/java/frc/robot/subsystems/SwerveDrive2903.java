package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class SwerveDrive2903 extends Subsystem {

  public SwerveModule2903 LeftFront;
  public SwerveModule2903 RightFront;
  public SwerveModule2903 LeftRear;
  public SwerveModule2903 RightRear;

  public List<SwerveModule2903> modules = new ArrayList<SwerveModule2903>();
  public int[] leftTurnAngles = {-135, -45, 135, 45}; //LF, RF, LR, RR
  public int[] rightTurnAngles = {45, 135, -45, -135};

  final int TICKS_PER_REV = 4096*6;
  final int DEG_PER_REV = 360;
  boolean isForward = true;

  double deadzone = 0.5; // joystick isn't actually in center, making sure doesn't move when not touched :)
  int targetAngle = 0;

  public void init() {
    LeftFront = new SwerveModule2903(RobotMap.LeftFrontForward, RobotMap.LeftFrontTurn, RobotMap.LeftFrontLimit);
    RightFront = new SwerveModule2903(RobotMap.TBD, RobotMap.TBD, RobotMap.TBD);
    LeftRear = new SwerveModule2903(RobotMap.TBD, RobotMap.TBD, RobotMap.TBD);
    RightRear = new SwerveModule2903(RobotMap.RightRearForward, RobotMap.RightRearTurn, RobotMap.RightRearLimit);

    modules.add(LeftFront);
    modules.add(RightFront);
    modules.add(LeftRear);
    modules.add(RightRear);
  }

  public void zeroModulesLimit() {
    for (SwerveModule2903 module : modules)
      module.zeroTurnMotor();
  }

  public void zeroModules() {
    for (SwerveModule2903 module : modules)
      module.setZero();
  }

  public void goToZero() {
    setTurnDegrees(0);
  }

  public int joystickAngle(double x, double y) {
    int angle = -1;
    if (Math.abs(x) > deadzone || Math.abs(y) > deadzone) {
      angle = (int)Math.toDegrees(Math.atan2(x, y));
      if (angle < 0)
        angle += 360;
    }
    return angle;
  }

  public double joystickMag(double x, double y) {
    return Math.sqrt(x*x+y*y);
  }

  public void setForward(double speed) {
    for (SwerveModule2903 module : modules)
      module.ForwardMotor.set(speed * (isForward ? 1 : -1));
  }

  public void setTurnDegrees(int degrees) {
    if (degrees == -1) return;
    int currentTargetTic = (int)LeftFront.TurnMotor.getClosedLoopTarget()-LeftFront.getJoyTurnTicks();
    int localTic = LeftFront.angleToTicks(degrees) - currentTargetTic % TICKS_PER_REV;
    if (localTic < -TICKS_PER_REV/2)
      localTic += TICKS_PER_REV;
    else if (localTic > TICKS_PER_REV/2)
      localTic -= TICKS_PER_REV;
    if (Math.abs(localTic) > TICKS_PER_REV/4) {
        isForward = !isForward;
        for (SwerveModule2903 module : modules) {
          module.setEncoder(currentTargetTic + TICKS_PER_REV/2*(isForward ? 1 : -1));
          module.TurnMotor.set(ControlMode.Position,currentTargetTic + TICKS_PER_REV/2*(isForward ? 1 : -1));
        }
        setTurnDegrees(degrees);
        return;
    }
    for (SwerveModule2903 module : modules)
      module.TurnMotor.set(ControlMode.Position,currentTargetTic+localTic+module.getJoyTurnTicks());
  }

  public void swerveDrive(double power, double angle, double turn, boolean fieldCentric) {

    int[] angles = (turn < 0) ? leftTurnAngles : rightTurnAngles;
    turn = Math.abs(turn);
    int angleIndex = (angle >= 315 || angle <= 45) ? 0 :
                    (angle >= 45 && angle <= 135) ? 1 :
                    (angle >= 135 && angle <= 225) ? 2 :
                    3;
    for (SwerveModule2903 module : modules) {
      module.setMaxJoyTurnDegrees(angles[angleIndex]);
      module.setJoyTurnPercent(turn);
      if (++angleIndex > angles.length - 1) angleIndex = 0;
    }
    
    if (angle != -1) {
      targetAngle = (int)angle;
      setTurnDegrees((int)(angle-((fieldCentric)?Robot.navXSubsystem.turnAngle():0)));
    } else {
      setTurnDegrees((int)(targetAngle-((fieldCentric)?Robot.navXSubsystem.turnAngle():0)));
    }

    //LeftRear.setForward(power);
    if (deadzone < Math.abs(power)) {
      setForward(power/5);
    }
    // RightFront.setForward(power);
    SmartDashboard.putNumber("Swerve Forward Speed", power);
    SmartDashboard.putNumber("Swerve Turn Speed", turn);
    SmartDashboard.putNumber("Swerve Current Angle", LeftFront.getTurnDegrees());
    SmartDashboard.putNumber("Swerve Target Angle", angle);
    SmartDashboard.putNumber("Swerve LeftFront Current", LeftFront.ticksToAngle(LeftFront.getAbsoluteTurnTicks())%360);
    SmartDashboard.putNumber("Swerve RightRear Current", RightRear.ticksToAngle(RightRear.getAbsoluteTurnTicks())%360);
    SmartDashboard.putBoolean("Swerve LeftFront Limit", LeftFront.getLimit());
    SmartDashboard.putBoolean("Swerve RightRear Limit", RightRear.getLimit());
    SmartDashboard.putBoolean("Field Centric?", fieldCentric);

    //if (SmartDashboard.getNumber("Swerve LeftFront Amperage", 0) < LeftFront.TurnMotor.getOutputCurrent())
      SmartDashboard.putNumber("Swerve LeftFront Amperage", LeftFront.TurnMotor.getOutputCurrent());
    //if (SmartDashboard.getNumber("Swerve RightRear Amperage", 0) < RightRear.TurnMotor.getOutputCurrent())
      SmartDashboard.putNumber("Swerve RightRear Amperage", RightRear.TurnMotor.getOutputCurrent());
      
    // if (LeftFront.TurnMotor.getOutputCurrent() > 45 || RightRear.TurnMotor.getOutputCurrent() > 45 ) {
    //   for (SwerveModule2903 module : modules)
    //     module.TurnMotor.set(ControlMode.PercentOutput, 0);
    //   disabled = true;
    // }
    // SmartDashboard.putBoolean("Wheels disabled?", disabled);
  }

  public void TankDrive(double left, double right) {
    goToZero();
    LeftFront.ForwardMotor.set(left * (isForward ? 1 : -1));
    RightRear.ForwardMotor.set(right * (isForward ? 1 : -1));
  }

  @Override
  protected void initDefaultCommand() {

  }
}