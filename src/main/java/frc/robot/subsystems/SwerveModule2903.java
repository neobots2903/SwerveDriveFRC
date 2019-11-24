package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule2903 {
  public CANSparkMax ForwardMotor;
  public WPI_TalonSRX TurnMotor;
  public DigitalInput limit;
  final int TICKS_PER_REV = 4096*6;
  final int DEG_PER_REV = 360;
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  boolean isForward = true;

  public SwerveModule2903(int forwardMotorId, int turnMotorId, int limitId) {
    ForwardMotor = new SafeCANSparkMax(forwardMotorId, MotorType.kBrushless);
    TurnMotor = new WPI_TalonSRX(turnMotorId);
    limit = new DigitalInput(limitId);
    TurnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);

    TurnMotor.configPeakCurrentLimit(45, kTimeoutMs);
    TurnMotor.configPeakCurrentDuration(1500, kTimeoutMs);
    TurnMotor.configContinuousCurrentLimit(27, kTimeoutMs);
    
    /**
     * Grab the 360 degree position of the MagEncoder's absolute
     * position, and intitally set the relative sensor to match.
     */
    int absolutePosition = TurnMotor.getSensorCollection().getPulseWidthPosition();
    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    setZero(absolutePosition);

    setPowerPercent(1); //set max turn power to just 100%;
    TurnMotor.set(ControlMode.Position, getTurnTicks());
    TurnMotor.config_kP(0, 0.15);
  }

  public void setForward(double speed) {
    ForwardMotor.set(speed * (isForward ? 1 : -1));
  }

  public boolean getLimit() {
    return limit.get();
  }

  public void zeroTurnMotor() {
    while (!limit.get()) {
      TurnMotor.set(ControlMode.PercentOutput, 1);
      SmartDashboard.putBoolean("Init Limit", getLimit());
    }
    TurnMotor.set(ControlMode.PercentOutput, 0);
    setZero(0);
  }

  public void setZero(int newZero) {
    TurnMotor.setSelectedSensorPosition(newZero, kPIDLoopIdx, kTimeoutMs);
  }

  public int getTurnTicks() {
   return TurnMotor.getSensorCollection().getQuadraturePosition();// % TICKS_PER_REV;
  }

  public int getAbsoluteTurnTicks() {
    return TurnMotor.getSensorCollection().getPulseWidthPosition();// % TICKS_PER_REV;
  }

  public int getTurnDegrees() {
    return ticksToAngle(getTurnTicks());
  }

  //oregano
  // public void setTurnDegrees(int degrees) {
  //   int localDeg = degrees - getTurnDegrees() % 360;
  //   if (localDeg < -180)
  //     localDeg += 360;
  //   else if (localDeg > 180)
  //     localDeg -= 360;
  //   TurnMotor.set(ControlMode.Position,angleToTicks(getTurnDegrees()+localDeg));
  // }
  
  public void setTurnDegrees(int degrees) {
    if (degrees == -1) return;
    int currentTargetDeg = ticksToAngle((int)TurnMotor.getClosedLoopTarget());
    int currentTargetTic = (int)TurnMotor.getClosedLoopTarget();
    int localDeg = degrees - currentTargetDeg % 360;
    if (localDeg < -180)
      localDeg += 360;
    else if (localDeg > 180)
      localDeg -= 360;
    if (Math.abs(localDeg) > 90) {
        isForward = !isForward;
        setZero(currentTargetTic + TICKS_PER_REV/2*(isForward ? 1 : -1));
        TurnMotor.set(ControlMode.Position,currentTargetTic + TICKS_PER_REV/2*(isForward ? 1 : -1));
        setTurnDegrees(degrees);
        return;
    }
    TurnMotor.set(ControlMode.Position,angleToTicks(currentTargetDeg+localDeg));
  }

  public int ticksToAngle (int ticks) {
    double remainder = ticks;// % TICKS_PER_REV;
    remainder /= TICKS_PER_REV;
    return (int)(remainder * DEG_PER_REV);
  }

  public int angleToTicks (int angle) {
    double remainder = angle;// % DEG_PER_REV;
    remainder /= DEG_PER_REV;
    return (int)(remainder * TICKS_PER_REV);
  }

  public void setPowerPercent(double val) {
    double value = 
      (val > 1) ? 1 : 
      (val < 0) ? 0 : val;

    TurnMotor.configPeakOutputForward(value, 0);
    TurnMotor.configPeakOutputReverse(-value, 0);
  }

  private class SafeCANSparkMax extends CANSparkMax {
    private int safetyTimeout = 200;
    private long endgame = System.currentTimeMillis();
    private boolean running = true;

    public SafeCANSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
        run();
    }

    @Override
    public void set(double speed) {
      super.set(speed);
      endgame = System.currentTimeMillis() + safetyTimeout;
    }

    public void destroySafety() {
      running = false;
    }

    public void setSafetyTimeout(int millis) {
      safetyTimeout = millis;
    }
 
    private void run() {
      new Thread(() -> {
        while (running) {
          try {
            if (endgame < System.currentTimeMillis()) {
              super.set(0);
            }
            Thread.sleep(10);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }).start();
    }
  }

}