package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

class SwerveModule2903 {
    private CANSparkMax ForwardMotor;
    private WPI_TalonSRX TurnMotor;
    final int TICKS_PER_REV = 4096*6;
    final int DEG_PER_REV = 360;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

  public SwerveModule2903(int forwardMotorId, int turnMotorId) {
    ForwardMotor = new SafeCANSparkMax(forwardMotorId, MotorType.kBrushless);
    TurnMotor = new WPI_TalonSRX(turnMotorId);

    TurnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
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
    ForwardMotor.set(speed);
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

  public void setTurnDegrees(int degrees) {
    int localDeg = degrees - getTurnDegrees() % 360;
    if (localDeg < -180)
    localDeg += 360;
    else if (localDeg > 180)
    localDeg -= 360;
    TurnMotor.set(ControlMode.Position,angleToTicks(getTurnDegrees()+localDeg));
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