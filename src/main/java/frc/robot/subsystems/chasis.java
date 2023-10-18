package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
private final boolean writeOutPuts = false;

public class chasis extends SubsystemBase {
  public chasis() {} 
   
  public CommandBase exampleMethodCommand() {
    
    return runOnce(
        () -> {
          
        });
  }

  public boolean exampleCondition() {
    
    return false;
  }

  public void writePeriodicOutputs() {
    //SWERVE
    mFrontRightSpeedMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_front_right_speed_motor);
    mFrontLeftSpeedMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_front_left_speed_motor);
    mBackRightSpeedMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_back_right_speed_motor);
    mBackLeftSpeedMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_back_left_speed_motor);

  }
//INTAKE
public void writePeriodicOutputs() {
    if(mPeriodicIO.angle > Constants.kIntakeTurningReverseLimit+10){
        mPeriodicIO.turning_motor = -0.1;
    }else if(mPeriodicIO.angle < Constants.kIntakeTurningForwardLimit-10){
        mPeriodicIO.turning_motor = 0.1;
    }
    
    if((Math.abs(turningEncoder.getStatusFramePeriod(CANCoderStatusFrame.SensorData)) < Util.kEpsilon && Math.abs(mPeriodicIO.turning_motor) > 0.05)){
        DeltaAngletimer += mPeriodicIO.deltaTime;
    }else DeltaAngletimer = 0;

    if(DeltaAngletimer>1){
        mPeriodicIO.turning_motor = 0;
    }

    //Send output to arm motors
    turningMotor.set(mPeriodicIO.turning_motor*1);
    turningMotor2.set(mPeriodicIO.turning_motor*-1);

    if(mPeriodicIO.cubeSwitcher){
        intakeLMotor.set(-0.04);
        intakeRMotor.set(0.04);
    }else {
        intakeLMotor.set(ControlType.kVelocity, -mPeriodicIO.velocity_setpoint);
        intakeRMotor.set(ControlType.kVelocity, mPeriodicIO.velocity_setpoint);
    }

    //Change leds states
    if(mPeriodicIO.pieceInRobot<0){
        LedsController.getInstance().SetState(LedsController.State.NoPieceInRobot);
    }else if(mPeriodicIO.pieceInRobot==0 || mPeriodicIO.pieceInRobot==1){
        LedsController.getInstance().SetState(LedsController.State.TakeConeP1);
    }else if(mPeriodicIO.pieceInRobot==2 || mPeriodicIO.pieceInRobot==3){
        LedsController.getInstance().SetState(LedsController.State.TakeConeP2);
    }else if(mPeriodicIO.pieceInRobot==4 || mPeriodicIO.pieceInRobot==5){
        LedsController.getInstance().SetState(LedsController.State.TakeConeP3);
    }else if(mPeriodicIO.pieceInRobot==6 || mPeriodicIO.pieceInRobot==7){
        LedsController.getInstance().SetState(LedsController.State.TakeCube);
    }
}
public void outputTelemetry(){
    if ( mCSVWriter != null ) {
        mCSVWriter.write();
    }
    if(writeOutPuts){
        SmartDashboard.putNumber("IntakeAngle", mPeriodicIO.angle);
        SmartDashboard.putNumber("IntakeAnglePWM", mPeriodicIO.turning_motor);
        SmartDashboard.putNumber("IntakeAngleSetPoint", mPeriodicIO.angleSetPoint);
        SmartDashboard.putNumber("kind Of Piece", mPeriodicIO.pieceInRobot);
        SmartDashboard.putNumber("Distance", mPeriodicIO.sensorDistanceFiltered);
        SmartDashboard.putNumber("Intake Speed", mPeriodicIO.mIntakeSpeed);
        SmartDashboard.putNumber("Intake State", takingLimitSpeed);
        SmartDashboard.putBoolean("NEO state", mPeriodicIO.cubeSwitcher);
       SmartDashboard.putNumber("potencia", potencia);
    }
    SmartDashboard.putNumber("kind Of Piece", mPeriodicIO.pieceInRobot);
    SmartDashboard.putString("PIEZA", driverPiece);
    SmartDashboard.putBoolean("ConeP2", hasConeP2);
    SmartDashboard.putBoolean("CubeP1", hasCubeP1);
    SmartDashboard.putBoolean("ConeP1", hasConeP1);
    SmartDashboard.putBoolean("INTAKE", !mPeriodicIO.cubeSwitcher);
}
  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
public synchronized void startLogging() {
    if (mCSVWriter == null) {
        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/Intake-LOGS.csv",PeriodicIO.class);
    }
}

/// Funcion para finalizar el objeto del logging /// 
public synchronized void stopLogging() {
    if (mCSVWriter != null) {
        mCSVWriter.flush();
        mCSVWriter = null;
    }
}
}
