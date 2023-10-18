package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
//import com.team3478.frc2023.subsystems.StateManager.GlobalState;
//import com.team3478.lib.drivers.TalonUtil;
//import com.team3478.lib.geometry.Pose2d;
//import com.team3478.lib.geometry.Rotation2d;
//import com.team3478.lib.geometry.Translation2d;
//import com.team3478.lib.swerve.ChassisSpeeds;
//import com.team3478.lib.swerve.SwerveDriveKinematics;
//import com.team3478.lib.swerve.SwerveDriveOdometry;
//import com.team3478.lib.swerve.SwerveModuleState;
//import com.team3478.lib.util.MotorUtil;
import frc.robot.util.ReflectingCSVWriter;
import frc.robot.util.util1;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class temple_subsystem extends Subsystem{
    public static temple_subsystem mInstance; //instancia única

    public synchronized static temple_subsystem getInstance() {
        if( mInstance == null ){
            mInstance = new temple_subsystem();
        }
        return mInstance;
    }

    //HardWare
    /*public AHRS navx;
    private CANCoder frontRightabsoluteEncoder;
    private CANCoder frontLeftabsoluteEncoder;
    private CANCoder backRightabsoluteEncoder;
    private CANCoder backLeftabsoluteEncoder;

    private TalonFX mFrontRightSpeedMotor, mFrontLeftSpeedMotor, mBackRightSpeedMotor, mBackLeftSpeedMotor; 
    private TalonFX mFrontRightSteeringMotor, mFrontLeftSteeringMotor, mBackRightSteeringMotor, mBackLeftSteeringMotor;*/

    public static enum State{
        IDLE,
        Balancing, 
        AligningPriorityLeft,
        AligningPriorityRight,
        Driving,
        PathPlannerDriving
    }

    //private final MotorUtil mMotorUtil; //quitar
    public PIDController balancerPIDX,balancerPIDY,robotYawPID,robotYawPIDdinamic,robotYawPIDdinamic2, allignPID;  //Pids para hacer el control del autoajuste en la balanza
    private PIDController aligningDynamicPID;
    private ProfiledPIDController mFrontRightTurnigController, mFrontLeftTurnigController, mBackRightTurnigController, mBackLeftTurnigController;
    //private SwerveDriveKinematics mSwerveDriveKinematics;
    //private ChassisSpeeds speeds;
    //private SwerveModuleState[] moduleStates;
    //private SwerveDriveOdometry odometry;  
    private State mCurrentState = State.Driving;
    private State mWantedState = State.Driving;
    //private final MotorUtil mMotorUtilSpeed;
    //private Translation2d[] moduleLocations;
    private double[] lastError = {0,0,0,0};
    private int numberOfModules = 4; //número de módulos que se van a usar

    public int drivingDirection = 0; //stopped
    public int drivingDirectionSetPoint = 0;
    public double drivingTimer = 0; //time moving
    private double timerABS = 0;
    private double rawX,rawY,turn=0;
    private int tag = 0;
    private boolean needUpdate = false;
    private double tempYawAngle = 0;

    private PIDController yController,xController;
    private PIDController yController_2,xController_2;
    public double mWantedDistance_y=0,mWantedDistance_x=0;
    //private Pose2d desiredPose;
    public double pwm_y=0,pwm_x=0;
    private double errorDistance = 0.025;
    private double alignOffSet = 0;
    private double TOF_OffSet = 0;
    //private Pose2d gridPose = new Pose2d();
    //private Pose2d Pose = new Pose2d();
    private double alianceColor = -1;
    private double[] target = new double[]{0,0,0};
    private int isRunningAutoAlign=0;
    private double[] targetSetpoint = new double[]{0,0,0};


    public PeriodicIO mPeriodicIO; //para almacenar los inputs/outputs del subsistema
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null; //para tener un output
    private final boolean isLogging = true; //definir si el subsistema va a loggear datos
    private final boolean writeOutPuts = true; //definir si tiene salidas para SmartDashBoard

    //declarar las variables con los inputs/outputs default del subsistema
    public static class PeriodicIO {
        // INPUTS
        public double timestamp = 0;
        public double deltaTime = 0;
        public double yawAngle;
        public double yawAngleSetPoint=0;
        public double rollAngle;
        public double pitchAngle;
        public double absFrontRightPosition;
        public double absFrontLeftPosition;
        public double absBackRightPosition;
        public double absBackLeftPosition;
        public double leftBackSpeedDis;
        public double leftFrontSpeedDis;
        public double rightBackSpeedDis;
        public double rightFrontSpeedDis;
        public double X_DrivePosition = 0;
        public double Y_DrivePosition = 0;
        public boolean frontDetected = false;
        public boolean backDetected = false;
        public double yawAngle360 = 0;
        public double odometry_X_time = 0;
        public double odometry_Y_time = 0;
        

        // OUTPUTS
        public double final_front_left_steering_motor;
        public double final_front_left_speed_motor;
        public double final_front_right_steering_motor;
        public double final_front_right_speed_motor;
        public double final_back_left_steering_motor;
        public double final_back_left_speed_motor;
        public double final_back_right_steering_motor;
        public double final_back_right_speed_motor;

        //TEST
        public double sensitivity;
    }

    //constructor del subsystema para iniciar navX y motores
    private temple_subsystem() {

        yController = new PIDController(0.6,0,0);
        xController = new PIDController(0.6,0,0);
        yController_2 = new PIDController(2,0,0);
        xController_2 = new PIDController(2,0,0);

        mPeriodicIO = new PeriodicIO();
       // mMotorUtil = new MotorUtil(Constants.kGearSpeedReduction);
        //mMotorUtilSpeed = new MotorUtil(Constants.kGearSpeedReduction);

        //mFrontRightTurnigController = new ProfiledPIDController(Constants.kPSteeringValue, 0, 0, new TrapezoidProfile.Constraints((3*Math.PI), (8*Math.PI)));
        //mFrontLeftTurnigController = new ProfiledPIDController(Constants.kPSteeringValue, 0, 0, new TrapezoidProfile.Constraints((3*Math.PI), (8*Math.PI)));
        //mBackRightTurnigController = new ProfiledPIDController(Constants.kPSteeringValue, 0, 0, new TrapezoidProfile.Constraints((3*Math.PI), (8*Math.PI)));
        //mBackLeftTurnigController = new ProfiledPIDController(Constants.kPSteeringValue, 0, 0, new TrapezoidProfile.Constraints((3*Math.PI), (8*Math.PI)));

        mFrontRightTurnigController.enableContinuousInput(-Math.PI, Math.PI);
        mFrontLeftTurnigController.enableContinuousInput(-Math.PI, Math.PI);
        mBackRightTurnigController.enableContinuousInput(-Math.PI, Math.PI);
        mBackLeftTurnigController.enableContinuousInput(-Math.PI, Math.PI);

        //Init Modules Positions
        //moduleLocations = new Translation2d[numberOfModules];
        //moduleLocations[0] = new Translation2d(Constants.kXFrontLeftLocation, Constants.kYFrontLeftLocation);
        //moduleLocations[1] = new Translation2d(Constants.kXFrontRightLocation, Constants.kYFrontRightLocation);
        //moduleLocations[2] = new Translation2d(Constants.kXBackRightLocation, Constants.kYBackRightLocation);
        //moduleLocations[3] = new Translation2d(Constants.kXBackLeftLocation, Constants.kYBackLeftLocation);

        //Init Modules States
        //moduleStates = new SwerveModuleState[numberOfModules];
        for(int i = 0; i < numberOfModules; i++){
            //moduleStates[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        }

        //speeds = new ChassisSpeeds(0, 0, 0);

        //mSwerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);
        //odometry = new SwerveDriveOdometry(mSwerveDriveKinematics,new Pose2d()); //set initial pose in 0,0 (el autonomo sobrescribe la posicion inicial dependiendo el path que se ejecuta)

        //Init Swerve Kinematics Object
        //mSwerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);

        //Get NavX
        try{
            //navx = new AHRS(SPI.Port.kMXP);
            Timer.delay(0.5);
            //navx.reset();
            Timer.delay(0.5);
        }
        catch(Exception e){
            System.out.println("navx not working");
        }
        
        //Init CAN Coders
        //frontRightabsoluteEncoder = new CANCoder(Constants.kfrontRightCANCoderID,"CANivore1");
        //frontLeftabsoluteEncoder = new CANCoder(Constants.kfrontLeftCANCoderID,"CANivore1");
        //backRightabsoluteEncoder = new CANCoder(Constants.kbackRightCANCoderID,"CANivore1");
        //backLeftabsoluteEncoder = new CANCoder(Constants.kbackLeftCANCoderID,"CANivore1");

        //frontRightabsoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        //frontLeftabsoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        //backRightabsoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        //backLeftabsoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        //Inicializacion de los pids para balancear
        balancerPIDX = new PIDController(0.00935,0,0); //0.0095
        balancerPIDY = new PIDController(0.6,0,0);
        //Inicializamos el pid para mantener el robot en el angulo deseado(yaw)
        robotYawPID = new PIDController(0.0027,0,0); //0.0025
        robotYawPIDdinamic = new PIDController(0.0075,0,0); //0.007
        robotYawPIDdinamic2 = new PIDController(0.008,0,0); //0.008
        //Inicializamos el pid para alinear el robot contra los targets
        allignPID = new PIDController(0.01,0,0); //en unity la tolerancia esta en metros, en el real estara en pixeles(ajustar)
        aligningDynamicPID = new PIDController(0.045, 0,0);

        //Init Speed Motors
        //mFrontRightSpeedMotor = new TalonFX(Constants.kFrontRightSpeedMotorID,"CANivore1");
        //mFrontLeftSpeedMotor = new TalonFX(Constants.kFrontLeftSpeedMotorID,"CANivore1");
        //mBackRightSpeedMotor = new TalonFX(Constants.kBackRightSpeedMotorID,"CANivore1");
        //mBackLeftSpeedMotor = new TalonFX(Constants.kBackLeftSpeedMotorID,"CANivore1");
        //Init Steering Motors
        //mFrontRightSteeringMotor = new TalonFX(Constants.kFrontRightSteeringMotorID,"CANivore1");
        //mFrontLeftSteeringMotor = new TalonFX(Constants.kFrontLeftSteeringMotorID,"CANivore1");
        //mBackRightSteeringMotor = new TalonFX(Constants.kBackRightSteeringMotorID,"CANivore1");
        //mBackLeftSteeringMotor = new TalonFX(Constants.kBackLeftSteeringMotorID,"CANivore1");

        //TalonUtil.checkError(mFrontLeftSpeedMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true, Constants.kSpeedMotorLimitVoltge, Constants.kSpeedMotorLimitVoltge, 0.2
            ), 100
        ), "Could not set stator drive current limits");
        //TalonUtil.checkError(mFrontRightSpeedMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true, Constants.kSpeedMotorLimitVoltge, Constants.kSpeedMotorLimitVoltge, 0.2
            ), 100
        ), "Could not set stator drive current limits");
        //TalonUtil.checkError(mBackLeftSpeedMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true, Constants.kSpeedMotorLimitVoltge, Constants.kSpeedMotorLimitVoltge, 0.2
            ), 100
        ), "Could not set stator drive current limits");
        //TalonUtil.checkError(mBackRightSpeedMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true, Constants.kSpeedMotorLimitVoltge, Constants.kSpeedMotorLimitVoltge, 0.2
            ), 100
        ), "Could not set stator drive current limits");

        //TalonUtil.checkError(mFrontLeftSteeringMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true, Constants.kSteeringMotorLimitVoltge, Constants.kSteeringMotorLimitVoltge, 0.2
            ), 100
        ), "Could not set stator drive current limits");
        //TalonUtil.checkError(mFrontRightSteeringMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true, Constants.kSteeringMotorLimitVoltge, Constants.kSteeringMotorLimitVoltge, 0.2
            ), 100
        ), "Could not set stator drive current limits");
        //TalonUtil.checkError(mBackLeftSteeringMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true, Constants.kSteeringMotorLimitVoltge, Constants.kSteeringMotorLimitVoltge, 0.2
            ), 100
        ), "Could not set stator drive current limits");
        //TalonUtil.checkError(mBackRightSteeringMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                true, Constants.kSteeringMotorLimitVoltge, Constants.kSteeringMotorLimitVoltge, 0.2
            ), 100
        ), "Could not set stator drive current limits");
    }

    //metodo para leer los inputs (sensores, encoders..)
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.deltaTime = Timer.getFPGATimestamp() - mPeriodicIO.timestamp;
        
        mPeriodicIO.odometry_X_time = getOdometry().getPoseMeters().getTranslation().x();
        mPeriodicIO.odometry_Y_time = getOdometry().getPoseMeters().getTranslation().y();

        mPeriodicIO.yawAngle = navx.getAngle();
        mPeriodicIO.yawAngle360 = Limit360Angle(mPeriodicIO.yawAngle);
        mPeriodicIO.rollAngle = navx.getRoll();
        mPeriodicIO.pitchAngle = navx.getPitch();

        mPeriodicIO.absFrontRightPosition = (- frontRightabsoluteEncoder.getAbsolutePosition() + Constants.kFrontRightEncoderInitPos);
        mPeriodicIO.absFrontLeftPosition = (- frontLeftabsoluteEncoder.getAbsolutePosition() + Constants.kFrontLeftEncoderInitPos);
        mPeriodicIO.absBackRightPosition = (- backRightabsoluteEncoder.getAbsolutePosition() + Constants.kBackRightEncoderInitPos);
        mPeriodicIO.absBackLeftPosition = (- backLeftabsoluteEncoder.getAbsolutePosition() + Constants.kBackLeftEncoderInitPos);

        mPeriodicIO.absFrontRightPosition = Limit360Angle( mPeriodicIO.absFrontRightPosition );
        mPeriodicIO.absFrontLeftPosition = Limit360Angle( mPeriodicIO.absFrontLeftPosition );
        mPeriodicIO.absBackRightPosition = Limit360Angle( mPeriodicIO.absBackRightPosition );
        mPeriodicIO.absBackLeftPosition = Limit360Angle( mPeriodicIO.absBackLeftPosition );

        mPeriodicIO.rightBackSpeedDis = (((mBackRightSpeedMotor.getSelectedSensorVelocity()*10)/Constants.kPulsesEncoder)/8.14)*Math.PI*Constants.wheelRadius*2;
        mPeriodicIO.rightFrontSpeedDis = (((mFrontRightSpeedMotor.getSelectedSensorVelocity()*10)/Constants.kPulsesEncoder)/8.14)*Math.PI*Constants.wheelRadius*2;
        mPeriodicIO.leftBackSpeedDis = (((mBackLeftSpeedMotor.getSelectedSensorVelocity()*10)/Constants.kPulsesEncoder)/8.14)*Math.PI*Constants.wheelRadius*2;
        mPeriodicIO.leftFrontSpeedDis = (((mFrontLeftSpeedMotor.getSelectedSensorVelocity()*10)/Constants.kPulsesEncoder)/8.14)*Math.PI*Constants.wheelRadius*2;

        //System.out.println("Wheel speed RB: "+mPeriodicIO.rightBackSpeedDis);

        //Update odometry(usando los estados reales de las llantas)
        SwerveModuleState[] tempStates = new SwerveModuleState[Constants.kNumberOfModules];
        tempStates[0] = new SwerveModuleState(mPeriodicIO.leftFrontSpeedDis,Rotation2d.fromDegrees(mPeriodicIO.absFrontLeftPosition));
        tempStates[1] = new SwerveModuleState(mPeriodicIO.rightFrontSpeedDis,Rotation2d.fromDegrees(mPeriodicIO.absFrontRightPosition));
        tempStates[2] = new SwerveModuleState(mPeriodicIO.rightBackSpeedDis,Rotation2d.fromDegrees(mPeriodicIO.absBackRightPosition));
        tempStates[3] = new SwerveModuleState(mPeriodicIO.leftBackSpeedDis,Rotation2d.fromDegrees(mPeriodicIO.absBackLeftPosition));
        odometry.updateWithTime(mPeriodicIO.timestamp,Rotation2d.fromDegrees(Limit360Angle(mPeriodicIO.yawAngle)),tempStates);

        //alianceColor = AutoModeSelector.color;
        /*mPeriodicIO.frontDetected = (mPeriodicIO.yawAngle360 > 290 || mPeriodicIO.yawAngle360<70) && NetWork.getInstance().mPeriodicIO.targetDetected1;
        mPeriodicIO.backDetected = (mPeriodicIO.yawAngle360 > 110 && mPeriodicIO.yawAngle360<250) && NetWork.getInstance().mPeriodicIO.targetDetected2;
        
        if(mPeriodicIO.backDetected || mPeriodicIO.frontDetected){
            needUpdate=false;
            tag = mPeriodicIO.backDetected ? Math.round(NetWork.getInstance().mPeriodicIO.tagId_2) : Math.round(NetWork.getInstance().mPeriodicIO.tagId_1);
            switch (tag){ //TODO: checar signo de alianza roja (negativo?)
                case 1:
                    if(alianceColor==1){
                        needUpdate=true;
                        target = new double[]{1.02, 1.07,0}; // X: 15.51, y: 1.07 //revisar signo
                    }
                    break;
                case 2:
                    if(alianceColor==1){
                        needUpdate=true;
                        target = new double[]{1.02, 2.74,0}; // X: 15.51, y: 2.748
                    }
                    break;
                case 3:
                    if(alianceColor==1){
                        needUpdate=true;
                        target = new double[]{1.02, 4.42,0}; // X: 15.51, y: 4.42
                    }
                    break;
                case 6:
                    if(alianceColor==0){
                        needUpdate=true;
                        target = new double[]{1.02,-4.42,0}; //correcto X: 1.027, y: 4.42
                    }
                    break;
                case 7:
                    if(alianceColor==0){
                        needUpdate=true;
                        target = new double[]{1.02,-2.74,0}; //mas correcto X: 1.027, Y: 2.748
                    }
                    break;
                case 8:
                    if(alianceColor==0){
                        needUpdate=true;
                        target = new double[]{1.02,-1.07,0}; //Correcto X: 1.027, y: 1.07
                    }
                    break;
                default:
                    break;
            }
            if(needUpdate){
                if(mPeriodicIO.backDetected){
                    Pose = new Pose2d(target[0]+NetWork.getInstance().mPeriodicIO.pose3d_2[2],target[1]+NetWork.getInstance().mPeriodicIO.pose3d_2[0],Rotation2d.fromDegrees(mPeriodicIO.yawAngle));
                    getOdometry().resetPosition(Pose);
                }else{
                    Pose = new Pose2d(target[0]+NetWork.getInstance().mPeriodicIO.pose3d_1[2],target[1]+NetWork.getInstance().mPeriodicIO.pose3d_1[0],Rotation2d.fromDegrees(mPeriodicIO.yawAngle));
                    getOdometry().resetPosition(Pose);
                }
            }
        }*/
    }

    //metodo para setear los outputs (velocidad motores...)
    public void writePeriodicOutputs() {

        mFrontRightSpeedMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_front_right_speed_motor);
        mFrontRightSteeringMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_front_right_steering_motor);

        mFrontLeftSpeedMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_front_left_speed_motor);
        mFrontLeftSteeringMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_front_left_steering_motor);

        mBackRightSpeedMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_back_right_speed_motor);
        mBackRightSteeringMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_back_right_steering_motor);

        mBackLeftSpeedMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_back_left_speed_motor);
        mBackLeftSteeringMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_back_left_steering_motor);
    }

    //metodo para registrar los loops del subsistema, las acciones al inicio, durante y final del teleop
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (SwerveDriveSDS.this) {
                    if( isLogging ) startLogging();
                    SetBrakeMode(true);
                    SetSwerveState(State.Driving);
                    stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                if(isLogging && mCSVWriter != null) mCSVWriter.add(mPeriodicIO);
                LoopRobotStates();
                Autobalance();
                //setGridLocation();
                //gridMainDrive(pwm_x, pwm_y, 0);
                swerveMainDrive();
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                if( isLogging ) stopLogging();
            }
        });
    }

    public void zeroSensors() {
        //reset sensors
    }

    //Iniciar a declarar funciones del subsistema------------------------------------------------------------    

    // Funcion para manejar con input de sticks
    public void swerveMainDrive(double _rawX, double _rawY, double _turn){
        rawX=_rawX*-1;
        rawY=_rawY*-1;
        turn=_turn*-1;
        SetBrakeMode(true);
    }

    public void swerveMainDrive(){

        double deltaDrivingTime = Timer.getFPGATimestamp() - timerABS;
        timerABS = Timer.getFPGATimestamp();

        if(mCurrentState!=State.Driving) return;

        double deltaAngle = Math.abs(mPeriodicIO.yawAngle - mPeriodicIO.yawAngleSetPoint);

        turn = Util.handleDeadband(turn,0.3);
        if(StateManager.getInstance().GetCurrentState()!=GlobalState.AUTO){
            if(Math.abs(turn)<1e-6){
                if(deltaAngle > 10){
                    turn = robotYawPIDdinamic.calculate(mPeriodicIO.yawAngle,mPeriodicIO.yawAngleSetPoint)*-1;
                }else{
                    turn = robotYawPID.calculate(mPeriodicIO.yawAngle,mPeriodicIO.yawAngleSetPoint)*-1;
                }
            }else{
                mPeriodicIO.yawAngleSetPoint = mPeriodicIO.yawAngle;
            }
        }
        else{
            if(Math.abs(turn)<1e-6){
                turn = robotYawPIDdinamic2.calculate(mPeriodicIO.yawAngle, mPeriodicIO.yawAngleSetPoint)*-1;
            }else{
                mPeriodicIO.yawAngleSetPoint = mPeriodicIO.yawAngle;
            }
        }
        
        //Se convierten los valores de los sticks a velocidad en m/s
        //Inversión de ejes
        double speedX = StickToVelocity(rawY);
        double speedY = StickToVelocity(rawX);
        //Convertir el valor de stick a velocidad angularF
        double turnangle = StickToAngularVelocity(turn);
        //Se dan vectores de referencia en la cancha
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, turnangle, Rotation2d.fromDegrees((mPeriodicIO.yawAngle)));

        double angleDriving = (Math.atan2(speedY, speedX) * 180)/Math.PI;
        double movingDrive = Math.hypot(rawY, rawX);
        double yawRotation = Limit360Angle(mPeriodicIO.yawAngle);
        double angleDrivingLimit = Limit360Angle(angleDriving);
        yawRotation = Limit360Angle(-yawRotation+angleDrivingLimit);
        angleDriving = yawRotation > 180 ? yawRotation-360 : yawRotation;

        if(StateManager.getInstance().GetCurrentState()!=GlobalState.AUTO){
            if(movingDrive < 0.7){
                if(drivingDirectionSetPoint != 0){ drivingTimer = 0;}
                    drivingDirectionSetPoint = 0;
                    drivingTimer += deltaDrivingTime;
                if(drivingTimer>=0.5){
                    drivingDirection = 0; //avanza hacia delante
                }
            }
            else{
                if((angleDriving <= 65 && angleDriving >= 0) || (angleDriving >= -65 && angleDriving <= 0)){
                    if(drivingDirectionSetPoint != 1){ drivingTimer = 0;}
                    drivingDirectionSetPoint = 1;
                    drivingTimer += deltaDrivingTime;
                    if(drivingTimer>=0.5){
                        drivingDirection = 1; //avanza hacia delante
                    }
                }else if((angleDriving <= 180 && angleDriving >= 115) || (angleDriving >= -180 && angleDriving <= -115)){
                    if(drivingDirectionSetPoint != -1){ drivingTimer = 0;}
                    drivingDirectionSetPoint = -1;
                    drivingTimer += deltaDrivingTime;
                    if(drivingTimer>=0.5){
                        drivingDirection = -1; //avanza hacia atrás
                    }
                }
            }
        }
        
        //Con Inverse Kinematics se convierten las velocidades a los estados de los modulos
        moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(speeds);
        //Normalizar las velocidades para que no superen el máximo de las velocidades
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, mMotorUtilSpeed.GetMaxVelocity(Constants.wheelRadius));
        //Optimizar los valores de cada angulo en los modulos considerando el cambio de direccion en velocidad parar reducir el giro
        moduleStates[0] = SwerveModuleState.optimize(moduleStates[0], Rotation2d.fromDegrees(mPeriodicIO.absFrontLeftPosition));
        moduleStates[1] = SwerveModuleState.optimize(moduleStates[1], Rotation2d.fromDegrees(mPeriodicIO.absFrontRightPosition));
        moduleStates[2] = SwerveModuleState.optimize(moduleStates[2], Rotation2d.fromDegrees(mPeriodicIO.absBackRightPosition));
        moduleStates[3] = SwerveModuleState.optimize(moduleStates[3], Rotation2d.fromDegrees(mPeriodicIO.absBackLeftPosition));
        //Actualizar el estado de las salidas
        UpdateDriveSpeed();
    }

    public void gridMainDrive(double _rawX, double _rawY, double _turn){

        if(mCurrentState!=State.AligningPriorityRight && mCurrentState!=State.AligningPriorityLeft) return;

        //Se convierten los valores de los sticks a velocidad en m/s
        //Inversión de ejes
        double speedX = StickToVelocity(_rawX*-1);
        double speedY = StickToVelocity(_rawY*-1);
        double turn = 0;
        turn = aligningDynamicPID.calculate(mPeriodicIO.yawAngle,mPeriodicIO.yawAngleSetPoint)*-1;
        double turnangle = StickToAngularVelocity(turn);
        //Se dan vectores de referencia en la cancha
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, turnangle, Rotation2d.fromDegrees((mPeriodicIO.yawAngle)));
        //Con Inverse Kinematics se convierten las velocidades a los estados de los modulos
        moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(speeds);
        //Normalizar las velocidades para que no superen el máximo de las velocidades
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, mMotorUtilSpeed.GetMaxVelocity(Constants.wheelRadius));
        //Optimizar los valores de cada angulo en los modulos considerando el cambio de direccion en velocidad parar reducir el giro
        moduleStates[0] = SwerveModuleState.optimize(moduleStates[0], Rotation2d.fromDegrees(mPeriodicIO.absFrontLeftPosition));
        moduleStates[1] = SwerveModuleState.optimize(moduleStates[1], Rotation2d.fromDegrees(mPeriodicIO.absFrontRightPosition));
        moduleStates[2] = SwerveModuleState.optimize(moduleStates[2], Rotation2d.fromDegrees(mPeriodicIO.absBackRightPosition));
        moduleStates[3] = SwerveModuleState.optimize(moduleStates[3], Rotation2d.fromDegrees(mPeriodicIO.absBackLeftPosition));
        //Actualizar el estado de las salidas
        UpdateDriveSpeed();
    }

    public boolean isInRange(){
        return getOdometry().getPoseMeters().distance(gridPose) <= 0.2 && 
               getOdometry().getPoseMeters().distance(gridPose) >= -0.2;
    }

    public void setGridLocation(){

        if(mCurrentState!=State.AligningPriorityRight && mCurrentState!=State.AligningPriorityLeft) return;
        tempYawAngle = mPeriodicIO.yawAngle;
        if(isRunningAutoAlign==0){
            if(mPeriodicIO.yawAngle360 > 290 || mPeriodicIO.yawAngle360<70){
                int turns = (int)(mPeriodicIO.yawAngle/360);
                double offsetturn = (mPeriodicIO.yawAngle%360)/(double)360;
                if(offsetturn>0.75){
                    mPeriodicIO.yawAngleSetPoint=(turns+1)*360;
                }else if(offsetturn<-0.75){
                    mPeriodicIO.yawAngleSetPoint=(turns-1)*360;
                }else{
                    mPeriodicIO.yawAngleSetPoint=turns*360;
                }
            }
            if(mPeriodicIO.yawAngle360 > 110 && mPeriodicIO.yawAngle360<250){
                if(mPeriodicIO.yawAngle<0) tempYawAngle-=180; //TODO: usar el setpoint?
                if(mPeriodicIO.yawAngle>=0) tempYawAngle+=180;
                int turns = (int)(tempYawAngle/360);
                double offsetturn = (tempYawAngle%360)/(double)360;
                if(offsetturn>0.75){
                    mPeriodicIO.yawAngleSetPoint=(turns+1)*360;
                }else if(offsetturn<-0.75){
                    mPeriodicIO.yawAngleSetPoint=(turns-1)*360;
                }else{
                    mPeriodicIO.yawAngleSetPoint=turns*360;
                }
                if(mPeriodicIO.yawAngleSetPoint<0) mPeriodicIO.yawAngleSetPoint+=180;
                if(mPeriodicIO.yawAngleSetPoint>=0) mPeriodicIO.yawAngleSetPoint-=180;
            }

            if(getOdometry().getPoseMeters().getTranslation().y()>=-1.91){  //region 1
                if(mCurrentState==State.AligningPriorityLeft){
                    if(alianceColor==0){
                        targetSetpoint = new double[]{1.86,-0.65,0};//TODO: calibrar
                    }else{
                        targetSetpoint = new double[]{1.86,0.61,0};//TODO: calibrar
                    }
                }else{
                    if(alianceColor==0){
                        targetSetpoint = new double[]{1.86,-1.53,0};//TODO: calibrar
                    }else{
                        targetSetpoint = new double[]{1.86,1.53,0};//TODO: calibrar
                    }
                }
            }else if(getOdometry().getPoseMeters().getTranslation().y()<-1.91 && getOdometry().getPoseMeters().getTranslation().y()>=(-1.91-1.68)){  //region 2
                if(mCurrentState==State.AligningPriorityLeft){
                    if(alianceColor==0){
                        targetSetpoint = new double[]{1.8,-2.28,0};//2.28
                    }else{
                        targetSetpoint = new double[]{1.8,2.28,0};//REVISAR en ROJO
                    }
                }else{
                    if(alianceColor==0){
                        targetSetpoint = new double[]{1.8,-3.25,0}; //3.2
                    }else{ 
                        targetSetpoint = new double[]{1.8,3.22,0};//REVISAR en ROJO
                    }
                }
            }else if(getOdometry().getPoseMeters().getTranslation().y()<(-1.91-1.68)){  //region 3
                if(mCurrentState==State.AligningPriorityLeft){
                    if(alianceColor==0){
                        targetSetpoint = new double[]{1.8,-3.7,0}; // 3.96
                    }else{
                        targetSetpoint = new double[]{1.8,3.76,0};//REVISAR en ROJO
                    }
                }else{
                    if(alianceColor==0){
                        targetSetpoint = new double[]{1.8,-4.88,0}; //4.98
                    }else{
                        targetSetpoint = new double[]{1.8, 4.88,0};//REVISAR en ROJO
                    }
                }
            }
        }
        isRunningAutoAlign++;
        //Intake tof offset to target
        TOF_OffSet = Intake.getInstance().mPeriodicIO.sensorDistanceFiltered;     
        TOF_OffSet = TOF_OffSet/1000;
        if(TOF_OffSet <= 0.1){
            TOF_OffSet = -0.10;
        }else if(TOF_OffSet > 0.1 && TOF_OffSet <= 0.17){
            TOF_OffSet = 0.0;
        }else TOF_OffSet = 0.12;
        
        gridPose = new Pose2d(targetSetpoint[0],targetSetpoint[1]+TOF_OffSet,Rotation2d.fromDegrees(mPeriodicIO.yawAngle));
        if(!isInRange()){
            pwm_y = yController.calculate(getOdometry().getPoseMeters().getTranslation().y(),gridPose.getTranslation().y());
            pwm_x = xController.calculate(getOdometry().getPoseMeters().getTranslation().x(),gridPose.getTranslation().x());
        }else{
            pwm_y = yController_2.calculate(getOdometry().getPoseMeters().getTranslation().y(),gridPose.getTranslation().y());
            pwm_x = xController_2.calculate(getOdometry().getPoseMeters().getTranslation().x(),gridPose.getTranslation().x());
        }
        if(pwm_y>0.8) pwm_y=0.8; 
        if(pwm_x>0.8) pwm_x=0.8; 
        if(pwm_y<-0.8) pwm_y=-0.8;
        if(pwm_x<-0.8) pwm_x=-0.8;

        if(getOdometry().getPoseMeters().distance(gridPose) <= errorDistance && getOdometry().getPoseMeters().distance(gridPose) >= -errorDistance){
            pwm_y = 0;
            pwm_x = 0;
            LedsController.getInstance().SetState(LedsController.State.Balance);
        }
    }

    //Funcion para leer el objeto qu tien la odometria
    public SwerveDriveOdometry getOdometry(){
        return odometry;
    }

    public void LoopRobotStates(){
        if( mWantedState != mCurrentState ){
            switch( mCurrentState ){
            case Balancing:
                //nada
                break;
            case AligningPriorityLeft:
                //nada
                break;
            case AligningPriorityRight:
                //nada
                break;
            case Driving:
                //nada
                break;
            case PathPlannerDriving:
                //nada
                break;
            }
            mCurrentState = mWantedState;
            switch( mCurrentState ){
            case Balancing:
                //nada
                break;
            case AligningPriorityLeft:
                isRunningAutoAlign=0;
                break;
            case AligningPriorityRight:
                isRunningAutoAlign=0;
                break;
            case Driving:
                //nada
                break;
            case PathPlannerDriving:
                //nada
                break;
            }
        }
    }

    private double StickToVelocity(double stickInput){
        return stickInput*mMotorUtil.GetMaxVelocity(Constants.wheelRadius);
    }

    private double StickToAngularVelocity(double stickInput){ 
        return stickInput*( mMotorUtil.GetMaxVelocity(Constants.wheelRadius) / (double)Constants.wheelTrack );
    }

    //Funcion para obtener el delta mas corto entre dos angulos
    private double DeltaAngle(double _target, double _actual){
        double deltadegrees = (_target-_actual);
        deltadegrees = ((deltadegrees+180) - (Math.floor((deltadegrees+180)/360f)*360)) - 180;
        return deltadegrees;
    }  

    //Funcion que limita el angulo de 0 a 360
    public double Limit360Angle(double _angle){
        _angle = _angle%360;
        if (_angle< 0) {
            _angle += 360;
        }
        return _angle;
    }

    //Funcion para actualizar el estado de salidas
    private void UpdateDriveSpeed(){
        double deltadegrees = 0;
        if(Math.abs(moduleStates[0].speedMetersPerSecond) < 0.1){
            mPeriodicIO.final_front_left_steering_motor = 0;
            mPeriodicIO.final_front_left_speed_motor = 0;
        }else{
            deltadegrees = DeltaAngle((double)Limit360Angle(moduleStates[0].angle.getDegrees()),mPeriodicIO.absFrontLeftPosition);
            mPeriodicIO.final_front_left_steering_motor = deltadegrees * Constants.kPSteeringValue + (deltadegrees-lastError[0])/mPeriodicIO.deltaTime * Constants.kDSteeringValue;
            lastError[0] = deltadegrees;
            mPeriodicIO.final_front_left_speed_motor = mMotorUtilSpeed.GetPercentageFromVelocity(moduleStates[0].speedMetersPerSecond,Constants.wheelRadius);
        }

        if(Math.abs(moduleStates[1].speedMetersPerSecond) < 0.1){
            mPeriodicIO.final_front_right_steering_motor = 0;
            mPeriodicIO.final_front_right_speed_motor = 0;
        }else{
            deltadegrees = DeltaAngle((double)Limit360Angle(moduleStates[1].angle.getDegrees()),mPeriodicIO.absFrontRightPosition);
            mPeriodicIO.final_front_right_steering_motor = deltadegrees * Constants.kPSteeringValue + (deltadegrees-lastError[1])/mPeriodicIO.deltaTime * Constants.kDSteeringValue;    
            lastError[1] = deltadegrees;
            mPeriodicIO.final_front_right_speed_motor = mMotorUtilSpeed.GetPercentageFromVelocity(moduleStates[1].speedMetersPerSecond,Constants.wheelRadius);
        }
        
        if(Math.abs(moduleStates[2].speedMetersPerSecond) < 0.1){
            mPeriodicIO.final_back_right_steering_motor = 0;
            mPeriodicIO.final_back_right_speed_motor = 0;
        }else{
            deltadegrees = DeltaAngle((double)Limit360Angle(moduleStates[2].angle.getDegrees()),mPeriodicIO.absBackRightPosition);
            mPeriodicIO.final_back_right_steering_motor = deltadegrees * Constants.kPSteeringValue + (deltadegrees-lastError[2])/mPeriodicIO.deltaTime * Constants.kDSteeringValue;    
            lastError[2] = deltadegrees;
            mPeriodicIO.final_back_right_speed_motor = mMotorUtilSpeed.GetPercentageFromVelocity(moduleStates[2].speedMetersPerSecond,Constants.wheelRadius);
        }

        if(Math.abs(moduleStates[3].speedMetersPerSecond) < 0.1){
            mPeriodicIO.final_back_left_steering_motor = 0;
            mPeriodicIO.final_back_left_speed_motor = 0;
        }else{
            deltadegrees = DeltaAngle((double)Limit360Angle(moduleStates[3].angle.getDegrees()),mPeriodicIO.absBackLeftPosition);
            mPeriodicIO.final_back_left_steering_motor = deltadegrees * Constants.kPSteeringValue + (deltadegrees-lastError[3])/mPeriodicIO.deltaTime * Constants.kDSteeringValue;    
            lastError[3] = deltadegrees;
            mPeriodicIO.final_back_left_speed_motor = mMotorUtilSpeed.GetPercentageFromVelocity(moduleStates[3].speedMetersPerSecond,Constants.wheelRadius);
        }
    }

    public void Autobalance(){
        if(mCurrentState!=State.Balancing) return;
        double valueX = balancerPIDX.calculate(mPeriodicIO.rollAngle, 0); //Gira sobre X
        valueX=Util.limit(valueX,-0.5, 0.5);
        swerveMainDrive(0, valueX, 0);
    }

    public void SetSwerveState(State swerveState){
        if( mWantedState == swerveState) return;
        mWantedState = swerveState;
    }

    public State GetSwerveState(){
        return mCurrentState;
    }

    public void SetBrakeMode(boolean enabled){
        NeutralMode mode = enabled? NeutralMode.Brake : NeutralMode.Coast;
        mFrontLeftSpeedMotor.setNeutralMode(mode);
        mFrontRightSpeedMotor.setNeutralMode(mode);
        mBackLeftSpeedMotor.setNeutralMode(mode);
        mBackRightSpeedMotor.setNeutralMode(mode);

        mFrontLeftSteeringMotor.setNeutralMode(mode);
        mFrontRightSteeringMotor.setNeutralMode(mode);
        mBackLeftSteeringMotor.setNeutralMode(mode);
        mBackRightSteeringMotor.setNeutralMode(mode);
    }
    
    public void SetInitZero(){
        if(Math.abs(mPeriodicIO.yawAngle)>1.5){LedsController.getInstance().SetState(LedsController.State.Auto);}
        else{LedsController.getInstance().SetState(LedsController.State.Disable);}
    }

    //-------------------------------------------------------------------------------------------
    
    // Funcion para inicializar el objeto del logging
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/SwerveDrive-LOGS.csv", PeriodicIO.class);
        }
    }

    // Funcion para finalizar el objeto del logging
    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public void stop(){
        rawX=0;
        rawY=0;
        turn=0;

        mPeriodicIO.final_front_right_speed_motor=0;
        mPeriodicIO.final_front_right_steering_motor=0;
        mPeriodicIO.final_front_left_speed_motor=0;
        mPeriodicIO.final_front_left_steering_motor=0;
        mPeriodicIO.final_back_right_speed_motor=0;
        mPeriodicIO.final_back_right_steering_motor=0;
        mPeriodicIO.final_back_left_speed_motor=0;
        mPeriodicIO.final_back_left_steering_motor=0;

        mFrontRightSpeedMotor.set(ControlMode.PercentOutput, 0);
        mFrontRightSteeringMotor.set(ControlMode.PercentOutput, 0);

        mFrontLeftSpeedMotor.set(ControlMode.PercentOutput, 0);
        mFrontLeftSteeringMotor.set(ControlMode.PercentOutput, 0);

        mBackRightSpeedMotor.set(ControlMode.PercentOutput, 0);
        mBackRightSteeringMotor.set(ControlMode.PercentOutput, 0);

        mBackLeftSpeedMotor.set(ControlMode.PercentOutput, 0);
        mBackLeftSteeringMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean checkSystem(){
        return true;
    }

    public void outputTelemetry(){
        if ( mCSVWriter != null ) {
            mCSVWriter.write();
        }
        if(writeOutPuts){
            SmartDashboard.putNumber("FR demand", mPeriodicIO.final_front_right_speed_motor);
            SmartDashboard.putNumber("FL demand", mPeriodicIO.final_front_left_speed_motor);
            SmartDashboard.putNumber("BR demand", mPeriodicIO.final_back_right_speed_motor);
            SmartDashboard.putNumber("BL demand", mPeriodicIO.final_back_left_speed_motor);

            SmartDashboard.putNumber("FR angle demand", mPeriodicIO.final_front_right_steering_motor);
            SmartDashboard.putNumber("FL angle demand", mPeriodicIO.final_front_left_steering_motor);
            SmartDashboard.putNumber("BR angle demand", mPeriodicIO.final_back_right_steering_motor);
            SmartDashboard.putNumber("BL angle demand", mPeriodicIO.final_back_left_steering_motor);

            SmartDashboard.putNumber("FR position", mPeriodicIO.absFrontRightPosition);
            SmartDashboard.putNumber("FL position", mPeriodicIO.absFrontLeftPosition);
            SmartDashboard.putNumber("BR position", mPeriodicIO.absBackRightPosition);
            SmartDashboard.putNumber("BL position", mPeriodicIO.absBackLeftPosition);

            SmartDashboard.putNumber("Yaw angle", mPeriodicIO.yawAngle);
            SmartDashboard.putNumber("Yaw angle setpoit", mPeriodicIO.yawAngleSetPoint);
            SmartDashboard.putNumber("Pitch angle", mPeriodicIO.pitchAngle);
            SmartDashboard.putNumber("Roll angle", mPeriodicIO.rollAngle);

            SmartDashboard.putNumber("Module 1 desired", moduleStates[0].angle.getDegrees());
            SmartDashboard.putNumber("Module 2 desired", moduleStates[1].angle.getDegrees());
            SmartDashboard.putNumber("Module 3 desired", moduleStates[2].angle.getDegrees());
            SmartDashboard.putNumber("Module 4 desired", moduleStates[3].angle.getDegrees());
        
            SmartDashboard.putNumber("Odometry_X", getOdometry().getPoseMeters().getTranslation().x());
            SmartDashboard.putNumber("Odometry_Y", getOdometry().getPoseMeters().getTranslation().y());
            SmartDashboard.putNumber("Odometry_Rotation", getOdometry().getPoseMeters().getRotation().getDegrees());

            SmartDashboard.putNumber("pwm_Y", pwm_y);
        }
    }
}