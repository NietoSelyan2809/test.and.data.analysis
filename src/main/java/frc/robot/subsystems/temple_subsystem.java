package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
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
    CANSparkMax motor1;
    CANSparkMax motor2;
    CANSparkMax motor3;
    RelativeEncoder encoderm1;
    RelativeEncoder encoderm2;
    RelativeEncoder encoderm3;
    MotorType tipo;
    

    public static enum State{
        IDLE,
        Balancing, 
        AligningPriorityLeft,
        AligningPriorityRight,
        Driving,
        PathPlannerDriving
    }
    // VARIABLES SUBSISTEMAS
    double getBusVoltage = 0;
    double getMotorTemperature = 0;
    double getVelocity = 0;

    //VARIABLES DEL WRITER
    public PeriodicIO mPeriodicIO; //para almacenar los inputs/outputs del subsistema
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null; //para tener un output
    private final boolean isLogging = true; //definir si el subsistema va a loggear datos
    private final boolean writeOutPuts = true; //definir si tiene salidas para SmartDashBoard

    //declarar las variables con los inputs/outputs default del subsistema
    public static class PeriodicIO {
        double timestamp;
        double deltaTime;

        // INPUTS
        double posenconder1;
        double posenconder2;
        double posenconder3;

        double getTemperature1;
        double getTemperature2;
        double getTemperature3;

        double getBusVoltage1;
        double getBusVoltage2;
        double getBusVoltage3;

        double getVelocity1;
        double getVelocity2;
        double getVelocity3;
        

        // OUTPUTS
       

        //TEST
    
    }

    //constructor del subsystema para iniciar navX y motores
    private temple_subsystem() {
        // PERIODIC IO
        mPeriodicIO = new PeriodicIO();

        //PONER VARIABLES
        
        //PONER HARDWARE
        motor1 = new CANSparkMax(1, MotorType.kBrushless);
        motor2 = new CANSparkMax(2, MotorType.kBrushless);
        motor3 = new CANSparkMax(3, MotorType.kBrushless);

        encoderm1 = motor1.getEncoder();
        encoderm2 = motor2.getEncoder();
        encoderm3 = motor3.getEncoder();

        //Get Sensors
        
        //CONFIGURACIONES

    }

    //metodo para leer los inputs (sensores, encoders..)
    public void readPeriodicInputs() {

        mPeriodicIO.getTemperature1 = motor1.getMotorTemperature();       
        mPeriodicIO.getTemperature2 = motor2.getMotorTemperature();
        mPeriodicIO.getTemperature3 = motor3.getMotorTemperature();

        mPeriodicIO.posenconder1 = encoderm1.getPosition();
        mPeriodicIO.posenconder1 = encoderm2.getPosition();
        mPeriodicIO.posenconder1 = encoderm3.getPosition();

        mPeriodicIO.getBusVoltage1 = motor1.getBusVoltage();
        mPeriodicIO.getBusVoltage1 = motor2.getBusVoltage();
        mPeriodicIO.getBusVoltage1 = motor3.getBusVoltage();

        mPeriodicIO.getVelocity1 = ((RelativeEncoder) motor1).getVelocity();
        mPeriodicIO.getVelocity2 = ((RelativeEncoder) motor1).getVelocity();
        mPeriodicIO.getVelocity3 = ((RelativeEncoder) motor1).getVelocity();

        //LECTURA DE TIMESTAMP 
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.deltaTime = Timer.getFPGATimestamp() - mPeriodicIO.timestamp;
        
        //LECTURA DEL HARDWARE Y SENSORES
        
        //PRINT WHEEL SPEED

        //Update odometry(usando los estados reales de las llantas)
        
        //COLOR DE ALIANZA
       }

    public void writePeriodicOutputs() {

        //metodo para setear los outputs (velocidad motores...)

    }

    public void registerEnabledLoops(ILooper in) {

        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (temple_subsystem.this) {
                    if( isLogging ) startLogging();

                    //Funciones que llamamos al inicio del loop
            
                }
            }

            @Override
            public void onLoop(double timestamp) {
                if(isLogging && mCSVWriter != null) mCSVWriter.add(mPeriodicIO);

                //FUNCIONES QUE LLAMAS EN LOOP

            }

            @Override
            public void onStop(double timestamp) {
                stop();
                if( isLogging ) stopLogging();
                //NO LLEVA FUNCIONES 
            }
        });

    }

    public void zeroSensors() {
        //ESTA VACIO
    }

    //Iniciar a declarar funciones del subsistema------------------------------------------------------------    

    public void swerveMainDrive(double _rawX, double _rawY, double _turn){

            // Funcion para manejar con input de sticks

    }

    //FUNCIONES DEL SUBSYSTEMA

    public void swerveMainDrive(){/* 

        
        //Se convierten los valores de los sticks a velocidad en m/s
        //Inversión de ejes
        
        //Convertir el valor de stick a velocidad angularF
      
        //Se dan vectores de referencia en la cancha
        
        //Con Inverse Kinematics se convierten las velocidades a los estados de los modulos

        //Normalizar las velocidades para que no superen el máximo de las velocidades

        //Optimizar los valores de cada angulo en los modulos considerando el cambio de direccion en velocidad parar reducir el giro
        
        //Actualizar el estado de las salidas
        
    */}
    //FUNCION DE SUBSYSTEMA
    public void gridMainDrive(double _rawX, double _rawY, double _turn){


        //Se convierten los valores de los sticks a velocidad en m/s
        //Inversión de ejes
                
        //Se dan vectores de referencia en la cancha

        //Con Inverse Kinematics se convierten las velocidades a los estados de los modulos

        //Normalizar las velocidades para que no superen el máximo de las velocidades

        //Optimizar los valores de cada angulo en los modulos considerando el cambio de direccion en velocidad parar reducir el giro
        
        //Actualizar el estado de las salidas
    }

    public boolean isInRange(){/* 
        //OBTEN ODOMETRIA
    */ return true;
        }
    // FUNCION DE SUBSISTEMA
    public void setGridLocation(){/* 

       */}

    //Funcion para leer el objeto qu tien la odometria
    /*public SwerveDriveOdometry getOdometry(){
        return odometry;
    }*/
    //FUNCION DE SUBSISTEMA
    public void LoopRobotStates(){

    }

    private double StickToVelocity(double stickInput){
        return 0;
        // RETURN STICKYVELOCITY
        
    }

    private double StickToAngularVelocity(double stickInput){ 

       // RETURN StickToAngularVelocity

         return 0;
    }

    private double DeltaAngle(double _target, double _actual){

            //Funcion para obtener el delta mas corto entre dos angulos

        return 0;
    }  

    public double Limit360Angle(double _angle){
        return 0;
            //Funcion que limita el angulo de 0 a 360

    }

    private void UpdateDriveSpeed(){

            //Funcion para actualizar el estado de salidas

    }

    public void Autobalance(){

        // FUNCION PARA AUTOBALANCE
        
    }

    public void SetSwerveState(State swerveState){

        //FUNCION PARA SetSwerveState
        
    }

    /*public State GetSwerveState(){

        //FUNCION PARA GetSwerveState
       
    }*/

    public void SetBrakeMode(boolean enabled){

        //PARA PONER LOS MOTORES EN BREAKE MODE
    }
    
    public void SetInitZero(){

        //FUNCION PARA LEDS AND PARA INICIALIZAR A 0
    }

    //-------------------------------------------------------------------------------------------
    
    public synchronized void startLogging() {
           // Funcion para inicializar el objeto del logging

    }

    public synchronized void stopLogging() {
            // Funcion para finalizar el objeto del logging

    }

    public void stop(){
        // SPEED MOTORS AND STEERING MOTORS
        // CONTROL MODE FOR SPEED AND STEERING MOTORS
    }

    public boolean checkSystem(){
        return true;
    }

    public void outputTelemetry(){
       //WRITER OF OUTPUTS
        //WRITE OUTPUTS}
}
}