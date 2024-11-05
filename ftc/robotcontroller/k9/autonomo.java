package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "anemona")
public class autonomo extends LinearOpMode {
  
  protected DcMotor rightDrive; 
  protected DcMotor leftDrive;
  protected DcMotor motorOmbro1; 
  protected DcMotor motorOmbro2;
  protected Servo GARRA;
  
   double kP = 0.0015; //0.0011
    double kI = 0.00015; //0.00011
    double IRange = 10; //10 //4466
    double kD = 0.0001; //0.00011
    double f = 0.0001;//0.0001
    double ticks_in_degree = 360 / 28.0;
    ElapsedTime timer = new ElapsedTime();
    
    double setPoint = 0;
    double lastError = 0;
    double integral = 0;
    double lastTimestamp = 0;
    
    double kP2 = 0.015; // 0.011
    double kI2 = 0.0015; // 0.0011
    double IRange2 = 15; // 10
    double kD2 = 0.015;// 0.0011
    double f2 = 0.005; //0.001
    double ticks_in_degree2 = 360 / 28.0; // Ticks por grau para motorOmbro2
    ElapsedTime timer2 = new ElapsedTime(); // Temporizador para motorOmbro2

    double setPoint2 = 0; // Ponto de ajuste para motorOmbro2
    double lastError2 = 0; // Último erro para motorOmbro2
    double integral2 = 0; // Integral para motorOmbro2
    double lastTimestamp2 = 0; // Último timestamp para motorOmbro2
 
  @Override
  public void runOpMode()throws InterruptedException {
      leftDrive = hardwareMap.get(DcMotor.class, "motor_esq");
      rightDrive = hardwareMap.get(DcMotor.class, "motor_dir");
      motorOmbro1 = hardwareMap.get(DcMotor.class, "motor_ombro1");
      motorOmbro2 = hardwareMap.get(DcMotor.class, "motor_ombro2");
      GARRA = hardwareMap.get(Servo.class, "servo_garra");
        
      leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
      rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
      motorOmbro1.setDirection(DcMotorSimple.Direction.REVERSE);
      motorOmbro2.setDirection(DcMotorSimple.Direction.REVERSE);

  waitForStart();
  
// GARRA.setPosition(0); (mover servo)

    
    
  moveForward(-0.390,1000); //420/ 0.355
  

  
  
  
  }
  
  public void moveForward (double power, int time){
    
    // leftDrive.setPower(-0.12); //0.2/ 0.15
    // rightDrive.setPower(0.12);
    // sleep(time);
    // leftDrive.setPower(power);
    // rightDrive.setPower(power);   
    // sleep(time);
    // leftDrive.setPower(0.0);
    motorOmbro1.setPower(1);
    motorOmbro2.setPower(1);
    // sleep(time);
    leftDrive.setPower(1.0); //frente e tras = 507
    rightDrive.setPower(-1.0);
    sleep(time);
    leftDrive.setPower(0.30);
    rightDrive.setPower(0.30);
    sleep(time);
    leftDrive.setPower(0.55);       
    rightDrive.setPower(-0.55); //0.45
    sleep(time);
    leftDrive.setPower(-0.15);       
    rightDrive.setPower(0.15); //0.2 aq
    sleep(time);
    leftDrive.setPower(0);       
    rightDrive.setPower(0);
    sleep(time);
    leftDrive.setPower(1.0);
    rightDrive.setPower(1.0);

    
  }                                                                                                                                                                                                                                                                                                                                                           
}

//giro = 0.420
//pra frente e pra tras (frente= 0.507)
// azul negativo, vermelho positivo