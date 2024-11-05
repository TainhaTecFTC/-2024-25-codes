package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "autismo-andar")
public class Autismo extends LinearOpMode {
  
  protected DcMotor rightDrive; 
  protected DcMotor leftDrive;
  protected DcMotor motorOmbro1; 
  protected DcMotor motorOmbro2;
  protected Servo GARRA;
 
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
      // GARRA.setPosition(0); (mover servo)

  waitForStart();
  

    
    
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
    leftDrive.setPower(0.45); //mover na diagonel
    rightDrive.setPower(-0.45);
    sleep(time);
    leftDrive.setPower(1);
    rightDrive.setPower(1);
    leftDrive.setPower(0.55);       
    rightDrive.setPower(0.55); //Pra frente do bloco azul
    sleep(time);
    leftDrive.setPower(-0.85);       
    rightDrive.setPower(0.85); //ficar rente ao bloco
    sleep(time);
    leftDrive.setPower(1);       
    rightDrive.setPower(1); // empurrar blocko e voltar de ré
    leftDrive.setPower(-1.0);
    rightDrive.setPower(-1.0);
    sleep(time);
    leftDrive.setPower(0.20);
    rightDrive.setPower(-0.20);
    leftDrive.setPower(0.20);
    rightDrive.setPower(0.20);
    leftDrive.setPower(-0.20);
    rightDrive.setPower(0.20);
    sleep(time);
    leftDrive.setPower(1);       
    rightDrive.setPower(1); // empurrar blocko e voltar de ré
    leftDrive.setPower(-1.0);
    rightDrive.setPower(-1.0);
    sleep(time);
    leftDrive.setPower(0.20);
    rightDrive.setPower(-0.20);
    leftDrive.setPower(0.20); //se alinhar
    rightDrive.setPower(0.20);
    leftDrive.setPower(-0.20);
    rightDrive.setPower(0.20);
    sleep(time);
    leftDrive.setPower(1);       
    rightDrive.setPower(1); // empurrar blocko e voltar de ré
    leftDrive.setPower(-1.0);
    rightDrive.setPower(-1.0);

    
  }                                                                                                                                                                                                                                                                                                                                                           
}

//giro = 0.420
//pra frente e pra tras (frente= 0.507)
// azul negativo, vermelho positivo