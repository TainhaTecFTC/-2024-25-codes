// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.Servo;

// @com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "anemona")
// public class autonomo extends LinearOpMode {
  
//   protected DcMotor rightDrive; 
//   protected DcMotor leftDrive;
 
  
//   double kP = 0.0015; //0.0011
//     double kI = 0.00015; //0.00011
//     double IRange = 10; //10 //4466
//     double kD = 0.0001; //0.00011
//     double f = 0.0001;//0.0001
//     double ticks_in_degree = 360 / 28.0;
//     ElapsedTime timer = new ElapsedTime();
    
//     double setPoint = 0;
//     double lastError = 0;
//     double integral = 0;
//     double lastTimestamp = 0;
    
//     double kP2 = 0.015; // 0.011
//     double kI2 = 0.0015; // 0.0011
//     double IRange2 = 15; // 10
//     double kD2 = 0.015;// 0.0011
//     double f2 = 0.005; //0.001
//     double ticks_in_degree2 = 360 / 28.0; // Ticks por grau para motorOmbro2
//     ElapsedTime timer2 = new ElapsedTime(); // Temporizador para motorOmbro2

//     double setPoint2 = 0; // Ponto de ajuste para motorOmbro2
//     double lastError2 = 0; // Último erro para motorOmbro2
//     double integral2 = 0; // Integral para motorOmbro2
//     double lastTimestamp2 = 0; // Último timestamp para motorOmbro2
 
//   @Override
//   public void runOpMode()throws InterruptedException {
//       leftDrive = hardwareMap.get(DcMotor.class, "motor_esq");
//       rightDrive = hardwareMap.get(DcMotor.class, "motor_dir");

//       leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
//       rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
      
//   waitForStart();
  
// // GARRA.setPosition(0); (mover servo)

    
    
//   moveForward(1000,1000); //420/ 0.355
  

  
  
  
//   }
  
//   public void moveForward (double power, int time){
    
//     // leftDrive.setPower(-0.12); //0.2/ 0.15
//     // rightDrive.setPower(0.12);
//     // sleep(time);
//     // leftDrive.setPower(power);
//     // rightDrive.setPower(power);   
//     // sleep(time);
//     // leftDrive.setPower(0.0);

//     // sleep(time);
//     leftDrive.setPower(0.24);
//     rightDrive.setPower(-0.19);
//     sleep(time);
//     leftDrive.setPower(3.0);
//     rightDrive.setPower(0.0);
//     sleep(time);
//     leftDrive.setPower(0.6);
//     rightDrive.setPower(0.0);
//     sleep(time);
//     leftDrive.setPower(0.80);       
//     rightDrive.setPower(-0.80); //0.45
//     sleep(time);
//     leftDrive.setPower(-5);       
//     rightDrive.setPower(5); //0.45
//     sleep(time);
//     leftDrive.setPower(-0.90);       
//     rightDrive.setPower(0.0); //0.45
//     sleep(time);


    
//   }                                                                                                                                                                                                                                                                                                                                                           
// }

// //giro = 0.420
// //pra frente e pra tras (frente= 07)
// // azul negativo, vermelho positivo


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "anemona")
public class autonomo extends LinearOpMode {

    protected DcMotor rightDrive; 
    protected DcMotor leftDrive;

    // PID constants for first motor
    double kP = 0.0015; // Proporcional
    double kI = 0.00015; // Integral
    double kD = 0.0001; // Derivativo

    // PID constants for second motor
    double kP2 = 0.015; // Proporcional
    double kI2 = 0.0015; // Integral
    double kD2 = 0.015; // Derivativo
    
    // Ticks por rotação (ajuste conforme necessário)
    int ticksPerRevolution = 360; // Ajuste conforme o seu motor
    double wheelDiameterCm = 9.0; // Diâmetro da roda em centímetros
    double wheelCircumference = Math.PI * wheelDiameterCm; // Perímetro da roda

    ElapsedTime timer = new ElapsedTime();
    
    // Variáveis PID
    double lastError = 0;
    double integral = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.get(DcMotor.class, "motor_esq");
        rightDrive = hardwareMap.get(DcMotor.class, "motor_dir");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        
        waitForStart();
        
        // Sequência de movimentos
        moveForward(25); 
        moveBackward(98);
        rotateLeft(180); 
        moveBackward(68);
        moveForward(68);
        moveRight(68);    
        rotateRight(90);  
    }

    public void moveForward(double distanceCm) {
        double rotationsNeeded = distanceCm / wheelCircumference; // Rotações necessárias
        int targetTicks = (int)(rotationsNeeded * ticksPerRevolution); // Ticks necessários

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + targetTicks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(2.0); // Poder do motor
        rightDrive.setPower(2.0);

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            double leftPower = calculatePID(leftDrive.getCurrentPosition(), leftDrive.getTargetPosition(), kP, kI, kD);
            double rightPower = calculatePID(rightDrive.getCurrentPosition(), rightDrive.getTargetPosition(), kP2, kI2, kD2);
            
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        stopMotors();
    }

    public void moveBackward(double distanceCm) {
        double rotationsNeeded = distanceCm / wheelCircumference; // Rotações necessárias
        int targetTicks = (int)(rotationsNeeded * ticksPerRevolution); // Ticks necessários

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - targetTicks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(-1.0); // Poder do motor para ré
        rightDrive.setPower(-1.0);

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            double leftPower = calculatePID(leftDrive.getCurrentPosition(), leftDrive.getTargetPosition(), kP, kI, kD);
            double rightPower = calculatePID(rightDrive.getCurrentPosition(), rightDrive.getTargetPosition(), kP2, kI2, kD2);
            
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        stopMotors();
    }

    public void moveRight(double distanceCm) {
        int targetTicks = calculateTicks(distanceCm);
        
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + targetTicks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(1.0); // Poder do motor esquerdo
        rightDrive.setPower(-1.0); // Poder do motor direito

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            double leftPower = calculatePID(leftDrive.getCurrentPosition(), leftDrive.getTargetPosition(), kP, kI, kD);
            double rightPower = calculatePID(rightDrive.getCurrentPosition(), rightDrive.getTargetPosition(), kP2, kI2, kD2);
            
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        stopMotors();
    }

    public void rotateLeft(double degrees) {
        int targetTicks = calculateRotationTicks(degrees);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - targetTicks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(-1.0); // Poder do motor esquerdo
        rightDrive.setPower(1.0);  // Poder do motor direito

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            double leftPower = calculatePID(leftDrive.getCurrentPosition(), leftDrive.getTargetPosition(), kP, kI, kD);
            double rightPower = calculatePID(rightDrive.getCurrentPosition(), rightDrive.getTargetPosition(), kP2, kI2, kD2);
            
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        stopMotors();
    }

    public void rotateRight(double degrees) {
        int targetTicks = calculateRotationTicks(degrees);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + targetTicks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(1.0); // Poder do motor esquerdo
        rightDrive.setPower(-1.0);  // Poder do motor direito

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            double leftPower = calculatePID(leftDrive.getCurrentPosition(), leftDrive.getTargetPosition(), kP, kI, kD);
            double rightPower = calculatePID(rightDrive.getCurrentPosition(), rightDrive.getTargetPosition(), kP2, kI2, kD2);
            
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        stopMotors();
    }

    private int calculateTicks(double distanceCm) {
        double rotationsNeeded = distanceCm / wheelCircumference; // Rotações necessárias
        return (int)(rotationsNeeded * ticksPerRevolution); // Ticks necessários
    }

    private int calculateRotationTicks(double degrees) {
        double wheelBase = 10; // Distância entre os eixos das rodas (ajuste conforme necessário)
        double rotationCircumference = Math.PI * wheelBase; // Perímetro do círculo de rotação

        double rotationsNeeded = degrees / 360; // Rotações necessárias
        return (int)(rotationsNeeded * (rotationCircumference / wheelCircumference) * ticksPerRevolution); // Ticks necessários
    }

    private void stopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public double calculatePID(int currentPosition, int targetPosition, double kP, double kI, double kD) {
        double error = targetPosition - currentPosition;
        integral += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        lastError = error;
        timer.reset();

        return output;
    }
}
