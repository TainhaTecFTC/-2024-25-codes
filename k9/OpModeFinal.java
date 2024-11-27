package org.firstinspires.ftc.robotcontroller.k9;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Set;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Opmode Final", group = "Linear Opmode")
public class OpModeFinal extends LinearOpMode 
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor motorOmbro2 = null; // Slide
    private DcMotor motorOmbro4 = null; // Braço
    private CRServo GARRA = null;
    private CRServo PUNHO = null;
    boolean isRunning = false; 
    public double MotorSens = 1;
    
    //Parametros e Coeficientes do PID 
   
    //Parametros do PivotArm (motorOmbro4)
    double kP = 0.0011;   //0.0011
    double kI = 0.00011;  //0.00011
    double IRange = 10;   //10
    double kD = 0.00011;  //0.00011
    double f = 0.00010;   //0.00010
    double ticks_in_degree = 360 / 28.0;      //Ticks por grau
    ElapsedTime timer = new ElapsedTime();    //Temporizador
    double setPoint = 0;             //Ponto de ajuste para motorOmbro4
    double lastError = 0;            //Último erro para motorOmbro4
    double integral = 0;             //Integral para motorOmbro4
    double lastTimestamp = 0;        //Último timestamp para motorOmbro4
    
    //Parametros do Slide (motorOmbro2)
    double kP2 = 0.011; // 0.011
    double kI2 = 0.0011; // 0.0011
    double IRange2 = 10; // 10
    double kD2 = 0.0011;// 0.0011
    double f2 = 0.001; //0.001
    double ticks_in_degree2 = 360 / 28.0;      //Ticks por grau
    ElapsedTime timer2 = new ElapsedTime();    //Temporizador
    double setPoint2 = 0;            //Ponto de ajuste para motorOmbro2
    double lastError2 = 0;           //Último erro para motorOmbro2
    double integral2 = 0;            //Integral para motorOmbro2
    double lastTimestamp2 = 0;       //Último timestamp para motorOmbro2
    //Fim dos pids
        
    double tgtPower = 0;
    long servoDuration = 10; // 1000 = 1 segundo
    boolean servoActive = false;
    long servoStartTime = 0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized", "Feito por migel");
        telemetry.update();
        lastTimestamp = timer.seconds();
        leftDrive = hardwareMap.get(DcMotor.class, "motor_esq");
        rightDrive = hardwareMap.get(DcMotor.class, "motor_dir");
        PUNHO = hardwareMap.get(CRServo.class, "servo_punho");
        GARRA = hardwareMap.get(CRServo.class, "servo_garra");
        motorOmbro4 = hardwareMap.get(DcMotor.class, "motor_ombro4"); //trocar para PivotArm
        motorOmbro2 = hardwareMap.get(DcMotor.class, "motor_ombro2"); //trocar para Slider

        double PUNHO_ZERO = 0.0;
        double PUNHO_MIN = 0.0;
        double PUNHO_MAX = 1.0;

        double GARRA_ZERO = 0.0;
        double GARRA_MIN = 0.0;
        double GARRA_MAX = 1.0;

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        motorOmbro4.setDirection(DcMotor.Direction.REVERSE);
        motorOmbro4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOmbro4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Inicialização da Tração
            double leftPower;
            double rightPower;
            
            //Inicialização do PivotArm
            int position = motorOmbro4.getCurrentPosition();
            double sensorPos = position;
            double error = setPoint - sensorPos;
            double dt = timer.seconds() - lastTimestamp;
            double errorRate = (error - lastError) / dt;

            telemetry.addData("Error", error);
            integral += error * timer.seconds();
            
            double ff = Math.cos(Math.toRadians(setPoint / ticks_in_degree)) * f;
            timer.reset();
            double output = kP * error;

            if (Math.abs(error) <= IRange) {
                output += kI * integral;
            }

            if (Math.abs(error) >= 4) {
                double sign = Math.signum(output);
                motorOmbro4.setPower(Math.min(Math.abs(output), 1) * sign * 0.7);
            } else {
                motorOmbro4.setPower(0); 
            }
            
            double sensorPos2 = motorOmbro2.getCurrentPosition(); // Posição atual do motor
            double error2 = setPoint2 - sensorPos2; // Cálculo do erro
            double dt2 = timer.seconds() - lastTimestamp2; // Diferença de tempo
            double errorRate2 = (error2 - lastError2) / dt2; // Taxa de variação do erro

            integral2 += error2 * dt2; // Cálculo do termo integral
            double output2 = kP2 * error2; // Termo proporcional

            if (Math.abs(error2) <= IRange2) {
                output2 += kI2 * integral2; // Adiciona o termo integral se dentro do intervalo
            }   
            
            // Aplica a saída do PID ao motor
            if (Math.abs(error2) >= 4) {
                double sign2 = Math.signum(output2);
                motorOmbro2.setPower(Math.min(Math.abs(output2), 1) * sign2 * 1); // Limita a potência
            } else {
                motorOmbro2.setPower(0); // Para o motor se o erro for pequeno
            }
            // Atualiza o último erro e timestamp
            lastError2 = error2;
            lastTimestamp2 = timer.seconds();
            
            
            //Inicio do Codigo para Controle do Robo
            //Rotinas de PivotArm + SLide
            
             if (gamepad2.x) {
                LowBasket();
             } else if (gamepad2.y) {
                HighBasket();
             } else if (gamepad2.b) {
                IntakeSubmarino();
             } else if (gamepad2.a) {
                IntakeSpecimen();
             }
            
            //Rotinas de Slide
            if (gamepad2.dpad_up) {
                setPoint2 = 650;
             }
            if (gamepad2.dpad_down) {
                setPoint2 = 0;
             }
             if (gamepad2.dpad_right) {
                setPoint2 = 400;
             }
            
            // Servo Garra
            if (gamepad1.left_bumper){
                GARRA.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                GARRA.setPower(-1.0);
            } else{
                GARRA.setPower(0.0);
            } 
            
            // Servo Punho
            if (gamepad2.left_trigger < 0.5){
                PUNHO.setPower(1.0);
            } else if (gamepad2.right_trigger < -0.5) {
                PUNHO.setPower(-1.0);
            } else{
                PUNHO.setPower(0.0);
            }
  
            
            double drive = gamepad1.left_stick_y;
            double drive2 = gamepad1.right_stick_x;
            
            leftPower = Range.clip(drive + drive2, -1 * MotorSens, 1 * MotorSens);
            rightPower = Range.clip(drive - drive2, -1 * MotorSens, 1 * MotorSens);

            leftDrive.setPower(leftPower * 1);
            rightDrive.setPower(rightPower * 1);

            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
        public void HighBasket() {
            setPoint = 2500;
            // setPoint2 = 0;
        }
        
    
        public void LowBasket() {
            setPoint = 1900;
            // setPoint2 = 0;
        }
        
        public void IntakeSubmarino() {
            setPoint = 850;
            // setPoint2 = 0;
        }
        
        public void IntakeSpecimen() {
            setPoint = 0;
            // setPoint2 = 0;
        }
}
        
         //Dados mostrados no Display
            //telemetry.addData("Error motorOmbro2", error2);
            
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Sensibilidade", MotorSens);
            //telemetry.addData("Target Power", tgtPower);
            //telemetry.addData("Status", "Running");
            //telemetry.update();

            //telemetry.addData("Posição do Braço: ", setPoint);
            //telemetry.addData("kP: ", kP);
            //telemetry.addData("kI: ", kI);
            //telemetry.addData("kD: ", kD);
            
// Controles:

// gamepad1:
// lb = abaixar punho
// rb = levantar punho
// joysticks = controlar robo 

// gamepad2:
// x = high basket (nao vai controlar os servos, apenas o ombro e o elevador)
// y = climb 
// dpad_down = desce apenas o evelador
// b = volta a posição 0 de tudo 