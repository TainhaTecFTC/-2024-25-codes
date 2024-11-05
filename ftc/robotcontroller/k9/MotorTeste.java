package org.firstinspires.ftc.robotcontroller.k9;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Set;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "chupa chat gpt 😼 ඞ 🎏", group = "Linear Opmode")
public class MotorTeste extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    // private DcMotor motorOmbro1 = null;
    private DcMotor motorOmbro2 = null; 
    private DcMotor motorOmbro4 = null; // Braço 
    private Servo GARRA = null;
    private Servo BRACO = null;
    public double MotorSens = 1;
    private Servo AVIAO = null;
    private Servo ROBLOX = null;
    boolean isRunning = false; 
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
    
    double tgtPower = 0;
    long servoDuration = 1500; // 1000 = 1 segundo
    boolean servoActive = false;
    long servoStartTime = 0;
    
    // portas 0,2,3 sao os motores da elevação (2 e 3 do hub de expansão)

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        lastTimestamp = timer.seconds();
        leftDrive = hardwareMap.get(DcMotor.class, "motor_esq");
        rightDrive = hardwareMap.get(DcMotor.class, "motor_dir");
        ROBLOX = hardwareMap.get(Servo.class, "servo_roblox");
        motorOmbro4 = hardwareMap.get(DcMotor.class, "motor_ombro4"); //negocio do fio
        motorOmbro2 = hardwareMap.get(DcMotor.class, "motor_ombro2"); //negocio do fio


        double ROBLOX_ZERO = 0.0;
        double ROBLOX_MIN = 0.0;
        double ROBLOX_MAX = 1.0;
        
        // servo garra cima e baixo
        
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
            double leftPower;
            double rightPower;

            int position = motorOmbro4.getCurrentPosition();
            telemetry.addData("Posição do Braço: ", position);
            telemetry.addData("kP: ", kP);
            telemetry.addData("kI: ", kI);
            telemetry.addData("kD: ", kD);
            // telemetry.update();

            // teste slide aq
            
            // teste Servo garra roblox
            
          
    if (gamepad1.left_bumper) {
        ROBLOX.setPosition(1); 
        servoActive = true;
        servoStartTime = System.currentTimeMillis(); 
    
    } else if (gamepad1.right_bumper) {
        ROBLOX.setPosition(0); 
        servoActive = true;
        servoStartTime = System.currentTimeMillis();
    }


    if (servoActive && (System.currentTimeMillis() - servoStartTime >= servoDuration)) {
        ROBLOX.setPosition(0); // Set the servo to a neutral position after duration
        servoActive = false; // Reset the servo active state
    }
            
            // fim do teste roblox
            
            if (gamepad2.x) {
    setPoint2 = 0; // Exemplo de ajuste do ponto de ajuste
} else if (gamepad2.dpad_down) {
    setPoint2 = 0; // Exemplo de ajuste do ponto de ajuste
} else if (gamepad2.y) {
    setPoint2 = 700; // Exemplo de ajuste do ponto de ajuste = 900
}

double sensorPos2 = motorOmbro2.getCurrentPosition(); // Posição atual do motor
double error2 = setPoint2 - sensorPos2; // Cálculo do erro
double dt2 = timer.seconds() - lastTimestamp2; // Diferença de tempo
double errorRate2 = (error2 - lastError2) / dt2; // Taxa de variação do erro

integral2 += error2 * dt2; // Cálculo do termo integral

telemetry.addData("Error motorOmbro2", error2);

double output2 = kP2 * error2; // Termo proporcional

if (Math.abs(error2) <= IRange2) {
    output2 += kI2 * integral2; // Adiciona o termo integral se dentro do intervalo
}

// Aplica a saída do PID ao motor
if (Math.abs(error2) >= 4) {
    double sign2 = Math.signum(output2);
    motorOmbro2.setPower(Math.min(Math.abs(output2), 1) * sign2 * 0.5); // Limita a potência
} else {
    motorOmbro2.setPower(0); // Para o motor se o erro for pequeno
}

// Atualiza o último erro e timestamp
lastError2 = error2;
lastTimestamp2 = timer.seconds();
            
            //teste slide aq
            if (gamepad1.x) { //2995 no coiso normal (perderemos dedos)
                setPoint = 2985; //setPoint2 = 900; //x = high basket
            } else if (gamepad1.dpad_down) {
                setPoint = 600; // posiçao de inicio (se quiserem eu mudo)
            } else if (gamepad1.y) {
                setPoint = 0; //y = climb ///// setpoint =  4466
            } else if (gamepad1.dpad_up){
                setPoint = 3490; //low basket
            }
            
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
                motorOmbro4.setPower(Math.min(Math.abs(output), 1) * sign * 0.5);
            } else {
                motorOmbro4.setPower(0); 
            }

            if (gamepad1.a) {
                MotorSens = 0.5;
            } else {
                MotorSens = 1;
            }
            
    //         if(gamepad1.b) {
    //     // move to 0 degrees.
    //     ROBLOX.setPosition(0);
    // } else if (gamepad1.a) {
    //     // move to 180 degrees.
    //     ROBLOX.setPosition(1);
    // }

            double drive = gamepad1.left_stick_y;
            double drive2 = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + drive2, -1 * MotorSens, 1 * MotorSens);
            rightPower = Range.clip(drive - drive2, -1 * MotorSens, 1 * MotorSens);

            leftDrive.setPower(leftPower * -1);
            rightDrive.setPower(rightPower * -1);

            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Sensibilidade", MotorSens);
            telemetry.addData("Servo Position", ROBLOX.getPosition());
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
    
    
    
    
} 