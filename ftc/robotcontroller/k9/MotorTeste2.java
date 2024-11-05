package org.firstinspires.ftc.robotcontroller.k9;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Set;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teste Motor 2", group = "Linear Opmode")
public class MotorTeste2 extends LinearOpMode {
    
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MotorTeste = null; // Braço 
    boolean isRunning = false; 
    double kP = 0.03;
    double kI = 0.005;
    double IRange = 10;
    double kD = -0.01;
    double f = 0.0;
    double ticks_in_degree = 360 / 28.0;
    ElapsedTime timer = new ElapsedTime();
    
    double setPoint = 0;
    double lastError = 0;
    double integral = 0;
    double lastTimestamp = 0;
    
    
  
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        lastTimestamp = timer.seconds();
        MotorTeste = hardwareMap.get(DcMotor.class, "motor_ombro4"); //negocio do fio
        
        
        MotorTeste.setDirection(DcMotor.Direction.REVERSE);
        MotorTeste.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorTeste.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
           int position = MotorTeste.getCurrentPosition();
            
            telemetry.addData("Posição do Braço: ", position);
            telemetry.addData("kP: ", kP);
            telemetry.addData("kI: ", kI);
            telemetry.addData("kD: ", kD);
            telemetry.update();

            if (gamepad1.x){
                kI = (kI + 0.0001);
            } else if (gamepad1.b){
                kI = (kI - 0.0001);
            }
            if (gamepad1.y){
                kP = (kP + 0.01);
            } else if (gamepad1.a){
                kP = (kP - 0.01);
            }

            if (gamepad1.dpad_up) {
                setPoint = 60;
            } else if (gamepad1.dpad_down) {
                setPoint = -60;
            } else if (gamepad1.dpad_right) {
                setPoint = 0;
            }

            double sensorPos = position;
            double error = setPoint - sensorPos;
            double dt = timer.seconds() - lastTimestamp;
            double errorRate = (error - lastError) / dt;

            telemetry.addData("Error", error);

            integral += error * timer.seconds();
            
            //double ff = Math.cos(Math.toRadians(setPoint / ticks_in_degree)) * f;
            
            timer.reset();
            double output = kP * error;

            if(Math.abs(error) <= IRange){
                output += kI * integral;
            }

            if (Math.abs(error) >= 4){

                MotorTeste.setPower(output);
            }else{
                MotorTeste.setPower(0);
            }
        }
    }

    }
