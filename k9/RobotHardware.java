package org.firstinspires.ftc.robotcontroller.k9;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotHardware {
    private LinearOpMode myOpMode = null; // Ganha acesso aos métodos no OpMode chamante.

    // Define objetos Motor e Servo (tornando-os privados para que não possam ser acessados externamente).
    private DcMotor motorOmbro1 = null;
    private DcMotor motorOmbro2 = null;
    private DcMotor motorOmbro3 = null;
    private DcMotor motorOmbro4 = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private CRServo servoPunho = null;
    private CRServo servoGarra = null;
    public double motorsens = 1;
    // private Servo PUNHO = null;


    // Define constantes de direção do servo. Torna-as públicas para que possam ser usadas pelo OpMode chamante.
    public static final double BRACO_ZERO = 0.0;
    public static final double BRACO_MIN = 0.0; // Define a taxa de movimento do servo
    public static final double BRACO_MAX = 0.5;

    public static final double GARRA_ZERO = 0.0;
    public static final double GARRA_MIN = 0.0; // Define a taxa de movimento do servo
    public static final double GARRA_MAX = 0.5;

    
    public static final double PUNHO_ZERO = 0.0;
    public static final double PUNHO_MIN = 0.0;
    public static final double PUNHO_MAX = 0.5;

    // Define um construtor que permite que o OpMode passe uma referência a si mesmo.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Inicialize todo o hardware do robô.
     * Este método deve ser chamado UMA VEZ quando o OpMode for inicializado.
     */
    public void init() {
        // Define e Inicializa os Motores (observe que precisamos usar a referência ao OpMode real).
        leftDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "motor_braco");
        motorOmbro1 = myOpMode.hardwareMap.get(DcMotor.class, "motor_ombro1");
        motorOmbro2 = myOpMode.hardwareMap.get(DcMotor.class, "motor_ombro2");
        motorOmbro3 = myOpMode.hardwareMap.get(DcMotor.class, "motor_ombro3");
        motorOmbro4 = myOpMode.hardwareMap.get(DcMotor.class, "motor_ombro4");
        // PUNHO = myOpMode.hardwareMap.get(Servo.class, "servo_punho");

        // Para avançar, a maioria dos robôs precisa que o motor de um lado seja invertido, pois os eixos apontam em direções opostas.
        // Empurrar o joystick esquerdo para a frente DEVE fazer o robô avançar. Portanto, ajuste estas duas linhas com base no primeiro teste de direção.
        // Nota: As configurações aqui pressupõem direção direta nas rodas esquerda e direita. Engrenagens ou unidades de 90 graus podem exigir inversões de direção.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Se houver codificadores conectados, mude para o modo RUN_USING_ENCODER para maior precisão
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Defina e inicialize TODOS os servos instalados.
        // servoBraco = myOpMode.hardwareMap.get(Servo.class, "servo_braco");
        servoGarra = myOpMode.hardwareMap.get(CRServo.class, "servo_garra");
        servoPunho = myOpMode.hardwareMap.get(CRServo.class, "servo_punho");
        // servoGarra.setPosition(GARRA_ZERO);
        // servoPunho.setPosition(PUNHO_ZERO);

        myOpMode.telemetry.addData(">", "Hardware Inicializado");
        myOpMode.telemetry.update();
    }
}




