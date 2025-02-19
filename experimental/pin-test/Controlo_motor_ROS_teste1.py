import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32

# Definir o pino de controle do motor (pino 5)
motor_pin = 5  # Pin 5 (substituído por 12 ou 11 conforme necessário)

# Variável de controle do PWM
motor_pwm = 50  # Inicialmente parado

def pwm_callback(msg):
    global motor_pwm
    motor_pwm = msg.data
    
    if motor_pwm < 0:
        motor_pwm = 0
    elif motor_pwm > 100:
        motor_pwm = 100
    
    pwm.ChangeDutyCycle(motor_pwm)
    
    if motor_pwm == 50:
        rospy.loginfo("Motor parado: PWM = 50%")
    elif motor_pwm < 50:
        rospy.loginfo(f"Motor para trás: PWM = {motor_pwm}%")
    elif motor_pwm > 50:
        rospy.loginfo(f"Motor para frente: PWM = {motor_pwm}%")

def main():
    global pwm  # Tornar pwm global para ser acessado na callback

    # Inicializar o nó ROS
    rospy.init_node('motor_control_node')

    # Configuração dos pinos:
    GPIO.setmode(GPIO.BOARD)  # Modo de numeração dos pinos da placa (BOARD)
    GPIO.setup(motor_pin, GPIO.OUT)

    # Iniciar PWM no pino com duty cycle de 50% (motor parado)
    pwm = GPIO.PWM(motor_pin, 100)  # Frequência de 100Hz
    pwm.start(50)  # Começa com 50% de duty cycle

    # Criar o subscriber para o tópico do PWM
    rospy.Subscriber('motor_pwm', Int32, pwm_callback)

    rospy.loginfo("Iniciando o demo ROS! Pressione CTRL+C para sair")

    # Manter o nó rodando até que o Ctrl+C seja pressionado
    try:
        rospy.spin()  # Espera pelas mensagens
    except KeyboardInterrupt:
        pass
    finally:
        pwm.stop()  # Parar o PWM ao sair
        GPIO.cleanup()  # Limpar a configuração dos pinos

if __name__ == '__main__':
    main()
