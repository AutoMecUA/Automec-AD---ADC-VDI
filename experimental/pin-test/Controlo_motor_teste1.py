import RPi.GPIO as GPIO
import time

# Definir o pino de controle do motor (pino 5)
motor_pin = 5  # Pin 5 (substituído por 12 ou 11 conforme necessário)

# Variáveis de controle
frente = False  # Inicialmente inativo
traz = False    # Inicialmente inativo

def main():
    # Configuração dos pinos:
    GPIO.setmode(GPIO.BOARD)  # Modo de numeração dos pinos da placa (BOARD)

    # Configurar o pino como saída
    GPIO.setup(motor_pin, GPIO.OUT)

    # Iniciar PWM no pino com duty cycle de 50% (50% da potência)
    pwm = GPIO.PWM(motor_pin, 100)  # Frequência de 100Hz
    pwm.start(50)  # Começa com 50% de duty cycle

    print("Iniciando o demo! Pressione CTRL+C para sair")

    try:
        while True:
            # Se a variável frente estiver ativa, definir PWM para 100%
            if frente:
                print("Frente ativa: PWM a 100%")
                pwm.ChangeDutyCycle(100)  # PWM a 100%

                # Pode incluir um `while` aqui, se necessário
                while frente:
                    time.sleep(0.1)  # Pequena espera para evitar uso excessivo de CPU

            # Se a variável traz estiver ativa, definir PWM para 0%
            elif traz:
                print("Traz ativo: PWM a 0%")
                pwm.ChangeDutyCycle(0)  # PWM a 0%

                # Pode incluir um `while` aqui, se necessário
                while traz:
                    time.sleep(0.1)  # Pequena espera para evitar uso excessivo de CPU

            # Se nenhuma das variáveis estiver ativa, manter o PWM em 50%
            else:
                print("Nenhuma direção ativa: PWM a 50%")
                pwm.ChangeDutyCycle(50)  # PWM a 50%

            time.sleep(0.5)  # Espera de 0.5 segundos antes de verificar novamente

    finally:
        pwm.stop()  # Parar o PWM ao sair
        GPIO.cleanup()  # Limpar a configuração dos pinos

if __name__ == '__main__':
    main()