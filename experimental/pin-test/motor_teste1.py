import RPi.GPIO as GPIO
import time

# Definir os pinos do motor (12 e 11)
motor_pin_a = 12  # Pin 12
motor_pin_b = 11  # Pin 11

def main():
    # Configuração dos pinos:
    # Modo de numeração dos pinos da placa (BOARD)
    GPIO.setmode(GPIO.BOARD)

    # Configurar os pinos como saídas e iniciar em LOW
    GPIO.setup(motor_pin_a, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(motor_pin_b, GPIO.OUT, initial=GPIO.LOW)

    print("Iniciando o demo! Pressione CTRL+C para sair")
    curr_value_pin_a = GPIO.HIGH  # Valor atual do pino A (inicia em HIGH)
    curr_value_pin_b = GPIO.LOW   # Valor atual do pino B (inicia em LOW)

    try:
        while True:
            time.sleep(5)  # Aguardar 5 segundos
            # Exibir os estados atuais dos pinos
            print("Saída: {} para pino {} E {} para pino {}".format(
                curr_value_pin_a, motor_pin_a, curr_value_pin_b, motor_pin_b))

            # Atualizar a saída dos pinos
            GPIO.output(motor_pin_a, curr_value_pin_a)
            GPIO.output(motor_pin_b, curr_value_pin_b)

            # Alternar os valores entre HIGH e LOW
            curr_value_pin_a = GPIO.LOW if curr_value_pin_a == GPIO.HIGH else GPIO.HIGH
            curr_value_pin_b = GPIO.LOW if curr_value_pin_b == GPIO.HIGH else GPIO.HIGH

    finally:
        GPIO.cleanup()  # Limpar a configuração dos pinos ao sair

if __name__ == '__main__':
    main()

