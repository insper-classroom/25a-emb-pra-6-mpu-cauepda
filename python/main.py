#!/usr/bin/env python3
import serial
import threading
import pyautogui
import time

# Configuração da porta serial (ajuste conforme seu SO)
SERIAL_PORT = '/dev/tty.usbmodem102'  # ou 'COM3' no Windows
BAUD_RATE   = 115200
pyautogui.FAILSAFE = False
pyautogui.PAUSE = 0

def reader_thread(ser):
    """
    Lê linhas no formato:
    roll,pitch,yaw,click
    e converte em movimentos+click.
    """
    while True:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            continue
        try:
            roll, pitch, yaw, click = line.split(',')
            roll  = float(roll)
            pitch = float(pitch)
            yaw   = float(yaw)
            click = bool(int(click))
        except ValueError:
            continue

        # Mapeia ângulos em pixels
        # você pode calibrar estes ganhos
        dx = roll  *  2.0
        dy = pitch * -2.0

        # executa movimento
        pyautogui.moveRel(dx, dy, duration=0)
        pyautogui.moveRel(dx, dy, duration=0, _pause=False)

        # executa click se solicitado
        if click:
            pyautogui.click()

if __name__ == '__main__':
    # abre serial
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # espera inicialização da placa

    # inicia thread de leitura
    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    print("Pointer IMU rodando. Pressione Ctrl+C para sair.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nEncerrando.")
        ser.close()