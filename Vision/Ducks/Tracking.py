import cv2
import numpy as np

# Importar las bibliotecas necesarias

def detectar_circulos(frame, rangos_color_bgr, rangos_color_hsv, colores):
    # Función para detectar y dibujar círculos de diferentes colores en un frame

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Convertir el frame de BGR a HSV para facilitar la detección del color

    mascaras = []
    for rango_bgr, rango_hsv in zip(rangos_color_bgr, rangos_color_hsv):
        mascara_bgr = cv2.inRange(frame, rango_bgr[0], rango_bgr[1])
        mascara_hsv = cv2.inRange(hsv, rango_hsv[0], rango_hsv[1])
        mascara = cv2.bitwise_and(mascara_bgr, mascara_hsv)
        mascaras.append(mascara)

    # Crear máscaras para cada rango de color y combinarlas mediante operaciones bitwise

    contornos = []
    for mascara in mascaras:
        contornos_color, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contornos.append(contornos_color)

    # Encontrar los contornos de cada color mediante el uso de las máscaras

    nombres_colores = ['Naranja', 'Verde', 'Azul', 'Morado', 'Amarillo', 'Cian', 'Rojo']
    for i, (contornos_color, color, nombre_color) in enumerate(zip(contornos, colores, nombres_colores), start=1):
        for contorno in contornos_color:
            area = cv2.contourArea(contorno)
            if area > 100:
                x, y, w, h = cv2.boundingRect(contorno)
                centro_x = int(x + w / 2)
                centro_y = int(y + h / 2)
                cv2.circle(frame, (centro_x, centro_y), int((w + h) / 4), color, 2)
                cv2.putText(frame, f'{nombre_color} ({i})', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                cv2.putText(frame, f'({centro_x}, {centro_y})', (x, y + h + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # Dibujar círculos y etiquetas de color y coordenadas en el frame

    return frame, contornos


# Rangos de color en formato BGR y HSV para cada color
rangos_color_bgr = [
    (np.array([1, 65, 220]), np.array([5, 76, 238])),
    (np.array([20, 255, 40]), np.array([25, 255, 43])),
    (np.array([230, 0, 0]), np.array([250, 0, 3])),
    (np.array([165, 0, 170]), np.array([180, 0, 180])),
    (np.array([9, 250, 245]), np.array([14, 255, 251])),      # Rango BGR para Amarillo (por completar)
    (np.array([248, 253, 20]), np.array([255, 255, 25])),      # Rango BGR para Lila (por completar)
    (np.array([3, 0, 229]), np.array([7, 0, 234]))       # Rango BGR para Rojo (por completar)
]
rangos_color_hsv = [
    (np.array([5, 220, 220]), np.array([10, 253, 240])),
    (np.array([50, 220, 253]), np.array([117, 235, 255])),
    (np.array([100, 255, 230]), np.array([130, 255, 250])),
    (np.array([140, 255, 175]), np.array([150, 255, 180])),
    (np.array([26, 240, 251]), np.array([34, 246, 255])),      # Rango HSV para Amarillo (por completar)
    (np.array([85, 228, 250]), np.array([93, 235, 255])),      # Rango HSV para Lila (por completar)
    (np.array([175, 253, 230]), np.array([182, 255, 236]))       # Rango HSV para Rojo (por completar)
]

# Colores correspondientes a cada rango de color
colores = [
    (5, 75, 255),    # Rojo
    (0, 255, 0),    # Verde
    (255, 0, 0),    # Azul
    (255, 0, 255),  # Morado
    (0, 255, 255),  # Amarillo (por completar)
    (255, 255, 23),  # Lila (por completar)
    (0, 0, 255)     # Rojo (por completar)
]

captura = cv2.VideoCapture("ID_6.mp4")
frame_num = 0

# Abrir el archivo de video para captura

while True:
    ret, frame = captura.read()
    if not ret:
        break
    frame_num += 1
    frame, contornos = detectar_circulos(frame, rangos_color_bgr, rangos_color_hsv, colores)
    cv2.imshow('Seguimiento de Círculos', frame)

    # Leer el siguiente frame y detectar los círculos en él

    for i, contornos_color in enumerate(contornos, start=1):
        for contorno in contornos_color:
            area = cv2.contourArea(contorno)
            if area > 100:
                x, y, w, h = cv2.boundingRect(contorno)
                centro_x = int(x + w / 2)
                centro_y = int(y + h / 2)
                print(f'Frame {frame_num}: Círculo {i}: ({centro_x}, {centro_y})')

    # Imprimir las coordenadas de los círculos detectados en la consola

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Salir del bucle si se presiona la tecla 'q'

captura.release()
cv2.destroyAllWindows()
