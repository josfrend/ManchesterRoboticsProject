import cv2
import numpy as np

def detect_intersections(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    gray = cv2.GaussianBlur(image, (3,3),0)

    # Apply Canny edge detection to detect edges
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # Apply Hough transform to detect lines
    lines = cv2.HoughLines(edges, 0.8, np.pi / 170, 190)

    # Store the coordinates of intersections
    intersections = []

    # Draw the detected lines and find intersections
    if lines is not None:
        lines = lines[:, 0, :]  # Extract line coordinates
        for i in range(len(lines)):
            rho1, theta1 = lines[i]
            for j in range(i + 1, len(lines)):
                rho2, theta2 = lines[j]
                a1, b1 = np.cos(theta1), np.sin(theta1)
                a2, b2 = np.cos(theta2), np.sin(theta2)
                denom = a1 * b2 - a2 * b1
                if denom != 0:
                    x = int((b2 * rho1 - b1 * rho2) / denom)
                    y = int((a1 * rho2 - a2 * rho1) / denom)

                    # Check if the current point is close to any other point
                    merged = False
                    for intersection in intersections:
                        if np.sqrt((x - intersection[0]) ** 2 + (y - intersection[1]) ** 2) < 10:
                            # Merge the current point with the nearby point
                            intersection[0] = (intersection[0] + x) // 2
                            intersection[1] = (intersection[1] + y) // 2
                            merged = True
                            break

                    # If not merged, add the intersection as a new point
                    if not merged:
                        intersections.append([x, y])

        # Draw the points on the intersections
        for intersection in intersections:
            x, y = intersection
            cv2.circle(image, (x, y), 2, (0, 255, 0), -1)
            

    return image

# Abrir el video
video = cv2.VideoCapture("patitos.mp4")

# Obtener el número de fotogramas por segundo del video
fps = video.get(cv2.CAP_PROP_FPS)

# Obtener el ancho y alto de los fotogramas del video
frame_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Crear un objeto VideoWriter para guardar el video procesado
output = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (frame_width, frame_height))

# Procesar cada fotograma del video
while True:
    # Leer el fotograma actual
    ret, frame = video.read()

    # Verificar si se llegó al final del video
    if not ret:
        break

    # Detectar las intersecciones utilizando la transformada de Hough
    result = detect_intersections(frame)

    # Escribir el fotograma procesado en el archivo de salida
    output.write(result)

    # Mostrar el fotograma procesado en una ventana
    cv2.imshow('Intersections Detection', result)

    # Salir del bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar los recursos utilizados
video.release()
output.release()
cv2.destroyAllWindows()
