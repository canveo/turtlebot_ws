import cv2
import numpy as np

# Define el rango de colores rojos en el espacio de color RGB
# RGB_LOW = np.array([0, 0, 100], dtype=np.uint8)
# RGB_HIGH = np.array([100, 100, 255], dtype=np.uint8)
RGB_LOW = np.array([0, 0, 0], dtype=np.uint8)
RGB_HIGH = np.array([50, 50, 50], dtype=np.uint8)

# Captura de video desde la cámara
cap = cv2.VideoCapture(0)

while True:
    # Lee un fotograma del video
    ret, frame = cap.read()
    
    # Aplica la segmentación de color en el rango de rojo
    mask = cv2.inRange(frame, RGB_LOW, RGB_HIGH)
    
    # Muestra el resultado
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)

    # Rompe el bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera los recursos y cierra las ventanas
cap.release()
cv2.destroyAllWindows()
cv2.destroyAllWindows()
