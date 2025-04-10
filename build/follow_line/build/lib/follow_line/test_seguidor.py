import cv2

captura = cv2.VideoCapture(2)

while (captura.isOpened()):
  ret, imagen = captura.read()
  print(imagen.shape)
  if ret == True:
    cv2.imshow('video', imagen)
    if cv2.waitKey(1) & 0xFF == ord('s'):
      break
  else: break

captura.release()
cv2.destroyAllWindows()