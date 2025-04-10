import numpy as np
import matplotlib as plt

class TrajectoryGenerator:
    def __init__(self, N=100):
        self.div = round(N / 4)
        self.pointX = [0, 1.0, 1.0, 0, 0]
        self.pointY = [0, 0, 1.0, 1.0, 0]
        self.pointYaw = np.radians([0.1, 90, 179, -90, -0.1])

        self.px = []
        self.py = []
        self.pyaw = []
        self.generate_trajectory()

    def generate_trajectory(self):
        for p in range(len(self.pointX) - 1):
            self.px.extend(np.linspace(self.pointX[p], self.pointX[p + 1], self.div))
            self.py.extend(np.linspace(self.pointY[p], self.pointY[p + 1], self.div))
            self.pyaw.extend(np.linspace(self.pointYaw[p], self.pointYaw[p + 1], self.div))

    def get_trajectory(self):
        return self.px, self.py, self.pyaw
    

trajectory = TrajectoryGenerator(N=200)
waypoints_x, waypoints_y, waypoints_yaw = trajectory.get_trajectory()

# Gráfico 1: Trayectoria (x, y)
# plt.figure(figsize=(10, 5))
plt.plot(waypoints_x, waypoints_y, marker='o', linestyle='-', color='b', label='Trayectoria')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Trayectoria en el Plano XY')
plt.grid(True)
plt.legend()
plt.show()

# Gráfico 2: Yaw en función del tiempo (índice)
# plt.figure(figsize=(10, 5))
plt.plot(range(len(waypoints_yaw), waypoints_yaw, marker='x', linestyle='-', color='r', label='Yaw'))
plt.xlabel('Índice de Tiempo')
plt.ylabel('Yaw [rad]')
plt.title('Orientación Yaw')
plt.grid(True)
plt.legend()
plt.show()




print(len(waypoints_yaw))