import numpy as np
import matplotlib.pyplot as plt

class RobotArm2D:
    def __init__(self, L1, L2):
        self.L1 = L1
        self.L2 = L2
        self.DOF = 2
    
    def create_htm(self, theta, length):
        theta_rad = np.radians(theta)
        c = np.cos(theta_rad)
        s = np.sin(theta_rad)
        return np.array([
            [c, -s, 0, length * c],
            [s, c, 0, length * s],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, theta1, theta2):
        T1 = self.create_htm(theta1, self.L1)
        T2 = self.create_htm(theta2, self.L2)
        T_total = T1 @ T2
        
        joint1 = (T1[0, 3], T1[1, 3])
        end = (T_total[0, 3], T_total[1, 3])
        return joint1, end
    
    def inverse_kinematics(self, x, y):
        d = np.sqrt(x**2 + y**2)
        
        if d > self.L1 + self.L2 or d < abs(self.L1 - self.L2):
            print("tidak bisa dijangkau")
            return None
        
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = np.clip(cos_theta2, -1, 1)
        theta2 = np.arccos(cos_theta2)
        
        alpha = np.arctan2(y, x)
        beta = np.arctan2(self.L2 * np.sin(theta2), self.L1 + self.L2 * np.cos(theta2))
        theta1 = alpha - beta
        
        return np.degrees(theta1), np.degrees(theta2)
    
    def plot(self, theta1, theta2):
        joint1, end = self.forward_kinematics(theta1, theta2)
        
        plt.figure(figsize=(10, 8))
        plt.plot([0, joint1[0]], [0, joint1[1]], 'r-', linewidth=6, marker='o', markersize=10)
        plt.plot([joint1[0], end[0]], [joint1[1], end[1]], 'g-', linewidth=6, marker='o', markersize=10)
        plt.plot(end[0], end[1], 'bo', markersize=12)
        
        plt.text(0, 0, '  Base', fontsize=10)
        plt.text(joint1[0], joint1[1], f'  Joint 1\n  ({joint1[0]:.2f}, {joint1[1]:.2f})', fontsize=9)
        plt.text(end[0], end[1], f'  End Effector\n  ({end[0]:.2f}, {end[1]:.2f})', fontsize=9)
        
        info = f'DoF: {self.DOF}\nθ1={theta1}°, θ2={theta2}°\nL1={self.L1}, L2={self.L2}'
        plt.text(0.02, 0.98, info, transform=plt.gca().transAxes, fontsize=10,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        max_range = self.L1 + self.L2
        plt.xlim(-5, max_range + 10)
        plt.ylim(-5, max_range + 10)
        plt.grid(True, alpha=0.3)
        plt.axhline(0, color='k', linewidth=0.5)
        plt.axvline(0, color='k', linewidth=0.5)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('2-DOF Robot Arm')
        plt.axis('equal')
        plt.tight_layout()
        plt.show()


# Data
L1 = 28  # Femur  
L2 = 40  # Tibia    
theta1 = 40
theta2 = 30

robot = RobotArm2D(L1, L2)

print(f"L1={L1}, L2={L2}, DoF={robot.DOF}")
print(f"Diketahui: θ1={theta1}°, θ2={theta2}°\n")

mode = input("FK/IK? ").upper()

if mode == "FK":
    joint1, end = robot.forward_kinematics(theta1, theta2)
    print(f"End Effector: ({end[0]:.2f}, {end[1]:.2f})")
    robot.plot(theta1, theta2)

elif mode == "IK":
    x = float(input("Target X: "))
    y = float(input("Target Y: "))
    
    result = robot.inverse_kinematics(x, y)
    if result:
        t1, t2 = result
        print(f"θ1={t1:.2f}°, θ2={t2:.2f}°")
        robot.plot(t1, t2)

else:
    print("Input tidak valid")