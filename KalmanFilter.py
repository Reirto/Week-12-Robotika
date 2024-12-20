from controller import Robot

TIME_STEP = 32

# Fungsi Kalman Filter
def kalman_filter(z, u, x, P):
    # Prediksi langkah
    x_pred = x + u
    P_pred = P + 0.1  # Noise proses

    # Koreksi langkah
    K = P_pred / (P_pred + 1)  # Gain Kalman
    x = x_pred + K * (z - x_pred)  # Pembaruan posisi
    P = (1 - K) * P_pred  # Pembaruan ketidakpastian
    return x, P

# Inisialisasi robot
robot = Robot()

# Motor roda
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  # Mode kecepatan
right_motor.setPosition(float('inf'))  # Mode kecepatan
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Encoder roda
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# Sensor jarak (proximity sensors)
proximity_sensors = []
for i in range(8):  # e-puck typically has 8 proximity sensors
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(TIME_STEP)
    proximity_sensors.append(sensor)

# Variabel untuk Kalman Filter
x = 0.0  # Posisi awal
P = 1.0  # Ketidakpastian awal

# Loop utama
while robot.step(TIME_STEP) != -1:
    # Ambil nilai encoder
    left_distance = left_encoder.getValue()
    right_distance = right_encoder.getValue()

    # Estimasi pergerakan robot (input u)
    u = (left_distance + right_distance) / 2.0

    # Ambil pengukuran dari sensor jarak (z)
    # Menggunakan rata-rata pembacaan sensor jarak
    z = sum(sensor.getValue() for sensor in proximity_sensors) / len(proximity_sensors)

    # Terapkan Kalman Filter
    x, P = kalman_filter(z, u, x, P)

    print(f"Estimasi Posisi Robot: {x}")
