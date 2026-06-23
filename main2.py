from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
# import plotly.graph_objects as go
import numpy as np
import math


### From here on, the actual code begins. The previous stuff was just test code. ###

class Sensor: #This class is for the cone of vision
    def __init__(self, fov_angle, vis_range):
        self.fov_angle = fov_angle
        self.vis_range = vis_range


class UAV:
    def __init__(self, x, y, z, vx, vy, vz, ax, ay, az):
        self.pos = np.array([x, y, z], dtype=float)
        self.vel = np.array([vx, vy, vz], dtype=float)
        self.accel = np.array([ax, ay, az], dtype=float)

        self.history = [self.pos.copy()]
    
    def update(self, dt):
        self.pos += self.vel * dt + 0.5 * self.accel * dt**2
        self.vel += self.accel * dt
        self.history.append(self.pos.copy())

class Guidance:
    def __init__(self, N):
        self.N = N

    def get_acceleration(self, interceptor, target):
        relPos = relative_position(interceptor, target)
        relVel = relative_velocity(interceptor, target)

        R = np.linalg.norm(relPos)

        Vc = -np.dot(relPos, relVel) / R #closing velocity
        omega = np.cross(relPos, relVel) / R**2 #rotation vector of the line of sight

        speed = np.linalg.norm(interceptor.vel)
        if speed < 1e-6:
            return np.zeros(3), R, Vc, omega, relPos, relVel
        v_hat = interceptor.vel / speed #interceptor velcocity unit vector (direction of its velocity)

        t_go = R / max(Vc, 0.1)
        self.N = 2 + 20/(t_go + 2) #time-to-go adaptivity for N

        accel_cmd = self.N * Vc * np.cross(omega, v_hat)

        a_max = 30.0 #max possible acceleration of interceptor

        a_mag = np.linalg.norm(accel_cmd)

        if a_mag > a_max:
            accel_cmd *= a_max / a_mag

        return accel_cmd, R, Vc, omega, relPos, relVel, self.N, t_go
        

def relative_position(interceptor, target):
    return np.array(target.pos - interceptor.pos)

def relative_velocity(interceptor, target):
    return np.array(target.vel - interceptor.vel)

def safe_sqrt(val):
    if val < 0:
        raise ValueError("Cannot compute sqrt of a negative number.")
    return math.sqrt(val)

def plotly_Interceptor_demo():

    #Position : m (ie all assigned position numbers are in m)
    #Velocity : m/s (ie relVel, object.vel etc are in m/s)
    #Time : s (ie dt, total_time, etc are in s)
    #omega : rad/s

    # uav_target = UAV(200, 470, 50, 
    #                  0, 0, 0, 
    #                  0, 0, 0)

    uav_target = UAV(20, 0, 50, 
                     51.389, 0, 0, 
                     0, 0, 0)
    uav_attack = UAV(0, 0, 0, 
                     60, 0, 0, 
                     0, 0, 0)
    uav_attack.Sensor = Sensor(np.radians(20), 20)

    N_value = 2.7
    PN = Guidance(N_value)

    total_time = 60 #simulated time total (how long the scenario lasts)
    playback_time = 20 #in seconds, how long the sim playback lasts
    dt = 0.05 #timestep, influences physics accuracy
    hit_radius = 5.0 #in meters, obv

    # ==========================
    # 3D SIMULATION WINDOW
    # ==========================

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])

    sim_win = gl.GLViewWidget()
    sim_win.setWindowTitle("Interceptor Simulation")
    sim_win.resize(800, 900)
    sim_win.move(0, 0)

    # Camera settings
    sim_win.opts['distance'] = 250
    sim_win.opts['elevation'] = 20
    sim_win.opts['azimuth'] = -60

    sim_win.show()
    sim_win.pan(250, 0, 50)
    # SIM WINDOW
    sim_win.resize(800, 900)
    sim_win.move(0, 0)



    # Ground grid
    grid = gl.GLGridItem()
    grid.scale(50, 50, 50)
    sim_win.addItem(grid)

    axis = gl.GLAxisItem()
    axis.setSize(200, 200, 200)
    sim_win.addItem(axis)

    # Target marker
    target_point = gl.GLScatterPlotItem(
        pos=np.array([[0.0, 0.0, 0.0]]),
        size=10,
        color=(1, 0, 0, 1)
    )
    sim_win.addItem(target_point)

    # Interceptor marker
    attack_point = gl.GLScatterPlotItem(
        pos=np.array([[0.0, 0.0, 0.0]]),
        size=10,
        color=(0, 0, 1, 1)
    )
    sim_win.addItem(attack_point)

    # Target trajectory
    target_trail = gl.GLLinePlotItem(
        pos=np.zeros((1, 3)),
        color=(1, 0, 0, 1),
        width=2,
        mode='line_strip'
    )
    sim_win.addItem(target_trail)

    # Interceptor trajectory
    attack_trail = gl.GLLinePlotItem(
        pos=np.zeros((1, 3)),
        color=(0, 0, 1, 1),
        width=2,
        mode='line_strip'
    )
    sim_win.addItem(attack_trail)

    # LOS line
    los_line = gl.GLLinePlotItem(
        pos=np.zeros((2, 3)),
        color=(0, 1, 0, 1),
        width=1,
        mode='lines'
    )
    sim_win.addItem(los_line)

    # Floating info window
    info_label = QtWidgets.QLabel()
    info_label.setWindowTitle("Simulation Data")
    info_label.resize(600, 450)
    info_label.move(20, 20)
    info_label.show()

    # ==========================
    # TELEMETRY WINDOW
    # ==========================

    tel_win = pg.GraphicsLayoutWidget(
        title="Telemetry"
    )

    tel_win.resize(1100, 900)
    # TELEMETRY WINDOW
    tel_win.resize(1100, 900)
    tel_win.move(820, 0)

    # Top row
    range_plot = tel_win.addPlot(
        row=0,
        col=0,
        title="Range"
    )

    vc_plot = tel_win.addPlot(
        row=0,
        col=1,
        title="Closing Velocity"
    )

    accel_plot = tel_win.addPlot(
        row=0,
        col=2,
        title="Acceleration Command"
    )

    # Bottom row
    omega_plot = tel_win.addPlot(
        row=1,
        col=0,
        title="LOS Rotation Rate"
    )

    speed_plot = tel_win.addPlot(
        row=1,
        col=1,
        title="Interceptor Speed"
    )

    tgo_plot = tel_win.addPlot(
        row=1,
        col=2,
        title="Time To Go"
    )

    tel_win.show()

    range_plot.setLabel('left', 'Range (m)')
    range_plot.setLabel('bottom', 'Time (s)')

    vc_plot.setLabel('left', 'Vc (m/s)')
    vc_plot.setLabel('bottom', 'Time (s)')

    accel_plot.setLabel('left', 'Accel (m/s²)')
    accel_plot.setLabel('bottom', 'Time (s)')

    omega_plot.setLabel('left', 'Omega (rad/s)')
    omega_plot.setLabel('bottom', 'Time (s)')

    speed_plot.setLabel('left', 'Speed (m/s)')
    speed_plot.setLabel('bottom', 'Time (s)')

    tgo_plot.setLabel('left', 't_go (s)')
    tgo_plot.setLabel('bottom', 'Time (s)')

    for p in [
        range_plot,
        vc_plot,
        accel_plot,
        omega_plot,
        speed_plot,
        tgo_plot
    ]:
        p.showGrid(x=True, y=True)
    ###########

    range_history = []
    Vc_history = []
    time_history = []
    accel_history = []
    omega_history = []
    int_speed_history = []
    t_go_history =[]

    range_curve = range_plot.plot(pen='y')
    vc_curve = vc_plot.plot(pen='c')
    accel_curve = accel_plot.plot(pen='m')

    omega_curve = omega_plot.plot(pen='g')
    speed_curve = speed_plot.plot(pen='b')
    tgo_curve = tgo_plot.plot(pen='r')


    def update(frame):

        # uav_target.vel = np.array([
        #     1.0,
        #     np.sin(frame),
        #     np.cos(frame)
        # ])

        # uav_attack.vel = np.array([
        #     1,
        #     0,
        #     0
        # ])

        uav_target.update(dt)
        uav_attack.accel, R, Vc, omega, relPos, relVel, N_value, t_go = PN.get_acceleration(uav_attack, uav_target) 
        uav_attack.update(dt)

        time_history.append(frame * dt)
        range_history.append(R)
        Vc_history.append(Vc)
        accel_history.append(np.linalg.norm(uav_attack.accel))
        omega_history.append(np.linalg.norm(omega))
        int_speed_history.append(np.linalg.norm(uav_attack.vel))
        t_go_history.append(t_go)

        #target UAV
        target_point.setData(
            pos=np.array([uav_target.pos]),
            color=(1, 0, 0, 1),
            size=10
        )

        #one way attack
        attack_point.setData(
            pos=np.array([uav_attack.pos]),
            color=(0, 0, 1, 1),
            size=10
        )

        history_target = np.array(uav_target.history)
        history_attack = np.array(uav_attack.history)

        #target trail
        target_trail.setData(
            pos=history_target
        )
        #attack trail
        attack_trail.setData(
            pos=history_attack
        )

        los_line.setData(
            pos=np.array([
                uav_attack.pos,
                uav_target.pos
            ])
        )

        info_label.setText(
            f"TARGET\n"
            f"Pos (m): ({uav_target.pos[0]:.2f}, {uav_target.pos[1]:.2f}, {uav_target.pos[2]:.2f})\n"
            f"Vel (m/s): ({uav_target.vel[0]:.2f}, {uav_target.vel[1]:.2f}, {uav_target.vel[2]:.2f})\n"
            f"Accel (m/s2): ({uav_target.accel[0]:.2f}, {uav_target.accel[1]:.2f}, {uav_target.accel[2]:.2f})\n\n"
            f"INTERCEPTOR\n"
            f"Pos (m): ({uav_attack.pos[0]:.2f}, {uav_attack.pos[1]:.2f}, {uav_attack.pos[2]:.2f})\n"
            f"Vel (m/s): ({uav_attack.vel[0]:.2f}, {uav_attack.vel[1]:.2f}, {uav_attack.vel[2]:.2f})\n"
            f"Accel (m/s2): ({uav_attack.accel[0]:.2f}, {uav_attack.accel[1]:.2f}, {uav_attack.accel[2]:.2f})\n\n"
            f"Relative Position (m): ({relPos[0]:.2f}, {relPos[1]:.2f}, {relPos[2]:.2f})\n"
            f"Relative Velocity (m/s): ({relVel[0]:.2f}, {relVel[1]:.2f}, {relVel[2]:.2f})\n"
            f"Distance b/w Target and Interceptor (m) : {R:.2f}\n"
            f"Time to go (s) : {t_go:.2f}\n"
            f"Distance of closest approach (m) : {min(range_history):.2f}\n"
            f"N value for accel_cmd in PN : {N_value:.2f}"
        )

        if R <= hit_radius:
            info_label.setText(
                f"Intercept ACHIEVED!\n\n"
                f"Time : {frame * dt:.2f} s\n"
                f"Range : {R:.2f} m"
            )
            timer.stop()
            return
        if frame % 5 == 0: 
            range_curve.setData(
                time_history,
                range_history
            )

            vc_curve.setData(
                time_history,
                Vc_history
            )

            accel_curve.setData(
                time_history,
                accel_history
            )

            omega_curve.setData(
                time_history,
                omega_history
            )

            speed_curve.setData(
                time_history,
                int_speed_history
            )

            tgo_curve.setData(
                time_history,
                t_go_history
            )

        # INFO WINDOW
        info_label.resize(600, 450)
        info_label.move(1940, 0)


    frame_counter = {"value": 0}

    timer = QtCore.QTimer()

    def timer_update():
        update(frame_counter["value"])

        if frame_counter["value"] % 50 == 0:
            print(frame_counter["value"])

        frame_counter["value"] += 1

        if frame_counter["value"] >= int(total_time / dt):
            timer.stop()

    timer.timeout.connect(timer_update)
    frame_count = int(total_time / dt)

    interval_ms = int(
        playback_time * 1000 / frame_count
    )

    timer.start(interval_ms)

    # point_target, = ax_sim.plot([], [], [], 'ro', label='Target')
    # point_attack, = ax_sim.plot([], [], [], 'bo', label='Interceptor')

    # ax_sim.legend()

    app.exec()


if __name__ == "__main__":
    try:
        plotly_Interceptor_demo()
    except Exception as e:
        print(e)
