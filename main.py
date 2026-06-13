import matplotlib.pyplot as plt
import matplotlib.animation as anim
import plotly.graph_objects as go
import numpy as np
import math

def main():
    print("Hello from interceptor-study!")

def plt3D_plot_test():
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    z = np.linspace(0, 15, 100)
    y = np.sin(z)
    x = np.cos(z)

    ax.plot3D(x, y, z, 'gray', label = '3D Spiral Path')
    ax.scatter3D(x, y, z, c=z, cmap='viridis', s=20, label='Data Points')

    plt.show()


def plotly3D_plot_test():
    z = np.linspace(0, 15, 100)
    y = np.sin(z)
    x = np.cos(z)

    fig = go.Figure()

    # Spiral line
    fig.add_trace(
        go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode='lines',
            line=dict(color='gray', width=4),
            name='3D Spiral Path'
        )
    )

    # Colored points
    fig.add_trace(
        go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode='markers',
            marker=dict(
                size=4,
                color=z,
                colorscale='Viridis',
                colorbar=dict(title='z'),
            ),
            name='Data Points'
        )
    )

    fig.update_layout(
        title='3D Spiral',
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        ),
        width=800,
        height=600
    )

    fig.show()

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

        accel_cmd = self.N * Vc * np.cross(omega, v_hat)

        return accel_cmd, R, Vc, omega, relPos, relVel
        

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

    uav_target = UAV(200, 100, 0, 
                     0, 0, 0, 
                     0, 0, 0)
    uav_attack = UAV(0, 0, 0, 
                     40, 0, 0, 
                     0, 0, 0)
    uav_attack.Sensor = Sensor(np.radians(20), 20)

    N_value = 3
    PN = Guidance(N_value)

    total_time = 20 #simulated time total (how long the scenario lasts)
    dt = 0.05 #timestep, influences physics accuracy
    hit_radius = 5.0 #in meters, obv

    #MAIN INTERCEPTOR-TARGET ANIM PLOT
    fig_sim = plt.figure(figsize=(14,8))
    ax_sim = fig_sim.add_subplot(111, projection='3d')

    ax_sim.set_xlim(0, 500)
    ax_sim.set_ylim(-200, 200)
    ax_sim.set_zlim(-50, 200)

    ax_sim.set_box_aspect([500, 400, 250]) #When I run the code, this defines how the "zoom" is by default

    ax_sim.view_init(
        elev=20,   # vertical angle
        azim=-60   # horizontal angle
    )
    ###

    #TELEMETRY PLOT WINDOW
    fig_tel = plt.figure(figsize=(12, 8))
    fig_tel.tight_layout()

    ax_range = fig_tel.add_subplot(231)
    ax_vc    = fig_tel.add_subplot(232)
    ax_accel = fig_tel.add_subplot(233)
    ax_omega = fig_tel.add_subplot(234)
    ax_int_speed = fig_tel.add_subplot(235)

    range_history = []
    Vc_history = []
    time_history = []
    accel_history = []
    omega_history = []
    int_speed_history = []

    range_line, = ax_range.plot([], [])
    Vc_line,    = ax_vc.plot([], [])
    accel_line, = ax_accel.plot([], [])
    omega_line, = ax_omega.plot([], [])
    int_speed_line, = ax_int_speed.plot([], [])

    ax_range.set_title("Range")
    ax_range.set_ylabel("m")
    ax_range.grid(True)

    ax_vc.set_title("Closing Velocity")
    ax_vc.set_ylabel("m/s")
    ax_vc.grid(True)

    ax_accel.set_title("Acceleration Command")
    ax_accel.set_ylabel("m/s2")
    ax_accel.set_xlabel("Time (s)")
    ax_accel.grid(True)

    ax_omega.set_title("Rate of LOS rotation")
    ax_omega.set_ylabel("rad/s")
    ax_omega.set_xlabel("Time (s)")
    ax_omega.grid(True)

    ax_int_speed.set_title("Interceptor Speed")
    ax_int_speed.set_ylabel("m/s")
    ax_int_speed.set_xlabel("Time (s)")
    ax_int_speed.grid(True)
    ###

    #POSITION AND SIZE OF BOTH PLOTS ON THE SCREEN
    fig_sim.canvas.manager.window.wm_geometry("800x900+0+0")
    fig_tel.canvas.manager.window.wm_geometry("1100x900+785+0")
    ###

    point_target, = ax_sim.plot([], [], [], 'ro', markersize=8)
    trail_target, = ax_sim.plot([], [], [], 'r-', linewidth=2)

    point_attack, = ax_sim.plot([], [], [], 'bo', markersize=8)
    trail_attack, = ax_sim.plot([], [], [], 'b-', linewidth=2)

    info_text = ax_sim.text2D(
        -0.1,
        0.8,
        "",
        transform=ax_sim.transAxes,
        bbox=dict(
        boxstyle="round",
        facecolor="white",
        alpha=0.6
        )
    )

    los_line, = ax_sim.plot([], [], [], 'g--', linewidth=1.5)

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
        uav_attack.accel, R, Vc, omega, relPos, relVel = PN.get_acceleration(uav_attack, uav_target) 
        uav_attack.update(dt)

        time_history.append(frame * dt)
        range_history.append(R)
        Vc_history.append(Vc)
        accel_history.append(np.linalg.norm(uav_attack.accel))
        omega_history.append(np.linalg.norm(omega))
        int_speed_history.append(np.linalg.norm(uav_attack.vel))

        #target UAV
        point_target.set_data([uav_target.pos[0]], [uav_target.pos[1]])
        point_target.set_3d_properties([uav_target.pos[2]])
        #one way attack
        point_attack.set_data([uav_attack.pos[0]], [uav_attack.pos[1]])
        point_attack.set_3d_properties([uav_attack.pos[2]])

        history_target = np.array(uav_target.history)
        history_attack = np.array(uav_attack.history)

        #target trail
        trail_target.set_data(history_target[:, 0], history_target[:, 1])
        trail_target.set_3d_properties(history_target[:, 2])
        #attack trail
        trail_attack.set_data(history_attack[:, 0], history_attack[:, 1])
        trail_attack.set_3d_properties(history_attack[:, 2])

        los_line.set_data(
            [uav_attack.pos[0], uav_target.pos[0]],
            [uav_attack.pos[1], uav_target.pos[1]]
        )

        los_line.set_3d_properties(
            [uav_attack.pos[2], uav_target.pos[2]]
        )

        info_text.set_text(
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
            f"N value for accel_cmd in PN : {N_value}"
        )

        #TELEMETRY PLOT UPDATES
        range_line.set_data(time_history, range_history)
        Vc_line.set_data(time_history, Vc_history)
        accel_line.set_data(time_history, accel_history)
        omega_line.set_data(time_history, omega_history)
        int_speed_line.set_data(time_history, int_speed_history)

        for ax in [ax_range, ax_vc, ax_accel, ax_omega, ax_int_speed]:
            ax.relim()
            ax.autoscale_view()

        fig_tel.canvas.draw_idle()
        ###

        if R <= hit_radius:
            info_text.set_text(
                f"Intercept ACHIEVED!\n"
                f"Time : {frame * dt :.2f} s\n"
                f"Range : {R :.2f} m"
            )
            ani.event_source.stop()

        return (point_target,
                trail_target,
                point_attack,
                trail_attack,
                info_text,
                los_line,
                range_line,
                Vc_line,
                accel_line,
                omega_line,
                int_speed_line)

    ani = anim.FuncAnimation(
        fig_sim,
        update,
        frames=int(total_time / dt),
        interval=20,
        blit=False,
        repeat=False
    )

    # point_target, = ax_sim.plot([], [], [], 'ro', label='Target')
    # point_attack, = ax_sim.plot([], [], [], 'bo', label='Interceptor')

    # ax_sim.legend()

    plt.show()


if __name__ == "__main__":
    plotly_Interceptor_demo()
