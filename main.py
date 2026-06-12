import matplotlib.pyplot as plt
import matplotlib.animation as anim
import plotly.graph_objects as go
import numpy as np

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
        self.history.append(self.pos.copy())

def relative_position(interceptor, target):
    return np.array(target.pos - interceptor.pos)

def relative_velocity(interceptor, target):
    return np.array(target.vel - interceptor.vel)

def plotly_Interceptor_demo():

    uav_target = UAV(0, 0, 1, 
               1, 0, 0, 
               0, 0, 0)
    uav_attack = UAV(0, 0, -5, 
                     1, 0, 0, 
                     0, 0, 0)
    uav_attack.Sensor = Sensor(np.radians(20), 20)

    dt = 1

    fig = plt.figure(figsize=(14,8))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim(0, 100)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)

    point_target, = ax.plot([], [], [], 'ro', markersize=8)
    trail_target, = ax.plot([], [], [], 'r-', linewidth=2)

    point_attack, = ax.plot([], [], [], 'bo', markersize=8)
    trail_attack, = ax.plot([], [], [], 'b-', linewidth=2)

    info_text = ax.text2D(
        -0.5,
        0.6,
        "",
        transform=ax.transAxes,
        bbox=dict(
        boxstyle="round",
        facecolor="white",
        alpha=0.6
        )
    )


    def update(frame):

        uav_target.vel = np.array([
            1,
            np.sin(frame),
            np.cos(frame)
        ])

        uav_attack.vel = np.array([
            0,
            0,
            0
        ])

        uav_target.update(dt)
        uav_attack.update(dt)

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

        rel_pos = relative_position(uav_attack, uav_target)
        rel_vel = relative_velocity(uav_attack, uav_target)

        info_text.set_text(
            f"TARGET\n"
            f"Pos: ({uav_target.pos[0]:.2f}, {uav_target.pos[1]:.2f}, {uav_target.pos[2]:.2f})\n"
            f"Vel: ({uav_target.vel[0]:.2f}, {uav_target.vel[1]:.2f}, {uav_target.vel[2]:.2f})\n"
            f"Accel: ({uav_target.accel[0]:.2f}, {uav_target.accel[1]:.2f}, {uav_target.accel[2]:.2f})\n\n"
            f"INTERCEPTOR\n"
            f"Pos: ({uav_attack.pos[0]:.2f}, {uav_attack.pos[1]:.2f}, {uav_attack.pos[2]:.2f})\n"
            f"Vel: ({uav_attack.vel[0]:.2f}, {uav_attack.vel[1]:.2f}, {uav_attack.vel[2]:.2f})\n"
            f"Accel: ({uav_attack.accel[0]:.2f}, {uav_attack.accel[1]:.2f}, {uav_attack.accel[2]:.2f})\n\n"
            f"Relative Position: ({rel_pos[0]:.2f}, {rel_pos[1]:.2f}, {rel_pos[2]:.2f})\n"
            f"Relative Velocity: ({rel_vel[0]:.2f}, {rel_vel[1]:.2f}, {rel_vel[2]:.2f})"
        )

        return (point_target,
                trail_target,
                point_attack,
                trail_attack,
                info_text)

    ani = anim.FuncAnimation(
        fig,
        update,
        frames=range(100),
        interval=50,
        blit=False,
        repeat=False
    )

    # point_target, = ax.plot([], [], [], 'ro', label='Target')
    # point_attack, = ax.plot([], [], [], 'bo', label='Interceptor')

    # ax.legend()

    plt.show()


if __name__ == "__main__":
    plotly_Interceptor_demo()
