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



class UAV:
    def __init__(self, x, y, z, vx, vy, vz):
        self.pos = np.array([x, y, z], dtype=float)
        self.vel = np.array([vx, vy, vz], dtype=float)

        self.history = [self.pos.copy()]
    
    def update(self, dt):
        self.pos += self.vel * dt
        self.history.append(self.pos.copy())



def plotly_Interceptor_demo():

    uav = UAV(0, 0, 1, 0, 0, 0)

    dt = 1

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim(0, 50)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-5, 5)

    point, = ax.plot([], [], [], 'ro', markersize=8)
    trail, = ax.plot([], [], [], 'r-', linewidth=2)


    def update(frame):

        uav.vel = np.array([
            1,
            np.sin(frame),
            np.cos(frame)
        ])

        uav.update(dt)

        point.set_data([uav.pos[0]], [uav.pos[1]])
        point.set_3d_properties([uav.pos[2]])

        history = np.array(uav.history)

        trail.set_data(history[:, 0], history[:, 1])
        trail.set_3d_properties(history[:, 2])

        return point, trail

    ani = anim.FuncAnimation(
        fig,
        update,
        frames=range(50),
        interval=50,
        blit=False,
        repeat=False
    )

    plt.show()


if __name__ == "__main__":
    plotly_Interceptor_demo()
