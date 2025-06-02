import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import imageio
from matplotlib.animation import PillowWriter
import matplotlib as mpl
mpl.rcParams['animation.ffmpeg_path'] = 'C:/FFmpeg/bin/ffmpeg.exe'
# Define the list of quaternions (w, x, y, z)
def draw_quaternion(quaternions, name="quaternion_rotation"):
    # quaternions = [
    #     [1, 0, 0, 0],
    #     [0.9239, 0.3827, 0, 0],   # 45 deg around x
    #     [0.7071, 0.7071, 0, 0],   # 90 deg around x
    #     [0.7071, 0, 0.7071, 0],   # 90 deg around y
    #     [0.7071, 0, 0, 0.7071],   # 90 deg around z
    # ]

    # Convert quaternions to scipy Rotation objects
    rotations = [R.from_quat([x, y, z, w]) for (w, x, y, z) in quaternions]

    # Arrow definition: one pointing along +X
    base_vector = np.array([1.0, 0.0, 0.0])

    # Create figure and 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Draw reference axes
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', linestyle='--', length=1)
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', linestyle='--', length=1)
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', linestyle='--', length=1)

    # Animated arrow
    arrow = ax.quiver(0, 0, 0, 1, 0, 0, color='k', linewidth=3)
    endpoint_text = ax.text(0, 0, 0, "", color='purple')

    from tqdm import tqdm

    print("Creating animation...")
    pbar = tqdm(total=len(rotations), desc="Animating", unit="frame")

    def update(frame):
        nonlocal arrow, endpoint_text, pbar
        arrow.remove()
        endpoint_text.remove()
        pbar.update(1)
        vec = rotations[frame].apply(base_vector)
        arrow = ax.quiver(0, 0, 0, vec[0], vec[1], vec[2], color='k', linewidth=3)
        # Display the current arrow end point
        endpoint_text = ax.text(
            vec[0], vec[1], vec[2],
            f"({vec[0]:.2f}, {vec[1]:.2f}, {vec[2]:.2f})",
            color='purple'
        )


    ani = FuncAnimation(fig, update, frames=len(rotations), interval=1)

    # Save the animation as a video
    print("Saving animation...")
    ani.save("quaternion_rotation.gif", writer=PillowWriter(fps=30))
    writer = FFMpegWriter(fps=60)
    ani.save(f"{name}.mp4", writer=writer)
    # ani.save(f"{name}.gif", writer='pillow')
    print(f"Animation saved as {name}.gif")
    plt.close()
