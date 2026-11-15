"""Native velocity and steering controls for the physical Gazebo hatchback."""
import argparse
import math
import signal
import threading

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist

WHEELBASE = 2.5165
STEERING_ANGLE = 0.45


class Car:
    def __init__(self, world="shapes", model="hatchback_blue"):
        self.node = Node()
        self.world, self.model = world, model
        self.publisher = self.node.advertise(f"/model/{model}/cmd_vel", Twist)
        self.pose = Pose()

    def connected(self):
        return self.publisher.has_connections()

    def command(self, speed=0.0, steering=0.0):
        message = Twist()
        message.linear.x = speed
        # Ackermann plugin accepts yaw velocity, not steering angle.
        message.angular.z = speed * math.tan(steering) / WHEELBASE
        if not self.publisher.publish(message):
            raise RuntimeError("Could not send the car command.")

    def refresh(self):
        ready = threading.Event()
        pose = Pose()

        def receive(message):
            for item in message.pose:
                if item.name == self.model:
                    pose.CopyFrom(item)
                    ready.set()
                    break

        topic = f"/world/{self.world}/pose/info"
        self.node.subscribe(Pose_V, topic, receive)
        received = ready.wait(3)
        self.node.unsubscribe(topic)
        if not received:
            raise RuntimeError(f"No live pose for {self.model}; check Gazebo.")
        self.pose.CopyFrom(pose)
        return self.pose


def run_window(world, model):
    import tkinter as tk

    car = Car(world, model)
    root = tk.Tk()
    root.title("Gazebo Car Controls")
    root.geometry("390x450+1100+180")
    root.resizable(False, False)
    root.configure(bg="#182231")
    root.attributes("-topmost", True)
    status = tk.StringVar(value="Connecting to the car...")
    speed = tk.DoubleVar(value=1.0)
    held = set()
    releases = {}
    mouse_action = [None]

    def stop(event=None):
        held.clear()
        mouse_action[0] = None
        for timer in releases.values():
            root.after_cancel(timer)
        releases.clear()
        car.command()
        status.set("Stopped")

    def press(key):
        if key in releases:
            root.after_cancel(releases.pop(key))
        held.add(key)

    def release(key):
        # X11 key repeat emits release/press pairs; defer the release briefly.
        def finish():
            held.discard(key)
            releases.pop(key, None)
        if key in releases:
            root.after_cancel(releases.pop(key))
        releases[key] = root.after(25, finish)

    def tick():
        actions = held | ({mouse_action[0]} if mouse_action[0] else set())
        steering = (int('Left' in actions) - int('Right' in actions)) * STEERING_ANGLE
        direction = int('Up' in actions) - int('Down' in actions)
        if steering and not ({'Up', 'Down'} & actions):
            direction = 1
        try:
            if car.connected():
                car.command(direction * speed.get(), steering)
                if direction:
                    status.set(('Forward' if direction > 0 else 'Reverse') +
                               (' · left' if steering > 0 else ' · right' if steering < 0 else ''))
                else:
                    status.set("Stopped · connected to Gazebo")
            else:
                held.clear()
                mouse_action[0] = None
                status.set("Waiting for the drivable car in Gazebo...")
        except Exception as exc:
            held.clear()
            mouse_action[0] = None
            status.set(str(exc))
        root.after(50, tick)

    def close():
        stop()
        root.after(100, root.destroy)

    tk.Label(root, text="CAR CONTROLS", font=("Sans", 17, "bold"),
             bg="#182231", fg="white").pack(pady=(16, 4))
    tk.Label(root, text="Hold buttons or arrow keys · release to stop",
             bg="#182231", fg="#c5d0df").pack(pady=(0, 12))
    grid = tk.Frame(root, bg="#182231")
    grid.pack(fill="x", padx=24)
    grid.columnconfigure(0, weight=1)
    grid.columnconfigure(1, weight=1)

    def button(label, action, row, column, span=1):
        item = tk.Button(grid, text=label, font=("Sans", 12, "bold"),
                         bg="#2d72b8", fg="white", activebackground="#438cce",
                         activeforeground="white", relief="flat", height=2)
        item.grid(row=row, column=column, columnspan=span, sticky="ew", padx=3, pady=4)
        item.bind('<ButtonPress-1>', lambda event: mouse_action.__setitem__(0, action))
        item.bind('<ButtonRelease-1>', lambda event: mouse_action.__setitem__(0, None))
        item.bind('<Leave>', lambda event: mouse_action.__setitem__(0, None))

    button("▲  FORWARD", 'Up', 0, 0, 2)
    button("◀  LEFT", 'Left', 1, 0)
    button("RIGHT  ▶", 'Right', 1, 1)
    button("▼  BACKWARD", 'Down', 2, 0, 2)
    tk.Button(root, text="■  STOP  (Space)", command=stop,
              font=("Sans", 12, "bold"), bg="#ba4141", fg="white",
              relief="flat").pack(fill="x", padx=27, pady=6)
    tk.Scale(root, from_=0.2, to=2.0, resolution=0.1, variable=speed,
             orient="horizontal", label="Speed (m/s)", bg="#182231",
             fg="white", highlightthickness=0, troughcolor="#43546b"
             ).pack(fill="x", padx=27)
    tk.Label(root, textvariable=status, wraplength=355,
             bg="#182231", fg="#94e0bc").pack(pady=5)
    tk.Label(root, text="Left / Right drive in an arc. Down + Left / Right reverse-turn.\n"
             "Physical wheels and body collisions enabled.", font=("Sans", 8),
             bg="#182231", fg="#c5d0df").pack()
    for key in ('Up','Down','Left','Right'):
        root.bind('<KeyPress-'+key+'>', lambda event, k=key: press(k))
        root.bind('<KeyRelease-'+key+'>', lambda event, k=key: release(k))
    root.bind('<space>', stop)
    root.bind('<Escape>', stop)
    root.bind('<FocusOut>', stop)
    root.protocol('WM_DELETE_WINDOW', close)
    signal.signal(signal.SIGTERM, lambda *_: close())
    signal.signal(signal.SIGINT, lambda *_: close())
    root.after(50, tick)
    try:
        root.mainloop()
    finally:
        car.command()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='shapes')
    parser.add_argument('--model', default='hatchback_blue')
    args = parser.parse_args()
    run_window(args.world, args.model)
