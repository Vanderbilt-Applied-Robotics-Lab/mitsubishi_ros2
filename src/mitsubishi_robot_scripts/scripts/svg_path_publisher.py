#!/usr/bin/env python3

# ROS node/script that prompts the user for a .svg file and some processing
# settings. Simplifies the .svg paths and converts them to (x,y) coordinate
# paths that are thereafter published to /svg_path for use by the separate
# C++ drawing node
#
# Author: Joshua Holden Turner

# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# SVG
from svg.path import parse_path
from xml.dom import minidom
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
from pathlib import Path
from scipy.spatial import KDTree
from tqdm import tqdm

# GUI
import threading
from tkinter import Tk, filedialog, Label, Scale, HORIZONTAL, Button, StringVar, DoubleVar


@dataclass
class Point:
    x: float
    y: float

    def dist(self, p: 'Point'):
        return np.sqrt((self.x - p.x)**2 + (self.y - p.y)**2)

    def __str__(self):
        return f"({self.x}, {self.y})"


class SvgPathPublisher(Node):
    def __init__(self):
        super().__init__("svg_path_publisher")
        self.pub = self.create_publisher(
            Float32MultiArray, '/svg_path', 10
        )
        self.make_gui()
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        self.tk.mainloop()

    def make_gui(self):
        self.tk = Tk()
        self.tk.title("SVG Path Generator")

        self.fname = StringVar()
        self.fname.set("No SVG selected")
        Button(self.tk, text="Select SVG", command=self.pick_image).pack(pady=8)
        Label(self.tk, textvariable=self.fname).pack()

        self.image_size = DoubleVar()
        self.image_size.set(200.0)
        Label(self.tk, text="Image size diagonal (mm):").pack()
        Scale(self.tk, variable=self.image_size, from_=5.0, to=2000, resolution=0.1, orient=HORIZONTAL).pack(fill='x')

        self.sampling_step = DoubleVar()
        self.sampling_step.set(0.5)
        Label(self.tk, text="Sampling step (px):").pack()
        Scale(self.tk, variable=self.sampling_step, from_=0.1, to=5.0, resolution=0.1, orient=HORIZONTAL).pack(fill='x')

        self.merge_step = DoubleVar()
        self.merge_step.set(0.01)
        Label(self.tk, text="Merge radius (mm):").pack()
        Scale(self.tk, variable=self.merge_step, from_=0.0, to=1.0, resolution=0.001, orient=HORIZONTAL).pack(fill='x')

        Button(self.tk, text="Process", command=self.process).pack(pady=8)
        Button(self.tk, text="Publish", command=self.publish).pack(pady=8)

    def pick_image(self):
        self.fname.set(filedialog.askopenfilename(filetypes=[("SVG files", "*.svg")]))

    def publish(self):
        if self.paths is None:
            return
        msg = self.paths_to_multiarray(self.paths)
        self.pub.publish(msg)
        self.get_logger().info(f"Published {len(msg.data)} points")

    def process(self):
        if self.fname.get() == "No SVG selected":
            return
        self.paths = self.generate_paths(self.fname.get(), self.image_size.get(), self.sampling_step.get(), self.merge_step.get())

    def generate_paths(self, fname: str, image_size: float, sampling_step: float, merge_step: float) -> List[List[Point]]:

        # Load image
        self.get_logger().info("Converting svg paths to points")
        paths = self.paths_from_svg(fname, sampling_step)

        # Normalize
        self.get_logger().info("Resizing points")
        paths, image_size_x, image_size_y = self.normalize_paths(
            paths, image_size)

        # Merge
        self.get_logger().info("Merging paths")
        paths = self.merge_paths(paths, merge_step)

        # Show image
        self.plot_paths(paths)

        # Image data
        self.get_logger().info(f"Image name       : {fname}")
        self.get_logger().info(f"Image size       : {1000*image_size_x:.2f}mm x {1000*image_size_y:.2f}mm")
        self.get_logger().info(f"Number of points : {sum([len(p) for p in paths])}")
        self.get_logger().info(f"Number of paths  : {len(paths)}")

        return paths

    def paths_to_multiarray(self, paths: List[List[Point]]) -> Float32MultiArray:
        floats = []
        for path in paths:
            for point in path:
                floats.extend([float(point.x), float(point.y)])
            floats.extend([np.nan, np.nan])
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = 'xy'
        dim.size = len(floats)
        dim.stride = len(floats)
        msg.layout.dim = [dim]
        msg.data = floats
        return msg

    def paths_from_svg(self, file_path: Path, res: float) -> List[List[Point]]:
        with open(file_path, 'r') as f:
            svg_data = f.read()
        svg_doc = minidom.parseString(svg_data)

        paths = []
        for element in svg_doc.getElementsByTagName('path'):
            for path in tqdm(parse_path(element.getAttribute("d"))):
                path_segments = int(path.length(error=1e-3) / res)
                path_segments = max(path_segments, 10)
                path_points = [path.point(pos)
                               for pos in np.linspace(0, 1, path_segments)]
                x = [p.real for p in path_points]
                y = [-p.imag for p in path_points]
                path_points = [Point(xi, yi) for xi, yi in zip(x, y)]
                paths.append(path_points)

        return paths

    def normalize_paths(self, paths: List[List[Point]], size: float) -> List[List[Point]]:
        # Find extents of paths
        x_min = 0
        x_max = 0
        y_min = 0
        y_max = 0
        for path in paths:
            x_min = min([x_min, *[p.x for p in path]])
            x_max = max([x_max, *[p.x for p in path]])
            y_min = min([y_min, *[p.y for p in path]])
            y_max = max([y_max, *[p.y for p in path]])

        # Calculate scaling factor
        dx = x_max - x_min
        dy = y_max - y_min
        sf = size / np.sqrt(dx**2 + dy**2)

        # Scale all points
        paths_normal = []
        for path in tqdm(paths):
            path_normal = []
            for point in path:
                path_normal.append(
                    Point((point.x-x_min)*sf, (point.y-y_min)*sf))
            paths_normal.append(path_normal)

        return paths_normal, dx*sf, dy*sf

    def merge_paths(self, paths: List[List[Point]], res: float) -> List[List[Point]]:
        # This function essentially takes all the paths in the image and merges
        # them if they start/end in roughly the same positions
        simple = True
        change = True
        iter = 0
        while (change):
            change = False

            # Merge ends and starts
            tmp, paths = self.merge(paths.copy(), res, False, False, simple)
            change |= tmp

            # Merge starts and starts
            tmp, paths = self.merge(paths.copy(), res, True, True, simple)
            change |= tmp

            # Merge ends and ends
            tmp, paths = self.merge(paths.copy(), res, True, False, simple)
            change |= tmp

            # Do all simple merges first
            if not change and simple:
                simple = False
                change = True
            iter += 1

        return paths

    def merge(self, paths: List[List[Point]], res: float, same: bool, s: bool, simple: bool) -> Tuple[bool, List[List[Point]]]:
        starts = [(p[0].x, p[0].y) for p in paths]
        ends = [(p[-1].x, p[-1].y) for p in paths]

        points = []
        kd = None
        if same:
            if s:
                points = starts
                kd = KDTree(starts, copy_data=True)
            else:
                points = ends
                kd = KDTree(ends, copy_data=True)
        else:
            points = ends
            kd = KDTree(starts, copy_data=True)

        merge_list_indices = []
        selected_paths = set()
        # For all points, find if there is another point within res distance
        # that is not itself and that has not already been selected
        for path1_i, point1 in enumerate(points):
            # Skip already selected paths
            if path1_i in selected_paths:
                continue
            path2_i = kd.query_ball_point(point1, res)
            # Filter out already selected starts and self-matches
            path2_i = list(filter(
                lambda x: x not in selected_paths and x != path1_i, path2_i))
            # Ensure there is a pairing to merge with
            if len(path2_i) != 0:
                # In a simple merge, only merge paths that have 1 option
                # that is, paths without intersections
                if simple and len(path2_i) != 1:
                    continue
                # Select the longest path to merge with
                path2_i_len = 0
                for i in path2_i:
                    if path2_i_len < len(paths[i]):
                        path2_i_len = len(paths[i])
                        path2_i = i
                # The path indices to merge
                merge_list_indices.append((path1_i, path2_i))
                # You can't select the same start to merge with
                # add it to set to filter above
                selected_paths.add(path2_i)

        if len(merge_list_indices) == 0:
            return False, paths

        # Merge
        # When we are merging opposite ends (starts to ends etc) then
        # there is a chance a chain of such merges will be selected
        # In this case, the indices for path merging will be disrupted.
        # To resolve this, a map of merge indices is kept
        merge_map = {}
        for i in range(len(paths)):
            merge_map[i] = i
        # Merge logic
        for path1_i, path2_i in merge_list_indices:
            path1_i = merge_map[path1_i]
            path2_i = merge_map[path2_i]
            if same:
                # start/start and end/end merges require a reversal in 1 path
                paths[path2_i].reverse()
                paths[path1_i].extend(paths[path2_i])
            else:
                # end/start merges are straightforward
                paths[path1_i].extend(paths[path2_i])
                merge_map[path2_i] = path1_i
        # Remove merged paths
        paths = [p for i, p in enumerate(paths) if i not in selected_paths]
        return True, paths

    def plot_paths(self, paths: List[List[Point]]):
        # Clear
        plt.cla()

        # Draw paths
        for path in paths:
            x = [p.x for p in path]
            y = [p.y for p in path]
            plt.plot(x, y)

        plt.show(block=False)


def main(args=None):
    rclpy.init(args=args)
    SvgPathPublisher()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
