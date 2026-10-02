"""
Track loading and trajectory parsing module for kinematic bicycle simulation.
Pure Python module without mandatory ROS dependencies for easy unit testing.
"""

import json
import math
import os

try:
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
except ImportError:
    get_package_share_directory = None

    class PackageNotFoundError(Exception):
        pass


DEFAULT_TRACK_FILE = 'global_waypoints.json'


class Track:
    """Represents a racetrack loaded from global_waypoints.json"""

    def __init__(self, track_file=None, trajectory_type='centerline', close_loop=True):
        self.track_file = track_file or DEFAULT_TRACK_FILE
        self.trajectory_type = trajectory_type  # 'centerline', 'sp', 'iqp'
        self.close_loop = close_loop

        self.file_path = self.resolve_track_path(self.track_file)
        if not self.file_path or not os.path.isfile(self.file_path):
            raise FileNotFoundError(f"Track file '{self.track_file}' could not be resolved.")

        with open(self.file_path, 'r') as f:
            self.raw_data = json.load(f)

        # list of dicts: {'x': ..., 'y': ..., 'psi': ..., 'kappa': ..., 's': ...}
        self.waypoints = []
        self._load_trajectory()
        self.trackbounds_markers = self.raw_data.get('trackbounds_markers', {}).get('markers', [])

    @staticmethod
    def get_search_paths(file_name):
        """Returns candidate paths to search for the track file."""
        candidates = []
        if os.path.isabs(file_name):
            return [file_name]

        # 1. Package share directory (installed)
        if get_package_share_directory is not None:
            for pkg in ('track_environment', 'kinematic_bicycle'):
                try:
                    share_dir = get_package_share_directory(pkg)
                    candidates.append(os.path.join(share_dir, 'tracks', file_name))
                    candidates.append(os.path.join(share_dir, file_name))
                except PackageNotFoundError:
                    pass

        # 2. Module relative directory (source tree)
        module_dir = os.path.dirname(os.path.abspath(__file__))
        package_root = os.path.abspath(os.path.join(module_dir, '..'))
        candidates.append(os.path.join(package_root, 'tracks', file_name))
        candidates.append(os.path.join(module_dir, 'tracks', file_name))
        candidates.append(os.path.join(module_dir, file_name))

        cwd = os.getcwd()
        candidates.append(os.path.join(cwd, 'tracks', file_name))
        candidates.append(os.path.join(cwd, file_name))
        candidates.append(os.path.join(cwd, 'src', 'track_environment', 'tracks', file_name))
        candidates.append(os.path.join(cwd, 'src', 'kinematic_bicycle', 'tracks', file_name))
        return candidates

    @classmethod
    def resolve_track_path(cls, file_name):
        """Finds the first existing candidate file path."""
        for path in cls.get_search_paths(file_name):
            if os.path.isfile(path):
                return path
        return None

    def _load_trajectory(self):
        """Extracts waypoints based on trajectory_type."""
        key_map = {
            'centerline': 'centerline_waypoints',
            'sp': 'global_traj_wpnts_sp',
            'shortest_path': 'global_traj_wpnts_sp',
            'iqp': 'global_traj_wpnts_iqp',
            'min_curvature': 'global_traj_wpnts_iqp'
        }
        json_key = key_map.get(self.trajectory_type, 'centerline_waypoints')
        traj_data = self.raw_data.get(json_key, {})
        wpnts = traj_data.get('wpnts', [])

        if not wpnts and 'centerline_waypoints' in self.raw_data:
            # Fallback to centerline if requested trajectory is not found
            wpnts = self.raw_data['centerline_waypoints'].get('wpnts', [])

        if len(wpnts) < 2:
            raise ValueError(f"Track contains fewer than 2 waypoints in {json_key}")

        self.waypoints = []
        for wp in wpnts:
            x = float(wp.get('x_m', 0.0))
            y = float(wp.get('y_m', 0.0))
            psi = float(wp.get('psi_rad', 0.0))
            kappa = float(wp.get('kappa_radpm', 0.0))
            s = float(wp.get('s_m', 0.0))
            vx = float(wp.get('vx_mps', 0.0))
            ax = float(wp.get('ax_mps2', 0.0))

            self.waypoints.append({
                'x': x,
                'y': y,
                'psi': psi,
                'kappa': kappa,
                's': s,
                'vx': vx,
                'ax': ax
            })

        # Calculate or refine headings if missing/zero
        n = len(self.waypoints)
        for i in range(n):
            if self.waypoints[i]['psi'] == 0.0:
                next_i = (i + 1) % n
                dx = self.waypoints[next_i]['x'] - self.waypoints[i]['x']
                dy = self.waypoints[next_i]['y'] - self.waypoints[i]['y']
                self.waypoints[i]['psi'] = math.atan2(dy, dx)

        # Close the loop if requested
        if self.close_loop and n > 0:
            first = self.waypoints[0].copy()
            last = self.waypoints[-1]
            dist_close = math.hypot(first['x'] - last['x'], first['y'] - last['y'])
            if dist_close > 1e-4:
                first['s'] = last['s'] + dist_close
                self.waypoints.append(first)

    @property
    def x(self):
        return [wp['x'] for wp in self.waypoints]

    @property
    def y(self):
        return [wp['y'] for wp in self.waypoints]

    @property
    def psi(self):
        return [wp['psi'] for wp in self.waypoints]

    @property
    def vx(self):
        return [wp['vx'] for wp in self.waypoints]

    @property
    def kappa(self):
        return [wp['kappa'] for wp in self.waypoints]

    @property
    def total_length(self):
        """Calculates total perimeter along the waypoints."""
        total = 0.0
        for i in range(len(self.waypoints) - 1):
            total += math.hypot(
                self.waypoints[i + 1]['x'] - self.waypoints[i]['x'],
                self.waypoints[i + 1]['y'] - self.waypoints[i]['y']
            )
        return total

    @property
    def start_pose(self):
        """Returns (x, y, psi) of the start waypoint."""
        if not self.waypoints:
            return 0.0, 0.0, 0.0
        return self.waypoints[0]['x'], self.waypoints[0]['y'], self.waypoints[0]['psi']
