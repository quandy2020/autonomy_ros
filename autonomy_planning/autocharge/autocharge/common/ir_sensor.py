"""IR sensor model: mount, FOV check, DockIR message, and RViz markers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from jdbot_interfaces.msg import DockIR
from visualization_msgs.msg import Marker, MarkerArray


def wrap_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2D:
    """2D pose in world frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class IRSensor:
    """Single IR receiver mounted on robot frame."""

    name: str
    x: float
    y: float
    yaw: float
    fov_deg: float
    max_range_m: float
    token: int
    mode: str = 'sector'


@dataclass(frozen=True)
class IRSensorOutput:
    """Result of one sampling step."""

    left: int
    right: int
    code: int = 0
    message: str = 'ok'


class IRSensorArray:
    """Dual-IR array model with detection, encoding, and visualization."""

    def __init__(
        self,
        *,
        robot_length_m: float = 0.40,
        robot_width_m: float = 0.35,
        spacing_m: float = 0.0109,
        mount_x_m: Optional[float] = None,
        mount_center_y_m: float = 0.0,
        mount_yaw_deg: float = 0.0,
        fov_deg: float = 15.0,
        max_range_m: float = 1.5,
        left_range_m: Optional[float] = None,
        right_range_m: Optional[float] = None,
        bit_left: int = 0x69,
        bit_center: int = 0x6B,
        bit_right: int = 0x6F,
        use_explicit_pair_pose: bool = False,
        left_x_m: float = 0.0,
        left_y_m: float = 0.0,
        left_yaw_deg: float = 0.0,
        left_fov_deg: Optional[float] = None,
        left_mode: str = 'normal',
        right_x_m: float = 0.0,
        right_y_m: float = 0.0,
        right_yaw_deg: float = 0.0,
        right_fov_deg: Optional[float] = None,
        right_mode: str = 'normal',
        footprint_color: Tuple[float, float, float, float] = (0.2, 0.9, 0.3, 1.0),
        left_sensor_color: Tuple[float, float, float, float] = (0.2, 0.6, 1.0, 1.0),
        right_sensor_color: Tuple[float, float, float, float] = (1.0, 0.4, 0.2, 1.0),
        fov_color: Tuple[float, float, float, float] = (1.0, 1.0, 0.2, 0.9),
        fov_fill_color: Tuple[float, float, float, float] = (1.0, 1.0, 0.2, 0.20),
        enable_fov_fill: bool = True,
    ) -> None:
        # Robot/body geometry.
        self.robot_length_m = float(robot_length_m)
        self.robot_width_m = float(robot_width_m)
        self.spacing_m = float(spacing_m)
        self.mount_x_m = (
            0.5 * self.robot_length_m if mount_x_m is None else float(mount_x_m)
        )
        self.mount_center_y_m = float(mount_center_y_m)
        self.mount_yaw_deg = float(mount_yaw_deg)
        self.fov_deg = float(fov_deg)
        self.max_range_m = float(max_range_m)
        self.left_range_m = self.max_range_m if left_range_m is None else float(left_range_m)
        self.right_range_m = self.max_range_m if right_range_m is None else float(right_range_m)

        # Marker colors.
        self.footprint_color = footprint_color
        self.left_sensor_color = left_sensor_color
        self.right_sensor_color = right_sensor_color
        self.fov_color = fov_color
        self.fov_fill_color = fov_fill_color
        self.enable_fov_fill = bool(enable_fov_fill)

        # Output tokens.
        self.bit_left = int(bit_left) & 0xFF
        self.bit_center = int(bit_center) & 0xFF
        self.bit_right = int(bit_right) & 0xFF
        self._low_mask = 0x1F

        if bool(use_explicit_pair_pose):
            lfov = self.fov_deg if left_fov_deg is None else float(left_fov_deg)
            rfov = self.fov_deg if right_fov_deg is None else float(right_fov_deg)
            lmode = str(left_mode).strip().lower() or 'normal'
            rmode = str(right_mode).strip().lower() or 'normal'
            self.left_sensor = self._build_sensor(
                name='left',
                x_m=float(left_x_m),
                y_m=float(left_y_m),
                yaw_deg=float(left_yaw_deg),
                fov_deg=lfov,
                max_range_m=self.left_range_m,
                token=self.bit_left,
                mode=lmode,
            )
            self.right_sensor = self._build_sensor(
                name='right',
                x_m=float(right_x_m),
                y_m=float(right_y_m),
                yaw_deg=float(right_yaw_deg),
                fov_deg=rfov,
                max_range_m=self.right_range_m,
                token=self.bit_right,
                mode=rmode,
            )
        else:
            half = 0.5 * self.spacing_m
            self.left_sensor = self._build_sensor(
                name='left',
                x_m=self.mount_x_m,
                y_m=self.mount_center_y_m + half,
                yaw_deg=self.mount_yaw_deg,
                fov_deg=self.fov_deg,
                max_range_m=self.left_range_m,
                token=self.bit_left,
                mode='normal',
            )
            self.right_sensor = self._build_sensor(
                name='right',
                x_m=self.mount_x_m,
                y_m=self.mount_center_y_m - half,
                yaw_deg=self.mount_yaw_deg,
                fov_deg=self.fov_deg,
                max_range_m=self.right_range_m,
                token=self.bit_right,
                mode='normal',
            )

    def _build_sensor(
        self,
        *,
        name: str,
        x_m: float,
        y_m: float,
        yaw_deg: float,
        fov_deg: float,
        max_range_m: float,
        token: int,
        mode: str,
    ) -> IRSensor:
        """Build one sensor record with normalized numeric fields."""
        return IRSensor(
            name=name,
            x=float(x_m),
            y=float(y_m),
            yaw=math.radians(float(yaw_deg)),
            fov_deg=float(fov_deg),
            max_range_m=float(max_range_m),
            token=int(token),
            mode=str(mode),
        )

    @staticmethod
    def _transform(pose: Pose2D, px: float, py: float) -> Tuple[float, float]:
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        return (pose.x + c * px - s * py, pose.y + s * px + c * py)

    def sensor_positions_world(self, pose: Pose2D) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Return left/right sensor positions in world frame."""
        lw = self._transform(pose, self.left_sensor.x, self.left_sensor.y)
        rw = self._transform(pose, self.right_sensor.x, self.right_sensor.y)
        return lw, rw

    def in_fov(self, pose: Pose2D, sensor: IRSensor, target_x: float, target_y: float) -> bool:
        """Check if a target point is inside one sensor FOV and range."""
        sx, sy = self._transform(pose, sensor.x, sensor.y)
        dx = float(target_x) - sx
        dy = float(target_y) - sy
        dist = math.hypot(dx, dy)
        if dist <= 1e-9:
            return True
        if dist > sensor.max_range_m:
            return False
        heading = wrap_pi(pose.yaw + sensor.yaw)
        ang = math.atan2(dy, dx)
        err = abs(wrap_pi(ang - heading))
        return err <= math.radians(0.5 * sensor.fov_deg)

    def sample_target_point(self, pose: Pose2D, target_x: float, target_y: float) -> IRSensorOutput:
        """Sample one target point and produce low-level left/right token."""
        hit_l = self.in_fov(pose, self.left_sensor, target_x, target_y)
        hit_r = self.in_fov(pose, self.right_sensor, target_x, target_y)
        if hit_l and hit_r:
            t = self.bit_center
            return IRSensorOutput(left=t, right=t)
        if hit_l:
            return IRSensorOutput(left=self.bit_left, right=0)
        if hit_r:
            return IRSensorOutput(left=0, right=self.bit_right)
        return IRSensorOutput(left=0, right=0)

    def sample_targets(
        self,
        pose: Pose2D,
        targets: Sequence[Tuple[float, float]],
    ) -> IRSensorOutput:
        """Sample multiple target points and merge by priority center>side>none."""
        l_any = False
        r_any = False
        for tx, ty in targets:
            l_any = l_any or self.in_fov(pose, self.left_sensor, tx, ty)
            r_any = r_any or self.in_fov(pose, self.right_sensor, tx, ty)
        if l_any and r_any:
            t = self.bit_center
            return IRSensorOutput(left=t, right=t)
        if l_any:
            return IRSensorOutput(left=self.bit_left, right=0)
        if r_any:
            return IRSensorOutput(left=0, right=self.bit_right)
        return IRSensorOutput(left=0, right=0)

    def to_dock_ir_msg(self, out: IRSensorOutput, stamp: Optional[Time] = None) -> DockIR:
        """Convert sensor output to DockIR message."""
        msg = DockIR()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.left = int(out.left) & 0xFF
        msg.right = int(out.right) & 0xFF
        msg.code = int(out.code)
        msg.message = str(out.message)
        return msg

    def build_markers(
        self,
        pose: Pose2D,
        *,
        frame_id: str,
        stamp: Time,
        ns: str = 'ir_sensor',
        visualize_left: bool = True,
        visualize_right: bool = True,
    ) -> MarkerArray:
        """Build footprint/sensor/FOV markers for RViz."""
        mks = MarkerArray()
        mks.markers.append(self._mk_footprint(pose, frame_id, stamp, ns, 0))
        left_xy, right_xy = self.sensor_positions_world(pose)
        if visualize_left:
            mks.markers.append(self._mk_sensor(left_xy, frame_id, stamp, ns, 1, True))
        if visualize_right:
            mks.markers.append(self._mk_sensor(right_xy, frame_id, stamp, ns, 2, False))
        mks.markers.append(
            self._mk_fov_lines(
                pose,
                left_xy,
                right_xy,
                frame_id,
                stamp,
                ns,
                3,
                visualize_left=visualize_left,
                visualize_right=visualize_right,
            )
        )
        if self.enable_fov_fill:
            mks.markers.append(
                self._mk_fov_fill(
                    pose,
                    left_xy,
                    right_xy,
                    frame_id,
                    stamp,
                    ns,
                    4,
                    visualize_left=visualize_left,
                    visualize_right=visualize_right,
                )
            )
        return mks

    def _mk_footprint(self, pose: Pose2D, frame_id: str, stamp: Time, ns: str, mid: int) -> Marker:
        half_l = 0.5 * self.robot_length_m
        half_w = 0.5 * self.robot_width_m
        corners = [
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, -half_w),
            (-half_l, half_w),
            (half_l, half_w),
        ]
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        m.color.r, m.color.g, m.color.b, m.color.a = self.footprint_color
        for px, py in corners:
            wx, wy = self._transform(pose, px, py)
            m.points.append(Point(x=wx, y=wy, z=0.03))
        return m

    def _mk_sensor(
        self, xy: Tuple[float, float], frame_id: str, stamp: Time, ns: str, mid: int, is_left: bool
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x = xy[0]
        m.pose.position.y = xy[1]
        m.pose.position.z = 0.04
        m.scale.x = 0.003
        m.scale.y = 0.003
        m.scale.z = 0.003
        if is_left:
            m.color.r, m.color.g, m.color.b, m.color.a = self.left_sensor_color
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = self.right_sensor_color
        return m

    def _mk_fov_lines(
        self,
        pose: Pose2D,
        left_xy: Tuple[float, float],
        right_xy: Tuple[float, float],
        frame_id: str,
        stamp: Time,
        ns: str,
        mid: int,
        *,
        visualize_left: bool,
        visualize_right: bool,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.002
        m.color.r, m.color.g, m.color.b, m.color.a = self.fov_color

        sensors = []
        if visualize_left:
            sensors.append((left_xy, self.left_sensor))
        if visualize_right:
            sensors.append((right_xy, self.right_sensor))
        for xy, sensor in sensors:
            half = math.radians(0.5 * float(sensor.fov_deg))
            heading = pose.yaw + float(sensor.yaw)
            mode = str(sensor.mode).strip().lower()
            if mode in ('trapezoid', 'trap'):
                near = 0.05 * float(sensor.max_range_m)
                far = float(sensor.max_range_m)
                near_w = math.tan(half) * near
                far_w = math.tan(half) * far
                ux = math.cos(heading)
                uy = math.sin(heading)
                vx = -math.sin(heading)
                vy = math.cos(heading)
                p_nl = Point(x=xy[0] + ux * near - vx * near_w, y=xy[1] + uy * near - vy * near_w, z=0.03)
                p_nr = Point(x=xy[0] + ux * near + vx * near_w, y=xy[1] + uy * near + vy * near_w, z=0.03)
                p_fl = Point(x=xy[0] + ux * far - vx * far_w, y=xy[1] + uy * far - vy * far_w, z=0.03)
                p_fr = Point(x=xy[0] + ux * far + vx * far_w, y=xy[1] + uy * far + vy * far_w, z=0.03)
                m.points.extend([p_nl, p_nr, p_nl, p_fl, p_nr, p_fr, p_fl, p_fr])
            else:
                for sign in (-1.0, 1.0):
                    ang = heading + sign * half
                    p0 = Point(x=xy[0], y=xy[1], z=0.03)
                    p1 = Point(
                        x=xy[0] + float(sensor.max_range_m) * math.cos(ang),
                        y=xy[1] + float(sensor.max_range_m) * math.sin(ang),
                        z=0.03,
                    )
                    m.points.extend([p0, p1])
        return m

    def _mk_fov_fill(
        self,
        pose: Pose2D,
        left_xy: Tuple[float, float],
        right_xy: Tuple[float, float],
        frame_id: str,
        stamp: Time,
        ns: str,
        mid: int,
        *,
        visualize_left: bool,
        visualize_right: bool,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = self.fov_fill_color

        sensors = []
        if visualize_left:
            sensors.append((left_xy, self.left_sensor))
        if visualize_right:
            sensors.append((right_xy, self.right_sensor))

        for xy, sensor in sensors:
            half = math.radians(0.5 * float(sensor.fov_deg))
            heading = pose.yaw + float(sensor.yaw)
            mode = str(sensor.mode).strip().lower()
            if mode in ('trapezoid', 'trap'):
                near = 0.05 * float(sensor.max_range_m)
                far = float(sensor.max_range_m)
                near_w = math.tan(half) * near
                far_w = math.tan(half) * far
                ux = math.cos(heading)
                uy = math.sin(heading)
                vx = -math.sin(heading)
                vy = math.cos(heading)
                p_nl = Point(x=xy[0] + ux * near - vx * near_w, y=xy[1] + uy * near - vy * near_w, z=0.02)
                p_nr = Point(x=xy[0] + ux * near + vx * near_w, y=xy[1] + uy * near + vy * near_w, z=0.02)
                p_fl = Point(x=xy[0] + ux * far - vx * far_w, y=xy[1] + uy * far - vy * far_w, z=0.02)
                p_fr = Point(x=xy[0] + ux * far + vx * far_w, y=xy[1] + uy * far + vy * far_w, z=0.02)
                m.points.extend([p_nl, p_nr, p_fl, p_nr, p_fr, p_fl])
            else:
                center = Point(x=xy[0], y=xy[1], z=0.02)
                seg_n = 12
                for i in range(seg_n):
                    a0 = heading - half + (2.0 * half) * (i / seg_n)
                    a1 = heading - half + (2.0 * half) * ((i + 1) / seg_n)
                    p0 = Point(
                        x=xy[0] + float(sensor.max_range_m) * math.cos(a0),
                        y=xy[1] + float(sensor.max_range_m) * math.sin(a0),
                        z=0.02,
                    )
                    p1 = Point(
                        x=xy[0] + float(sensor.max_range_m) * math.cos(a1),
                        y=xy[1] + float(sensor.max_range_m) * math.sin(a1),
                        z=0.02,
                    )
                    m.points.extend([center, p0, p1])
        return m
