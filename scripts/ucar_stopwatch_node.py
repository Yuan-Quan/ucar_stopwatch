#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf
import os
import math
import hashlib
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point32, PolygonStamped
from time import monotonic
from datetime import datetime
from textual.app import App, ComposeResult
from textual.containers import Container, Horizontal
from textual.reactive import reactive
from textual.widgets import Button, Header, Footer, Static, DataTable, Checkbox
from textual.message import Message, MessageTarget


class RosClockWatcherNode(object):

    def __init__(self):
        # ROS Infastructure
        rospy.init_node('ucar_stopwatch_node')


class StopwatchNode(object):

    def __init__(self):
        # ROS Infastructure
        rospy.init_node('ucar_stopwatch_node')
        self.tf_listener = tf.TransformListener()
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.pub_scan_region = rospy.Publisher(
            "scan_region", PolygonStamped, queue_size=1)
        self.pub_park_1_region = rospy.Publisher(
            "park_1_region", PolygonStamped, queue_size=1)
        self.pub_park_2_region = rospy.Publisher(
            "park_2_region", PolygonStamped, queue_size=1)
        self.pub_park_3_region = rospy.Publisher(
            "park_3_region", PolygonStamped, queue_size=1)

        self.package_path = rospy.get_param("~package_path", "")
        # VARS
        self.position = Point32()
        self.vel = 0.0
        self.global_frame_id = "map"
        self.robot_frame_id = "base_link"

        pt1 = Point32()
        pt2 = Point32()
        pt3 = Point32()
        pt4 = Point32()

        self.scan_region = PolygonStamped()
        self.park_1_region = PolygonStamped()
        self.park_2_region = PolygonStamped()
        self.park_3_region = PolygonStamped()

        _scan_region = [[2.173, -3.406], [2.173, -2.768],
                        [3.416, -2.768], [3.416, -3.406]]

        pt1 = Point32()
        pt2 = Point32()
        pt3 = Point32()
        pt4 = Point32()
        pt1.x = _scan_region[0][0]
        pt1.y = _scan_region[0][1]
        pt2.x = _scan_region[1][0]
        pt2.y = _scan_region[1][1]
        pt3.x = _scan_region[2][0]
        pt3.y = _scan_region[2][1]
        pt4.x = _scan_region[3][0]
        pt4.y = _scan_region[3][1]

        self.scan_region.header.frame_id = self.global_frame_id
        self.scan_region.polygon.points.append(pt1)
        self.scan_region.polygon.points.append(pt2)
        self.scan_region.polygon.points.append(pt3)
        self.scan_region.polygon.points.append(pt4)

# [INFO][1669197865.563598546, 1080.665000000]: Setting goal: Frame: map, Position(-0.372, -1.612, 0.000), Orientation(0.000, 0.000, -0.004, 1.000) = Angle: -0.008
#
# [INFO][1669197869.910638276, 1084.973000000]: Setting goal: Frame: map, Position(1.384, -0.974, 0.000), Orientation(0.000, 0.000, -0.001, 1.000) = Angle: -0.003

        dx = 1.756
        dy = .638

        offset_x = -.372
        offset_y = -1.612

        self.park_1_region.header.frame_id = self.global_frame_id
        self.park_1_region.polygon.points.append(
            Point32(offset_x, offset_y, 0.0))
        self.park_1_region.polygon.points.append(
            Point32(offset_x, offset_y + dy, 0.0))
        self.park_1_region.polygon.points.append(
            Point32(offset_x + dx/3, offset_y + dy, 0.0))
        self.park_1_region.polygon.points.append(
            Point32(offset_x + dx/3, offset_y, 0.0))

        self.park_2_region.header.frame_id = self.global_frame_id
        self.park_2_region.polygon.points.append(
            Point32(offset_x + dx/3, offset_y, 0.0))
        self.park_2_region.polygon.points.append(
            Point32(offset_x + dx/3, offset_y + dy, 0.0))
        self.park_2_region.polygon.points.append(
            Point32(offset_x + 2*dx/3, offset_y + dy, 0.0))
        self.park_2_region.polygon.points.append(
            Point32(offset_x + 2*dx/3, offset_y, 0.0))

        self.park_3_region.header.frame_id = self.global_frame_id
        self.park_3_region.polygon.points.append(
            Point32(offset_x + 2*dx/3, offset_y, 0.0))
        self.park_3_region.polygon.points.append(
            Point32(offset_x + 2*dx/3, offset_y + dy, 0.0))
        self.park_3_region.polygon.points.append(
            Point32(offset_x + 3*dx/3, offset_y + dy, 0.0))
        self.park_3_region.polygon.points.append(
            Point32(offset_x + 3*dx/3, offset_y, 0.0))

    def odom_callback(self, odom: Odometry) -> None:
        self.vel = math.sqrt(pow(odom.twist.twist.linear.x, 2) +
                             pow(odom.twist.twist.linear.y, 2))

    def _nav_start_srv_client(request):
        try:
            sp = rospy.ServiceProxy('nav_start', SetBool)
            response = sp(True)
            return response.success, "Commander nav_start invoked successfully"
        except rospy.ServiceException as ex:
            return False, ("Service call to start commander failed: " + str(ex))

    def commander_start(self):
        stat, message = self._nav_start_srv_client()
        # to avoid mess with the rich markup
        message = message.replace('[', ' ')
        message = message.replace(']', ' ')
        return stat, message

    def _publish_visulization(self):
        self.pub_scan_region.publish(self.scan_region)
        self.pub_park_1_region.publish(self.park_1_region)
        self.pub_park_2_region.publish(self.park_2_region)
        self.pub_park_3_region.publish(self.park_3_region)

    def _get_robot_pose(self):
        # lookup tf and get robot pose in map frame
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.global_frame_id, self.robot_frame_id, rospy.Time(0))
            self.position.x = trans[0]
            self.position.y = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            # rospy.logwarn("tf lookup failed")

    def is_in_region(self, trigger_region):
        # check is robot in trigger region
        x_min = trigger_region.polygon.points[0].x
        x_max = trigger_region.polygon.points[0].x
        y_min = trigger_region.polygon.points[0].y
        y_max = trigger_region.polygon.points[0].y
        # find the x_min, y_min, x_max, y_max of the trigger region
        for point in trigger_region.polygon.points:
            if point.x < x_min:
                x_min = point.x
            if point.x > x_max:
                x_max = point.x
            if point.y < y_min:
                y_min = point.y
            if point.y > y_max:
                y_max = point.y

        # check is robot in trigger region
        if self.position.x >= x_min and \
                self.position.x <= x_max and \
                self.position.y >= y_min and \
                self.position.y <= y_max:
            return True
        else:
            return False

    def get_ros_status(self):
        self.update()
        return self.get_park_status(), self.vel

    def get_park_status(self) -> int:
        if self.is_in_region(self.scan_region):
            return 0
        elif self.is_in_region(self.park_1_region):
            return 1
        elif self.is_in_region(self.park_2_region):
            return 2
        elif self.is_in_region(self.park_3_region):
            return 3
        else:
            return -1

    def update(self) -> None:
        self._get_robot_pose()
        self._publish_visulization()


class RosStatus(Static):
    """ to run ros backend """

    class Parked(Message):
        def __init__(self, sender: MessageTarget, park_point: int) -> None:
            self.park_point = park_point
            super().__init__(sender)

    node = StopwatchNode()
    park_stat = reactive(-1)
    park_point_str = ""
    robot_vel = reactive(0.0)

    def on_mount(self) -> None:
        self.update_ros_timer = self.set_interval(
            1 / 100, self.update_ros)

    def update_ros(self) -> None:
        self.park_stat, self.robot_vel = self.node.get_ros_status()

    def watch_park_stat(self, park_stat: int) -> None:
        if park_stat == -1:
            self.park_point_str = "[dim]none[/]"
        elif park_stat == 0:
            self.park_point_str = "[blue]scan_qr_code[/]"
        elif park_stat == 1:
            self.park_point_str = "[green]parkpoint_1[/]"
        elif park_stat == 2:
            self.park_point_str = "[green]parkpoint_2[/]"
        elif park_stat == 3:
            self.park_point_str = "[green]parkpoint_3[/]"
        self.update(
            f"Active Parkpoint: {self.park_point_str}, Robot Velocity: {self.robot_vel:.2f} m/s")

    def watch_robot_vel(self, vel: float) -> None:
        self.update(
            f"[bold]ROS Status:[/] \n    active parkpoint: {self.park_point_str} \n    robot velocity: [yellow]{self.robot_vel:.2f}[/] m/s")
        if self.park_stat >= 1 and self.park_stat <= 3 and self.robot_vel < 0.0005:
            self.emit_no_wait(self.Parked(self, self.park_stat))


class LogDisplay(DataTable):

    def on_mount(self):
        self.add_column("TIME", width=10)
        self.add_column("MESSAGE", width=66)
        self.add_row(datetime.now().strftime(
            "%H:%M:%S"), "stopwatch initialized")

    def new_message(self, message: str) -> None:
        self.add_row(datetime.now().strftime("%H:%M:%S"), message)


class RosTimeDisplay(Static):
    """A widget to display elapsed time."""

    node = RosClockWatcherNode()

    start_time = reactive(rospy.get_time())
    time = reactive(0.00)

    def on_mount(self) -> None:
        """Event handler called when widget is added to the app."""
        self.update_timer = self.set_interval(
            1 / 60, self.update_time, pause=True)
        self.start_time = rospy.get_time()

    def update_time(self) -> None:
        """Method to update time to current."""
        self.time = rospy.get_time() - self.start_time

    def watch_time(self, time: float) -> None:
        """Called when the time attribute changes."""
        minutes, seconds = divmod(time, 60)
        hours, minutes = divmod(minutes, 60)
        self.update(
            f"ROS Time Elapsed:   [blue bold]{hours:02.0f}:{minutes:02.0f}:{seconds:05.2f}[/]")

    def start(self) -> None:
        """Method to start (or resume) time updating."""
        self.node = RosClockWatcherNode()
        self.start_time = rospy.get_time()
        self.update_timer.resume()

    def stop(self):
        """Method to stop the time display updating."""
        elapsed = self.time
        self.update_timer.pause()
        return elapsed

    def reset(self):
        """Method to reset the time display to zero."""
        self.time = 0.00
        pass


class WallTimeDisplay(Static):
    """A widget to display elapsed time."""

    start_time = reactive(monotonic)
    time = reactive(0.00)

    def on_mount(self) -> None:
        """Event handler called when widget is added to the app."""
        self.update_timer = self.set_interval(
            1 / 60, self.update_time, pause=True)

    def update_time(self) -> None:
        """Method to update time to current."""
        self.time = monotonic() - self.start_time

    def watch_time(self, time: float) -> None:
        """Called when the time attribute changes."""
        minutes, seconds = divmod(time, 60)
        hours, minutes = divmod(minutes, 60)
        self.update(
            f"Wall Time Elapsed:  [blue bold]{hours:02.0f}:{minutes:02.0f}:{seconds:05.2f}[/]")

    def start(self) -> None:
        """Method to start (or resume) time updating."""
        self.start_time = monotonic()
        self.update_timer.resume()

    def stop(self):
        """Method to stop the time display updating."""
        elapsed = self.time
        self.update_timer.pause()
        return elapsed

    def reset(self):
        """Method to reset the time display to zero."""
        self.time = 0.0
        pass


class StopwatchApp(App):
    """A Textual app to manage stopwatches."""

    node = StopwatchNode()
    is_timer_started = False
    use_sim_time = True
    send_start = True
    CSS_PATH = os.path.join(
        node.package_path, "scripts/ucar_stopwatch_node.css")

    BINDINGS = [("space", "start_timer", "Start Timer"),
                ("d", "toggle_dark", "Toggle dark mode"),
                ("q", "exit", "Quit Stopwatch")]

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Event handler called when a button is pressed."""
        button_id = event.button.id
        ros_time_display = self.query_one(RosTimeDisplay)
        wall_time_display = self.query_one(WallTimeDisplay)
        log_display = self.query_one(LogDisplay)
        if button_id == "start":
            if self.send_start:
                (stat, msg) = self.node.commander_start()
                if self.is_timer_started:
                    pass
                elif stat:
                    ros_time_display.start()
                    wall_time_display.start()
                    self.add_class("started")
                    log_display.new_message("Nav Start!!")
                    self.is_timer_started = True
                else:
                    log_display.new_message(f"[warning] {msg} [/warning]")
            else:
                ros_time_display.start()
                wall_time_display.start()
                self.add_class("started")
                log_display.new_message("Timer Start!!")
                self.is_timer_started = True

        elif button_id == "reset":
            self.remove_class("started")
            ros_time_display.reset()
            wall_time_display.reset()
            self.is_timer_started = False
            # time_display.reset()
        elif button_id == "stop":
            ros_time_display = self.query_one(RosTimeDisplay)
            wall_time_display = self.query_one(WallTimeDisplay)
            log_disp = self.query_one(LogDisplay)
            sim_time = ros_time_display.stop()
            wall_time = wall_time_display.stop()
            self.is_timer_started = False
            if self.use_sim_time:
                elapsed = sim_time
            else:
                elapsed = wall_time
            log_disp.new_message(
                f"Manually stoped, Time: {elapsed:.3f} sec")
            self.remove_class("started")

        elif button_id == "md5sum":
            file_name = os.path.join(
                self.node.package_path, '../gazebo_pkg/world/race.world')
            # Open,close, read file and calculate MD5 on its contents
            with open(file_name, 'rb') as file_to_check:
                # read contents of the file
                data = file_to_check.read()
                # pipe contents of the file through
                md5_returned = hashlib.md5(data).hexdigest()

            log_disp = self.query_one(LogDisplay)
            log_disp.new_message("md5sum world:   " + md5_returned)

            file_name = os.path.join(
                self.node.package_path, '../mecanum_sim/nexus_4wd_mecanum_description/urdf/nexus_4wd_mecanum.xacro')
            # Open,close, read file and calculate MD5 on its contents
            with open(file_name, 'rb') as file_to_check:
                # read contents of the file
                data = file_to_check.read()
                # pipe contents of the file through
                md5_returned = hashlib.md5(data).hexdigest()

            log_disp.new_message("md5sum urdf:    " + md5_returned)

        elif button_id == "dump":
            log_disp = self.query_one(LogDisplay)
            log_disp.new_message("topic and tf dump is not implemented, yet.")

    def on_ros_status_parked(self, message: RosStatus.Parked) -> None:
        """ when robot is in park point and velocity is 0"""
        if not self.is_timer_started:
            return
        ros_time_display = self.query_one(RosTimeDisplay)
        wall_time_display = self.query_one(WallTimeDisplay)
        log_disp = self.query_one(LogDisplay)
        sim_time = ros_time_display.stop()
        wall_time = wall_time_display.stop()
        self.is_timer_started = False
        if self.use_sim_time:
            elapsed = sim_time
        else:
            elapsed = wall_time
        log_disp.new_message(
            f"Parked at park_point_{message.park_point}, Time: {elapsed:.3f} sec")
        self.remove_class("started")

    def on_checkbox_changed(self, event: Checkbox.Changed) -> None:
        box_id = event.input.id
        value = event.value
        if box_id == "sim_time_check":
            if value:
                self.use_sim_time = True
            else:
                self.use_sim_time = False
                log_disp = self.query_one(LogDisplay)
                log_disp.new_message(
                    f"You have disabled sim time, this is most likely unwanted.")
                log_disp.new_message(
                    f"Only use this option when ROS /Clock is not avaliable")
        elif box_id == "send_start_check":
            self.send_start = value

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()
        yield RosStatus(id="ros_status")
        yield Container(
            Container(
                RosTimeDisplay(),
                WallTimeDisplay(),
                id="time_display"
            ),
            Container(
                Button("Start", id="start", variant="success"),
                Button("Stop", id="stop", variant="error"),
                Button("Reset", id="reset"),
                id="start_reset_panel"
            ),
            id='panel_1'
        )

        yield Container(
            Container(
                Static("[b]Configuration:\n", classes="config_title"),
                Horizontal(
                    Static("  use sim time:       ", classes="label"),
                    Checkbox(value=True, id="sim_time_check"),
                    classes="configure",
                ),
                Horizontal(
                    Static("  send /nav_start:    ", classes="label"),
                    Checkbox(value=True, id="send_start_check"),
                    classes="configure",
                ),
                id="config_left"
            ),
            Container(
                Button("verify md5 sum", id="md5sum"),
                Button("dump topics and tf tree", id="dump"),
                id="config_right"
            ),
            id="config_panel"
        )
        yield LogDisplay()
        yield Footer()

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark

    def action_start_timer(self) -> None:
        ros_time_display = self.query_one(RosTimeDisplay)
        wall_time_display = self.query_one(WallTimeDisplay)
        log_display = self.query_one(LogDisplay)
        if self.send_start:
            (stat, msg) = self.node.commander_start()
            if self.is_timer_started:
                pass
            elif stat:
                ros_time_display.start()
                wall_time_display.start()
                self.add_class("started")
                log_display.new_message("Nav Start!!")
                self.is_timer_started = True
            else:
                log_display.new_message(f"[warning] {msg} [/warning]")
        else:
            ros_time_display.start()
            wall_time_display.start()
            self.add_class("started")
            log_display.new_message("Timer Start!!")
            self.is_timer_started = True

    def action_exit(self) -> None:
        exit(0)


def main():
    app = StopwatchApp()
    app.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
