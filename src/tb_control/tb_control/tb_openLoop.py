#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time


class tb_openloop_node(Node):
    def __init__(self):
        # Initialize the node and create the publisher
        super().__init__("tb_openloop_node")
        self.get_logger().info("TurtleBot3 Open Loop Control Node has started")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Read the scenario from user
        scenario = input("Select the scenario to run (1 or 2): ")
        self.scenario = int(scenario)
        self.get_logger().info(f"Selected Scenario-{self.scenario}")
        distance = input("Enter distance to travel in m: ")

        if self.scenario == 1:
            # Scenario 1:
            self.get_logger().info("The TurtleBot moves at a constant velocity of 0.1 m/s")

            # Time of travel computation:
            # 1. The Turtlebot starts from rest but is given an instantaneous constant velocity of 0.1 m/s
            # 2. The Time of travel is computed using the expression T = d / v where T = time of travel,
            #   d = distance covered and v is the velocity
            self.curr_speed = 0.1  # Fixed speed
            self.time_of_travel = float(distance) / self.curr_speed
            self.get_logger().info(
                "Expected time of travel: " + str(self.time_of_travel)
            )
            
            # Initially publish zero to put the Turtlebot to rest
            msg = Twist()
            msg.linear.x = 0.0
            self.cmd_vel_pub.publish(msg)

        elif self.scenario == 2:
            # Scenario 2:
            self.get_logger().info(
                "The Turtlebot moves from rest with Uniform Acceleration -> Steady state motion at 0.22 m/s -> Uniform deceleration to stop"
            )

            #  Velocity profile and time of travel computation:
            # 1. The Turtlebot starts moving with uniform acceleration for the first quarter of the motion,
            #   during which velocity increases from rest (0 m/s) to a max value (0.22 m/s)
            # 2. This is then followed by a constant velocity motion at max velocity (0.22 m/s) in the second
            #   and third quarters (central half) of the motion
            # 3. This is then followed by uniform deceleration for the last quarter of the motion,
            #   during which velocity decreases from the max value (0.22 m/s) to complete halt (0 m/s)
            # 4. Initially the total time of motion and the average velocity are computed after receiving
            #   distance as input from the user
            # 5. For the assumed profile, we get Total time, T = t1 + t2 + t3 (Three time periods)
            # 6. Total distance, D = d1 + d2 + d3 = 0.5*t1*vmax + t2*vmax + 0.5*t3*vmax = (t1 + t2) * vmax = (3/4)*T*vmax
            #   (Because t1 = t3 = T/4 and t2 = T/2)
            # 7. Average velocity = vavg = Total Distance / Total time = D/T = (3/4)*T*vmax/T = (3/4)*0.22 = 0.165 m/s
            # 8. Hence, the total time of travel = T = Total Distance / 0.165

            self.curr_speed = 0.0  # Assuming initial velocity is zero
            self.time_of_travel = float(distance) / 0.165
            self.get_logger().info(
                "Expected time of travel: " + str(self.time_of_travel)
            )

            # Initially publish zero to put the Turtlebot to rest
            msg = Twist()
            msg.linear.x = 0.0
            self.cmd_vel_pub.publish(msg)

        else:
            raise ValueError("Invalid option selected")

        self.started_at = time.time()
        threading.Thread(target=self.timer_callback).start()

    def timer_callback(self):
        if self.scenario == 1:
            # Publishing the Twist message in cmd_vel topic
            msg = Twist()
            time_now = int(time.time())

            if time_now >= (self.started_at + self.time_of_travel):
                # Turtlebot has already reached the goal, allow it to come to a complete halt
                msg.linear.x = self.curr_speed = 0.0
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info("Turtlebot has reached the goal")
                return

            # Keep moving at constant velocity until the time as elapsed
            msg.linear.x = self.curr_speed
            threading.Timer(0.01, self.timer_callback).start()
            self.cmd_vel_pub.publish(msg)

        elif self.scenario == 2:
            # Computing speed and publishing the twist message in cmd_vel topic:
            # 1. From Equations of motion, v = u + at, we get a = (v - u) / t
            # 2. The acceleration is computed by plugging in initial (assumed 0 m/s, open loop) and final velocities (0.22 m/s)
            # 3. So we get a = (0.22 - 0) / (T/4) = 0.88 / T , where T is the total time of travel and
            #   t = T/4 because the Turtlebot accelerates for one quarter of the time to reach max velocity
            # 4. Given that we enter the control loop at 100Hz (Timer frequency), the speed is increased every 0.01 seconds to
            #   achieve uniform acceleration
            # 5. Substituting the known values in a = dv/dt, we get 0.88/T = (next_speed-curr_velocity)/0.01
            # 6. Then next_speed = curr_velocity + 0.0088/T while accelerating
            # 7. This velocity gets stepped up by a factor of 0.0088/T every 0.01 seconds until it reaches its max value (0.22 m/s)
            # 8. Once max velocity is reached, it is maintained for a time period of T/2
            # 9. Similarly while decelerating, a = -0.88/T
            # 10. Then next_speed = curr_velocity - 0.0088/T while decelerating
            # 7. This velocity gets stepped down by a factor of 0.0088/T every 0.01 seconds until it reaches its min value (0 m/s)

            msg = Twist()
            next_speed = 0.0
            time_now = time.time()

            if time_now >= (self.started_at + self.time_of_travel):
                # Turtlebot has already reached the goal, allow it to come to a complete halt
                next_speed = 0.0
                msg.linear.x - next_speed
                self.cmd_vel_pub.publish(msg)
                self.get_logger().info("Turtlebot has reached the goal")
                return

            if time_now < (self.started_at + self.time_of_travel / 4):
                # In the first quarter, maintain uniform acceleration
                next_speed = self.curr_speed + (0.0088 / self.time_of_travel)
            elif time_now < (self.started_at + self.time_of_travel * 3 / 4):
                # In the central half, maintain constant speed of 0.22 m/s
                next_speed = self.curr_speed = 0.22
            elif time_now < (self.started_at + self.time_of_travel):
                # In the last quarter, maintain uniform deceleration
                next_speed = self.curr_speed - (0.0088 / self.time_of_travel)

            msg.linear.x = next_speed
            self.curr_speed = next_speed
            self.cmd_vel_pub.publish(msg)
            threading.Timer(0.01, self.timer_callback).start()

        else:
            raise ValueError("Invalid option selected")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = tb_openloop_node()
        rclpy.spin(node)
    except ValueError as e:
        print(f"Error: {e}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
