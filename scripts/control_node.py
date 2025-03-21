#!/usr/bin/env python3
import rospy, re
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist

class ControlNode:
    def __init__(self):
        rospy.init_node('control_node')
        self.energy = 100
        self.is_moving = False
        self.energy_pub = rospy.Publisher('/energy', Int32, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/voice_commands', String, self.process_command)
        self.timer = None
        self.number_words = {
            'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4,
            'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9,
            'ten': 10
        }
        rospy.loginfo("Control Node Ready")

    def process_command(self, msg):
        cmd = msg.data.strip().lower()
        
        if cmd == "hill":
            #Here,i used hill to heal.Because whenever I say say heal it recognizer showed me hill
            self.energy = min(100, self.energy + 20)
            self.update_energy()
            rospy.loginfo(f"Healed! Energy: {self.energy}")
        elif cmd == "status":
            #It was not mentioned to include but i included as per my wish
            rospy.loginfo(f"ENERGY STATUS: {self.energy}/100")
        elif any(word in cmd for word in ["forward", "backward", "left", "right"]):
            self.handle_movement(cmd)
        elif "spin" in cmd:
            self.handle_spin(cmd)

    def parse_duration(self, cmd):
        """Parse numeric and word durations with/without time units"""
        # Numeric format (Like if i say "5s")
        digit_match = re.search(r'(\d+)\s*s', cmd)
        if digit_match:
            return int(digit_match.group(1))
        
        # Word format for saying like forward 2s------>two seconds
        word_match = re.search(
            r'\b(zero|one|two|three|four|five|six|seven|eight|nine|ten)\b(?:\s*seconds?)?', 
            cmd
        )
        if word_match:
            return self.number_words.get(word_match.group(1).lower(), 0)
        
        # Default durations
        return 2 if any(w in cmd for w in ["forward", "backward"]) else 1

    def handle_movement(self, cmd):
        if self.is_moving:
            rospy.logwarn("Already moving! Wait for completion.")
            return

        duration = self.parse_duration(cmd)
        move_type = "linear" if any(w in cmd for w in ["forward", "backward"]) else "angular"
        cost = duration * (10 if move_type == "linear" else 5)
        #keeping track of energy to control future interplaneter rover bot
        if self.energy < cost:
            rospy.logwarn(f"Need {cost} energy, only {self.energy} left!")
            return
            
        self.energy = max(0, self.energy - cost)
        self.update_energy()
        self.is_moving = True

        twist = Twist()
        if "forward" in cmd:
            twist.linear.x = 0.2
            rospy.loginfo(f"Forward {duration}s (Cost: {cost} energy)")
        elif "backward" in cmd:
            twist.linear.x = -0.2
            rospy.loginfo(f"Backward {duration}s (Cost: {cost} energy)")
        elif "left" in cmd:
            twist.angular.z = 1.0
            rospy.loginfo(f"Left {duration}s (Cost: {cost} energy)")
        elif "right" in cmd:
            twist.angular.z = -1.0
            rospy.loginfo(f"Right {duration}s (Cost: {cost} energy)")

        self.cmd_vel_pub.publish(twist)
        self.set_timer(duration)

    def handle_spin(self, cmd):
        if self.is_moving:
            rospy.logwarn("Movement in progress!")
            return

        duration = self.parse_duration(cmd) or 3  # Default 3s spin
        cost = duration * 15  # Higher energy cost for spinning

        if self.energy < cost:
            rospy.logwarn(f"Need {cost} energy to spin, only {self.energy} left!")
            return
            
        self.energy = max(0, self.energy - cost)
        self.update_energy()
        self.is_moving = True

        twist = Twist()
        twist.angular.z = 2.0  # Faster rotation
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"SPINNING {duration}s (Cost: {cost} energy)")
        self.set_timer(duration)

    def set_timer(self, duration):
        if self.timer: self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(duration), self.stop_robot, oneshot=True)

    def stop_robot(self, event):
        self.cmd_vel_pub.publish(Twist())
        self.is_moving = False
        rospy.loginfo(f"Movement complete. Energy: {self.energy}")

    def update_energy(self):
        self.energy_pub.publish(Int32(self.energy))

if __name__ == '__main__':
    ControlNode()
    rospy.spin()