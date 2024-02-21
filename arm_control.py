import sys
import rospy
import moveit_commander
import geometry_msgs.msg

class RobotController:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robot_controller", anonymous=True)

        robot = moveit_commander.RobotCommander()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        scene = moveit_commander.PlanningSceneInterface()

        self.move_group = move_group
        self.robot = robot
        self.scene = scene

    def move_to_joint_state(self, joint_goal):
        print("Moving to joint state:", joint_goal)
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def move_to_pose_goal(self, pose_goal):
        print("Moving to pose goal:", pose_goal)
        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def add_box(self, box_pose, box_size, timeout=4):
        print("Adding box to the scene")
        self.scene.add_box("box", box_pose, size=box_size)
        rospy.sleep(1)

    def attach_box(self):
        print("Attaching box to the robot's end effector")
        eef_link = self.move_group.get_end_effector_link()
        touch_links = self.robot.get_link_names("panda_hand")
        self.scene.attach_box(eef_link, "box", touch_links=touch_links)
        rospy.sleep(1)

    def detach_box(self):
        print("Detaching box from the robot's end effector")
        self.scene.remove_attached_object(self.move_group.get_end_effector_link(), name="box")
        rospy.sleep(1)

    def remove_box(self):
        print("Removing box from the scene")
        self.scene.remove_world_object("box")
        rospy.sleep(1)

    def execute_plan(self, plan):
        print("Executing the planned path")
        self.move_group.execute(plan, wait=True)
        rospy.sleep(1)

def main():
    try:
        controller = RobotController()

        # Move to a predefined joint state
        joint_goal = [0, -1.0, 0, -2.0, 0, 1.5, 0]
        controller.move_to_joint_state(joint_goal)

        # Move to a specific pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.2
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        controller.move_to_pose_goal(pose_goal)

        # Add a box to the scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11
        box_size = (0.075, 0.075, 0.075)
        controller.add_box(box_pose, box_size)

        # Attach the box to the robot's end effector
        controller.attach_box()

        # Move to a goal position
        goal_pose = geometry_msgs.msg.Pose()
        goal_pose.orientation.w = 1.0
        goal_pose.position.x = -0.5
        goal_pose.position.y = -0.5
        goal_pose.position.z = 0.5
        controller.move_to_pose_goal(goal_pose)

        # Detach the box from the robot's end effector
        controller.detach_box()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
