#!/usr/bin/python3
import rospy
import os
import rospkg
from pydantic import BaseModel
from spawn_formation import *


class LaunchParams(BaseModel):
    """
    Represents the launch parameters for the robots.

    Attributes:
        num_of_robots (int): The number of robots.
        robot_model (str): The model of the robot.
        distribution (str): The distribution method.
        distribution_list (dict): The distribution list.
    """
    num_of_robots: int
    robot_model: str
    distribution_type: str
    distribution_params: dict

class LaunchFileGenerator:
    """
    A class to generate a launch file for spawning robots in Gazebo.

    Attributes
    ----------
    params : LaunchParams
        An instance of the LaunchParams class that contains the parameters for the launch file.
    num_robots : int
        The number of robots to be spawned.
    model : str
        The model of the robots to be spawned.
    start_formation_type : str
        The type of formation in which the robots should be spawned.
    distribution : dict
        The parameters for the distribution of the robots.
    output_file_name : str
        The path to the output launch file.

    Methods
    -------
    __init__(self, params: LaunchParams):
        Initializes the LaunchFileGenerator with parameters from a LaunchParams instance.

    load_params(config: dict) -> LaunchParams:
        Reads parameters from a dictionary and returns a LaunchParams instance.

    generate_launch_file(self):
        Generates a launch file for spawning robots in Gazebo.

    run(self):
        Runs the LaunchFileGenerator.
    """
   
    def __init__(self, params: LaunchParams):
        """Initializes the LaunchFileGenerator with parameters from a YAML file."""
        self.params = params
        self.num_robots = self.params.num_of_robots
        self.model = self.params.robot_model
        self.start_formation_type = self.params.distribution_type
        self.distribution = self.params.distribution_params
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('gazebo_multi_robot_spawn')  # Replace with your package name
        self.output_file_name = os.path.join(os.path.join(package_path, 'launch'), 'spawn_multi_robot.launch')

    @staticmethod
    def load_params(config: dict) -> LaunchParams:
        """Reads parameters from a dictionary and returns a LaunchParams instance."""
        return LaunchParams(**config)

    def generate_launch_file(self):
        """Generates a launch file for spawning robots in Gazebo."""
        with open(self.output_file_name, "w") as f:
            f.write('<launch>\n')
            f.write('    <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>\n')

            for k in range(self.num_robots):
                if self.start_formation_type == 'circle':
                    formation_class = Circle(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                elif self.start_formation_type == 'line':
                    formation_class = Line(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                elif self.start_formation_type == 'two_lines':
                    formation_class = TwoLines(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                elif self.start_formation_type == 'three_lines':
                    formation_class = ThreeLines(**self.distribution)
                    x, y = formation_class.distribute(k, self.num_robots)
                f.write(f'    <arg name="robot_{k}_pos_x" default="{x}"/>\n')
                f.write(f'    <arg name="robot_{k}_pos_y" default="{y}"/>\n')
                f.write(f'    <arg name="robot_{k}_pos_z" default="0.0"/>\n')
                f.write(f'    <arg name="robot_{k}_yaw" default="0"/>\n')

            f.write('    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />\n')

            for k in range(self.num_robots):
                f.write(f'    <group ns="robot_{k}">\n')
                f.write('        <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">\n')
                f.write('            <param name="publish_frequency" type="double" value="50.0" />\n')
                f.write(f'            <param name="tf_prefix" value="robot_{k}" />\n')
                f.write('        </node>\n')
                f.write(f'        <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -model robot_{k} -x $(arg robot_{k}_pos_x) -y $(arg robot_{k}_pos_y) -z $(arg robot_{k}_pos_z) -Y $(arg robot_{k}_yaw) -param /robot_description" />\n')
                f.write('    </group>\n')

            f.write('</launch>\n')

    def run(self):
        """Runs the LaunchFileGenerator."""
        rospy.init_node('launch_file_generator', anonymous=True)
        print('running')
        self.generate_launch_file()
        rospy.loginfo("Launch file generated at: %s", self.output_file_name)
        rospy.signal_shutdown("Work Completed")


if __name__ == "__main__":
    try:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('gazebo_multi_robot_spawn')  # Replace with new package name if altered. 
        config = {
            "num_of_robots": rospy.get_param('num_of_robots'),
            "robot_model": rospy.get_param('robot_model'),
            "distribution_type": rospy.get_param('distribution_type'),
            "distribution_params": rospy.get_param('distribution_params')
        }
        params = LaunchFileGenerator.load_params(config)
        generator = LaunchFileGenerator(params)
        generator.run()
    except rospy.ROSInterruptException:
        pass
