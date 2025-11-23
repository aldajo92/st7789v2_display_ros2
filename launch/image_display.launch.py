from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for both topics
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Raw image topic to subscribe to'
    )
    
    processed_topic_arg = DeclareLaunchArgument(
        'processed_topic',
        default_value='/cv/processed_image',
        description='Processed image topic to subscribe to'
    )
    
    # Create the node with parameters
    image_display_node = Node(
        package="st7789v2_display",
        executable="image_display_node",
        name="image_display_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'processed_topic': LaunchConfiguration('processed_topic')
        }]
    )
    
    return LaunchDescription([
        image_topic_arg,
        processed_topic_arg,
        image_display_node
    ])

