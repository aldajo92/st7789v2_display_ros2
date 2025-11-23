# ST7789V2 Display ROS2 Package

This package provides a ROS2 node for controlling a 1.69" ST7789V2 LCD display (240x280) over SPI on Raspberry Pi.

## Features

- Direct SPI communication using bcm2835 library (no external dependencies needed)
- Subscribe to text messages and display them on the screen
- 240x280 pixel display support (1.69" LCD)
- Waveshare GUI library with professional font rendering
- RGB565 color support
- Multi-line text with automatic wrapping

## Hardware Requirements

- Raspberry Pi (tested on Pi 4)
- Waveshare 1.69" LCD Display (240x280 pixels, ST7789V2 controller)
- SPI connections:
  - MOSI: Pin 19 (GPIO 10)
  - SCLK: Pin 23 (GPIO 11)
  - CE0: Pin 24 (GPIO 8)
  - DC: Pin 18 (GPIO 25)
  - RST: Pin 22 (GPIO 27)
  - BL: Pin 12 (GPIO 18) - Backlight

## Build

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select st7789v2_display
source install/setup.bash
```

## Usage

### Text Display Node

Displays text messages on the LCD.

Using the launch file (recommended):

```bash
sudo -E bash -c "source /ros2_ws/install/setup.bash && ros2 launch st7789v2_display display.launch.py"
```

Or directly:

```bash
sudo -E bash -c "source /ros2_ws/install/setup.bash && ros2 run st7789v2_display display_node"
```

### Image Display Node

Displays camera images on the LCD (subscribes to `/camera/image_raw`).

Using the launch file (recommended):

```bash
sudo -E bash -c "source /ros2_ws/install/setup.bash && ros2 launch st7789v2_display image_display.launch.py"
```

Or directly:

```bash
sudo -E bash -c "source /ros2_ws/install/setup.bash && ros2 run st7789v2_display image_display_node"
```

### Publishing Text to Display

From another terminal:

```bash
source /ros2_ws/install/setup.bash
ros2 topic pub /st7789v2_display/text std_msgs/msg/String "data: 'Hello ROS2!'"
```

Or publish once:

```bash
ros2 topic pub --once /st7789v2_display/text std_msgs/msg/String "data: 'Hello World'"
```

### Multi-line Text

```bash
ros2 topic pub --once /st7789v2_display/text std_msgs/msg/String "data: 'Line 1\nLine 2\nLine 3'"
```

## Topics

### Text Display Node

**Subscribed Topics:**
- `/st7789v2_display/text` (`std_msgs/msg/String`): Text to display on the screen

### Image Display Node

**Subscribed Topics:**
- `/camera/image_raw` (`sensor_msgs/msg/Image`): Camera images to display on the screen

## Features

### Text Display Node Features
- **Automatic text wrapping**: Text automatically wraps to the next line when reaching the edge
- **Multi-line support**: Use `\n` for explicit line breaks
- **Professional fonts**: Uses Waveshare font library (Font12, Font16, Font20, Font24)
- **Color display**: White background with colored text
- **Bordered layout**: Blue border around the display area
- **Status indicator**: Shows "Updated!" when new messages arrive

### Image Display Node Features
- **Real-time camera display**: Shows live camera feed on the LCD
- **Automatic scaling**: Images are automatically resized to fit the 240x280 display
- **RGB565 conversion**: Converts camera images to the display's native color format
- **High performance**: Optimized for smooth video display
- **Any image source**: Works with any ROS2 image publisher

## Dependencies

- rclcpp
- std_msgs
- sensor_msgs
- cv_bridge
- OpenCV
- bcm2835 library (included in the package)
- Waveshare LCD library (included in the package)

## Notes

- **Root Access Required**: The node requires root privileges to access GPIO and SPI hardware
- **SPI Speed**: Configured for optimal performance with the ST7789V2 controller
- **Display Resolution**: 240x280 pixels (1.69" display)
- **Display Initialization**: On startup, shows "ST7789V2 Display ROS2 Ready!"
- **Clean Shutdown**: Displays "Shutting down..." before closing
- **Docker Privileged Mode**: When running in Docker, the container must be started with `--privileged` flag

## Troubleshooting

If you see "Failed to initialize hardware!":
1. Make sure you are running as root (sudo)
2. Verify SPI is enabled: `ls /dev/spi*`
3. Check hardware connections
4. Ensure Docker container is running with `--privileged` flag

Enable SPI if needed:
```bash
sudo raspi-config
# Go to Interface Options -> SPI -> Enable
```

## Display Specifications

- **Size**: 1.69 inches
- **Resolution**: 240×280 pixels
- **Controller**: ST7789V2
- **Communication**: 4-wire SPI
- **Colors**: RGB565 (65K colors)
- **Backlight**: PWM controllable

## Package Structure

```
st7789v2_display/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── display.launch.py           (Text display node)
│   └── image_display.launch.py     (Image display node)
└── src/
    ├── display_node_waveshare.cpp  (Text display ROS2 node)
    ├── image_display_node.cpp      (Image display ROS2 node)
    ├── bcm2835.c/h                 (SPI/GPIO library)
    └── waveshare/                  (Waveshare LCD library)
        ├── DEV_Config.c/h          (Hardware abstraction)
        ├── LCD_1in69.c/h           (1.69" display driver)
        ├── GUI_Paint.c/h           (Graphics library)
        └── font*.c/fonts.h         (Font library)
```

## License

This package includes the Waveshare LCD library and bcm2835 library by Mike McCauley, licensed under GPL V3.
