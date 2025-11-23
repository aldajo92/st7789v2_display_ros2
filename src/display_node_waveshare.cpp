#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

extern "C" {
    #include "waveshare/DEV_Config.h"
    #include "waveshare/LCD_1in69.h"
    #include "waveshare/GUI_Paint.h"
    #include "waveshare/fonts.h"
}

class DisplayNode : public rclcpp::Node
{
public:
    DisplayNode() : Node("st7789v2_display_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ST7789V2 display...");
        
        // Initialize hardware
        if(DEV_ModuleInit() != 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize hardware!");
            RCLCPP_ERROR(this->get_logger(), "Make sure you are running as root (sudo) and SPI is enabled");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize LCD
        LCD_1IN69_Init(VERTICAL);
        LCD_1IN69_Clear(WHITE);
        LCD_SetBacklight(1023);  // Maximum brightness
        
        // Allocate image buffer
        UDOUBLE Imagesize = LCD_1IN69_HEIGHT * LCD_1IN69_WIDTH * 2;
        image_buffer_ = (UWORD *)malloc(Imagesize);
        if(image_buffer_ == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate image buffer!");
            DEV_ModuleExit();
            rclcpp::shutdown();
            return;
        }
        
        // Initialize Paint library
        Paint_NewImage(image_buffer_, LCD_1IN69_WIDTH, LCD_1IN69_HEIGHT, 0, WHITE, 16);
        Paint_Clear(WHITE);
        Paint_SetRotate(ROTATE_0);
        
        // Show startup message
        Paint_Clear(BLACK);
        Paint_DrawString_EN(10, 10, "ST7789V2 Display", &Font20, BLACK, WHITE);
        Paint_DrawString_EN(10, 40, "ROS2 Ready!", &Font20, BLACK, GREEN);
        Paint_DrawString_EN(10, 70, "Waiting for text...", &Font16, BLACK, CYAN);
        LCD_1IN69_Display(image_buffer_);
        
        RCLCPP_INFO(this->get_logger(), "Display initialized successfully");
        
        // Create subscription
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "st7789v2_display/text", 
            10,
            std::bind(&DisplayNode::text_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to st7789v2_display/text topic");
    }
    
    ~DisplayNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down display node...");
        
        if (image_buffer_) {
            // Show shutdown message
            Paint_Clear(BLACK);
            Paint_DrawString_EN(10, 140, "Shutting down...", &Font20, BLACK, RED);
            LCD_1IN69_Display(image_buffer_);
            DEV_Delay_ms(1000);
            
            // Clean up
            free(image_buffer_);
            image_buffer_ = nullptr;
        }
        
        DEV_ModuleExit();
    }

private:
    void text_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received text: '%s'", msg->data.c_str());
        
        // Clear screen
        Paint_Clear(WHITE);
        
        // Draw border
        Paint_DrawRectangle(5, 5, LCD_1IN69_WIDTH - 5, LCD_1IN69_HEIGHT - 5, BLUE, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
        
        // Draw title
        Paint_DrawString_EN(10, 10, "Message Received:", &Font16, WHITE, BLACK);
        
        // Draw the message text (support multi-line)
        drawMultilineText(15, 35, msg->data.c_str(), &Font20, WHITE, RED);
        
        // Draw timestamp indicator
        Paint_DrawString_EN(10, LCD_1IN69_HEIGHT - 25, "Updated!", &Font12, WHITE, GREEN);
        
        // Display the buffer to LCD
        LCD_1IN69_Display(image_buffer_);
    }
    
    void drawMultilineText(int x, int y, const char* text, sFONT* font, UWORD bg_color, UWORD text_color)
    {
        int current_y = y;
        int current_x = x;
        const int line_height = font->Height + 5;
        const int max_width = LCD_1IN69_WIDTH - 30;  // Leave margin
        
        for (int i = 0; text[i] != '\0'; i++) {
            if (text[i] == '\n') {
                current_y += line_height;
                current_x = x;
                continue;
            }
            
            // Check if we need to wrap to next line
            if (current_x + font->Width > max_width) {
                current_y += line_height;
                current_x = x;
            }
            
            // Stop if we're running out of vertical space
            if (current_y + font->Height > LCD_1IN69_HEIGHT - 30) {
                break;
            }
            
            // Draw character
            char str[2] = {text[i], '\0'};
            Paint_DrawString_EN(current_x, current_y, str, font, bg_color, text_color);
            current_x += font->Width;
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    UWORD *image_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DisplayNode>();
    
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    
    rclcpp::shutdown();
    return 0;
}

