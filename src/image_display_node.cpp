#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

extern "C" {
    #include "waveshare/DEV_Config.h"
    #include "waveshare/LCD_1in69.h"
    #include "waveshare/GUI_Paint.h"
    #include "waveshare/fonts.h"
}

class ImageDisplayNode : public rclcpp::Node
{
public:
    ImageDisplayNode() : Node("image_display_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ST7789V2 image display...");
        
        // Initialize hardware
        if(DEV_ModuleInit() != 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize hardware!");
            RCLCPP_ERROR(this->get_logger(), "Make sure you are running as root (sudo) and SPI is enabled");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize LCD
        LCD_1IN69_Init(VERTICAL);
        LCD_1IN69_Clear(BLACK);
        LCD_SetBacklight(1023);
        
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
        Paint_NewImage(image_buffer_, LCD_1IN69_WIDTH, LCD_1IN69_HEIGHT, 0, BLACK, 16);
        Paint_Clear(BLACK);
        
        // Show startup message
        Paint_DrawString_EN(10, 130, "Camera Display", &Font24, BLACK, WHITE);
        Paint_DrawString_EN(10, 160, "Waiting...", &Font16, BLACK, CYAN);
        LCD_1IN69_Display(image_buffer_);
        
        RCLCPP_INFO(this->get_logger(), "Display initialized successfully");
        
        // Create subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 
            10,
            std::bind(&ImageDisplayNode::image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /camera/image_raw topic");
    }
    
    ~ImageDisplayNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down image display node...");
        
        if (image_buffer_) {
            Paint_Clear(BLACK);
            Paint_DrawString_EN(10, 130, "Shutting down...", &Font20, BLACK, RED);
            LCD_1IN69_Display(image_buffer_);
            DEV_Delay_ms(1000);
            
            free(image_buffer_);
            image_buffer_ = nullptr;
        }
        
        DEV_ModuleExit();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // Calculate aspect ratio and resize to fit display while maintaining ratio
            cv::Mat fitted = fitImageToDisplay(cv_ptr->image);
            
            // Convert to RGB565 and display
            convertAndDisplay(fitted);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    cv::Mat fitImageToDisplay(const cv::Mat& image)
    {
        // Reserve space for topic name at top (25 pixels for text)
        const int text_height = 25;
        const int available_height = LCD_1IN69_HEIGHT - text_height;
        
        // Calculate scaling factor to fit image in remaining space while maintaining aspect ratio
        float scale_width = static_cast<float>(LCD_1IN69_WIDTH) / image.cols;
        float scale_height = static_cast<float>(available_height) / image.rows;
        float scale = std::min(scale_width, scale_height);
        
        // Calculate new size maintaining aspect ratio
        int new_width = static_cast<int>(image.cols * scale);
        int new_height = static_cast<int>(image.rows * scale);
        
        // Resize image with high-quality interpolation
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
        
        // Create black canvas of display size
        cv::Mat canvas = cv::Mat::zeros(LCD_1IN69_HEIGHT, LCD_1IN69_WIDTH, image.type());
        
        // Calculate position - centered horizontally, below text at top
        int x_offset = (LCD_1IN69_WIDTH - new_width) / 2;
        int y_offset = text_height;  // Start after text area
        
        // Copy resized image to canvas below text area
        resized.copyTo(canvas(cv::Rect(x_offset, y_offset, new_width, new_height)));
        
        return canvas;
    }
    
    void convertAndDisplay(const cv::Mat& image)
    {
        // First, clear the entire buffer
        memset(image_buffer_, 0, LCD_1IN69_HEIGHT * LCD_1IN69_WIDTH * 2);
        
        // Convert BGR to RGB565 format with proper byte order
        for (int y = 0; y < image.rows && y < LCD_1IN69_HEIGHT; y++) {
            for (int x = 0; x < image.cols && x < LCD_1IN69_WIDTH; x++) {
                cv::Vec3b pixel = image.at<cv::Vec3b>(y, x);
                
                // OpenCV uses BGR order, extract channels
                uint8_t b = pixel[0]; // Blue
                uint8_t g = pixel[1]; // Green  
                uint8_t r = pixel[2]; // Red
                
                // Convert 8-bit BGR to 5-6-5 RGB format
                // RGB565 format: RRRR RGGG GGGB BBBB (big-endian)
                uint8_t r5 = (r >> 3) & 0x1F;  // 5 bits for red
                uint8_t g6 = (g >> 2) & 0x3F;  // 6 bits for green
                uint8_t b5 = (b >> 3) & 0x1F;  // 5 bits for blue
                
                // Pack into RGB565 - swap bytes for correct display order
                uint16_t rgb565 = (r5 << 11) | (g6 << 5) | b5;
                
                // Swap bytes for proper display (little-endian to big-endian)
                image_buffer_[y * LCD_1IN69_WIDTH + x] = ((rgb565 & 0xFF) << 8) | ((rgb565 >> 8) & 0xFF);
            }
        }
        
        // Initialize Paint library with the buffer
        Paint_NewImage(image_buffer_, LCD_1IN69_WIDTH, LCD_1IN69_HEIGHT, 0, BLACK, 16);
        
        // Draw topic name at the top
        Paint_DrawString_EN(5, 5, "/camera/image_raw", &Font16, BLACK, WHITE);
        
        // Display the buffer
        LCD_1IN69_Display(image_buffer_);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    UWORD *image_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImageDisplayNode>();
    
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    
    rclcpp::shutdown();
    return 0;
}

