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
        
        // Declare parameters for both topics
        this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("processed_topic", "/cv/processed_image");
        image_topic_ = this->get_parameter("image_topic").as_string();
        processed_topic_ = this->get_parameter("processed_topic").as_string();
        
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
        
        // Create subscriptions for both topics using SensorDataQoS for compatibility
        subscription_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 
            rclcpp::SensorDataQoS(),
            std::bind(&ImageDisplayNode::raw_image_callback, this, std::placeholders::_1));
        
        subscription_processed_ = this->create_subscription<sensor_msgs::msg::Image>(
            processed_topic_, 
            rclcpp::SensorDataQoS(),
            std::bind(&ImageDisplayNode::processed_image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s and %s topics", 
                    image_topic_.c_str(), processed_topic_.c_str());
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
    void raw_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            raw_image_ = cv_ptr->image.clone();
            has_raw_image_ = true;
            updateDisplay();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void processed_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            processed_image_ = cv_ptr->image.clone();
            has_processed_image_ = true;
            updateDisplay();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void updateDisplay()
    {
        // Create combined display with both images
        cv::Mat fitted = fitBothImagesToDisplay();
        convertAndDisplay(fitted);
    }
    
    cv::Mat fitBothImagesToDisplay()
    {
        // Layout heights
        const int username_height = 22;
        const int topic_height = 18;
        
        // Calculate available space for each image
        const int header_height = username_height + topic_height;
        const int middle_topic_height = topic_height;
        const int available_for_images = LCD_1IN69_HEIGHT - header_height - middle_topic_height;
        const int image_height = available_for_images / 2;  // Split remaining space
        
        // Create black canvas
        cv::Mat canvas = cv::Mat::zeros(LCD_1IN69_HEIGHT, LCD_1IN69_WIDTH, CV_8UC3);
        
        int current_y = header_height;
        
        // Process and place first image (raw)
        if (has_raw_image_ && !raw_image_.empty()) {
            cv::Mat resized = resizeToFit(raw_image_, LCD_1IN69_WIDTH, image_height);
            int x_offset = (LCD_1IN69_WIDTH - resized.cols) / 2;
            resized.copyTo(canvas(cv::Rect(x_offset, current_y, resized.cols, resized.rows)));
            current_y += image_height;
        } else {
            current_y += image_height;
        }
        
        current_y += middle_topic_height;
        
        // Process and place second image (processed)
        if (has_processed_image_ && !processed_image_.empty()) {
            cv::Mat resized = resizeToFit(processed_image_, LCD_1IN69_WIDTH, image_height);
            int x_offset = (LCD_1IN69_WIDTH - resized.cols) / 2;
            if (current_y + resized.rows <= LCD_1IN69_HEIGHT) {
                resized.copyTo(canvas(cv::Rect(x_offset, current_y, resized.cols, resized.rows)));
            }
        }
        
        return canvas;
    }
    
    cv::Mat resizeToFit(const cv::Mat& image, int max_width, int max_height)
    {
        float scale_width = static_cast<float>(max_width) / image.cols;
        float scale_height = static_cast<float>(max_height) / image.rows;
        float scale = std::min(scale_width, scale_height);
        
        int new_width = static_cast<int>(image.cols * scale);
        int new_height = static_cast<int>(image.rows * scale);
        
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
        
        return resized;
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
        
        // Draw username at the very top in yellow, centered horizontally
        const char* username = "@aldajo92";
        int username_width = strlen(username) * Font20.Width;
        int username_x = (LCD_1IN69_WIDTH - username_width) / 2;
        Paint_DrawString_EN(username_x, 2, username, &Font20, BLACK, YELLOW);
        
        // Draw first topic name below username
        std::string display_topic1 = image_topic_;
        if (display_topic1.length() > 35) {
            display_topic1 = display_topic1.substr(0, 32) + "...";
        }
        Paint_DrawString_EN(5, 24, display_topic1.c_str(), &Font12, BLACK, CYAN);
        
        // Draw second topic name in the middle
        std::string display_topic2 = processed_topic_;
        if (display_topic2.length() > 35) {
            display_topic2 = display_topic2.substr(0, 32) + "...";
        }
        Paint_DrawString_EN(5, 140, display_topic2.c_str(), &Font12, BLACK, MAGENTA);
        
        // Display the buffer
        LCD_1IN69_Display(image_buffer_);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_raw_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_processed_;
    UWORD *image_buffer_;
    std::string image_topic_;
    std::string processed_topic_;
    cv::Mat raw_image_;
    cv::Mat processed_image_;
    bool has_raw_image_ = false;
    bool has_processed_image_ = false;
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

