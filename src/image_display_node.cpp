#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

extern "C" {
    #include "waveshare/DEV_Config.h"
    #include "waveshare/LCD_1in69.h"
    #include "waveshare/LCD_1in69_fast.h"
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
        
        // Start rendering thread
        running_ = true;
        render_thread_ = std::thread(&ImageDisplayNode::renderLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "Rendering thread started at 20 FPS");
    }
    
    ~ImageDisplayNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down image display node...");
        
        // Stop rendering thread
        running_ = false;
        if (render_thread_.joinable()) {
            render_thread_.join();
        }
        
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
            
            // Fast downsample immediately if image is larger than display
            cv::Mat image = cv_ptr->image;
            if (image.cols > LCD_1IN69_WIDTH * 2 || image.rows > LCD_1IN69_HEIGHT * 2) {
                // Quick 2x downsample using pyrDown (very fast)
                cv::pyrDown(image, image);
                if (image.cols > LCD_1IN69_WIDTH * 2 || image.rows > LCD_1IN69_HEIGHT * 2) {
                    cv::pyrDown(image, image);  // Downsample again if still too large
                }
            }
            
            // Resize to exact fit
            const int available_height = (LCD_1IN69_HEIGHT - 40 - 18) / 2;
            cv::Mat resized = resizeToFit(image, LCD_1IN69_WIDTH, available_height);
            
            // Store preprocessed image in buffer
            std::lock_guard<std::mutex> lock(image_mutex_);
            raw_image_ = resized;
            has_raw_image_ = true;
            new_frame_available_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void processed_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // Fast downsample immediately if image is larger than display
            cv::Mat image = cv_ptr->image;
            if (image.cols > LCD_1IN69_WIDTH * 2 || image.rows > LCD_1IN69_HEIGHT * 2) {
                // Quick 2x downsample using pyrDown (very fast)
                cv::pyrDown(image, image);
                if (image.cols > LCD_1IN69_WIDTH * 2 || image.rows > LCD_1IN69_HEIGHT * 2) {
                    cv::pyrDown(image, image);  // Downsample again if still too large
                }
            }
            
            // Resize to exact fit
            const int available_height = (LCD_1IN69_HEIGHT - 40 - 18) / 2;
            cv::Mat resized = resizeToFit(image, LCD_1IN69_WIDTH, available_height);
            
            // Store preprocessed image in buffer
            std::lock_guard<std::mutex> lock(image_mutex_);
            processed_image_ = resized;
            has_processed_image_ = true;
            new_frame_available_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void renderLoop()
    {
        // Rendering loop running in separate thread
        const auto frame_duration = std::chrono::milliseconds(50);  // Reduced to 10 FPS for stability
        
        while (running_) {
            // Only update display if we have new frames
            if (new_frame_available_ && (has_raw_image_ || has_processed_image_)) {
                // Get latest PREPROCESSED images from buffers (already resized!)
                cv::Mat raw_copy, processed_copy;
                bool has_raw, has_processed;
                
                {
                    std::lock_guard<std::mutex> lock(image_mutex_);
                    if (has_raw_image_) {
                        raw_copy = raw_image_.clone();
                    }
                    if (has_processed_image_) {
                        processed_copy = processed_image_.clone();
                    }
                    has_raw = has_raw_image_;
                    has_processed = has_processed_image_;
                    new_frame_available_ = false;  // Reset flag
                }
                
                // Combine preprocessed images and display (fast!)
                cv::Mat combined = combineBothImages(raw_copy, processed_copy, has_raw, has_processed);
                convertAndDisplay(combined);
            }
            
            // Fixed sleep duration for consistent frame pacing
            std::this_thread::sleep_for(frame_duration);
        }
    }
    
    cv::Mat combineBothImages(const cv::Mat& raw_img, const cv::Mat& proc_img, 
                               bool has_raw, bool has_proc)
    {
        // Images are already resized in callbacks, just combine them
        const int header_height = 40;
        const int middle_topic_height = 18;
        const int image_height = (LCD_1IN69_HEIGHT - header_height - middle_topic_height) / 2;
        
        // Create black canvas
        cv::Mat canvas = cv::Mat::zeros(LCD_1IN69_HEIGHT, LCD_1IN69_WIDTH, CV_8UC3);
        
        int current_y = header_height;
        
        // Place first image (already resized)
        if (has_raw && !raw_img.empty()) {
            int x_offset = (LCD_1IN69_WIDTH - raw_img.cols) / 2;
            int y_center = current_y + (image_height - raw_img.rows) / 2;
            raw_img.copyTo(canvas(cv::Rect(x_offset, y_center, raw_img.cols, raw_img.rows)));
        }
        
        current_y += image_height + middle_topic_height;
        
        // Place second image (already resized)
        if (has_proc && !proc_img.empty()) {
            int x_offset = (LCD_1IN69_WIDTH - proc_img.cols) / 2;
            int y_center = current_y + (image_height - proc_img.rows) / 2;
            if (y_center + proc_img.rows <= LCD_1IN69_HEIGHT) {
                proc_img.copyTo(canvas(cv::Rect(x_offset, y_center, proc_img.cols, proc_img.rows)));
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
        // Use faster interpolation for better performance
        cv::resize(image, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);
        
        return resized;
    }
    
    void convertAndDisplay(const cv::Mat& image)
    {
        // Optimized RGB565 conversion with batch SPI transfer
        const int total_pixels = LCD_1IN69_WIDTH * LCD_1IN69_HEIGHT;
        
        // Fast conversion loop - optimized pixel access
        int idx = 0;
        for (int y = 0; y < image.rows && y < LCD_1IN69_HEIGHT; y++) {
            const cv::Vec3b* row_ptr = image.ptr<cv::Vec3b>(y);
            for (int x = 0; x < image.cols && x < LCD_1IN69_WIDTH; x++, idx++) {
                const cv::Vec3b& pixel = row_ptr[x];
                
                // Fast RGB565 conversion with bit operations
                uint16_t rgb565 = ((pixel[2] & 0xF8) << 8) |   // Red (5 bits)
                                  ((pixel[1] & 0xFC) << 3) |   // Green (6 bits)
                                  ((pixel[0] >> 3));           // Blue (5 bits)
                
                // Byte swap for display
                image_buffer_[idx] = ((rgb565 & 0xFF) << 8) | ((rgb565 >> 8) & 0xFF);
            }
            // Fill rest of row with black
            for (int x = image.cols; x < LCD_1IN69_WIDTH; x++, idx++) {
                image_buffer_[idx] = 0;
            }
        }
        
        // Fill remaining rows with black
        for (int i = idx; i < total_pixels; i++) {
            image_buffer_[i] = 0;
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
        
        // Display the buffer using fast single-transfer method
        LCD_1IN69_Display_Fast(image_buffer_);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_raw_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_processed_;
    UWORD *image_buffer_;
    std::string image_topic_;
    std::string processed_topic_;
    
    // Rendering thread
    std::thread render_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> new_frame_available_;
    
    // Image buffers protected by mutex
    std::mutex image_mutex_;
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

