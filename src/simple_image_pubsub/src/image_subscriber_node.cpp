/*
Simple subscriber to subscribe to the pubished images
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <softsys_msgs/msg/steer.hpp>
#include <softsys_msgs/msg/throttle.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdio.h>
#include <stdarg.h>
#include <std_msgs/msg/float64.hpp>
#define PI 3.1415926

void pointGen(cv::Mat inputImage, cv::Mat& outputImage, int& latErr, float& yawErr);
void IPMTool(cv::Mat inputImage, cv::Mat& OutputImage);
void HSVTool(cv::Mat inputImage, cv::Mat& outputImage);
void PIDController(float Kp, float Ki, float Kd, float dt, float yawError, float& controlOutput);

float proportional;
float integral;
float derivative;
float previousError;
float previousSteerAngle;
float error;
float MAXSTEER = 1; // steer is -100 to 100 and is a float
float MAXSTEERANGLE = 20; // degrees
float MAXTHROTTLE = 1; // maximum throttle value is from 0 to 100 and is a float
//int cameray = 720;
//int camerax = 1280;

class ImageSubscriberNode : public rclcpp::Node
{
public:
  ImageSubscriberNode() : Node("image_subscriber_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/softsys/image_raw", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        processImageMessage(msg);
      });
    publisher_s_ = this->create_publisher<softsys_msgs::msg::Steer>("/softsys/steer_cmd", 10);
    publisher_t_ = this->create_publisher<softsys_msgs::msg::Throttle>("/softsys/throttle_cmd", 10);

  }

private:
  void processImageMessage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
        
    int latError;
    float yawError;
    float throttleOutput;
    float controlOutput;

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat originalImage = cv_ptr->image;

        cv::Mat outputImage;
        cv::Mat IPMImage;
        cv::Mat HSVImage;
        cv::Mat croppedImage;
        cv::Mat combined;

        //IPMTool(originalImage, IPMImage);
        
        HSVTool(originalImage, HSVImage);

	//std::cout << "HSV tool has run" << "\n";

        //croppedImage = HSVImage(cv::Range(350, 500), cv::Range(540,740));
        //imgage(Range(start_row, end_row), Range(start_col, end_col)) 
        // the image is indexed from the top left corner of the matrix

        pointGen(HSVImage, outputImage,latError, yawError);
        
        //std::cout << "Pointgen has run" << "\n";

        PIDController(0.02,0.00001,0.0008,0.01, yawError, controlOutput);
        //std::cout << controlOutput << "\n";
        //std::cout << yawError << "\n";
        
        //std::cout << "PID has run" << "\n";



        //Declaring the messages and the time stamp portion of the header
        softsys_msgs::msg::Steer steer_msg;
        softsys_msgs::msg::Throttle throttle_msg;

        steer_msg.steer_angle = controlOutput;

        throttleOutput = 1.0;

        if (abs(controlOutput) >0.4){
            throttleOutput = 0.8- ((0.12 * abs(controlOutput)));
        }

        throttle_msg.throttle = throttleOutput;

        steer_msg.header.stamp = get_clock()->now();
        throttle_msg.header.stamp = get_clock()->now();

        publisher_s_->publish(steer_msg);
        publisher_t_->publish(throttle_msg);
        
        
        //std::cout << "Values have been published" << "\n";

        // display image
        //cv::imshow("IPM Image", IPMImage);
        //cv::imshow("HSV Image", HSVImage);
        //cv::imshow("output Image", outputImage);
        //cv::imshow("Original Image", originalImage);
        
        //cv::waitKey(25);
        
        //std::cout << "imshow has run" << "\n";
        
        

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error converting sensor_msgs::Image to cv::Mat: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<softsys_msgs::msg::Steer>::SharedPtr publisher_s_;
  rclcpp::Publisher<softsys_msgs::msg::Throttle>::SharedPtr publisher_t_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



void pointGen(cv::Mat inputImage, cv::Mat& outputImage, int& latErr, float& yawErr){
  // Bring in image from file, must be in folder, and displaying the original
        cv::Mat source = inputImage.clone();
        cv::Mat Original = inputImage.clone();
        //cv::cvtColor(source,source,CV_BGR2HSV);
        //cv::inRange(source, cv::Scalar(0,0,133), cv::Scalar(102,255,256), source);
        
    // mat container to receive images
        cv::Mat destination;        //Destination is the Image we are operating on 
        int Centx;              //Variable for Storing X Value
        cv::Mat eElement=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2,2)); //Errosion Element
        cv::Mat dElement=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)); //Dialate


    //Sliders
        //Slider Box for REGION of INTREST:
        int Rectx=340,Recty=200,Rectw=600,Recth=200, ELx=(source.cols/2)-50;

        //Creating a destination to represent the new frame 
            destination=source.clone();
        //Creating an image to draw on 
            cv::Mat Drawing = Original.clone();
            //cv::cvtColor(Drawing,Drawing, CV_BGR2GRAY);
            //cv::cvtColor(Drawing,Drawing, CV_GRAY2RGB);
        //Defining the Region of Interest Rectangle;
            cv::Rect rect(Rectx,Recty,Rectw,Recth);
        //Erroding and Dialating the Image
            //imshow("Before Erosion",destination);
            cv::erode(destination,destination,eElement);
            cv::dilate(destination,destination,dElement);
        //Determining the Contours
            std::vector<std::vector<cv::Point> > contours;
            cv::findContours( destination, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE );
            cv::Mat contourImage(destination.size(), CV_8UC1, cv::Scalar(0));
        //Determining Contour with Max Area
            double maxArea=0;
            int maxElement;
            if(contours.size()>0){
                //int numObjects=contours.size();
                for(auto i=0; i<contours.size(); i++){
                    double newArea = cv::contourArea(contours[i]);
                    if (newArea>maxArea){
                        maxArea = newArea;
                        maxElement=i;
                    }
                }
            }
            //Drawing the Largest Contour                            
            cv::drawContours(contourImage, contours, maxElement, cv::Scalar(255), CV_FILLED);
            //imshow("Contour",contourImage);
         //Defining the Region of Interest
            cv::Mat ROI(contourImage, rect);
            //imshow("ROI",ROI);
        //Finding the Contours in the Region of Interest:
            std::vector<std::vector<cv::Point> > ROIcontours;
            cv::findContours( ROI, ROIcontours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE );
            cv::Moments m = moments(ROI,true);
            Centx=m.m10/m.m00;   
        

        //If Statement that prevents Segfault if there are is no line detected in REGION of Interest
            if(ROIcontours.size()>0){
                //Outputting the Mean Pixel Value X Position in ROI frame
                    //std::cout<<Centx<<std::endl;
                //Lateral Error Determination 
                    int LatE=Centx+Rectx-ELx;
                //Yaw Error Determination  -- To use atan values must be doubles
                    double LatEDoub = LatE, RowsDoub=destination.rows;
                    double OA = LatEDoub/(RowsDoub-(Recty-Recth/2));
                    float YawE=atan(OA)*180/PI;
                    //std::cout << YawE << "\n";
                    
                //Drawing Things 
                    //Region of Interest
                        cv::rectangle(Drawing, rect, cv::Scalar(  252, 153, 255),0.5,0 );
                    //Egoline
                        cv::line (Drawing, cv::Point(ELx,1), cv::Point(ELx,Drawing.rows-1),  cv::Scalar(158,230,117), 2, cv::LINE_8, 0);
                    //Line Between Point and Egoline
                        cv::line (Drawing, cv::Point(ELx,Recty+Recth/2), cv::Point(Centx+Rectx,Recty+Recth/2), cv::Scalar(  128, 200, 255), 2, cv::LINE_8, 0);
                    //Line Between Point and Base of Egoline
                        cv::line (Drawing, cv::Point(ELx,destination.rows-1),cv::Point(Centx+Rectx,Recty+Recth/2),cv::Scalar(  255, 160, 153), 2, cv::LINE_8, 0);
                    //Circle of Point on Line (Detected Point)
                        cv::circle(Drawing,cv::Point(Centx+Rectx,Recty+Recth/2),1,cv::Scalar(  89, 100, 255),2,4,0 );
                    //Circle at Base of Ego-Line
                        cv::circle(Drawing,cv::Point(ELx,destination.rows-1),1,cv::Scalar(  89, 100, 255),2,4,0 );
                    //Printing Lateral Error on Image
                        std::string LatEstring=std::to_string(LatE);
                        putText(Drawing, LatEstring , cv::Point(15, 15), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(  128, 200, 255),2,0);
                    //Printing Yaw Error on Image
                        std::string YawEstring=std::to_string(YawE);
                        putText(Drawing, YawEstring , cv::Point(15, 35), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 160, 153),2,0);
                    
                    yawErr = YawE;
                    latErr = LatE;
            }
            //Displaying Drawing
                outputImage = Drawing.clone();
                
    


}




void HSVTool(cv::Mat inputImage, cv::Mat& mask){
  //cv::Mat mask;
  cv::Mat hsv_image;

  cv::cvtColor(inputImage, hsv_image, cv::COLOR_BGR2HSV);

  cv::Scalar hsv_min(79,38,126);
  cv::Scalar hsv_max(132,155,256);

  
  cv::inRange(hsv_image, hsv_min, hsv_max, mask);

  //cv::bitwise_and(inputImage, inputImage, outputImage, mask);
}






void IPMTool(cv::Mat inputImage, cv::Mat& outputImage){

// IPM Tool:

        //Parameters:
          int alpha_ = 278;
          int beta_ = 90;
          int gamma_ = 90;
          int f_ = 742;
          int dist_ = 133;

        //Converting Parameters to Degrees
            double focalLength, dist, alpha, beta, gamma; 
        
            alpha = alpha_ * PI/180;
            beta =((double)beta_ -90) * PI/180;
            gamma =((double)gamma_ -90) * PI/180;
            focalLength = (double)f_;
            dist = (double)dist_;


        //Resizing
            cv::Size image_size = inputImage.size();
            double w = (double)image_size.width;
            double h = (double)image_size.height;


        // Projecion matrix 2D -> 3D
            cv::Mat A1 = (cv::Mat_<float>(4, 3)<< 
                1, 0, -w/2,
                0, 1, -h/2,
                0, 0, 0,
                0, 0, 1 );


        // Rotation matrices Rx, Ry, Rz

            cv::Mat RX = (cv::Mat_<float>(4, 4) << 
                1, 0, 0, 0,
                0, cos(alpha), -sin(alpha), 0,
                0, sin(alpha), -cos(alpha), 0,
                0, 0, 0, 1 );

            cv::Mat RY = (cv::Mat_<float>(4, 4) << 
                cos(beta), 0, -sin(beta), 0,
                0, 1, 0, 0,
                sin(beta), 0, cos(beta), 0,
                0, 0, 0, 1  );

            cv::Mat RZ = (cv::Mat_<float>(4, 4) << 
                cos(gamma), -sin(gamma), 0, 0,
                sin(gamma), cos(gamma), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1  );


        // R - rotation matrix
            cv::Mat R = RX * RY * RZ;

        // T - translation matrix
            cv::Mat T = (cv::Mat_<float>(4, 4) << 
                1, 0, 0, 0,  
                0, 1, 0, 0,  
                0, 0, 1, dist,  
                0, 0, 0, 1); 

        // K - intrinsic matrix 
            cv::Mat K = (cv::Mat_<float>(3, 4) << 
                focalLength, 0, w/2, 0,
                0, focalLength, h/2, 0,
                0, 0, 1, 0
                ); 


        cv::Mat transformationMat = K * (T * (R * A1));

        cv::warpPerspective(inputImage, outputImage, transformationMat, image_size, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

}


void PIDController(float Kp, float Ki, float Kd, float dt, float yawError, float& controlOutput){

    
    
    //error = previousSteerAngle - yawError;
    error = -yawError;

    proportional = Kp * error;
    integral += Ki * error;
    derivative = Kd * ((error - previousError)/dt);

    controlOutput = proportional + integral + derivative;

    if (controlOutput > MAXSTEER) {
        controlOutput = MAXSTEER;
    } else if(controlOutput < -MAXSTEER) {
        controlOutput = -MAXSTEER;
    }

    previousError = error;
    previousSteerAngle = (controlOutput/100)*MAXSTEERANGLE;
}