#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <time.h>

/*                                                                                        */
/* -----------Thresholding constants for detection (1 = ELP, 2 = Red Dropzone)----------- */
/*                                                                                        */                                                                            

#define BLUR_KERNEL_WIDTH 1
#define LOW_H1 24
#define HIGH_H1 39
#define LOW_H2 0
#define HIGH_H2 180
#define LOW_S1 0
#define HIGH_S1 255
#define LOW_S2 97
#define HIGH_S2 209
#define LOW_V1 155
#define HIGH_V1 255
#define LOW_V2 213
#define HIGH_V2 255
#define OPENING_KERNEL_WIDTH 1
#define OPENING_APPLICATION_COUNT 0
#define CLOSING_KERNEL_WIDTH 1
#define CLOSING_APPLICATION_COUNT 0
static const cv::String WIN_CAPTURE_NAME = "Video Capture";
static const cv::String WIN_DETECTION_NAME = "Object Detection";


/*---------------------------------------MAIN---------------------------------------------*/

int main(int argc, char *argv[]){


    /*---------Camera and Window Initialization---------*/
    // cv::VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
    cv::VideoCapture cap("/dev/video2");
    cv::Mat frame, frame_HSV, frame_threshold;
    cv::namedWindow(WIN_CAPTURE_NAME);
    cv::namedWindow(WIN_DETECTION_NAME);


    /*----------------FPS Counter Variables----------------*/
    float fps = cap.get(cv::CAP_PROP_FPS), seconds, fps_live;
    int num_frames = 1;
    std::time_t start, end;


    while (true) {
        start = clock();
        cap >> frame;
        if (frame.empty()){
            break;
        }
        // Convert BGR to HSV colorspaces
        cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);

        // Detect object based on HSV range values {YELLOW FOR ELP} 
        // cv::inRange(frame_HSV, cv::Scalar(LOW_H1, LOW_S1, LOW_V1), 
        //             cv::Scalar(HIGH_H1, HIGH_S1, HIGH_V1), 
        //             frame_threshold);
        cv::inRange(frame_HSV, cv::Scalar(LOW_H2, LOW_S2, LOW_V2), 
                    cv::Scalar(HIGH_H2, HIGH_S2, HIGH_V2), 
                    frame_threshold);
        end = clock();

        // Time elapsed 
        seconds = (float(end)-float(start)) / float(CLOCKS_PER_SEC);

        fps_live = num_frames/seconds;

        cv::putText(frame_threshold,"FPS: " + std::to_string(fps_live), {50, 50}, cv::FONT_HERSHEY_COMPLEX, 1.5, (255,255,255), 2);

        // Displaying frames    
        cv::imshow(WIN_CAPTURE_NAME, frame);
        cv::imshow(WIN_DETECTION_NAME, frame_threshold);

        char key = (char) cv::waitKey(1);
        if (key == 'q' || key == 27){
            break;
        }
    }
    return 0;
}