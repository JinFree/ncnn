// Tencent is pleased to support the open source community by making ncnn available.
//
// Copyright (C) 2018 THL A29 Limited, a Tencent company. All rights reserved.
//
// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#include <stdio.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "platform.h"
#include "net.h"
#if NCNN_VULKAN
#include "gpu.h"
#endif // NCNN_VULKAN
#include "LaneDetection.h"
ncnn::Net yolov3;
struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

static int detect_yolov3(const cv::Mat& bgr, std::vector<Object>& objects)
{

    ncnn::Extractor ex = yolov3.create_extractor();
    ex.set_num_threads(4);
    // original pretrained model from https://github.com/eric612/MobileNet-YOLO
    // param : https://drive.google.com/open?id=1V9oKHP6G6XvXZqhZbzNKL6FI_clRWdC-
    // bin : https://drive.google.com/open?id=1DBcuFCr-856z3FRQznWL_S5h-Aj3RawA
    

    const int target_size = 352;

    int img_w = bgr.cols;
    int img_h = bgr.rows;

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR, bgr.cols, bgr.rows, target_size, target_size);

    const float mean_vals[3] = {127.5f, 127.5f, 127.5f};
    const float norm_vals[3] = {0.007843f, 0.007843f, 0.007843f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    

    ex.input("data", in);

    ncnn::Mat out;
    ex.extract("detection_out", out);

    objects.clear();
    for (int i=0; i<out.h; i++)
    {
        const float* values = out.row(i);

        Object object;
        object.label = values[0];
        object.prob = values[1];
        object.rect.x = values[2] * img_w;
        object.rect.y = values[3] * img_h;
        object.rect.width = values[4] * img_w - object.rect.x;
        object.rect.height = values[5] * img_h - object.rect.y;

        objects.push_back(object);
    }

    return 0;
}

static void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects)
{
    static const char* class_names[] = {"background",
        "aeroplane", "bicycle", "bird", "boat",
        "bottle", "bus", "car", "cat", "chair",
        "cow", "diningtable", "dog", "horse",
        "motorbike", "person", "pottedplant",
        "sheep", "sofa", "train", "tvmonitor"};

    cv::Mat image = bgr.clone();

    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];

        fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
                obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);

        cv::rectangle(image, obj.rect, cv::Scalar(255, 0, 0));

        char text[256];
        sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > image.cols)
            x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y),
                                      cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), -1);

        cv::putText(image, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
	image.copyTo(bgr);
    //cv::imshow("image", image);
    //cv::waitKey(0);
}

void imageProcessing(cv::Mat &m, cv::Mat &result)
{
	cv::Mat temp, lane, laneResult;
	m.copyTo(temp);
	m.copyTo(lane);
	ld::imageProcessing(lane, laneResult);
#if NCNN_VULKAN
    ncnn::create_gpu_instance();
#endif // NCNN_VULKAN
	
	std::vector<Object> objects;
    detect_yolov3(temp, objects);

#if NCNN_VULKAN
    ncnn::destroy_gpu_instance();
#endif // NCNN_VULKAN

    draw_objects(laneResult, objects);
    laneResult.copyTo(result);
    return;
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s [videopath]\n", argv[0]);
        return -1;
    }

    const char* videopath = argv[1];
	
	cv::VideoCapture cap(videopath);
    if(cap.isOpened())
    {
        printf("Video Opened\n");
    }
    else
    {
        printf("Video Not Opened\n");
        printf("Program Abort\n");
        exit(-1);
    }
    yolov3.load_param("mobilenetv2_yolov3.param");
    yolov3.load_model("mobilenetv2_yolov3.bin");
    std::string savePath = "output.mp4";
    int fps = cap.get(cv::CAP_PROP_FPS);
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fourcc = cap.get(cv::CAP_PROP_FOURCC);
    cv::VideoWriter out(savePath.c_str(), fourcc, fps, cv::Size(width, height), true);
    cv::namedWindow("Input", cv::WINDOW_GUI_EXPANDED);
    cv::namedWindow("Output", cv::WINDOW_GUI_EXPANDED);
    cv::Mat frame, output;
    while(cap.read(frame))
    {
        imageProcessing(frame, output);
        out.write(output);
        cv::imshow("Input", frame);
        cv::imshow("Output", output);
        char c = (char)cv::waitKey(int(1000.0/fps));
        if (c==27)
            break;
    }
    cap.release();
    out.release();
    cv::destroyAllWindows();
    
    return 0;
}
