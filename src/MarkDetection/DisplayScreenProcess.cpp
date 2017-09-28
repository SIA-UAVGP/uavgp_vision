/*
 * @file	: main.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/30
 */

#include "DisplayScreenProcess.h"
void ShowTime(Mat& img, int k, float shrink);
int EraseImpossibleResult(RoiAreaInfo& roiAreaInfo);

static float GetAverageValue(Mat& input_img, bool non_zero_pixel, float thres = 0)
{
    float sum = 0;
    int count_num = 0;
    for(int i=0;i<(int)input_img.rows;++i)
    {
        for(int j=0;j<(int)input_img.cols;++j)
        {
            int value = input_img.at<uchar>(i,j);
            if (true == non_zero_pixel)
            {
                if (value > thres)
                {
                        sum += input_img.at<uchar>(i,j);
                        count_num++;
                }
            }
            else
            {
                sum += input_img.at<uchar>(i,j);
                count_num++;
            }
        }
    }
    if (0 == count_num)
        return 0;
    else
        return int(sum/count_num);
}
static int GetMaxValue(Mat& input_img)
{
    int max = 0;
    for(int i=0;i<(int)input_img.rows;++i)
    {
        for(int j=0;j<(int)input_img.cols;++j)
        {
            int value = input_img.at<uchar>(i,j);
            if (value > max)
            {
                max = value;
            }
        }
    }

    return max;
}

static void strengthenContrast(Mat& gray_img, float contrast_ratio)
{
    for(int i=0;i<gray_img.rows;++i)
    {
        for(int j=0;j<gray_img.cols;++j)
        {
            gray_img.at<uchar>(i,j) = saturate_cast<uchar>(float(gray_img.at<uchar>(i,j))*contrast_ratio * float(gray_img.at<uchar>(i,j)));
        }
    }
}

//judge rect1 is inside rect2 or not
bool Rect1IsInside(Rect rect1, Rect rect2)
{
    return (rect1 == (rect1&rect2));
}

DisplayScreenProcessType::DisplayScreenProcessType( )
{
    imgNo = 0;
    shrink = 0.8;
    rect_filter_two_side_ratio_max = 1.0;   ////0.9
    rect_filter_two_side_ratio_min = 0.25;   ////0.2
    min_bounding_rect_height_ratio = 0.1;  ////0.1
    min_precision_ratio_thres = 80.0;    ////80
    min_knn_distance_thres = 260;
    return;
}

int DisplayScreenProcessType::DisplayScreenProcess(Mat& input_img, basicOCR* KNNocr, const char* baseDir, int color_filter_value)
{
    red_filter_value = color_filter_value;
    rawCameraImg = input_img.clone();
    resize(rawCameraImg,rawCameraImg,Size(rawCameraImg.cols*shrink,rawCameraImg.rows*shrink),0,0,INTER_AREA);

    Mat color_filtered_img;
    DisplayScreenProcessType::ColorFilter(rawCameraImg, color_filtered_img);

    vector<Rect> preprocess_rois;
    DisplayScreenProcessType::GetPossibleRois(color_filtered_img, preprocess_rois);
    if (preprocess_rois.size() < 1)
    {
        resize(rawCameraImg,show_img,Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
        ShowTime(show_img, imgNo, 0.5);
        imshow("Display screen", show_img);
        imgNo++;
        return -1;
    }

    Mat median_blur_light_img;
    Mat imgBinary;
    DisplayScreenProcessType::ThresholdProcess(color_filtered_img, preprocess_rois, median_blur_light_img, imgBinary);

    vector< vector<Point> > all_contours;
    findContours( imgBinary, all_contours, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE );//RETR_LIST; CV_RETR_CCOMP ; CV_RETR_EXTERNAL

    vector< vector<Point> > contours;
    DisplayScreenProcessType::ContoursPreFilter(all_contours, contours);

    vector<RoiAreaInfo> roiAreaInfos;
    DisplayScreenProcessType::GetDigitRoi(contours, median_blur_light_img, roiAreaInfos);
    DisplayScreenProcessType::DigitClassify(roiAreaInfos, rawCameraImg, KNNocr, baseDir);
    DisplayScreenProcessType::DigitSort(rawCameraImg, roiAreaInfos);

    resize(rawCameraImg,show_img,Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    ShowTime(show_img, imgNo, 0.5);
    imgNo++;

    if (roiAreaInfos.size() >= 1)
        return roiAreaInfos[0].digitNo;
    else
        return -1;
}


void DisplayScreenProcessType::ColorFilter(Mat& rawCameraImg, Mat& color_filtered_img)
{
    Mat hsv_img;
    cvtColor(rawCameraImg,hsv_img,CV_BGR2HSV_FULL);
    vector<Mat> channels;
    split(hsv_img, channels);
    Mat color_img;
    Mat saturation_img;
    Mat light_img;
    color_img = channels.at(0);
    saturation_img = channels.at(1);
    light_img = channels.at(2);
    for(int i=0;i<rawCameraImg.rows;++i)
    {
        for(int j=0;j<rawCameraImg.cols;++j)
        {
            int color = color_img.at<uchar>(i,j);
            int saturation = saturation_img.at<uchar>(i,j);
            int lightness = light_img.at<uchar>(i,j);

            if(lightness < 50)
                light_img.at<uchar>(i,j) = 0;
            if(saturation < red_filter_value) //120
                light_img.at<uchar>(i,j) = 0;
            if( color>60 && color<220 ) //red: 0~15, 221~255; yellow: 16~48;
                light_img.at<uchar>(i,j) = 0;
        }
    }
    light_img.copyTo(color_filtered_img);
    Mat resized_light_img;
    resize(light_img,resized_light_img,Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    imshow("color filtered img",resized_light_img);
    return;
}


void DisplayScreenProcessType::GetPossibleRois(Mat& color_filtered_img, vector<Rect>& preprocess_rois)
{
    Mat img = color_filtered_img.clone();
    //equalizeHist(color_filtered_img,img);
    Mat imgBinary;
    int min_size = 100; //100, 80
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(img, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); // ADAPTIVE_THRESH_GAUSSIAN_C, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,    THRESH_OTSU
    double s = 11*shrink;
    int size = ( 1 == int(s)%2 ) ? int(s) : int(s)+1;
    if (size < 3)
        size = 3;
    Mat element;
    element=getStructuringElement(MORPH_ELLIPSE, Size( size,size ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
    morphologyEx(imgBinary, imgBinary, MORPH_CLOSE ,element);

    Mat show_binary;
    resize(imgBinary,show_binary,Size(1384*0.5,1032*0.5));
    //imshow("Possible Rois adaptive Threshold", show_binary);

    vector< vector<Point> > contours;
    findContours( imgBinary, contours, RETR_LIST, CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL

    if (contours.size() < 1)
        return;

    for(int i=0;i<(int)contours.size();++i)
    {
        if (contours[i].size() < color_filtered_img.rows*0.05)
            continue;
        Rect minBoundingRect = boundingRect( Mat(contours[i]) );
        if (minBoundingRect.area() < pow(color_filtered_img.rows*0.05, 2)*2)
            continue;
        if (float(minBoundingRect.width)/minBoundingRect.height > rect_filter_two_side_ratio_max)
            continue;
        if (float(minBoundingRect.width)/minBoundingRect.height < rect_filter_two_side_ratio_min)
            continue;

        for(int j=0;j<(int)preprocess_rois.size();++j)
        {
            Rect rect = minBoundingRect & preprocess_rois[j];
            if (rect.area() > 10)     //(minBoundingRect.area() > preprocess_rois[i].area() ? preprocess_rois[j].area() : minBoundingRect.area())*0.005
            {
                minBoundingRect = minBoundingRect | preprocess_rois[j];
            }
        }
        preprocess_rois.push_back(minBoundingRect);
    }

    //rois filter
    for(int i=0;i<(int)preprocess_rois.size();++i)
    {
        for(int j=i+1;j<(int)preprocess_rois.size();++j)
        {
            if (preprocess_rois[i].area() < preprocess_rois[j].area())
                swap(preprocess_rois[i],preprocess_rois[j]);
        }
    }
    for(int i=0;i<(int)preprocess_rois.size();++i)
    {
        for(int j=i+1;j<(int)preprocess_rois.size();++j)
        {
            bool is_inside = Rect1IsInside(preprocess_rois[j], preprocess_rois[i]);
            if (true == is_inside)
            {
                preprocess_rois.erase(preprocess_rois.begin()+j);
                j--;
                continue;
            }
        }
    }
    //printf("Possible roi num=%d;\n", (int)preprocess_rois.size());
    return;
}

void DisplayScreenProcessType::ThresholdProcess(Mat& color_filtered_img, vector<Rect>& preprocess_rois, Mat& median_blur_light_img, Mat& imgBinary)
{
    Mat input_img = color_filtered_img.clone();
    median_blur_light_img = Mat::zeros(color_filtered_img.rows,color_filtered_img.cols,CV_8UC1);

    for(int i=0;i<(int)preprocess_rois.size();++i)
    {
        Mat roi_img = input_img(preprocess_rois[i]);
        medianBlur(roi_img,roi_img,7);

        //float cal_thres = 80;
        //float value = GetAverageValue( roi_img,true,cal_thres );
        float cal_thres = 50;
        float value = GetAverageValue( roi_img,false,cal_thres );
        //printf("AverageValue = %0.2f\n", value);

        //float contrast_ratio = 0.025 * 20/value;
        float contrast_ratio = 0.005 * 12/value;
        strengthenContrast(roi_img, contrast_ratio);
        //imshow("before THRESH_OTSU",roi_img);

        threshold(roi_img,roi_img,100,255,THRESH_OTSU);
        //if (value <= cal_thres*1.75)
            //equalizeHist(roi_img,roi_img);
        //if (value <= cal_thres*0.4)
        //    equalizeHist(roi_img,roi_img);

        Mat element;
        element=getStructuringElement(MORPH_ELLIPSE, Size( 1,3 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
        dilate(roi_img, roi_img, element);//MORPH_CLOSE, MORPH_GRADIENT
        element=getStructuringElement(MORPH_ELLIPSE, Size( 3,1 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
        erode(roi_img, roi_img, element);//MORPH_CLOSE, MORPH_GRADIENT

        char window_name[50];
        sprintf(window_name,"possible roi img %d", i);
        //imshow(window_name,roi_img);
        Mat median_blur_light_img_roi = median_blur_light_img(preprocess_rois[i]);
        roi_img.copyTo(median_blur_light_img_roi,roi_img);
    }

    Mat resized_median_blur_light_img;
    resize(median_blur_light_img,resized_median_blur_light_img,Size(1384*0.45,1032*0.45));
    imshow("strengthen Contrast median_blur_light_img",resized_median_blur_light_img);

    int min_size = 80; //100, 80
    int thresh_size = (min_size/4)*2 + 1;
    adaptiveThreshold(median_blur_light_img, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV
    double s = 11*shrink;
    int size = ( 1 == int(s)%2 ) ? int(s) : int(s)+1;
    if (size < 3)
        size = 3;
    Mat element;
    element=getStructuringElement(MORPH_RECT, Size( size,size ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
    morphologyEx(imgBinary, imgBinary, MORPH_CLOSE ,element);//MORPH_CLOSE, MORPH_GRADIENT
    Mat imgBinaryShow;
    resize(imgBinary, imgBinaryShow, Size(1384*0.5,1032*0.5),0,0,INTER_AREA);
    //imshow("adaptive Threshold Img",imgBinaryShow);
    return;
}


void DisplayScreenProcessType::ContoursPreFilter(vector< vector<Point> >& all_contours, vector< vector<Point> >& contours)
{
    for (int i = 0; i < (int)all_contours.size(); ++i)
    {
        if ((int)all_contours[i].size() > rawCameraImg.rows*0.2 && (int)all_contours[i].size()<rawCameraImg.rows*8)
        {
            //float area = fabs( (float)contourArea(all_contours[i]) );
            //if (area > pow((float)imgBinary.rows*0.2,2))
                contours.push_back(all_contours[i]);
        }
    }

    for(int i=0;i<(int)contours.size();++i)
    {
        RotatedRect boxTemp = minAreaRect( Mat(contours[i]) );
        Point2f vertex[4];
        boxTemp.points(vertex);

        //vertex sort，clock wise, top left corner is 0；
        for(int m=0;m<4;++m)
        {
            for (int n=m+1;n<4;++n)
            {
                if (vertex[m].y > vertex[n].y)
                    std::swap(vertex[m], vertex[n]);
            }
        }
        if (vertex[0].x > vertex[1].x)
            std::swap(vertex[0], vertex[1]);
        if (vertex[3].x > vertex[2].x)
            std::swap(vertex[2], vertex[3]);

        float side0 = sqrt( pow(vertex[1].x - vertex[0].x, 2) + pow(vertex[1].y - vertex[0].y, 2));
        float side1 = sqrt( pow(vertex[2].x - vertex[1].x, 2) + pow(vertex[2].y - vertex[1].y, 2));
        float area = side0 * side1;
        //printf("side0/side1=%0.3f\n",side0/side1);
        if (area < pow(rawCameraImg.rows*0.05,2) || side0>rawCameraImg.cols*0.8 || side0/side1 > rect_filter_two_side_ratio_max || side0/side1 < rect_filter_two_side_ratio_min)
        {
            contours.erase(contours.begin() + i);
            i--;
            continue;
        }

        for(int j=0; j<4;++j)
        {
            line(rawCameraImg, vertex[j], vertex[(j+1)%4], Scalar(0,255,0), 1, 8);
            char strNumber[100];
            sprintf( strNumber,"%d",j);
            //putText(rawCameraImg,strNumber,vertex[j],CV_FONT_HERSHEY_COMPLEX_SMALL,1.5,Scalar(0,0,255),2);
        }
    }
    return;
}



void DisplayScreenProcessType::GetDigitRoi(vector< vector<Point> >& contours, Mat& input_img, vector<RoiAreaInfo>& roiAreaInfos)
{
    //line(rawCameraImg, Point(0,rawCameraImg.rows*0.15), Point(rawCameraImg.cols,rawCameraImg.rows*0.15), Scalar(0,255,0), 1, 8);

    for(int i=0;i<(int)contours.size();++i)
    {
        Rect minBoundingRect = boundingRect( Mat(contours[i]) );
        if ((minBoundingRect.height < rawCameraImg.cols*min_bounding_rect_height_ratio) || ((minBoundingRect.width*1.0/minBoundingRect.height) > rect_filter_two_side_ratio_max) || ((minBoundingRect.width*1.0/minBoundingRect.height) < rect_filter_two_side_ratio_min))
            continue;
        if (minBoundingRect.x+minBoundingRect.width/2 > rawCameraImg.cols*0.8 || minBoundingRect.x+minBoundingRect.width/2 < rawCameraImg.cols*0.2)
            continue;
        if (minBoundingRect.y+minBoundingRect.height/2 < rawCameraImg.rows*0.1)
            continue;

        rectangle( rawCameraImg, minBoundingRect, Scalar(0,255,255), 2, 8);

        int height = minBoundingRect.height*1.1;
        int y = int(minBoundingRect.y + minBoundingRect.height*0.5 - height*0.5);
        int width = int(minBoundingRect.height * 2.7/4 * 1.2);
        int x = int(minBoundingRect.x + minBoundingRect.width*0.5 - width*0.5);
        if (x<=0)
            x = 0;
        if (y<=0)
            y = 0;
        if ((x + width) >= (rawCameraImg.cols-1))
            width = rawCameraImg.cols - x;
        if ((y + height) >= (rawCameraImg.rows-1))
            height = rawCameraImg.rows - y;
        Rect roi = Rect(x, y, width, height);
        if ((int)roi.area() < 10)
            continue;

        Mat roi_img;
        input_img(roi).copyTo(roi_img);

        resize(roi_img,roi_img,Size(128,128),0,0,INTER_AREA);
        medianBlur(roi_img,roi_img,5);
        //imshow("before GaussianBlur", roi_img);
        GaussianBlur(roi_img,roi_img,Size(5,5),0,0);

        //equalizeHist(roi_img,roi_img);
        Rect roi_img_roi = Rect(0+roi_img.cols*0.1, 0+roi_img.rows*0.1, roi_img.cols*0.89, roi_img.rows*0.89);
        Mat img = roi_img(roi_img_roi);
        float value = GetAverageValue( img,true );
        float contrast_ratio = 0.025 * 20/value; //// 0.025*20/value;
        strengthenContrast(roi_img, contrast_ratio);
        //imshow("strengthenContrast_light_img_roi", roi_img);

        float average_value = GetAverageValue( roi_img,false );
        //printf("average_value=%d\n",int(average_value));

        if (average_value < 40)
        {
            Mat element=getStructuringElement(MORPH_ELLIPSE, Size( 11,11 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
            int morphology_option = MORPH_CLOSE;
            morphologyEx(roi_img, roi_img, morphology_option ,element);
        }
        else
        {
            Mat element=getStructuringElement(MORPH_ELLIPSE, Size( 5,5 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
            int morphology_option = MORPH_OPEN;
            morphologyEx(roi_img, roi_img, morphology_option ,element);
            //erode(roi_img,roi_img,element);
        }

        double min_filter_thres = 150*pow(average_value,3)/pow(150,3)*19;
        int roi_pixel_max_value = GetMaxValue(roi_img);
        if (min_filter_thres >= roi_pixel_max_value-10)
            min_filter_thres = roi_pixel_max_value - 10;
        //printf("MaxValue = %d\n",roi_pixel_max_value);
        threshold(roi_img, roi_img, min_filter_thres, 255, THRESH_BINARY_INV);   //120
        Mat element_;
        element_=getStructuringElement(MORPH_ELLIPSE, Size( 5,5 ) );  //Size( 9,9 ) //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
        erode(roi_img,roi_img,element_);
        //imshow("wait classify roi", roi_img);
        RoiAreaInfo roiAreaInfo;
        roiAreaInfo.minBoundingRect = minBoundingRect;
        roiAreaInfo.roi_img = roi_img;
        roiAreaInfos.push_back(roiAreaInfo);
    }
    //sort by area
    for(int i=0;i<int(roiAreaInfos.size());++i)
    {
        for(int j=i+1;j<(int)roiAreaInfos.size();++j)
        {
            if (roiAreaInfos[i].minBoundingRect.area() < roiAreaInfos[j].minBoundingRect.area())
                swap(roiAreaInfos[i],roiAreaInfos[j]);
        }
    }
    //erase inside rects
    for(int i=0;i<int(roiAreaInfos.size());++i)
    {
        for(int j=i+1;j<(int)roiAreaInfos.size();++j)
        {
            bool is_inside = Rect1IsInside(roiAreaInfos[j].minBoundingRect, roiAreaInfos[i].minBoundingRect);
            if (true == is_inside)
            {
                roiAreaInfos.erase(roiAreaInfos.begin()+j);
                j--;
                continue;
            }
        }
    }
    return;
}

void DisplayScreenProcessType::DigitClassify(vector<RoiAreaInfo>& roiAreaInfos, Mat& rawCameraImg, basicOCR* KNNocr, const char* baseDir)
{
    int num = 0;
    for(int i=0;i<(int)roiAreaInfos.size();++i)
    {
        IplImage ipl_img(roiAreaInfos[i].roi_img);
        float classResult = KNNocr->classify(&ipl_img,1);
        float precisionRatio = KNNocr->knn_result.precisionRatio;
        float min_distance[10] = {1000};
        int count = 0;
        for(int j=0;j<10;++j)
        {
            if (int(classResult) == int(KNNocr->knn_result.nearest_label[j]))
            {
                min_distance[count] = KNNocr->knn_result.min_distance[j];
                count++;
            }
        }
        roiAreaInfos[i].digitNo = (int)classResult;

        char digit[500];
        sprintf(digit, "%s/digit_image/displayscreen_%d_accuracy_%d_dist_%d_%d_No_%06d.pbm", baseDir, int(classResult), int(precisionRatio), int(min_distance[0]), int(min_distance[1]), imgNo);
        imwrite(digit,  roiAreaInfos[i].roi_img );

        if (min_distance[0] >= min_knn_distance_thres || min_distance[0] <= 1)
        {
            roiAreaInfos.erase(roiAreaInfos.begin() + i);
            i--;
            continue;
        }
        if (precisionRatio <= min_precision_ratio_thres)
        {
            roiAreaInfos.erase(roiAreaInfos.begin() + i);
            i--;
            continue;
        }
        if (0 == EraseImpossibleResult(roiAreaInfos[i]))
        {
            roiAreaInfos.erase(roiAreaInfos.begin() + i);
            i--;
            continue;
        }
        if (1 == roiAreaInfos[i].digitNo)
        {
            if (min_distance[0] >= 160)
            {
                roiAreaInfos.erase(roiAreaInfos.begin() + i);
                i--;
                continue;
            }
        }

        rectangle( rawCameraImg, roiAreaInfos[i].minBoundingRect, Scalar(255,255,0), 5, 8);
        //printf("digit=%d; accuracy=%d%%; dist=%d,%d,%d\n",(int)classResult,(int)precisionRatio, int(min_distance[0]), int(min_distance[1]), int(min_distance[2]));
        sprintf(digit,"%d",(int)classResult);
        Point showCenter = Point(roiAreaInfos[i].minBoundingRect.x + roiAreaInfos[i].minBoundingRect.width + 60, roiAreaInfos[i].minBoundingRect.y + roiAreaInfos[i].minBoundingRect.height*0.5);
        putText(rawCameraImg, digit, showCenter,CV_FONT_HERSHEY_DUPLEX,5.2*shrink,Scalar(0,0,255), int(5.5*shrink));
        char window_name[50];
        sprintf(window_name,"digit_%d", num);
        imshow(window_name,roiAreaInfos[i].roi_img);
        num++;
    }
    return;
}

int EraseImpossibleResult(RoiAreaInfo& roiAreaInfo)
{
    Mat img = roiAreaInfo.roi_img.clone();
    GaussianBlur(img,img,Size(3,3),0,0);
    Canny(img,img,120,30);
    Mat img_find_contours = img.clone();
    vector< vector<Point> > contours;
    findContours(img_find_contours,contours,CV_RETR_EXTERNAL, CHAIN_APPROX_NONE );// RETR_LIST
    vector<Rect> rects;
    float img_area = img.cols*img.rows;
    for(int j=0;j<(int)contours.size();++j)
    {
        Rect minBoundingRect = boundingRect( Mat(contours[j]) );
        float area_thres = 0.3;
        float height_thres = 0.5;
        //float width_thres = 0.4;
        if (1 == roiAreaInfo.digitNo)
        {
            area_thres = 0.1;
            height_thres = 0.6;
        }
        if (minBoundingRect.area() < img_area*area_thres)
            continue;
        if (minBoundingRect.height < img.rows*height_thres)
            continue;
        Point rect_center = Point(minBoundingRect.x+minBoundingRect.width*0.5, minBoundingRect.y+minBoundingRect.height*0.5);
        Point img_center = Point(img.cols*0.5,img.rows*0.5);
        float dis = sqrt(pow(rect_center.x-img_center.x,2)+pow(rect_center.y-img_center.y,2));
        float thres = img.rows*0.5*0.5;
        //printf("dis_center = %0.1f\n", dis);
        if (dis > thres)
            continue;

        rects.push_back(minBoundingRect);
        //printf("rect.area = %0.2f, rect.height = %0.2f, rect.width = %0.2f\n", rects[0].area()/img_area, (float)rects[0].height/img.rows, (float)rects[0].width/img.cols);
    }
    if (rects.size() < 1)
        return 0;
    else
        return 1;
}


void DisplayScreenProcessType::DigitSort(Mat& input_img, vector<RoiAreaInfo>& roiAreaInfos)
{
    for(int i=0;i<(int)roiAreaInfos.size();++i)
    {
        for(int j=i+1;j<(int)roiAreaInfos.size();++j)
        {
            int x_i = roiAreaInfos[i].minBoundingRect.x + roiAreaInfos[i].minBoundingRect.width/2;
            int y_i = roiAreaInfos[i].minBoundingRect.y + roiAreaInfos[i].minBoundingRect.height/2;
            int x_j = roiAreaInfos[j].minBoundingRect.x + roiAreaInfos[j].minBoundingRect.width/2;
            int y_j = roiAreaInfos[j].minBoundingRect.y + roiAreaInfos[j].minBoundingRect.height/2;
            float dis_c_i = sqrt(pow(x_i - input_img.cols/2,2) + pow(y_i - input_img.rows*0.5,2));
            float dis_c_j = sqrt(pow(x_j - input_img.cols/2,2) + pow(y_j - input_img.rows*0.5,2));
            if (dis_c_i > dis_c_j)
            {
                swap(roiAreaInfos[i],roiAreaInfos[j]);
            }
        }
    }
    return;
}
