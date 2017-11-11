/*
 * @file	: JugdeRectKind.cpp
 * @auhtor	: xiaobin <xiaobin619@126.com>
 * @time	: 2016/09/12
 */

#include "declare.h"


char windowName[100];
//对所有包围矩形按面积排序，逆序，大的在前
void BoundingRectSortByAreaSize( vector< Rect>& minBoundingRect );
//对包围矩形的种类做判断
int RectKind(  Mat& inputImg, Mat& outputImg, vector< Rect>& minBoundingRect  );
//两个矩形之间的像素平均值
double GetPixelAverageValueBetweenTwoRect( Mat inputImg, Mat outputImg, Rect& rect_outside, Rect& rect_inside);
//按矩形类别存储数字区域图像
void GetDigitRoiImg( Mat& binaryImg, vector< Rect>& minBoundingRect, int rectKind,  vector<RectMark>&  rectCategory );

Mat img;
//对透视变换后的图像做处理，判断出检测到的矩形的类别
void GetRectKinds( vector< vector<RectMark> >&  rectCategory )
{
    for (int i=0; i<(int)rectCategory.size(); ++i)
    {
        rectCategory[i][0].perspectiveImg.copyTo( img );
        Mat showImg = img.clone();

        Mat srcGray;
        cvtColor(img,srcGray,CV_BGR2GRAY);
        Mat imgBinary;
        int min_size = 100;
        int thresh_size = (min_size/4)*2 + 1;
        adaptiveThreshold(srcGray, imgBinary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, thresh_size, thresh_size/3); //THRESH_BINARY_INV
        //morphologyEx(imgBinary, imgBinary, MORPH_CLOSE, Mat());

        ////根据重映射前的矩形大小来设置闭运算的windowsize
        Mat element=getStructuringElement(MORPH_ELLIPSE, Size(5,5) ); //MORPH_RECT=0, MORPH_CROSS=1, MORPH_ELLIPSE=2
        morphologyEx(imgBinary, imgBinary, MORPH_CLOSE ,element);
        imgBinary.copyTo(rectCategory[i][0].possibleRectBinaryImg);

        sprintf(windowName,"imgBinary-%d",i);
        //imshow(windowName, imgBinary);
        Mat img_bin = imgBinary.clone();

        vector< vector<Point> > contours;
        vector<Vec4i> hierarchy;
        vector<RotatedRect>  box;
        vector<Rect> minBoundingRect;
        minBoundingRect.push_back( Rect(0,0,img.cols,img.rows) );
        //查找轮廓
        findContours( img_bin, contours, hierarchy ,CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );//CV_RETR_CCOMP ; CV_RETR_EXTERNAL
        //cout<<"轮廓总个数=" << all_contours.size() <<endl;
        for (int k = 0; k < (int)contours.size(); ++k)
        {
            if ( (int)contours[k].size() < img.cols || fabs( (float)contourArea(contours[k])<pow( img.cols*0.15,2) )  )
            {
                continue;
            }

            //RotatedRect boxTemp = minAreaRect( Mat(contours[i]) );
            //Point2f vertex[4];
            //boxTemp.points(vertex);
            //画出各个最小面积的包围矩形
            //for(int j=0; j<4;++j)
            //{
                //line(showImg, vertex[j], vertex[(j+1)%4], Scalar(0,255,0), 2, 8);
            //}

            //最小包围的正矩形
            Rect minBoundingRectTemp = boundingRect( Mat(contours[k]) );
            //用蓝色画出所有外包围矩形
            rectangle( showImg, minBoundingRectTemp, Scalar(255,0,0), 2, 8);
            //只存储中心点在图像中心的矩形
            Point2d minBoundingRectMiddlePoint = Point2d( minBoundingRectTemp.x + minBoundingRectTemp.width*0.5, minBoundingRectTemp.y + minBoundingRectTemp.height*0.5 );
            double err = sqrt( pow(minBoundingRectMiddlePoint.x - img.cols*0.5, 2) + pow(minBoundingRectMiddlePoint.y - img.rows*0.5, 2) );
            // ROS_INFO("err:%8.4f,cols:%8.4f",err,img.cols*0.09);

            if ( err < img.cols*0.09)
            {
                if ( minBoundingRectTemp.height < img.rows*0.9 )
                {
                    minBoundingRect.push_back(minBoundingRectTemp);
                }
            }
        }
        //空区域不感兴趣
        if ( minBoundingRect.size() <= 1)
        {
            continue;
        }

        //包围矩形按面积排序
        BoundingRectSortByAreaSize( minBoundingRect );

        for (int j=0;j<(int)minBoundingRect.size(); ++j)
        {
            //用红色画出所有中心点在图像中心的外包矩形
            rectangle( showImg, minBoundingRect[j], Scalar(0,0,255), 2, 8);
            circle(showImg, Point(minBoundingRect[j].x+minBoundingRect[j].width*0.5, minBoundingRect[j].y+minBoundingRect[j].height*0.5), 2, Scalar(0,0,255), -1, 8);
        }

        //分情形判断矩形种类
        int rectKind = RectKind( imgBinary, showImg, minBoundingRect );
        GetDigitRoiImg( imgBinary, minBoundingRect, rectKind,  rectCategory[i] );

        //imshow("minRect", showImg);
    }
    return;
}

//对所有包围矩形按面积排序，逆序，大的在前
void BoundingRectSortByAreaSize( vector< Rect>& minBoundingRect )
{
        for (int i=0;i<(int)minBoundingRect.size();++i)
        {
            for (int j=i+1;j<(int)minBoundingRect.size();++j)
            {
                float width = (minBoundingRect[j].width > minBoundingRect[i].width) ? minBoundingRect[j].width : minBoundingRect[i].width;
                if (abs(minBoundingRect[j].width - minBoundingRect[i].width) < width*0.1)
                {
                    minBoundingRect.erase(minBoundingRect.begin() + j);
                    j--;
                    continue;
                }
                if ( (minBoundingRect[j].width * minBoundingRect[j].height) > (minBoundingRect[i].width * minBoundingRect[i].height) )
                {
                    swap(minBoundingRect[j], minBoundingRect[i]);
                }
            }
        }
    return;
}

//对包围矩形的种类做判断
int RectKind(  Mat& inputImg, Mat& outputImg, vector< Rect>& minBoundingRect  )
{
    int kind = -1;
    double rectOutside_rectInside_pixelValue = GetPixelAverageValueBetweenTwoRect( inputImg, outputImg, minBoundingRect[0], minBoundingRect[1]);
    //该矩形为第二种情况，检测到的是中间矩形
    if (rectOutside_rectInside_pixelValue > 100 && minBoundingRect.size() >=3)
    {
        kind = 1;
        //认为是全黑的黑框
        rectangle(outputImg, minBoundingRect[0], Scalar(0,0,0), 4, 8);
        rectangle(outputImg, minBoundingRect[1], Scalar(0,0,0), 4, 8);
    } 
    //矩形为第一、第三种情况
    else if (rectOutside_rectInside_pixelValue < 50 )
    {
        //认为是全白的白框
        rectangle(outputImg, minBoundingRect[0], Scalar( 255, 255, 255 ), 4, 8);
        rectangle(outputImg, minBoundingRect[1], Scalar( 255, 255, 255 ), 4, 8);
        //如果只有两个外包矩形，肯定是第一种情况, 最内侧矩形
        if (minBoundingRect.size() <= 2)
        {
            kind = 0;            
        }
        else if (3 == minBoundingRect.size() )
        {
            //可能是数字0
            kind = 0;
        }
        //第三种情况，检测到了最外边的矩形
        else
        {
            double widthRatio0 = double(minBoundingRect[0].width)/minBoundingRect[1].width;
            double heightRatio0 = double(minBoundingRect[0].height)/minBoundingRect[1].height;
            double widthRatio1 = double(minBoundingRect[1].width)/minBoundingRect[2].width;
            double heightRatio1 = double(minBoundingRect[1].height)/minBoundingRect[2].height;
            double widthError0 = fabs( widthRatio0 - 60.0/40);  //
            double heightError0 = fabs( heightRatio0 - 70.0/50);
            double widthError1 = fabs( widthRatio1 - 40.0/30);
            double heightError1 = fabs( heightRatio1 - 50.0/40);
            if (widthError0 < 60.0/40*0.1 && heightError0<70.0/50*0.1)
            {
                if ( widthError0 < 40.0/30*0.1 && heightError0 < 50.0/40*0.1 )
                {
                    kind = 2;
                }
            }
        }
    } /*else if (minBoundingRect.size() ==2){
        kind = 0;
        printf("3 = %f | %d\n", rectOutside_rectInside_pixelValue,minBoundingRect.size());
    }else if (minBoundingRect.size() >=3){
        kind = 1;
        printf("4 = %f | %d\n", rectOutside_rectInside_pixelValue,minBoundingRect.size());
    }
    printf("5 = %f | %d\n", rectOutside_rectInside_pixelValue,minBoundingRect.size());
    */
    return kind;
}

//按矩形类别, 存储数字区域图像
void GetDigitRoiImg( Mat& binaryImg, vector< Rect>& minBoundingRect, int rectKind,  vector<RectMark>&  rectCategory_i )
{

    Mat img;
     Rect roi;

    if ( -1 == rectKind)
    {
        return;
     }
    if ( 0 == rectKind)
    {
        roi = minBoundingRect[1];
     }
    if ( 1 == rectKind)
    {
        roi = minBoundingRect[2];
     }
    if ( 2 == rectKind)
    {
        roi = minBoundingRect[3];
     }

        //roi区域扩大1.1倍
        Point middlePoint = Point(roi.x + roi.width/2, roi.y + roi.height/2);
        roi.height = roi.height * 1.15;  //1.1
        roi.width = roi.height * 2.7/4;
        roi.x = middlePoint.x - roi.width/2 * 1.0;
        roi.y = middlePoint.y - roi.height/2;

        if ( roi.x <= 0 )
            roi.x = 0;
        if ( roi.y <= 0 )
            roi.y = 0;
        if ( roi.x + roi.width >= binaryImg.cols )
            roi.width =  binaryImg.cols - roi.x;
        if ( roi.y + roi.height >= binaryImg.rows )
            roi.height = binaryImg.rows - roi.y;

        //二值化翻转
        binaryImg( roi ).copyTo(img);
        //imshow( "roi", img );
        threshold(img, img, 125, 255, THRESH_BINARY_INV);

        resize(img, img, Size(128,128));
        Mat element=getStructuringElement(MORPH_ELLIPSE, Size(7,7));
        erode( img, img, element);
        rectCategory_i[0].possibleDigitBinaryImg = img.clone();
        rectCategory_i[0].rectKind = rectKind;

        //imshow("digitROI", img);
    return;
}


//求两个包围矩形之间的像素平均值
double GetPixelAverageValueBetweenTwoRect( Mat inputImg, Mat outputImg, Rect& rect_outside, Rect& rect_inside)
{
    double totalValue = 0;
    int count = 0;
    for (int i=0;i<inputImg.rows;++i)
    {
        uchar* data= inputImg.ptr<uchar>(i);
        //上下横条
        if ( (i>= rect_outside.y && i<=rect_inside.y) || ( i >= (rect_inside.y + rect_inside.height) && i <= (rect_outside.y + rect_outside.height) ) )
        {
            for (int j=0;j<inputImg.cols;++j)
            {
                    totalValue += data[j];
                    count++;
                    //outputImg.at<Vec3b>( i,j )[0] = 255;
                    //outputImg.at<Vec3b>( i,j )[1] = 0;
                    //outputImg.at<Vec3b>( i,j )[2] = 0;
            }
        }
        //左右竖条
        if (  i > rect_inside.y  &&  i < (rect_inside.y + rect_inside.height) )
        {
            for (int j=0;j<inputImg.cols;++j)
            {
                if ( (j> rect_outside.x && j<rect_inside.x) || ( j > (rect_inside.x + rect_inside.width) && j < (rect_outside.x + rect_outside.width) ) )
                {
                    totalValue += data[j];
                    count++;
                    //outputImg.at<Vec3b>( i,j )[0] = 255;
                    //outputImg.at<Vec3b>( i,j )[1] = 0;
                    //outputImg.at<Vec3b>( i,j )[2] = 0;
                }
            }
        }
    }

    return (totalValue/count);
}
