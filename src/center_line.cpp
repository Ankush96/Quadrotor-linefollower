#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <quad/perpendicular.h>

using namespace cv;
using namespace std;
using namespace quad;
static const std::string OPENCV_WINDOW = "Image window";
perpendicular perp; //Message of custom message type perpendicular.msg
Mat canny_output,hough_output,keyPoints;
int lowThreshold=100,upperThreshold=100,max_lowThreshold=255,max_upperThreshold=255;
vector<Vec2f> houghLines,lines;

struct GROUP{
  float rho;
  float theta;
  vector<int> index;
};


///**********GIVEN TWO LINES THIS FUNCTION FINDS THE INTERSECTION POINT*********************//
void findInterscetion(float rho1,float theta1,float rho2,float theta2,int rows,int cols,Point *pt)
{
  double a1=cos(theta1);
  double a2=cos(theta2);
  double b1=sin(theta1);
  double b2=sin(theta2);
  pt->x=(rho1*b2-rho2*b1)/(b2*a1-b1*a2);
  pt->y=(rho2*a1-rho1*a2)/(b2*a1-a2*b1);
}
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher perpendicular_distance=nh_.advertise<perpendicular>("perpendicular_distance_center",10);;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1,&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
   // perpendicular_distance=nh_.advertise<perpendicular>("perpendicular_distance_centre",1);
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow("keyPoints");
    cv::namedWindow("lines");

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow("keyPoints");
    cv::destroyWindow("lines");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // Taking the ROS image in CV_8UC1 format
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

   
    
    //-----------Canny Detection---------------//
    blur( cv_ptr->image, canny_output, Size(3,3) );
    createTrackbar( "Min Threshold:", OPENCV_WINDOW, &lowThreshold, max_lowThreshold);
    createTrackbar( "Max Threshold:", OPENCV_WINDOW, &upperThreshold, max_upperThreshold);
    Canny(canny_output,canny_output,lowThreshold,upperThreshold);
    //------------------------------------------//


    //---------------Hough Lines-----------------//
    
    hough_output.create(canny_output.size(),canny_output.type());
    hough_output=Scalar::all(0);
    HoughLines(canny_output, houghLines, 1, CV_PI/180, 100, 0, 0 );


    //---------------finding mean of the all similar groups---------------//
      lines.clear();   // we push the mean of the two edges to a vector named lines
     
      
      GROUP group[50];
      int groupCount=0; 
      
      for( size_t i = 0; i < houghLines.size(); i++ )
      {
        float theta= houghLines[i][1] ,rho= houghLines[i][0];
       // cout<<rho<<" "<<theta<<" ";
        int flag=0;
        if(groupCount==0)
        {
              
              group[groupCount].rho=rho;
              group[groupCount].theta=theta;
              group[groupCount].index.clear();
              group[groupCount].index.push_back(i);
              ++groupCount;
            //  cout<<"initial";
        }
        else
        {
             for(int j=0;j<groupCount;j++)
              {
                float a=fabs(rho-group[j].rho);
                float b=fabs(theta-group[j].theta);
                //cout<<"diff a="<<a<<" b="<<b<<" ";

                if((a<35)&&(b< 0.15))
                  {
                    group[j].index.push_back(i);
                    flag=1;
                      // cout<<"old";
                    break;     
                  }
              }
              if(!flag)
              {
                   
                    group[groupCount].rho=rho;
                    group[groupCount].theta=theta;
                    group[groupCount].index.clear();
                    group[groupCount].index.push_back(i);
                    ++groupCount;
                    // cout<<"new";
              }
        }

       
      }

      for(int i=0;i<groupCount;i++)
      {
        float rhoSum=0,thetaSum=0;
       // cout<<"Group "<<i<<" : ";
        for(int j=0;j<group[i].index.size();j++)
        {
          //cout<<houghLines[group[i].index[j]][0]<<" "<<houghLines[group[i].index[j]][1]<<" ";
          rhoSum+=houghLines[group[i].index[j]][0];
          //thetaSum+=houghLines[group[i].index[j]][1];
        }
        //cout<<endl;
        float meanRho=rhoSum/group[i].index.size();
       // float meanTheta=thetaSum/group[i].index.size();
        //lines.push_back(Vec2f(meanRho,meanTheta));
        lines.push_back(Vec2f(meanRho,group[i].theta));
      }
     // cout<<houghLines.size()<<" "<<groupCount<<endl;
    //----------------displaying only the mean lines-------------------//
     for( size_t i = 0; i < lines.size(); i++ )
    {
       float rho = lines[i][0], theta = lines[i][1];
       Point pt1, pt2;
       double a = cos(theta), b = sin(theta);
       double x0 = a*rho, y0 = b*rho;
       pt1.x = cvRound(x0 + 1000*(-b));
       pt1.y = cvRound(y0 + 1000*(a));
       pt2.x = cvRound(x0 - 1000*(-b));
       pt2.y = cvRound(y0 - 1000*(a));
       line( hough_output, pt1, pt2, Scalar(255), 4, CV_AA);
    }
    //-------------------------------------------//
  
    //--------------finding the vertical line closest to center of image------------------//
     int x=(cv_ptr->image.cols)/2,y=(cv_ptr->image.rows)/2;
      Point imageCenter(x,y);
    vector<Vec2f> verticalLines;
    for(size_t i=0;i<lines.size();i++)
    {
      float rho = lines[i][0], theta = lines[i][1];
      if(theta<CV_PI/4||theta>0.75*CV_PI)
      {
        verticalLines.push_back(Vec2f(rho,theta));
      }
    }
    float min=cv_ptr->image.cols+cv_ptr->image.rows;
    int minPos=-1;
    for(size_t i=0;i<verticalLines.size();i++)
    {
      float rho = verticalLines[i][0],theta=verticalLines[i][1];
       double a = cos(theta), b = sin(theta);
       double perpendicularDist=fabs(x*a+y*b-rho);
      if(perpendicularDist<min)
      {
        min=perpendicularDist;
        minPos=i;
      }
    }


    //---------------------------------------------//
          Mat keyPoints(cv_ptr->image.rows,cv_ptr->image.cols,CV_8UC3,Scalar(0,0,0));
          float rho = verticalLines[minPos][0], theta = verticalLines[minPos][1];
           Point pt1, pt2;
           double a = cos(theta), b = sin(theta);
           double x0 = a*rho, y0 = b*rho;
           pt1.x = cvRound(x0 + 1000*(-b));
           pt1.y = cvRound(y0 + 1000*(a));
           pt2.x = cvRound(x0 - 1000*(-b));
           pt2.y = cvRound(y0 - 1000*(a));
           line( keyPoints, pt1, pt2, Scalar(255,255,255), 4, CV_AA);


         
         
          int projection_x,projection_y;        
          double perpendicularDist=fabs(x*a+y*b-rho);
          //------------finding out projection points--------------//
              if(theta==0||theta==CV_PI)
              {
                projection_x=(int)fabs(rho);
                projection_y=y;
              }
              else if(theta==CV_PI/2)
              {
                projection_x=x;
                projection_y=(int)fabs(rho);
              }
              else
              {
                 projection_x=(int)(rho*a+x*b*b-y*b*a);
                 projection_y=(int)(rho*b-x*a*b+y*a*a);
              }
              //--------------projection points have been found out.Now we need to publish all the data----------------//
          perp.rho=rho;
          perp.theta=theta;
          perp.imageCenter_x=x;
          perp.imageCenter_y=y;
          perp.projection_x=projection_x;
          perp.projection_y=projection_y;
          perp.perpendicular_distance=perpendicularDist;
          
          perpendicular_distance.publish(perp);      
          line(keyPoints,imageCenter, Point(projection_x,projection_y),Scalar(20,20,255));                   
          circle(keyPoints,imageCenter,(int)perpendicularDist,Scalar(238,238,175));   //A red circle having a radius equal to the perpendicular distance is drawn
         
    
    cv::imshow(OPENCV_WINDOW, canny_output);
    cv::imshow("keyPoints", keyPoints);
    cv::imshow("lines", hough_output);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
   
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
