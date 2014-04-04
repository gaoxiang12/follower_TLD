#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <iostream>
#include <sstream>
#include <TLD.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include "reading_image.h"

using namespace cv;
using namespace std;
//Global variables
Rect box;
bool drawing_box = false;
bool gotBB = false;
bool tl = true;
bool rep = false;
bool fromfile=false;
string video;

void readBB(char* file)
{
  ifstream bb_file (file);
  string line;
  getline(bb_file,line);
  istringstream linestream(line);
  string x1,y1,x2,y2;
  getline (linestream,x1, ',');
  getline (linestream,y1, ',');
  getline (linestream,x2, ',');
  getline (linestream,y2, ',');
  int x = atoi(x1.c_str());// = (int)file["bb_x"];
  int y = atoi(y1.c_str());// = (int)file["bb_y"];
  int w = atoi(x2.c_str())-x;// = (int)file["bb_w"];
  int h = atoi(y2.c_str())-y;// = (int)file["bb_h"];
  box = Rect(x,y,w,h);
}
//bounding box mouse callback
void mouseHandler(int event, int x, int y, int flags, void *param){
  switch( event ){
  case CV_EVENT_MOUSEMOVE:
    if (drawing_box){
        box.width = x-box.x;
        box.height = y-box.y;
    }
    break;
  case CV_EVENT_LBUTTONDOWN:
    drawing_box = true;
    box = Rect( x, y, 0, 0 );
    break;
  case CV_EVENT_LBUTTONUP:
    drawing_box = false;
    if( box.width < 0 ){
        box.x += box.width;
        box.width *= -1;
    }
    if( box.height < 0 ){
        box.y += box.height;
        box.height *= -1;
    }
    gotBB = true;
    break;
  }
}

void print_help(char** argv){
  printf("use:\n     %s -p /path/parameters.yml\n",argv[0]);
  printf("-s    source video\n-b        bounding box file\n-tl  track and learn\n-r     repeat\n");
}

void read_options(int argc, char** argv,VideoCapture& capture,FileStorage &fs){
  for (int i=0;i<argc;i++){
      if (strcmp(argv[i],"-b")==0){
          if (argc>i){
              readBB(argv[i+1]);
              gotBB = true;
          }
          else
            print_help(argv);
      }
      if (strcmp(argv[i],"-s")==0){
          if (argc>i){
              video = string(argv[i+1]);
              capture.open(video);
              fromfile = true;
          }
          else
            print_help(argv);

      }
      if (strcmp(argv[i],"-p")==0){
          if (argc>i){
              fs.open(argv[i+1], FileStorage::READ);
          }
          else
            print_help(argv);
      }
      if (strcmp(argv[i],"-no_tl")==0){
          tl = false;
      }
      if (strcmp(argv[i],"-r")==0){
          rep = true;
      }
  }
}

double average_depth(Mat& depth, BoundingBox box)
{
    //cout<<"channels:"<<depth.channels()<<endl;
    if (!depth.data)
    {
        cout<<"no data!"<<endl;
        return 0.0;
    }
    float sum=0.0;
    int n = 0;
    //cout<<"in depth loop"<<endl;
    for( int nrow = box.x; nrow<box.x+box.width; nrow++)
    {
        for( int ncol = box.y; ncol < box.y+box.height; ncol++)
        {
            int v = depth.at<int>(nrow, ncol);
            if (v < 10)
                continue;
            //cout<<nrow<<","<<ncol<<", dep="<<depth.at<int>(nrow,ncol)<<endl;
            n++;
            sum += v;
        }
    }
    cout<<"out depth loop"<<endl;
    if (n>0)
        return sum/n;
    return 0.0;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "follower_TLD");
    ros::NodeHandle n;
    ImageConverter ic;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 200);
    ros::Publisher dep_pub = n.advertise<geometry_msgs::Twist>("/average_depth", 200);
    ros::Rate waiting_rate(30);
    FileStorage fs;
    fs.open("/home/g-xiang/catkin_ws/src/follower/src/parameters.yml", FileStorage::READ);
    TLD tld;
    Mat frame = ic.curr_image;
    Mat last_gray;
    Mat first;
    FILE *bb_file;
    while(ic.ready == false)
    {
        ros::spinOnce();
        waiting_rate.sleep();
    }
    cout<<"Init OK"<<endl;
    if(ic.ready == true)
    {
        //Register mouse callback to draw the bounding box
        cvNamedWindow("TLD",CV_WINDOW_AUTOSIZE);
        cvSetMouseCallback( "TLD", mouseHandler, NULL );
        cout<<"cv set ok"<<endl;
        //read parameters file
        tld.read(fs.getFirstTopLevelNode());
        ic.curr_image.copyTo(frame);
        cvtColor(frame, last_gray, CV_RGB2GRAY);
        frame.copyTo(first);
        cout<<"frame OK"<<endl;
        //Initialization
GETBOUNDINGBOX:
        while(!gotBB)
        {
            first.copyTo(frame);
            cvtColor(frame, last_gray, CV_RGB2GRAY);
            drawBox(frame,box);
            imshow("TLD", frame);
            if (cvWaitKey(33) == 'q')
                return 0;
        }
        if (min(box.width,box.height)<(int)fs.getFirstTopLevelNode()["min_win"]){
            cout << "Bounding box too small, try again." << endl;
            gotBB = false;
            goto GETBOUNDINGBOX;
        }
        cout<<"BB ok"<<endl;
        cvSetMouseCallback( "TLD", NULL, NULL );
        //Output file
        printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n",box.x,box.y,box.width,box.height);
        bb_file = fopen("bounding_boxes.txt","w");
        tld.init(last_gray,box,bb_file);
        cout<<"tld init ok"<<endl;
    }

    Mat current_gray;
    BoundingBox pbox;
    vector<Point2f> pts1;
    vector<Point2f> pts2;
    bool status=true;
    int frames = 1;
    int detections = 1;

    float centerX = 320;
    
 REPEAT:
    while(ros::ok())
    {
        //get frame
        ic.curr_image.copyTo(frame);
        cvtColor(frame, current_gray, CV_RGB2GRAY);
        //Process Frame
        tld.processFrame(last_gray,current_gray,pts1,pts2,pbox,status,tl,bb_file);
        //Draw Points
        if (status)
        {
            drawPoints(frame,pts1);
            drawPoints(frame,pts2,Scalar(0,255,0));
            drawBox(frame,pbox);
            detections++;

            float delta = 0;
            if (pbox.x+pbox.width/2. > centerX+40)
                delta = -0.2;
            else if (pbox.x+pbox.width/2. < centerX-40)
                delta = 0.2;

            cout<<"computing ave_dep"<<endl;
            double ave_dep = average_depth(ic.curr_image_depth, pbox);
            cout<<"average depth = "<<ave_dep<<endl;
            geometry_msgs::Twist twi;
            /*
            if (ave_dep > 4000)
                twi.linear.x = -0.1;
            else if (ave_dep < 1000 && ave_dep>0.1)
                twi.linear.x = -0.1;
            */
            if (pbox.width<100)
                twi.linear.x = 0.1;
            else if (pbox.width> 150)
                twi.linear.x = -0.1;
            twi.linear.y = twi.linear.z = 0.0;
            twi.angular.x = twi.angular.y = 0;
            twi.angular.z = delta;
            vel_pub.publish(twi);

            geometry_msgs::Twist t;
            t.linear.x = ave_dep;
            dep_pub.publish(t);
        }
        //Display
        imshow("TLD", frame);
        waitKey(3);
        //swap points and images
        swap(last_gray,current_gray);
        pts1.clear();
        pts2.clear();
        frames++;
        ic.ready = false;

        while(ic.ready == false)
        {
            ros::spinOnce();
        }
    }
    if (rep)
    {
        rep = false;
        tl = false;
        fclose(bb_file);
        bb_file = fopen("final_detector.txt","w");
        //capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
        goto REPEAT;
    }
    fclose(bb_file); 
    return 0;
}
