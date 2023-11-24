/*******************************************************************************
 * test_yolov8.c
 *
 * History:
 *  2023/10/02  - [Alister Hsu] created
 *
 ******************************************************************************/
#include "yolov8_class.h"
#include "yolov8_utils/object.hpp"
#include "yolov8_utils/point.hpp"
#include "yolov8_utils/bounding_box.hpp"
#include "yolov8_utils/vision_tracker.hpp"
using namespace std;


int main(int argc, char **argv)
{
	int rval = 0;
	int sig_flag = 0;
	std::vector<BoundingBox> bboxList;
	YoloV8_Class yolov8(argc,argv);
	std::string configPath = "./config/config.txt";
	VisionTracker _track(configPath);
	int c = 0;
	ea_tensor_t *tensor;
	img_set_t *img_set;
	img_set = new img_set_t;
	// tensor = new ea_tensor_t;
	do
	{
		//img = XXX.getFrame()
		cv::Mat img;
		cout<<"Start yolov8.Get_img"<<endl;
		img = yolov8.Get_img();
		cout<<"End yolov8.Get_img"<<endl;
		sig_flag = yolov8.test_yolov8_run(); //RVAL_OK
		// cout<<"Start yolov8.yolov8_thread_join()"<<endl;
		// yolov8.yolov8_thread_join();
		// tensor = yolov8.live_ctx->thread_ctx.thread[0].nn_arm_ctx.bgr;
		cout << "sig_flag = " << sig_flag << endl;
		bboxList.clear();
	
		yolov8.Get_Yolov8_Bounding_Boxes(bboxList);
		printf("c = %d\n",c);
		cout<<"Start yolov8.Draw_Yolov8_Bounding_Boxes"<<endl;

		yolov8.Draw_Yolov8_Bounding_Boxes(bboxList,c,img);
		printf("[int main(int argc, char **argv)] Start _track.run(img,bboxList); \n");
		//if(bboxList.size()>0){
		_track.run(img,bboxList);
		//}
	
		printf("[int main(int argc, char **argv)] End _track.run(img,bboxList); \n");
		
		
		
		c+=1;
	}while(sig_flag==0);

	return rval;
}
