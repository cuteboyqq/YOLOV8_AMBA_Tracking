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


void showTrackedHumanResults(std::vector<TrackedObj>& objList)
{
	for (int i=0; i<objList.size(); i++)
  {
    TrackedObj& obj = objList[i];

	  cout << "Obj[" << obj.id << "] ";
    cout << "Type: " << obj.type << " ";
    cout << "Conf: " << obj.confidence << " ";
    cout << "Loc: (" << obj.pLoc.x << " m, " << obj.pLoc.y << " m, " << obj.pLoc.z << " m)" << endl;
  }
}


int main(int argc, char **argv)
{
	int rval = 0;
	int sig_flag = 0;
	std::vector<BoundingBox> bboxList;
	YoloV8_Class yolov8(argc,argv);
	VisionTracker vTracker("./config/config.txt");
	VisionTrackingResults result;
	int c = 0;
	ea_tensor_t *tensor;
	img_set_t *img_set;
	img_set = new img_set_t;
	do
	{
		cv::Mat img;
		img = yolov8.Get_img();
		sig_flag = yolov8.test_yolov8_run(); //RVAL_OK
		bboxList.clear();
		yolov8.Get_Yolov8_Bounding_Boxes(bboxList,img);
		// if(bboxList.size()>0){
			vTracker.run(img,bboxList);
		// }
		c+=1;
	}while(sig_flag==0);

	return rval;
}
