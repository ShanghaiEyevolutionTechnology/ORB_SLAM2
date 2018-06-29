#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

//Evo SDK header
#include <evo_global_define.h>//global define
#include <evo_stereocamera.h>//stereo camera
#include <evo_mat.h>//evo Mat define
#include <evo_matconverter.h>//evo Mat converter

using namespace std;
using namespace evo;
using namespace evo::bino;

bool genYaml(StereoParameters param, Resolution_FPS resfps){
	ofstream fs("leadsense.yaml", ios_base::out);
	if(!fs.is_open()){
		cerr << "error to open leadsense.yaml" << endl;
		return false;	
	}
	fs << "%YAML:1.0" << endl << "---" << endl << endl;

	fs << "Camera.fx: " << param.leftCam.focal.x << endl;
	fs << "Camera.fy: " << param.leftCam.focal.y << endl;
	fs << "Camera.cx: " << param.leftCam.center.x << endl;
	fs << "Camera.cy: " << param.leftCam.center.y << endl;
	fs << "Camera.k1: " << 0.0 << endl;
	fs << "Camera.k2: " << 0.0 << endl;
	fs << "Camera.p1: " << 0.0 << endl;
	fs << "Camera.p2: " << 0.0 << endl;
	fs << "Camera.width: " << resfps.width << endl;
	fs << "Camera.height: " << resfps.height << endl;
	// Camera frames per second 
	fs << "Camera.fps: " << resfps.fps << endl;
	// stereo baseline[m] times fx
	fs << "Camera.bf: " << param.baseline() / 1000 * param.leftCam.focal.x << endl;
	// Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
	fs << "Camera.RGB: " << 0 << endl;

	// Close/Far threshold. Baseline times.
	fs << "ThDepth: " << 48 << endl << endl;

	//--------------------------------------------------------------------------------------------
	// ORB Parameters
	//--------------------------------------------------------------------------------------------
	// ORB Extractor: Number of features per image
	fs << "ORBextractor.nFeatures: " << 1000 << endl;
	// ORB Extractor: Scale factor between levels in the scale pyramid 	
	fs << "ORBextractor.scaleFactor: " << 1.2 << endl;
	// ORB Extractor: Number of levels in the scale pyramid	
	fs << "ORBextractor.nLevels: " << 8 << endl;
	// ORB Extractor: Fast threshold
	// Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
	// Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
	// You can lower these values if your images have low contrast			
	fs << "ORBextractor.iniThFAST: " << 20 << endl;
	fs << "ORBextractor.minThFAST: " << 7 << endl << endl;

	//--------------------------------------------------------------------------------------------
	// Viewer Parameters
	//--------------------------------------------------------------------------------------------
	fs << "Viewer.KeyFrameSize: " << 0.6 << endl;
	fs << "Viewer.KeyFrameLineWidth: " << 2 << endl;
	fs << "Viewer.GraphLineWidth: " << 1 << endl;
	fs << "Viewer.PointSize: " << 2 << endl;
	fs << "Viewer.CameraSize: " << 0.7 << endl;
	fs << "Viewer.CameraLineWidth: " << 3 << endl;
	fs << "Viewer.ViewpointX: " << 0 << endl;
	fs << "Viewer.ViewpointY: " << -100 << endl;
	fs << "Viewer.ViewpointZ: " << -0.1 << endl;
	fs << "Viewer.ViewpointF: " << 2000 << endl << endl;
	fs.close();
	return true;
}

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        cerr << endl << "Usage: ./leadsense path_to_vocabulary [800|400(default)]" << endl;
        return 1;
    }
	std::cout << "init evo stereo camera... " << std::endl;
	StereoCamera camera;
	bool running = false;
	int width, height;

	int cnt = 0, limit = 100;
	double tempt = 0;
	StereoParameters stereoPara;//stereo parameter

	//open camera
	RESOLUTION_FPS_MODE res_mode = RESOLUTION_FPS_MODE_SD400_30;
	if(argc > 2)
	{
		int res_n = atoi(argv[2]);
		if(res_n == 800){
			res_mode = RESOLUTION_FPS_MODE_HD800_30;
		}
	}
	RESULT_CODE res = camera.open(res_mode);
	std::cout << "stereo camera open: " << result_code2str(res) << std::endl;
	//show image size
	width = camera.getImageSizeFPS().width;
	height = camera.getImageSizeFPS().height;
	std::cout << "image width:" << width << ", height:" << height << std::endl;

	if(genYaml(camera.getStereoParameters(), camera.getImageSizeFPS()) == false){
		return 2;
	}
 	// Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],"leadsense.yaml",ORB_SLAM2::System::STEREO,true);
	
	if (res == RESULT_CODE_OK)//open camera successed
	{
		//evo Mat
		Mat<unsigned char> evo_image;
		//cv Mat
		cv::Mat cv_image;
		cv::Mat imLeft, imRight;
    
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();


		//running flag
		running = true;
		//main loop
		while (running)
		{
			// Get frames and launch the computation
			if (camera.grab(true) == RESULT_CODE_OK)
			{
				evo_image = camera.retrieveImage(SIDE_SBS);
				//Mat convert
				cv_image = evoMat2cvMat(evo_image);
				
				// Read left and right images from file
				imLeft = cv_image(cv::Rect(0,0,width, height));
				imRight = cv_image(cv::Rect(width,0,width, height));
				double tframe = camera.getCurrentFrameTimeCode();

				// Pass the images to the SLAM system
				SLAM.TrackStereo(imLeft,imRight,tframe); 

				double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(std::chrono::steady_clock::now() - t1).count();
      			t1 = std::chrono::steady_clock::now();

				cnt++;
				tempt += ttrack;
				if(cnt == limit){
					cout << "avg loop time:" << tempt / limit << endl;
					cnt = 0; 
					tempt = 0;
				}
				//std::this_thread::sleep_for(std::chrono::microseconds(5));
			}      
		}
	}
    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
