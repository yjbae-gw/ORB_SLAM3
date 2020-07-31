/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
/**
 * arg1 : orb vacabulary
 * arg2 : yaml(setting)
 * arg3 : image path
 * arg4 : image timestamp
 * arg5 : result file name
 */
int main(int argc, char *argv[]) {

    if(argc < 5) {
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2; // num_seq is 1
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName) {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq; //per timestamp
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro; 
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages; //image num
    vector<int> nImu; //imu num
    vector<int> first_imu(num_seq,0); //해당 seq의 처음 imu (한 sequence에 여러개의 imu의 데이터가 들어온다.. time base)

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0; //total image
    //일반적으론 num_seq = 1, 따라서 seq = 0 이다.
    for (seq = 0; seq<num_seq; seq++) {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        //image path, timestamp path, 3~4 input
        LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size(); //images size
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size(); //imu isze

        if((nImages[seq]<=0)||(nImu[seq]<=0)) {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        // 이미지의 time과 비교하여, 초기 imu의 time을 지정한다. 
        // 이미지 time과 비교해서 첫번째로 크기 시작한 value => 로 초기화 된다.
        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images); //image num 만큼 tracking

    cout.precision(17);  // 가장 큰 자리수부터 17자리를 출력한다.
    /*cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "IMU data in the sequence: " << nImu << endl << endl;*/

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // threds : 1.tracking, 2.local mapping, 3.loop & map merging
    // orb voca, setting yaml, monocular, viewer 사용여부
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);

    int proccIm = 0;

    // image num 만큼 반복.
    for (seq = 0; seq < num_seq; seq++) {

        // Main loop
        cv::Mat im;
        // Imu 측정value
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        for(int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
            
            // read image from file, opencv mat 형태로.
            im = cv::imread(vstrImageFilenames[seq][ni],CV_LOAD_IMAGE_UNCHANGED);
            // image timestamp
            double tframe = vTimestampsCam[seq][ni];

            if(im.empty()) {
                cerr << endl << "Failed to load image at: " << vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni > 0) {
                // cout << "t_cam " << tframe << endl;

                //최초 image timestamp보다 늦은 time으로 초기화한 imu 보다 큰 value 입력.
                while(vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni]) {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
            }

            /*cout << "first imu: " << first_imu << endl;
            cout << "first imu time: " << fixed << vTimestampsImu[first_imu] << endl;
            cout << "size vImu: " << vImuMeas.size() << endl;*/
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;

            // 이미지 매트릭스, 타임스템프, IMU 측정value를 입력하여 tracking!!! 시작
            SLAM.TrackMonocular(im, tframe, vImuMeas); // TODO change to monocular_inertial

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif
            // tracking 이전, 이후 타임 입력하여 최종 ttrack_tot time 처리
            double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T = 0;
            if(ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni+1] - tframe;
            else if(ni > 0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack < T)
                usleep((T-ttrack) * 1e6); // 1e6
        }
        if(seq < num_seq - 1) {
            cout << "Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // slam 종료 후 결과 데이터 저장
    if (bFileName) {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    } else {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

/**
 * vstrImages : image full path list
 * vTimeStamps : image's timestamp list
 * 
 * 이미지 경로 및 해당 타임스탬프 정보를 읽어 vector 컨테이너에 등록한다.
 */
void LoadImages(const string &strImagePath, const string &strPathTimes, vector<string> &vstrImages, vector<double> &vTimeStamps) {
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    //reserve : container의 용량을 미리 할당해 둠)
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);

    //timestamp row 만큼 반복
    while(!fTimes.eof()) {
        string s;
        getline(fTimes,s);
        if(!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);
        }
    }
}

/**
 * vAcc : accelerometer x, y, z value list
 * vGyro : gyroscope x, y, z  value list
 * 
 * IMU 정보를 읽어 accelerometer x, y, z, gyroscope x, y, z 정보를 vector 컨테이너에 저장한다.
 */
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro) {
    
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof()) {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty()) {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}