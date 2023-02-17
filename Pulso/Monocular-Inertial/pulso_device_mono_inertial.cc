/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

#define PORT 4210
#define MAXLINE 24

// void LoadImages(const string &strImagePath, const string &strPathTimes,
//                 vector<string> &vstrImages, vector<double> &vTimeStamps);

const unsigned char *toChar(float x)
{
    const unsigned char *pf = reinterpret_cast<const unsigned char *>(&x);
    return pf;
}

double now()
{
    auto time = std::chrono::system_clock::now().time_since_epoch();
    std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(time);
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(time);
    return (double)seconds.count() + ((double)(ms.count() % 1000) / 1000.0);
}

int main(int argc, char **argv)
{

    cv::VideoCapture cap(2);

    ///////////////////////
    float arr[7] = {};
    unsigned char tempBuffer[28] = {};
///////////////////////

    int sockfd;
    char buffer[MAXLINE];
    const char *hello = "Hello from server";
    struct sockaddr_in servaddr, cliaddr;

    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr *)&servaddr,
             sizeof(servaddr)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    socklen_t len;
    int n;

    len = sizeof(cliaddr); // len is value/result

    /////////////////////////

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;
    int counter = 0;
    cv::Mat myImage;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    while (true)
    {
        n = recvfrom(sockfd, (char *)buffer, MAXLINE,
                     MSG_WAITALL, (struct sockaddr *)&cliaddr,
                     &len);
        // buffer[n] = '\0';
        // printf("Client : %s\n", buffer);
        float xR;
        float yR;
        float zR;
        float xA;
        float yA;
        float zA;
        char bxr[] = {buffer[0], buffer[1], buffer[2], buffer[3]};
        char byr[] = {buffer[4], buffer[5], buffer[6], buffer[7]};
        char bzr[] = {buffer[8], buffer[7], buffer[10], buffer[11]};

        char bxa[] = {buffer[12], buffer[13], buffer[14], buffer[15]};
        char bya[] = {buffer[16], buffer[17], buffer[18], buffer[19]};
        char bza[] = {buffer[20], buffer[21], buffer[22], buffer[23]};

        memcpy(&xR, &bxr, 4);
        memcpy(&yR, &byr, 4);
        memcpy(&zR, &bzr, 4);

        memcpy(&xA, &bxa, 4);
        memcpy(&yA, &bya, 4);
        memcpy(&zA, &bza, 4);
        std::cout << xR << ", " << yR << ", " << zR << ", " << xA << ", " << yA << ", " << zA << "\n";

        /////
        cap.read(myImage);
        if (myImage.empty())
        { // Breaking the loop if no video frame is detected//
            break;
        }

        char c = (char)cv::waitKey(25); // Allowing 25 milliseconds frame processing time and initiating break condition//
        if (c == 27)
        { // If 'Esc' is entered break the loop//
            break;
        }

        double timestamp = now();
        vImuMeas.clear();
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(xA,yA,zA,xR,yR,zR,timestamp));
                    

        Sophus::SE3f res = SLAM.TrackMonocular(myImage, timestamp, vImuMeas); // TODO change to monocular_inertial
        g2o::SE3Quat q = ORB_SLAM3::Converter::toSE3Quat(res);

        Eigen::Quaterniond quat = q.rotation();
        Eigen::Vector3d trans = q.translation();

        float x = quat.x();
        arr[0] = x;

        float y = quat.y();
        arr[1] = y;

        float z = quat.z();
        arr[2] = z;

        float w = quat.w();
        arr[3] = w;

        float xt = trans.x();
        arr[4] = xt;

        float yt = trans.y();
        arr[5] = yt;

        float zt = trans.z();
        arr[6] = zt;

        printf("---------------------------------------\n");
        cout << counter << " " << x << ", " << y << ", " << z << ", " << w << " | " << xt << ", " << yt << ", " << zt << endl;

        counter = counter + 1;
    }
    // if(!SLAM.mStrSaveAtlasToFile.empty())
    // {
    //     cout << "saving atlas" << endl;
    //     SLAM.SaveAtlas(1);
    // }
    // cap.release();
    // Stop all threads
    SLAM.Shutdown();
    // SLAM.SaveAtlas(1);

    return 0;
}
