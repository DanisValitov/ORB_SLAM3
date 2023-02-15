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

    cv::Mat myImage;
    cv::Mat myImage2;

    cv::VideoCapture cap(2);
    cv::VideoCapture cap2(4);

    // UDP STUFF
    /////////////////

    // #define DST_PORT 7071
    // #define SRC_PORT 59612

    // #define IP "192.168.1.117"

    struct sockaddr_in addr, srcaddr;
    int fd;

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket");
        exit(1);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("192.168.1.117");
    addr.sin_port = htons(7071);

    memset(&srcaddr, 0, sizeof(srcaddr));
    srcaddr.sin_family = AF_INET;
    srcaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    srcaddr.sin_port = htons(59612);

    if (bind(fd, (struct sockaddr *)&srcaddr, sizeof(srcaddr)) < 0)
    {
        perror("bind");
        exit(1);
    }
    ///////////////////////
    float arr[7] = {};
    unsigned char tempBuffer[28] = {};
    ///////////////////////

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    while (true)
    {

        cap >> myImage;
        cap2 >> myImage2;
        if (myImage.empty() || myImage2.empty())
        { // Breaking the loop if no video frame is detected//
            break;
        }

        char c = (char)cv::waitKey(25); // Allowing 25 milliseconds frame processing time and initiating break condition//
        if (c == 27)
        { // If 'Esc' is entered break the loop//
            break;
        }

        double timestamp = now();

        Sophus::SE3f res = SLAM.TrackStereo(myImage, myImage2, timestamp); // TODO change to monocular_inertial
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
        cout << x << ", " << y << ", " << z << ", " << w << " | " << xt << ", " << yt << ", " << zt << endl;

        int shift = 0;
        int step = 4;
        for (size_t i = 0; i < 7; i++)
        {
            const unsigned char *res = toChar(arr[i]);
            for (size_t j = 0; j < step; j++)
            {
                tempBuffer[(step - j - 1) + shift] = res[j];
            }
            shift = shift + step;
        }

        if (sendto(fd, tempBuffer, 28, 0, (struct sockaddr *)&addr,
                   sizeof(addr)) < 0)
        {
            perror("sendto");
            exit(1);
        }
    }
    // Stop all threads
    SLAM.Shutdown();
    close(fd);
    return 0;
}
