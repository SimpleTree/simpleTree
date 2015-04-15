/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, Jan Hackenberg, University of Freiburg.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "StemPointDetection.h"

StemPointDetection::StemPointDetection(PointCloudI::Ptr cloud,
                                       std::vector<bool> stem_pts_old, float max_distance, float max_intens) {
      tt.tic ();
    this->cloud = cloud;
    this->stem_pts_old = stem_pts_old;
    this->max_intens = max_intens;
    this->max_distance = max_distance;
    clusters.reset(new pcl::IndicesClusters);

    copyCloud();
    pcl::ConditionalEuclideanClustering<PointI> cec(true);
    cec.setInputCloud(cloud_temp);
    cec.setConditionFunction(&customRegionGrowing);
    cec.setClusterTolerance(max_distance);
    cec.setMinClusterSize(cloud_temp->points.size()/70);
    cec.segment(*clusters);
    resetStemPoints();
    result.append(QString("Stem point detected in ")).append(QString::number(tt.toc()/1000)).append(QString(" seconds.\n"));
}

StemPointDetection::~StemPointDetection() {
    // TODO Auto-generated destructor stub
}

void StemPointDetection::copyCloud() {
    cloud_temp.reset(new PointCloudI);
    pcl::copyPointCloud(*cloud, *cloud_temp);
    for (size_t i = 0; i < cloud_temp->points.size(); i++) {
        if (stem_pts_old.at(i))        {
            cloud_temp->points[i].intensity = 0;
        }
        else
        {
            cloud_temp->points[i].intensity = 255;
        }
    }
}

bool customRegionGrowing(const PointI& p1, const PointI& p2,
                         float squared_distance) {
    float dist = 0.05;
    if (squared_distance < dist) {
        if (p1.intensity < 0.5)
            return (true);
        if (p2.intensity < 0.5)
            return (true);
    }
    return false;
}

void StemPointDetection::resetStemPoints() {
    stem_pts_new.clear();
    for (size_t i = 0; i < stem_pts_old.size(); i++)
    {
        stem_pts_new.push_back(false);
    }
    for (size_t i = 0; i < clusters->size (); i++)
    {
        for (size_t j = 0; j < (*clusters)[i].indices.size(); j++)
        {
            stem_pts_new[(*clusters)[i].indices[j]] = true;
        }
    }
}
