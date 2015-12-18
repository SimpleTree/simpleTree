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

#ifndef STEMPOINTDETECTION_H_
#define STEMPOINTDETECTION_H_
#define PCL_NO_PRECOMPILE
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <QString>
typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;
bool
	customRegionGrowing(const PointI& p1, const PointI& p2, float squared_distance);
class StemPointDetection {
public:
	StemPointDetection(PointCloudI::Ptr cloud,	std::vector<bool> stem_pts_old
            , float max_distance =0.05, float max_intens =0.5f);
	virtual ~StemPointDetection();

    std::vector<bool>& getStemPtsNew() {
		return stem_pts_new;
	}

    void setStemPtsNew(std::vector<bool>& stemPtsNew) {
		stem_pts_new = stemPtsNew;
	}
    QString result;

private:
    pcl::console::TicToc tt;
	PointCloudI::Ptr cloud;
	std::vector<bool> stem_pts_old;
	std::vector<bool> stem_pts_new;
	PointCloudI::Ptr cloud_temp;
	pcl::IndicesClustersPtr clusters;
	float max_distance;
	float max_intens;
	void copyCloud();
    PointCloudI::Ptr downsampled;
    PointCloudI::Ptr
    down_sample();

	void
	resetStemPoints();

};

#endif /* STEMPOINTDETECTION_H_ */
