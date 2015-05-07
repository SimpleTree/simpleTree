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
#include "SphereFollowing.h"



SphereFollowing::SphereFollowing(PointCloudI::Ptr treeCloud,
		boost::weak_ptr<Controller> control) {
	this->control = control;
	this->treeCloud = treeCloud;
	maxIterations = 1;
	pcl::console::TicToc tt;
	tt.tic();
	computeCylindersFromTree(treeCloud);
	QString str;
	float f = tt.toc() / 1000;
	str.append("Done sphere-following in ").append(QString::number(f)).append(
			" seconds.\n");
	getControl()->getGuiPtr()->writeConsole(str);
}

SphereFollowing::SphereFollowing(PointCloudI::Ptr treeCloud,
		boost::weak_ptr<Controller> control, int iterations,Method_Coefficients coeff) {
	this->treeCloud = treeCloud;
	this->control = control;



     sphereRadiusMultiplier = coeff.sphere_radius_multiplier;
    epsilon_cluster_stem = coeff.epsilon_cluster_stem;
    epsilon_cluster_branch = coeff.epsilon_cluster_branch;
    epsilon_sphere = coeff.epsilon_sphere;
    minPts_Ransac_stem = coeff.minPts_ransac_stem;
    minPts_Ransac_branch = coeff.minPts_ransac_branch;
    minPts_cluster_stem = coeff.minPts_cluster_stem;
    minPts_cluster_branch = coeff.minPts_cluster_branch;
    min_radius_sphere_stem = coeff.min_radius_sphere_stem;
    min_radius_sphere_branch = coeff.min_radius_sphere_branch;
	maxIterations = iterations;

	pcl::console::TicToc tt;
	tt.tic();
	computeCylindersFromTree(treeCloud);
	QString str;
	float f = tt.toc() / 1000;
	str.append("Done sphere-following in ").append(QString::number(f)).append(
			" seconds.\n");
	getControl()->getGuiPtr()->writeConsole(str);
}


SphereFollowing::SphereFollowing(PointCloudI::Ptr treeCloud,
		boost::weak_ptr<Controller> control, int iterations) {
	this->treeCloud = treeCloud;
	this->control = control;
	maxIterations = iterations;

	pcl::console::TicToc tt;
	tt.tic();
	computeCylindersFromTree(treeCloud);
	QString str;
	float f = tt.toc() / 1000;
	str.append("Done sphere-following in ").append(QString::number(f)).append(
			" seconds.\n");
	getControl()->getGuiPtr()->writeConsole(str);
}

SphereFollowing::~SphereFollowing() {
	// TODO Auto-generated destructor stub
}

boost::shared_ptr<Controller> SphereFollowing::getControl() {
	return this->control.lock();
}

PointCloudI::Ptr SphereFollowing::extractUnfittedPoints(
		std::vector<float> distances, PointCloudI::Ptr points) {
	PointCloudI::Ptr unfittedPoints(new PointCloudI);
	for (size_t i = 0; i < distances.size(); i++) {
		if (std::abs(distances[i]) > maxDistToModel) {
			unfittedPoints->push_back(points->points[i]);
		}
	}
	return unfittedPoints;
}

std::vector<int> SphereFollowing::indexOfPointsNearCylinder(
		pcl::octree::OctreePointCloudSearch<PointI>& octree,
		boost::shared_ptr<simpleTree::Cylinder>& cylinder,
		float factorEnLarge) {
	PointI queryPoint = cylinder->getCenterPoint();
    float radius = cylinder->getHalfSize();
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
    octree.radiusSearch(queryPoint, radius + factorEnLarge,
			pointIdxRadiusSearch, pointRadiusSquaredDistance);
	std::vector<int> indices;
	for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
		int index = (pointIdxRadiusSearch.at(i));
		indices.push_back(index);
	}
	return indices;
}

std::vector<float> SphereFollowing::distancesToModel(PointCloudI::Ptr cloud) {
	pcl::console::TicToc tt;
	tt.tic();
	std::vector<float> distances(cloud->points.size());
	std::fill(distances.begin(), distances.end(), 100);

	pcl::octree::OctreePointCloudSearch<PointI> octree(0.02f);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
	for (std::vector<pcl::ModelCoefficients>::iterator it = cylinders.begin();
			it != cylinders.end(); it++) {

		pcl::ModelCoefficients cyl = *it;
		boost::shared_ptr<simpleTree::Cylinder> cylinder = boost::make_shared<
				simpleTree::Cylinder>(cyl);
		std::vector<int> indices = indexOfPointsNearCylinder(octree, cylinder,
                0.05);
		for (size_t i = 0; i < indices.size(); i++) {
			PointI point = cloud->points[indices[i]];
			float dist = cylinder->distToPoint(point);
			if (std::abs(dist) < std::abs(distances[indices[i]])) {
				distances[indices[i]] = dist;
			}
		}
	}
	QString str;
	float f = tt.toc() / 1000;
	str.append("Distance to model computation in ").append(QString::number(f)).append(
			" seconds.\n");
    getControl()->getGuiPtr()->writeConsole(str);
	return distances;
}

void SphereFollowing::computeCylindersFromTree(PointCloudI::Ptr& treeCloud) {
	while (iteration < maxIterations) {
		if (iteration == 0) {
			computeCylindersFromCluster(treeCloud, true);
			QString str;
			str.append("In the first sphere-following run ").append(
					QString::number(cylinders.size())).append(
					" cylinders were detected.\n");
			getControl()->getGuiPtr()->writeConsole(str);

//       std::cout << "In the first sphere-following run " << cylinders.size () << " cylinders were detected." << std::endl;
		} else {
			std::vector<float> distances = distancesToModel(treeCloud);
			PointCloudI::Ptr remainingPoints = extractUnfittedPoints(distances,
					treeCloud);
			QString str;
			str.append("Not fitted by cylinders were ").append(
					QString::number(remainingPoints->size())).append(
					" points.\n");
			getControl()->getGuiPtr()->writeConsole(str);
        //	std::cout << "Not fitted by cylinders were "
            //		<< remainingPoints->size() << " points." << std::endl;
			std::vector<PointCloudI::Ptr> clusters = clusterPoints(
					remainingPoints,true);
			for (size_t i = 0; i < clusters.size(); i++) {
				if (clusters[i]->size() > 100) {
					computeCylindersFromCluster(clusters[i], false);
				}
			}
		}
		iteration++;
	}
}

std::vector<PointCloudI::Ptr> SphereFollowing::clusterPoints(
		PointCloudI::Ptr points_in, bool isStem) {
	pcl::search::KdTree<PointI>::Ptr tree(new pcl::search::KdTree<PointI>);
	tree->setInputCloud(points_in);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointI> ec;
	if(isStem)
	{
		ec.setClusterTolerance(epsilon_cluster_stem);  // 2cm
		ec.setMinClusterSize(minPts_cluster_stem);
	} else {
		ec.setClusterTolerance(epsilon_cluster_branch);  // 2cm
		ec.setMinClusterSize(minPts_cluster_branch);
	}

	ec.setSearchMethod(tree);
	ec.setInputCloud(points_in);
	ec.extract(cluster_indices);
	std::vector<PointCloudI::Ptr> clusters;
	for (std::vector<pcl::PointIndices>::const_iterator it =
			cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		PointCloudI::Ptr cluster(new PointCloudI);

		for (std::vector<int>::const_iterator pit = it->indices.begin();
				pit != it->indices.end(); pit++) {
			cluster->points.push_back(points_in->points[*pit]);  //*
		}
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;
		clusters.push_back(cluster);
	}
	return clusters;
}

std::vector<PointCloudI::Ptr> SphereFollowing::intersectionSphereSurface(
		pcl::octree::OctreePointCloudSearch<PointI>& octree,
		pcl::ModelCoefficients sphere, PointCloudI::Ptr &cloud_in) {
	std::vector<PointCloudI::Ptr> both_clouds;
  //  std::cout << "startsphere" << sphere <<std::endl;
	PointCloudI::Ptr pointsOnSphereSurface_stem(new PointCloudI);
	PointCloudI::Ptr pointsOnSphereSurface_branch(new PointCloudI);
	PointI searchPoint;
	searchPoint.x = sphere.values[0];
	searchPoint.y = sphere.values[1];
	searchPoint.z = sphere.values[2];
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = sphere.values[3] + SphereFollowing::epsilon_sphere;
	float minSquaredDist = (sphere.values[3] - SphereFollowing::epsilon_sphere)
			* (sphere.values[3] - SphereFollowing::epsilon_sphere);
	float maxSquaredDist = (sphere.values[3] + SphereFollowing::epsilon_sphere)
			* (sphere.values[3] + SphereFollowing::epsilon_sphere);
	octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
			pointRadiusSquaredDistance);
  //  std::cout << pointIdxRadiusSearch.size() << std::endl;
	for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
		if (pointRadiusSquaredDistance[i] > minSquaredDist
				&& pointRadiusSquaredDistance[i] < maxSquaredDist) {
			std::vector<bool> isStem = getControl()->getIsStem();
			if (isStem.size() == treeCloud->points.size()) {
				if (isStem.at(pointIdxRadiusSearch[i])) {
					pointsOnSphereSurface_stem->push_back(
							cloud_in->points[pointIdxRadiusSearch[i]]);
				} else {
					pointsOnSphereSurface_branch->push_back(
							cloud_in->points[pointIdxRadiusSearch[i]]);
				}
			} else {
				pointsOnSphereSurface_branch->push_back(
						cloud_in->points[pointIdxRadiusSearch[i]]);
			}
		}
		octree.deleteVoxelAtPoint(pointIdxRadiusSearch[i]);
	}
	both_clouds.push_back(pointsOnSphereSurface_stem);
	both_clouds.push_back(pointsOnSphereSurface_branch);
  //  std::cout << "start" <<std::endl;
	return both_clouds;
}

bool SphereFollowing::lowestZCluster(PointCloudI cloud_in,
		PointCloudI& cloud_out) {
	pcl::console::TicToc tt;
    cloud_out.points.clear();
	float minSize = std::numeric_limits< float>::max();
	for (size_t i = 0; i < cloud_in.points.size(); i++) {
		if (cloud_in.points[i].z < minSize) {
			minSize = cloud_in.points[i].z;
		}

	}

	for (size_t i = 0; i < cloud_in.points.size(); i++) {
        if (cloud_in.points[i].z < (minSize + 0.1)
				&& cloud_in.points[i].z >= minSize) {
			cloud_out.push_back(cloud_in.points[i]);
		}
	}
	return true;
}

pcl::ModelCoefficients SphereFollowing::fitSphereToCluster(
        PointCloudI::Ptr cluster, float& radius, bool isStem) {
    float x, y, z ,r;
	x = 0;
	y = 0;
	z = 0;
	PointCloudI pts = *cluster;
	for (size_t i = 0; i < cluster->points.size(); i++) {
		x += cluster->points[i].x;
		y += cluster->points[i].y;
		z += cluster->points[i].z;
	}
	x /= cluster->points.size();
	y /= cluster->points.size();
	z /= cluster->points.size();

	std::vector<float> distances;

	for (size_t i = 0; i < cluster->points.size(); i++) {
		float dist = std::sqrt(
				(cluster->points[i].x - x) * (cluster->points[i].x - x)
						+ (cluster->points[i].y - y)
								* (cluster->points[i].y - y)
						+ (cluster->points[i].z - z)
								* (cluster->points[i].z - z));
		distances.push_back(dist);

	}
	std::nth_element(distances.begin(),
			distances.begin() + distances.size() / 2, distances.end());
	r = distances[distances.size() / 2];

    if (r > 0.035&&use_ransac_for_sphere) {
        pcl::ModelCoefficients::Ptr coefficients_circle3d(
                new pcl::ModelCoefficients);

        pcl::SACSegmentationFromNormals<PointI, PointI> seg;
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE3D);
        seg.setMethodType(pcl::SAC_MLESAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cluster);
        seg.setInputNormals(cluster);

        // Obtain the cylinder inliers and coefficients
        seg.segment(*inliers_cylinder, *coefficients_circle3d);
        if(coefficients_circle3d->values.size()!=0)
        {



        if(coefficients_circle3d->values[3]< r*3)
        {

            x = coefficients_circle3d->values[0];
            y = coefficients_circle3d->values[1];
            z = coefficients_circle3d->values[2];
            r = coefficients_circle3d->values[3];
        }
        }

    }
	pcl::ModelCoefficients coeffs;
	coeffs.values.clear();
	coeffs.values.push_back(x);
	coeffs.values.push_back(y);
	coeffs.values.push_back(z);
	if (r * sphereRadiusMultiplier > 0.05) {
		coeffs.values.push_back(r * sphereRadiusMultiplier);
	} else {
		if(isStem)
		{
			coeffs.values.push_back(min_radius_sphere_stem);
		} else {
			coeffs.values.push_back(min_radius_sphere_branch);
		}

	}
	radius = r;

	return coeffs;
}

void SphereFollowing::addConnectionCylinder(
		pcl::ModelCoefficients startSphere, float start_radius) {
    float minDist = std::numeric_limits<float>::max();
	pcl::PointXYZ candidateStartPoint(0, 0, 0);
	pcl::PointXYZ closestStartPoint(0, 0, 0);
	pcl::PointXYZ sphereCenter(startSphere.values[0], startSphere.values[1],
            startSphere.values[2]);
	for (size_t i = 0; i < cylinders.size(); i++) {
		pcl::ModelCoefficients cylinder = cylinders[i];
		candidateStartPoint.x = cylinder.values[0] + cylinder.values[3];
		candidateStartPoint.y = cylinder.values[1] + cylinder.values[4];
		candidateStartPoint.z = cylinder.values[2] + cylinder.values[5];
		float distance = sqrt(
				((candidateStartPoint.x - sphereCenter.x)
						* (candidateStartPoint.x - sphereCenter.x))
						+ ((candidateStartPoint.y - sphereCenter.y)
								* (candidateStartPoint.y - sphereCenter.y))
						+ ((candidateStartPoint.z - sphereCenter.z)
                                * (candidateStartPoint.z - sphereCenter.z)));
		if (distance < minDist) {
            minDist = distance;
			closestStartPoint = candidateStartPoint;
		}
    }
	pcl::ModelCoefficients connectionCylinder;

	connectionCylinder.values.push_back(closestStartPoint.x);
	connectionCylinder.values.push_back(closestStartPoint.y);
	connectionCylinder.values.push_back(closestStartPoint.z);
	connectionCylinder.values.push_back(sphereCenter.x - closestStartPoint.x);
	connectionCylinder.values.push_back(sphereCenter.y - closestStartPoint.y);
	connectionCylinder.values.push_back(sphereCenter.z - closestStartPoint.z);
	connectionCylinder.values.push_back(
			start_radius);

	//std::cout << startSphere<< std::endl;
	//std::cout << connectionCylinder<< std::endl;
	cylinders.push_back(connectionCylinder);

}

void SphereFollowing::computeCylindersFromCluster(
		PointCloudI::Ptr& pointCluster, bool isFirstRun) {

	pcl::ModelCoefficients currentSphere;
	pcl::ModelCoefficients oldSphere;
	std::deque<pcl::ModelCoefficients> nextLevelSpheres;

	float resolution = 0.025f;
	pcl::octree::OctreePointCloudSearch<PointI> octree(resolution);
	octree.setInputCloud(pointCluster);
	octree.addPointsFromInputCloud();

	PointCloudI::Ptr startCloud(new PointCloudI);
	lowestZCluster(*pointCluster, *startCloud);
   // std::cout << startCloud->points.size() << "start cluster" << std::endl;
    float start_radius = 0.5;
	pcl::ModelCoefficients startSphere = fitSphereToCluster(startCloud,
	                                                        start_radius,true);

	nextLevelSpheres.push_back(startSphere);
	if (!isFirstRun) {
		addConnectionCylinder(startSphere,start_radius);
	}
//  int count = 0;
	while (!nextLevelSpheres.empty()) {
		currentSphere = (nextLevelSpheres.front());
		nextLevelSpheres.pop_front();
		while (currentSphere.values.size() > 0) {

			oldSphere = currentSphere;
			spheres.push_back(currentSphere);
			std::vector<PointCloudI::Ptr> sphereSurface =
					intersectionSphereSurface(octree, currentSphere,
							pointCluster);

            if ( !sphereSurface[0]->points.empty()) {

				bool hasNextLevelSphere = false;
				std::vector<PointCloudI::Ptr> clusters = clusterPoints(
						sphereSurface[0],true);
				if (!clusters.empty()) {
					PointCloudI::Ptr cluster = clusters[0];
					pcl::ModelCoefficients nextSphere = fitSphereToCluster(
							cluster, start_radius,true);
					//std::cout << nextSphere << "\n";
					pcl::ModelCoefficients cylinder;
					cylinder.values.clear();
					cylinder.values.push_back(oldSphere.values[0]);
					cylinder.values.push_back(oldSphere.values[1]);
					cylinder.values.push_back(oldSphere.values[2]);
					cylinder.values.push_back(
							nextSphere.values[0] - oldSphere.values[0]);
					cylinder.values.push_back(
							nextSphere.values[1] - oldSphere.values[1]);
					cylinder.values.push_back(
							nextSphere.values[2] - oldSphere.values[2]);
					cylinder.values.push_back(start_radius);
					cylinders.push_back(cylinder);
					currentSphere = nextSphere;
					hasNextLevelSphere = true;
					//nextLevelSpheres.push_back (nextSphere);
					if (clusters.size() >= 2) {
						for (size_t i = 1; i < clusters.size(); i++)

						{
							PointCloudI::Ptr cluster = clusters[i];
							float radius = 0.1;
							nextSphere = fitSphereToCluster(cluster, radius,true);
							pcl::ModelCoefficients cylinder;
							cylinder.values.clear();
							cylinder.values.push_back(oldSphere.values[0]);
							cylinder.values.push_back(oldSphere.values[1]);
							cylinder.values.push_back(oldSphere.values[2]);
							cylinder.values.push_back(
									nextSphere.values[0] - oldSphere.values[0]);
							cylinder.values.push_back(
									nextSphere.values[1] - oldSphere.values[1]);
							cylinder.values.push_back(
									nextSphere.values[2] - oldSphere.values[2]);
							cylinder.values.push_back(radius);
							cylinders.push_back(cylinder);
							nextLevelSpheres.push_back(nextSphere);
						}
					}
				}
				clusters = clusterPoints(sphereSurface[1],false);
				if (!clusters.empty()) {
					if (clusters.size() >= 1) {
						if (hasNextLevelSphere) {
							for (size_t i = 0; i < clusters.size(); i++)

							{
								PointCloudI::Ptr cluster = clusters[i];
								float radius = 0.1;
								pcl::ModelCoefficients nextSphere =
										fitSphereToCluster(cluster, radius,false);
								pcl::ModelCoefficients cylinder;
								cylinder.values.clear();
								cylinder.values.push_back(oldSphere.values[0]);
								cylinder.values.push_back(oldSphere.values[1]);
								cylinder.values.push_back(oldSphere.values[2]);
								cylinder.values.push_back(
										nextSphere.values[0]
												- oldSphere.values[0]);
								cylinder.values.push_back(
										nextSphere.values[1]
												- oldSphere.values[1]);
								cylinder.values.push_back(
										nextSphere.values[2]
												- oldSphere.values[2]);
								cylinder.values.push_back(radius);
								cylinders.push_back(cylinder);
								nextLevelSpheres.push_back(nextSphere);
							}
						} else {
							PointCloudI::Ptr cluster = clusters[0];
							pcl::ModelCoefficients nextSphere =
									fitSphereToCluster(cluster, start_radius,false);
							pcl::ModelCoefficients cylinder;
							cylinder.values.clear();
							cylinder.values.push_back(oldSphere.values[0]);
							cylinder.values.push_back(oldSphere.values[1]);
							cylinder.values.push_back(oldSphere.values[2]);
							cylinder.values.push_back(
									nextSphere.values[0] - oldSphere.values[0]);
							cylinder.values.push_back(
									nextSphere.values[1] - oldSphere.values[1]);
							cylinder.values.push_back(
									nextSphere.values[2] - oldSphere.values[2]);
							cylinder.values.push_back(start_radius);
							cylinders.push_back(cylinder);
							currentSphere = nextSphere;
							hasNextLevelSphere = true;
							//nextLevelSpheres.push_back (nextSphere);
							if (clusters.size() >= 2) {
								for (size_t i = 1; i < clusters.size(); i++)

								{
									PointCloudI::Ptr cluster = clusters[i];
									float radius = 0.1;
									nextSphere = fitSphereToCluster(cluster,
											radius,false);
									pcl::ModelCoefficients cylinder;
									cylinder.values.clear();
									cylinder.values.push_back(
											oldSphere.values[0]);
									cylinder.values.push_back(
											oldSphere.values[1]);
									cylinder.values.push_back(
											oldSphere.values[2]);
									cylinder.values.push_back(
											nextSphere.values[0]
													- oldSphere.values[0]);
									cylinder.values.push_back(
											nextSphere.values[1]
													- oldSphere.values[1]);
									cylinder.values.push_back(
											nextSphere.values[2]
													- oldSphere.values[2]);
									cylinder.values.push_back(radius);
									cylinders.push_back(cylinder);
									nextLevelSpheres.push_back(nextSphere);
								}
							}
						}

					}
				}
				//std::cout << hasNextLevelSphere << std::endl;
				if(!hasNextLevelSphere)
				{
					currentSphere.values.clear();
				}
			} else if (!sphereSurface[1]->points.empty()) {
                                        //    std::cout << sphereSurface[1]->points.size() << std::endl;
				std::vector<PointCloudI::Ptr> clusters = clusterPoints(
						sphereSurface[1],false);
				if (!clusters.empty()) {
					PointCloudI::Ptr cluster = clusters[0];
					pcl::ModelCoefficients nextSphere = fitSphereToCluster(
							cluster, start_radius,false);
					pcl::ModelCoefficients cylinder;
					cylinder.values.clear();
					cylinder.values.push_back(oldSphere.values[0]);
					cylinder.values.push_back(oldSphere.values[1]);
					cylinder.values.push_back(oldSphere.values[2]);
					cylinder.values.push_back(
							nextSphere.values[0] - oldSphere.values[0]);
					cylinder.values.push_back(
							nextSphere.values[1] - oldSphere.values[1]);
					cylinder.values.push_back(
							nextSphere.values[2] - oldSphere.values[2]);
					cylinder.values.push_back(start_radius);
					cylinders.push_back(cylinder);
					currentSphere = nextSphere;
					//nextLevelSpheres.push_back (nextSphere);
					if (clusters.size() >= 2) {
						for (size_t i = 1; i < clusters.size(); i++)

						{
							PointCloudI::Ptr cluster = clusters[i];
							float radius = 0.1;
							nextSphere = fitSphereToCluster(cluster, radius,false);
							pcl::ModelCoefficients cylinder;
							cylinder.values.clear();
							cylinder.values.push_back(oldSphere.values[0]);
							cylinder.values.push_back(oldSphere.values[1]);
							cylinder.values.push_back(oldSphere.values[2]);
							cylinder.values.push_back(
									nextSphere.values[0] - oldSphere.values[0]);
							cylinder.values.push_back(
									nextSphere.values[1] - oldSphere.values[1]);
							cylinder.values.push_back(
									nextSphere.values[2] - oldSphere.values[2]);
							cylinder.values.push_back(radius);
							cylinders.push_back(cylinder);
							nextLevelSpheres.push_back(nextSphere);
						}
					}
				} else {
					currentSphere.values.clear();
				}
			} else {
				currentSphere.values.clear();
			}
		}
	}

}
