/*
 * method_coefficients.h
 *
 *  Created on: Feb 8, 2015
 *      Author: hackenberg
 */

#ifndef METHOD_COEFFICIENTS_H_
#define METHOD_COEFFICIENTS_H_

#include <QString>

struct Method_Coefficients{
        QString name;
        //float epsilon;
        float sphere_radius_multiplier=0;
        float epsilon_cluster_stem=0;
        float epsilon_cluster_branch=0;
        float epsilon_sphere=0;
        int minPts_ransac_stem = 0;
        int minPts_ransac_branch=0;
        int minPts_cluster_stem=0;
        int minPts_cluster_branch=0;
        float min_radius_sphere_stem=0;
        float min_radius_sphere_branch=0;
        float a=35.671;
        float b=2.204;
        float minRad=0.0025;
        float fact=2;
        float min_dist=0.0001;
        float max_iterations=2;
        float seeds_per_voxel=81;
	float stem_min_height = 0;
	float bin_width = 1;
	float circle_epsilon = 0.05;
        bool do_stem_optimization = true;
        bool find_more = false;


        QString
        toQString()
        {
          QString str;
          str.append("Sphere radius multiplier : ").append(QString::number(sphere_radius_multiplier)).append("\n");
          str.append("epsilon_cluster_stem : ").append(QString::number(epsilon_cluster_stem)).append("\n");
          str.append("epsilon_cluster_branch : ").append(QString::number(epsilon_cluster_branch)).append("\n");
          str.append("epsilon sphere : ").append(QString::number(epsilon_sphere)).append("\n");
          str.append("minPts_ransac_stem : ").append(QString::number(minPts_ransac_stem)).append("\n");
          str.append("minPts_ransac_branch : ").append(QString::number(minPts_ransac_branch)).append("\n");
          str.append("minPts_cluster_stem : ").append(QString::number(minPts_cluster_stem)).append("\n");
          str.append("minPts_cluster_branch : ").append(QString::number(minPts_cluster_branch)).append("\n");
          str.append("min_radius_sphere_stem : ").append(QString::number(min_radius_sphere_stem)).append("\n");
          str.append("min_radius_sphere_branch : ").append(QString::number(min_radius_sphere_branch)).append("\n");
          str.append("Sphere radius multiplier : ").append(QString::number(sphere_radius_multiplier)).append("\n");
          //here i added value for allom
          str.append("a : ").append(QString::number(a)).append("\n");
          str.append("b : ").append(QString::number(b)).append("\n");
          str.append("fact : ").append(QString::number(fact)).append("\n");
          str.append("minRad : ").append(QString::number(minRad)).append("\n");
          //added variable for otimizaition
                str.append("min_dist : ").append(QString::number(min_dist)).append("\n");
                str.append("max_iterations : ").append(QString::number(max_iterations)).append("\n");
                str.append("seeds_per_voxel : ").append(QString::number(seeds_per_voxel)).append("\n");
                str.append("stem_min_height : ").append(QString::number(stem_min_height)).append("\n");
                str.append("bin_width : ").append(QString::number(bin_width)).append("\n");
                str.append("circle_epsilon : ").append(QString::number(circle_epsilon)).append("\n");
                if(find_more)
                {
                    str.append("Iterative search for spherefollowing was activated").append("\n");
                }
                else
                {
                    str.append("Iterative search for spherefollowing was de-activated").append("\n");
                }
                if(do_stem_optimization)
                {
                    str.append("Stem modelling parameters were automatically searched for").append("\n");
                }
                else
                {
                    str.append("Stem modelling parameters have been set manually.").append("\n");
                }
          return str;
        }

};


#endif /* METHOD_COEFFICIENTS_H_ */
