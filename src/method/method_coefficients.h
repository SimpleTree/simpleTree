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
        float sphere_radius_multiplier;
        float epsilon_cluster_stem;
        float epsilon_cluster_branch;
        float epsilon_sphere;
        int minPts_ransac_stem;
        int minPts_ransac_branch;
        int minPts_cluster_stem;
        int minPts_cluster_branch;
        float min_radius_sphere_stem;
        float min_radius_sphere_branch;


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
          return str;

        }

};


#endif /* METHOD_COEFFICIENTS_H_ */
