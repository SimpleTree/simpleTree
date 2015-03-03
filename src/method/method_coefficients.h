/*
 * method_coefficients.h
 *
 *  Created on: Feb 8, 2015
 *      Author: hackenberg
 */

#ifndef METHOD_COEFFICIENTS_H_
#define METHOD_COEFFICIENTS_H_


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
};


#endif /* METHOD_COEFFICIENTS_H_ */
