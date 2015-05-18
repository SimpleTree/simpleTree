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

#include "Tree.h"

namespace simpleTree {

    Tree::Tree ( std::vector<pcl::ModelCoefficients> cylinders,
    pcl::PointCloud<PointI>::Ptr cloud_Ptr,
    std::string name,
    boost::weak_ptr<Controller> control , bool computeCrown ) {
        this->control = control;
        this->cloud_Ptr = cloud_Ptr;
        this->name = name;
        QString str;
        str.append ( "Building a tree model for " ).append ( QString::number ( cylinders.size () ) ).append ( " detected cylinders.\n" );
        getControl ()->getGuiPtr ()->writeConsole ( str );
        QCoreApplication::processEvents ();
        setCylinders ( cylinders );

        child_Cylinder_Extraction.reset(new ChildCylinderExtraction(this->cylinders));
        Cylinder rootCylinder = cylinders.at ( 0 );
        rootSegment = boost::make_shared<Segment> ();
        rootSegment->addCylinder ( rootCylinder );

        buildTree ( rootSegment, getRootCylinder () );
        setCylinders ();
        child_Cylinder_Extraction.reset(new ChildCylinderExtraction(this->cylinders));
        rootSegment = boost::make_shared<Segment> ();
        rootSegment->addCylinder ( rootCylinder );
        buildTree ( rootSegment, getRootCylinder () );
        reorderTree();
        mergeCylinders ();
        improveBranchJunctions ();
        improveFit ();
        closeGaps ();
        int cylinders_size = getCylinders ().size ();
        int removed_cylinders_size = cylinders.size () - cylinders_size;

        str = "";
        str.append ( QString::number ( removed_cylinders_size ) ).append ( " cylinders were removed due to bad allocation." ).append ( QString::number ( cylinders_size ) ).append (
                    "  cylinders are still stored in tree structure.\n" );
        getControl ()->getGuiPtr ()->writeConsole ( str );
        QCoreApplication::processEvents ();
        reorderTree();
        detectStem ();
        reset_stem();
        setIDforSegments();
        if(computeCrown)
        {
            detectCrown ();
            crown = boost::make_shared<Crown> ( crownPoints (), this->control );
        }
        improveByMedianCheck ();
                resetRoot();



        str = "";
        str.append ( "The total volume of the tree is " ).append ( QString::number ( getVolume () ) ).append ( "m^3.\n" );
        getControl ()->getGuiPtr ()->writeConsole ( str );
        QCoreApplication::processEvents ();
    }
    Tree::Tree () {

    }

    void
    Tree::reset_crown(float h)
    {
        std::vector<boost::shared_ptr<Cylinder> > stemCylinders = getStemCylinders();
        for(std::vector<boost::shared_ptr<Cylinder> >::iterator it = stemCylinders.begin(); it != stemCylinders.end(); it++)
        {
            boost::shared_ptr<Cylinder> cyl = *it;
            if(cyl->values[2]<=h && (cyl->values[2]+cyl->values[5])>=h)
            {
                firstCrownSegment = cyl->getSegment();
                crown = boost::make_shared<Crown> ( crownPoints (), this->control );
                getControl()->setTreePtr(getControl()->getTreePtr());
                QCoreApplication::processEvents ();
            }
        }
    }

    void
    Tree::reset_stem() {
        std::vector<boost::shared_ptr<Segment> > allLeaveSegments = getLeaveSegments ();
        topStemSegment = rootSegment;
                for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allLeaveSegments.begin (); it != allLeaveSegments.end (); it++ ) {
            boost::shared_ptr<Segment> leave = *it;

            if ( leave->getEnd().z > topStemSegment->getEnd().z ) {
                topStemSegment = leave;
            }
        }
        detectCrown ();
        crown = boost::make_shared<Crown> ( crownPoints (), this->control );
        improveByMedianCheck ();
        reorderTree();
        setIDforSegments();

        QString str = "";
        str.append ( "The total volume of the tree is " ).append ( QString::number ( getVolume () ) ).append ( "m^3.\n" );
        getControl ()->getGuiPtr ()->writeConsole ( str );
        QCoreApplication::processEvents ();
    }

    void
    Tree::resetRoot()
    {
        boost::shared_ptr<Cylinder> cylinder = rootSegment->getCylinders().at(0);
        float temp = cylinder->values[2];
        cylinder->values[2] = 0.1;
        cylinder->values[5] += (temp-0.1);
    }

    void Tree::setIDforSegments() {
        setBranchOrder ( getRootSegment(),0 );
        QCoreApplication::processEvents ();
        setSegmentID();
        QCoreApplication::processEvents ();
        setBranchID ( getRootSegment(),0 );
        QCoreApplication::processEvents ();
    }
    
    std::vector< boost::shared_ptr< Segment > > 
    Tree::getLeaveSegmentsFromSegment ( boost::shared_ptr< Segment > base ) {
	std::vector< boost::shared_ptr< Segment > > allSegments = getChildGenerationSegments(base);
	std::vector< boost::shared_ptr< Segment > > leaveSegments;
	for(size_t i = 0 ; i < allSegments.size(); i++)
	{
	  boost::shared_ptr< Segment > seg = allSegments.at(i);
	  if(seg->getChildren().size()==0)
	  {
	    leaveSegments.push_back(seg);
	  }
	}
	return leaveSegments;
    }
    
    
    boost::shared_ptr< Segment > Tree::getLeaveSegmentWithMostVolume ( std::vector< boost::shared_ptr< Segment > > leaves, boost::shared_ptr< Segment > base ) {
	boost::shared_ptr< Segment > leave = base;
	float maxVolume = base->getVolume();
	for(size_t i = 0; i < leaves.size(); i++)
	{
	  float volume = getVolumeToLeave(leaves.at(i),base);
	  if(volume > maxVolume)
	  {
	    maxVolume = volume;
	    leave = leaves.at(i);
	  }
	}
	return leave;
    }
    
    
    float Tree::getVolumeToLeave ( boost::shared_ptr< Segment > leave, boost::shared_ptr< Segment > base ) {
	float volume = 0;
	volume = leave->getVolume();
	while(leave != base)
	{
	  volume += leave->getParent()->getVolume();
	  leave = leave->getParent();
	}
	return volume;
    }


    
    
     bool
     Tree::compareChildren(boost::shared_ptr< Segment > child1, boost::shared_ptr< Segment > child2)
     {
	  Tree tree;
          std::vector<boost::shared_ptr<Segment> > leaves1 = tree.getLeaveSegmentsFromSegment(child1);
          boost::shared_ptr<Segment> leave1 = tree.getLeaveSegmentWithMostVolume( leaves1,child1 );
	  float volume1 = tree.getVolumeToLeave(leave1, child1);
	  std::vector<boost::shared_ptr<Segment> > leaves2 = tree.getLeaveSegmentsFromSegment(child2);
          boost::shared_ptr<Segment> leave2 = tree.getLeaveSegmentWithMostVolume( leaves2,child2 );
	  float volume2 = tree.getVolumeToLeave(leave2, child2);
	  return(volume2<volume1);
     }
    
    void
    Tree::reorderTree(){
      std::vector<boost::shared_ptr<Segment> > allSegments = getSegments();
      for(size_t i = 0 ; i < allSegments.size(); i++)
      {
	boost::shared_ptr<Segment> segment = allSegments.at(i);
	if(segment->getChildren().size()>0)
	{
	  std::vector<boost::shared_ptr<Segment> > children = segment->getChildren();

      std::sort (children.begin(), children.end(), compareChildren);

	  segment->setChildren(children);
    }
      }
    }
    
    

     
    void Tree::setBranchOrder ( boost::shared_ptr< Segment > segment, float order ) {
        if(order == 0)
        {
            segment->branchOrder = order;
            if ( segment->getChildren().size() >0 ) {
                std::vector<boost::shared_ptr<Segment> > stemSegments;
                boost::shared_ptr<Segment> stemSegment = getStemTopSegment ();
                while ( stemSegment ) {
                    stemSegments.push_back(stemSegment);
                    stemSegment = stemSegment->getParent ();
                }

                for(size_t i = 0; i < segment->getChildren().size(); i++)
                {
                    boost::shared_ptr<Segment> child = segment->getChildren().at ( i );
                    bool isStem = false;
                    for(size_t i = 0; i < stemSegments.size(); i ++)
                    {
                        if(stemSegments.at(i)==child)
                        {
                            isStem = true;
                        }
                    }
                    if(isStem)
                    {
                        setBranchOrder ( child,0 );
                    } else {
                        setBranchOrder(child, order+1);
                    }
                }



            }
        } else {
            segment->branchOrder = order;
            if ( segment->getChildren().size() >0 ) {
                boost::shared_ptr<Segment> firstChild = segment->getChildren().at ( 0 );
                setBranchOrder ( firstChild,order );
                if ( segment->getChildren().size() >1 ) {
                    for ( size_t i = 1; i < segment->getChildren().size(); i++ ) {
                        boost::shared_ptr<Segment> child = segment->getChildren().at ( i );
                        setBranchOrder ( child,order+1 );
                    }
                }
            }
        }


    }


    void Tree::setSegmentID() {
        std::vector<boost::shared_ptr<Segment> > segments = getSegments();
        for ( size_t i = 0; i < segments.size(); i++ ) {
            boost::shared_ptr<Segment> seg = segments.at ( i );
            int j = i+1;
            seg->segmentID = j ;
        }
    }


    void Tree::setBranchID ( boost::shared_ptr<Segment> segment,  float ID ) {
        if ( ID==0 ) {
            std::vector< boost::shared_ptr<Segment> >stemSegments;
            boost::shared_ptr<Segment> currentSegment = this->getStemTopSegment();
            while ( currentSegment) {
                stemSegments.push_back ( currentSegment );
                currentSegment->branchID = 0;
                currentSegment = currentSegment->getParent();
            }
            int branchID = 1;
            for ( size_t i = 0; i < stemSegments.size(); i++ ) {
                boost::shared_ptr<Segment> currentSegment = stemSegments.at ( i );
                if ( currentSegment->getChildren().size() >0 ) {
                    for ( size_t j = 0; j < currentSegment->getChildren().size(); j++ ) {
                        boost::shared_ptr<Segment> child = currentSegment->getChildren().at ( j );
                        bool isStem = false;
                        for(size_t k = 0; k < stemSegments.size(); k ++)
                        {
                            if(stemSegments.at(k)==child)
                            {
                                isStem = true;
                            }
                        }
                        if(isStem)
                        {
                            segment->branchID = 0;
                        } else {
                            setBranchID(child, branchID);
                        }
                        branchID ++;
                    }
                }
            }
        } else {
            segment->branchID = ID;
            for ( size_t j = 0; j < segment->getChildren().size(); j++ ) {
                boost::shared_ptr<Segment> childSegment = segment->getChildren().at ( j );
                setBranchID ( childSegment, ID );
            }
        }

    }


    boost::shared_ptr<Controller>
    Tree::getControl () {
        return this->control.lock ();
    }

    std::string
    Tree::string () {
        QString str = QString::fromStdString ( this->name );
        str.append ( "\n" );
        str.append ( "Height (m): " );
        str.append ( QString::number ( getHeight () ) );
        str.append ( "\n" );
        str.append ( "DBH (cm): " );
        str.append ( QString::number ( getDBH () ) );
        str.append ( "\n" );
        str.append ( "Volume (l)" );
        str.append ( QString::number ( getVolume () * 1000 ) );
        str.append ( "\n" );
        return str.toStdString ();
    }

    boost::shared_ptr<Segment>
    Tree::getFirstCrownSegment () {
        return this->firstCrownSegment;
    }

    float
    Tree::getHeightAboveGround () {
        return this->getRootCylinder ()->values[2];
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    Tree::crownPoints () {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr ( new pcl::PointCloud<pcl::PointXYZ> );
        std::vector<boost::shared_ptr<Segment> > allCrownSegments = getChildGenerationSegments ( getFirstCrownSegment () );
        std::vector<boost::shared_ptr<Cylinder> > cylinders;
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allCrownSegments.begin (); it != allCrownSegments.end (); it++ ) {
            boost::shared_ptr<Segment> seg = *it;
            if(seg!=getFirstCrownSegment())
            {
                cylinders.insert ( cylinders.end (), seg->getCylinders ().begin (), seg->getCylinders ().end () );
            }
        }
        for ( std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
            boost::shared_ptr<Cylinder> cyl = *it;
            cloud_ptr->push_back ( cyl->getEnd () );
        }
        return cloud_ptr;
    }

    Tree::~Tree () {
        // TODO Auto-generated destructor stub
    }

    boost::shared_ptr<Segment>
    Tree::getStemTopSegment () {
        return topStemSegment;
    }

    void
    Tree::detectCrown () {
        firstCrownSegment = getStemTopSegment ();
        boost::shared_ptr<Segment> segment = getStemTopSegment ();
        while ( segment ) {
            std::vector<boost::shared_ptr<Segment> > children = segment->getChildren ();
            if ( children.size () > 1 ) {
                int counterChilds = 0;
                for ( size_t i = 0; i < children.size (); i++ ) {
                    boost::shared_ptr<Segment> child = children[i];
                    std::vector<boost::shared_ptr<Segment> > segments = getChildGenerationSegments ( child );
                    float volume = 0;
                    for ( std::vector<boost::shared_ptr<Segment> >::iterator it = segments.begin (); it != segments.end (); it++ ) {
                        boost::shared_ptr<Segment> segment = *it;
                        volume += segment->getVolume ();
                    }

                    if ( volume >= minVolumeForFirstCrownBranch ) {
                        counterChilds++;
                    }

                }
                if ( counterChilds >= minNumberOfFirstCrownBranches ) {
                    firstCrownSegment = segment;
                }
            }
            segment = segment->getParent ();

        }
        QString str;
        // float f = tt.toc () / 1000;
        str.append ( QString::number ( firstCrownSegment->getEnd ().z ) ).append ( "m is the crown height..\n" );
        getControl ()->getGuiPtr ()->writeConsole ( str );
        QCoreApplication::processEvents ();

    }

    float
    Tree::getDBH () {
        float DBH = 0;
        float distanceOfCylinderToDBH = std::numeric_limits<float>::max();
        std::vector<boost::shared_ptr<Cylinder> > cylinders = getStemCylinders ();
        for ( std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {

            boost::shared_ptr<Cylinder> cylinder = *it;

            float dist = 0;

//            if ( cylinder->values[2] > 1.3 ) {
//                //BHD = cylinder->values[6] * 200;

//            }

            if ( ( cylinder->values[2] <= 1.3 ) && ( ( cylinder->values[2] + cylinder->values[5] ) >= 1.3 ) ) {
                //BHD = cylinder->values[6] * 200;
                dist = 0;
            } else if(cylinder->values[2] > 1.3){
               dist = cylinder->values[2] - 1.3;
            } else if((cylinder->values[2]+ cylinder->values[5]) < 1.3 ){
                dist = 1.3 - (cylinder->values[2]+ cylinder->values[5]) ;
            }
            if(dist < distanceOfCylinderToDBH){
                distanceOfCylinderToDBH = dist;
                DBH = cylinder->values[6] * 200;
            }

        }
        return DBH;
    }

    float
    Tree::getBaseDiameter () {
        boost::shared_ptr<Cylinder> cylinder = getRootCylinder ();
        float dia = cylinder->values[6] * 200;
        return dia;
    }

    float
    Tree::getHeight () {

        boost::shared_ptr<Cylinder> top = topStemSegment->getCylinders ().back ();
        float height = 0;
        height = top->values[2] + top->values[5];
        return height;

    }

    float
    Tree::getLength () {
        float length = 0;
        std::vector<boost::shared_ptr<Cylinder> > cylinders = getStemCylinders ();
        for ( std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
            boost::shared_ptr<Cylinder> cylinder = *it;
            length += cylinder->getLength ();
        }
        length += getHeightAboveGround ();
        return length;
    }

    float
    Tree::getRootSegmentVolume () {
        return rootSegment->getVolume ();
    }

    float
    Tree::getSolidVolume () {
        float volume = 0;
        std::vector<boost::shared_ptr<Cylinder> > cylinders = getStemCylinders ();
        for ( std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
            boost::shared_ptr<Cylinder> cylinder = *it;
            if ( cylinder->values[6] >= solidRadius ) {
                volume += cylinder->getVolume ();
            }
        }
        return volume;
    }

    std::vector<float>
    Tree::distancesToModel () {
        pcl::console::TicToc tt;
        tt.tic ();
        std::vector<float> distances ( cloud_Ptr->points.size () );
        std::fill ( distances.begin (), distances.end (), 0.1 );

        pcl::octree::OctreePointCloudSearch<PointI> octree ( octreeResolution );
        octree.setInputCloud ( cloud_Ptr );
        octree.addPointsFromInputCloud ();
        std::vector<boost::shared_ptr<Cylinder> > cylinders = getCylinders ();
        for ( std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {

            boost::shared_ptr<Cylinder> cylinder = *it;
            std::vector<int> indices = indexOfPointsNearCylinder ( octree, cylinder, 1.5f );
            for ( size_t i = 0; i < indices.size (); i++ ) {
                PointI point = cloud_Ptr->points[indices[i]];
                float dist = cylinder->distToPoint ( point );
                if ( std::abs ( dist ) < std::abs ( distances[indices[i]] ) ) {
                    distances[indices[i]] = dist;
                }
            }
        }
        return distances;
    }

    std::vector<int>
    Tree::indexOfPointsNearCylinder ( pcl::octree::OctreePointCloudSearch<PointI>& octree,
                                      boost::shared_ptr<Cylinder>& cylinder,
                                      float factorEnLarge ) {
        PointI queryPoint = cylinder->getCenterPoint ();
        float radius = cylinder->getHalfSize ();
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        float r = std::max<float> ( radius * factorEnLarge, minSearchRadius );
        octree.radiusSearch ( queryPoint, r, pointIdxRadiusSearch, pointRadiusSquaredDistance );
        std::vector<int> indices;
        for ( size_t i = 0; i < pointIdxRadiusSearch.size (); i++ ) {
            int index = ( pointIdxRadiusSearch.at ( i ) );
            indices.push_back ( index );
        }
        return indices;
    }

    pcl::PointCloud<PointI>::Ptr
    Tree::pointsNearCylinder ( pcl::octree::OctreePointCloudSearch<PointI> &octree,
                               boost::shared_ptr<Cylinder> &cylinder,
                               float factorEnLarge ) {
        PointI queryPoint = cylinder->getCenterPoint ();
        float radius = cylinder->getHalfSize ();
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        octree.radiusSearch ( queryPoint, radius * factorEnLarge, pointIdxRadiusSearch, pointRadiusSquaredDistance );
        pcl::PointCloud<PointI>::Ptr cylinderPoints ( new pcl::PointCloud<PointI> );
        for ( size_t i = 0; i < pointIdxRadiusSearch.size (); i++ ) {
            int index = ( pointIdxRadiusSearch.at ( i ) );
            cylinderPoints->push_back ( cloud_Ptr->points[index] );
        }
        return cylinderPoints;
    }

    pcl::ModelCoefficients::Ptr
    Tree::doRANSAC ( pcl::PointCloud<PointI>::Ptr cylinderPoints ) {
        pcl::ModelCoefficients::Ptr coeff ( new pcl::ModelCoefficients );
        pcl::SACSegmentationFromNormals<PointI, PointI> seg;
        seg.setOptimizeCoefficients ( true );
        seg.setModelType ( pcl::SACMODEL_CYLINDER );
        seg.setMethodType ( SACMETHOD_TYPE );
        seg.setNormalDistanceWeight ( 0.12 );
        seg.setMaxIterations ( 1000 );
        seg.setDistanceThreshold ( 0.05 );
        seg.setRadiusLimits ( 0, 2.2 );
        seg.setInputCloud ( cylinderPoints );
        seg.setInputNormals ( cylinderPoints );
        pcl::PointIndices::Ptr inliers_cylinder ( new pcl::PointIndices );
        seg.segment ( *inliers_cylinder, *coeff );
        return coeff;
    }

    void
    Tree::projectStartAndEndPoint ( pcl::PointXYZ& start,
                                    pcl::PointXYZ& end,
                                    pcl::ModelCoefficients::Ptr& coeff ) {

        pcl::ModelCoefficients::Ptr coefficients_line ( new pcl::ModelCoefficients );
//         coefficients_line->values.clear ();
        coefficients_line->values.push_back ( coeff->values[0] );
        coefficients_line->values.push_back ( coeff->values[1] );
        coefficients_line->values.push_back ( coeff->values[2] );
        coefficients_line->values.push_back ( coeff->values[3] );
        coefficients_line->values.push_back ( coeff->values[4] );
        coefficients_line->values.push_back ( coeff->values[5] );

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected ( new pcl::PointCloud<pcl::PointXYZ> );
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_toProject ( new pcl::PointCloud<pcl::PointXYZ> );

        cloud_toProject->push_back ( start );
        cloud_toProject->push_back ( end );

        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType ( pcl::SACMODEL_LINE );
        proj.setInputCloud ( cloud_toProject );
        proj.setModelCoefficients ( coefficients_line );
        proj.filter ( *cloud_projected );
        start = cloud_projected->points[0];
        end = cloud_projected->points[1];

    }

    void
    Tree::improveWithRANSACCoefficients ( pcl::ModelCoefficients::Ptr coeff,
                                          boost::shared_ptr<Cylinder> cylinder ) {
        pcl::PointXYZ start ( cylinder->getValues () [0], cylinder->getValues () [1], cylinder->getValues () [2] );
        pcl::PointXYZ end ( cylinder->getValues () [0] + cylinder->getValues () [3], cylinder->getValues () [1] + cylinder->getValues () [4],
                            cylinder->getValues () [2] + cylinder->getValues () [5] );
        projectStartAndEndPoint ( start, end, coeff );
        float x, y, z, x2, y2, z2, r;

        x = start.x;
        y = start.y;
        z = start.z;
        x2 = end.x - start.x;
        y2 = end.y - start.y;
        z2 = end.z - start.z;
        r = coeff->values[6];

        cylinder->values.clear ();
        cylinder->values.push_back ( x );
        cylinder->values.push_back ( y );
        cylinder->values.push_back ( z );
        cylinder->values.push_back ( x2 );
        cylinder->values.push_back ( y2 );
        cylinder->values.push_back ( z2 );
        cylinder->values.push_back ( r );
    }
    void
    Tree::mergeCylinders ( int numberIterations ) {
        int i = 0;
        while ( i < numberIterations ) {
            i++;
            std::vector<boost::shared_ptr<Segment> > allSegments = getSegments ();
            for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
                boost::shared_ptr<Segment> seg = *it;
                std::vector<boost::shared_ptr<Cylinder> > mergedCylinders;
                std::vector<boost::shared_ptr<Cylinder> > cylinders = seg->getCylinders ();
                if ( cylinders.size () > 1 && cylinders.at(0)->getLength()<0.1) {
                    for ( size_t i = 0; i < ( cylinders.size () - 1 ); i++ ) {
                        boost::shared_ptr<Cylinder> first = cylinders.at ( i );
                        i++;
                        boost::shared_ptr<Cylinder> second = cylinders.at ( i );
                        pcl::ModelCoefficients coeff;
                        coeff.values.push_back ( first->getValues () [0] );
                        coeff.values.push_back ( first->getValues () [1] );
                        coeff.values.push_back ( first->getValues () [2] );
                        coeff.values.push_back ( first->getValues () [3] + second->getValues () [3] );
                        coeff.values.push_back ( first->getValues () [4] + second->getValues () [4] );
                        coeff.values.push_back ( first->getValues () [5] + second->getValues () [5] );
                        coeff.values.push_back ( (first->getValues()[6] + second->getValues () [6])/2.0f );
                        boost::shared_ptr<Cylinder> mergedCylinder = boost::make_shared<Cylinder> ( coeff );
                        mergedCylinder->setSegment ( *it );
                        mergedCylinders.push_back ( mergedCylinder );
                    }
                    if ( cylinders.size () % 2 == 1 ) {
                        mergedCylinders.push_back ( cylinders.back () );
                    }                 
                    seg->setCylinders ( mergedCylinders );
                }
            }
        }
    }

    void
    Tree::improveWithMedianCoefficients ( pcl::PointCloud<PointI>::Ptr cylinderPoints,
                                          boost::shared_ptr<Cylinder> cylinder ) {
        std::vector<float> distances;
        int size = cylinderPoints->points.size ();
        for ( int i = 0; i < size; i++ ) {
            PointI p = cylinderPoints->points[i];
            Eigen::Vector4f point ( p.x, p.y, p.z, 0 );
            Eigen::Vector4f pointOnLine ( cylinder->values[0], cylinder->values[1], cylinder->values[2], 0 );
            Eigen::Vector4f axis ( cylinder->values[3], cylinder->values[4], cylinder->values[5], 0 );
            float dist = sqrt ( pcl::sqrPointToLineDistance ( point, pointOnLine, axis ) );
            distances.push_back ( dist );
        }
        std::nth_element ( distances.begin (), distances.begin () + ( distances.size () >> 1 ), distances.end () );
        float r = ( distances[distances.size () >> 1] );
        if ( ( r < cylinder->values[6] * 1.5 ) && ( r > cylinder->values[6] * 0.1 ) ) {
            cylinder->values[6] = r;
        }

    }

    void
    Tree::improveFit () {

        pcl::console::TicToc tt;
        tt.tic ();

        pcl::octree::OctreePointCloudSearch<PointI> octree ( octreeResolution );
        octree.setInputCloud ( cloud_Ptr );
        octree.addPointsFromInputCloud ();
        std::vector<boost::shared_ptr<Cylinder> > cylinders = getCylinders ();
        int counterNoImprovement = 0;
        int counterRANSAC = 0;
        int counterMedian = 0;
        for ( std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {

            boost::shared_ptr<Cylinder> cylinder = *it;
            pcl::PointCloud<PointI>::Ptr cylinderPoints = pointsNearCylinder ( octree, cylinder );
            if ( cylinderPoints->size () > minPtsForCylinderImprovement ) {
                pcl::ModelCoefficients::Ptr coeff = doRANSAC ( cylinderPoints );
                if ( ( coeff->values.size () == 7 ) && ( coeff->values[6] < cylinder->values[6] * 1.5 ) && ( coeff->values[6] > cylinder->values[6] * 0.5 ) ) {
                    improveWithRANSACCoefficients ( coeff, cylinder );
                    counterRANSAC++;
                } else {
                    improveWithMedianCoefficients ( cylinderPoints, cylinder );
                    counterMedian++;
                }
            } else {
                counterNoImprovement++;
            }
        }
        QString str;
        str.append ( QString::number ( tt.toc () / 1000 ) ).append ( " seconds for fit improvement.\n" );
        getControl ()->getGuiPtr ()->writeConsole ( str );
        QCoreApplication::processEvents ();
        str = "";
        str.append ( QString::number ( counterRANSAC ) ).append ( "  cylinders were improved by RANSAC," ).append ( QString::number ( counterMedian ) ).append (
            " cylinders were improved with median method, " ).append ( QString::number ( counterNoImprovement ) ).append ( "cylinders were not improved at all.\n" );
        getControl ()->getGuiPtr ()->writeConsole ( str );
        QCoreApplication::processEvents ();
    }

    void
    Tree::improveByMedianCheck () {
        std::vector<boost::shared_ptr<Segment> > allSegments = getSegments ();
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
            boost::shared_ptr<Segment> segment = *it;
            pcl::PointXYZ start = segment->getCylinders().at(0)->getStart();
                    if(segment != this->rootSegment && start.z >= 1.0)
            {
            segment->correctRadiusByMedianCheck ();
            }
        }
    }

    std::vector<boost::shared_ptr<Segment> >
    Tree::getLeaveSegments () {
        std::vector<boost::shared_ptr<Segment> > allSegments = getSegments ();
        std::vector<boost::shared_ptr<Segment> > allLeaveSegments;
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
            boost::shared_ptr<Segment> segment = *it;
            if ( segment->isLeave () ) {
                allLeaveSegments.push_back ( segment );
            }
        }
        return allLeaveSegments;
    }
    std::vector<boost::shared_ptr<Cylinder> >
    Tree::getStemCylinders () {
        std::vector<boost::shared_ptr<Cylinder> > stemCylinders;
        boost::shared_ptr<Segment> segment = getStemTopSegment ();
        while ( segment ) {
            stemCylinders.insert ( stemCylinders.end (), segment->getCylinders ().begin (), segment->getCylinders ().end () );
            segment = segment->getParent ();
        }
        return stemCylinders;
    }

    float
    Tree::getStemVolume () {
        return getVolumeToRoot ( getStemTopSegment () );
    }

    float
    Tree::getVolumeToRoot ( boost::shared_ptr<Segment> segment ) {
        float volume = 0;
        while ( segment ) {
            volume += segment->getVolume ();
            segment = segment->getParent ();
        }
        return volume;
    }

    float
    Tree::getVolumeToSegment ( boost::shared_ptr<Segment> leave,
                               boost::shared_ptr<Segment> segment ) {
        float volume = 0;
        while ( leave != segment ) {
            volume += leave->getVolume ();
            leave = leave->getParent ();
        }
        volume += segment->getVolume ();
        return volume;
    }

    float
    Tree::getVolume () {
        float volume = 0;
        std::vector<boost::shared_ptr<Segment> > allSegments = getSegments ();
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
            boost::shared_ptr<Segment> seg = *it;
            volume += seg->getVolume ();

        }
        return volume;

    }
    void
    Tree::improveBranchJunctions () {
        std::vector<boost::shared_ptr<Segment> > allSegments = getSegments ();
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
            boost::shared_ptr<Segment> segment = *it;
            if ( segment->getParent() !=0 ) {
                if(segment!=rootSegment)
                {
                boost::shared_ptr<Segment> parent_segment = segment->getParent();
                if ( parent_segment->medianRadius() <0.5*segment->medianRadius() ) {
                    for ( std::vector<boost::shared_ptr<Cylinder> >::iterator cit = segment->getCylinders().begin (); cit != segment->getCylinders().end (); cit++ ) {
                        boost::shared_ptr<Cylinder>  cylinder= *cit;
                        cylinder->values[6] = parent_segment->medianRadius();
                    }
                }
                }
            }
//            else if ( segment->getChildren().size() !=0 ) {
//                boost::shared_ptr<Segment> child_segment = segment->getChildren().at ( 0 );
//                if ( child_segment->medianRadius() <0.5*segment->medianRadius() ) {
//                    for ( std::vector<boost::shared_ptr<Cylinder> >::iterator cit = segment->getCylinders().begin (); cit != segment->getCylinders().end (); cit++ ) {
//                        boost::shared_ptr<Cylinder>  cylinder= *cit;
//                        cylinder->values[6] = child_segment->medianRadius();
//                    }
//                }
//            }

            segment->correctBranchingSection ();
        }
    }

    void
    Tree::detectStem () {
        std::vector<boost::shared_ptr<Segment> > allLeaveSegments = getLeaveSegments ();
        //float maxVolume = 0;
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allLeaveSegments.begin (); it != allLeaveSegments.end (); it++ ) {
            boost::shared_ptr<Segment> leave = *it;
            if(leave->branchOrder==0)
            {
                topStemSegment = leave;

            }
        }
        if(topStemSegment==0)
        {
            reset_stem();
        }
    }

    void
    Tree::setCylinders () {
        std::vector<Cylinder> cylinders;
        std::vector<boost::shared_ptr<Cylinder> > cylinders_ptr = getCylinders ( true );
        for ( std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders_ptr.begin (); it != cylinders_ptr.end (); it++ ) {
            boost::shared_ptr<Cylinder> cylinder_ptr = *it;
            cylinders.push_back ( *cylinder_ptr );
        }
        this->cylinders = cylinders;
    }

    void
    Tree::setCylinders ( std::vector<pcl::ModelCoefficients> coeff_cylinders ) {
        std::vector<Cylinder> cylinders;
        for ( std::vector<pcl::ModelCoefficients>::iterator it = coeff_cylinders.begin (); it != coeff_cylinders.end (); ++it ) {

            Cylinder cylinder ( *it );
            cylinders.push_back ( cylinder );
        }
        this->cylinders = cylinders;
    }

    boost::shared_ptr<Segment>
    Tree::getRootSegment () {
        return rootSegment;
    }

    boost::shared_ptr<Cylinder>
    Tree::getRootCylinder () {
        return ( rootSegment->getCylinders ().at ( 0 ) );
    }

    std::vector<boost::shared_ptr<Segment> >
    Tree::getSegments () {
        return getChildGenerationSegments ( getRootSegment () );
    }

    std::vector<boost::shared_ptr<Segment> >
    Tree::getChildGenerationSegments ( boost::shared_ptr<Segment> segment ) {
        std::vector<boost::shared_ptr<Segment> > allSegments;
        allSegments.push_back ( segment );
        std::vector<boost::shared_ptr<Segment> > children = segment->getChildren ();
        if ( children.size () > 0 ) {
            for ( std::vector<boost::shared_ptr<Segment> >::iterator it = children.begin (); it != children.end (); it++ ) {
                boost::shared_ptr<Segment> child = *it;
                std::vector<boost::shared_ptr<Segment> > childsChildren = getChildGenerationSegments ( child );
                allSegments.insert ( allSegments.end (), childsChildren.begin (), childsChildren.end () );
            }
        } else {
        }
        return allSegments;
    }

    float
    Tree::getGrowthVolume ( boost::shared_ptr<Cylinder> cylinder ) {
        float volume = 0;

        boost::shared_ptr<Segment> segment = cylinder->getSegment ();
        volume += segment->getVolume ();
        std::vector<boost::shared_ptr<Segment> > children = segment->getChildren ();
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = children.begin (); it != children.end (); it++ ) {
            boost::shared_ptr<Segment> child = *it;
            std::vector<boost::shared_ptr<Segment> > childGenerations = getChildGenerationSegments ( child );
            for ( std::vector<boost::shared_ptr<Segment> >::iterator it2 = childGenerations.begin (); it2 != childGenerations.end (); it2++ ) {
                boost::shared_ptr<Segment> child = *it2;
                volume += child->getVolume ();
            }
        }
        std::vector<boost::shared_ptr<Cylinder> > cylinders = segment->getCylinders ();
        int index = -1;
        for ( size_t i = 0; i < cylinders.size (); i++ ) {
            boost::shared_ptr<Cylinder> cylinderTemp = cylinders[i];
            if ( *cylinderTemp == *cylinder ) {
                index = i;
            }
        }
        if ( index > 0 ) {
            for ( int i = ( index - 1 ); i > -1; i-- ) {
                boost::shared_ptr<Cylinder> cylinder = cylinders[i];
                volume -= cylinder->getVolume ();
            }
        }
        return volume;
    }

    void
    Tree::closeGaps () {
        pcl::octree::OctreePointCloudSearch<PointI> octree ( octreeResolution );
        octree.setInputCloud ( cloud_Ptr );
        octree.addPointsFromInputCloud ();
        std::vector<boost::shared_ptr<Segment> > allSegments = getSegments ();
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
            boost::shared_ptr<Segment> segment = *it;

            if ( segment->getParent () != 0 ) {
                boost::shared_ptr<Cylinder> cylinder_second = segment->getCylinders ().at ( 0 ); //segment->correctBranchingSection();
                boost::shared_ptr<Segment> parent_segment = segment->getParent ();
                boost::shared_ptr<Cylinder> cylinder_first = parent_segment->getCylinders ().back ();
                float x, y, z, x2, y2, z2, r;
                x = cylinder_first->getValues () [0] + cylinder_first->getValues () [3];
                y = cylinder_first->getValues () [1] + cylinder_first->getValues () [4];
                z = cylinder_first->getValues () [2] + cylinder_first->getValues () [5];
                x2 = -x + ( cylinder_second->getValues () [0] + cylinder_second->getValues () [3] );
                y2 = -y + ( cylinder_second->getValues () [1] + cylinder_second->getValues () [4] );
                z2 = -z + ( cylinder_second->getValues () [2] + cylinder_second->getValues () [5] );
                r = cylinder_second->getValues () [6];

                cylinder_second->values.clear ();
                cylinder_second->values.push_back ( x );
                cylinder_second->values.push_back ( y );
                cylinder_second->values.push_back ( z );
                cylinder_second->values.push_back ( x2 );
                cylinder_second->values.push_back ( y2 );
                cylinder_second->values.push_back ( z2 );
                cylinder_second->values.push_back ( r );
            }

        }
    }

    void
    Tree::buildTree ( boost::shared_ptr<Segment> currentSegment,
                      boost::shared_ptr<Cylinder> currentCylinder ) {

//          std::cout << "----------------" << std::endl;

        if(this->cylinders.size()>11111111)
        {
            std::vector<Cylinder>  children = child_Cylinder_Extraction->getChildren(currentCylinder);


            if ( children.size () == 1 ) {
                Cylinder child = (children.at ( 0 ));
                currentSegment->addCylinder ( child );
                buildTree ( currentSegment, boost::make_shared<Cylinder> ( child ) );
            } else if ( children.size () > 1 ) {
                for ( std::vector<Cylinder>::iterator it = children.begin (); it != children.end (); it++ ) {
                    Cylinder child = (*it);
                    boost::shared_ptr<Segment> childSegment = boost::make_shared<Segment> ();
                    childSegment->addCylinder ( child );
                    childSegment->connectWithParent ( currentSegment );
                    buildTree ( childSegment, boost::make_shared<Cylinder> ( child ) );

                }
            }

            for(size_t i = 0; i < children.size() ; i++)
            {
                Cylinder cylinder = children.at(i);
            }
        }
        else
        {
            std::vector<Cylinder>  children = child_Cylinder_Extraction->getChildren(currentCylinder);

            if ( children.size () == 1 ) {
                Cylinder child = children.at ( 0 );
                currentSegment->addCylinder ( child );
                buildTree ( currentSegment, boost::make_shared<Cylinder> ( child ) );
            } else if ( children.size () > 1 ) {
                for ( std::vector<Cylinder>::iterator it = children.begin (); it != children.end (); it++ ) {
                    Cylinder child = *it;
                    boost::shared_ptr<Segment> childSegment = boost::make_shared<Segment> ();
                    childSegment->addCylinder ( child );
                    childSegment->connectWithParent ( currentSegment );
                    buildTree ( childSegment, boost::make_shared<Cylinder> ( child ) );
                }
            }
        }


    }

    std::vector<boost::shared_ptr<Cylinder> >
    Tree::getCylinders ( bool removeOneLengthLeaveSegments ) {
        std::vector<boost::shared_ptr<Cylinder> > cylinders;
        std::vector<boost::shared_ptr<Segment> > allSegments = getSegments ();
        for ( std::vector<boost::shared_ptr<Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
            boost::shared_ptr<Segment> seg_Ptr = *it;
            if ( removeOneLengthLeaveSegments ) {
                if ( seg_Ptr->getChildren ().size () > 0 ) {
                    cylinders.insert ( cylinders.end (), seg_Ptr->getCylinders ().begin (), seg_Ptr->getCylinders ().end () );
                } else if ( seg_Ptr->getCylinders ().size () > 1 ) {
                    cylinders.insert ( cylinders.end (), seg_Ptr->getCylinders ().begin (), seg_Ptr->getCylinders ().end () );
                }

            } else {
                cylinders.insert ( cylinders.end (), seg_Ptr->getCylinders ().begin (), seg_Ptr->getCylinders ().end () );
            }
        }
        return cylinders;
    }

} /* namespace simpleTree */
