/*
 * Copyright (c) 2015, Jan Hackenberg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Jan Hackenberg ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Jan Hackenberg BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "writecsv.h"

WriteCSV::WriteCSV ( boost::shared_ptr<simpleTree::Tree> tree,
                     std::string fileName ) {
    this->fileName = fileName;
    this->tree = tree;
    exportSegments ();
    exportCylinders ();
    exportStem ();
    exportStats ();
    exportDistances ();
    exportGrowthVolume ();
    exportAll();
}

WriteCSV::~WriteCSV () {

}

void
WriteCSV::exportDistances () {
    std::vector<float> distances = this->tree->distancesToModel ();
    std::ofstream myfile;

    std::ostringstream stream;
    stream << "../output/" << this->fileName << "_dist.csv";

    std::string str = stream.str ();

    char *a = new char[str.size () + 1];
    a[str.size ()] = 0;
    memcpy ( a, str.c_str (), str.size () );

    myfile.open ( a );
    for ( size_t i = 0; i < distances.size (); i++ ) {
        myfile << distances[i] * 1000 << std::endl;
    }
    myfile.close ();
}

void WriteCSV::exportAll() {
    std::ofstream myfile;

    std::ostringstream stream;
    stream << "../output/" << this->fileName << "_detailed.csv";

    std::string str = stream.str ();

    char *a = new char[str.size () + 1];
    a[str.size ()] = 0;
    memcpy ( a, str.c_str (), str.size () );

    myfile.open ( a );
    myfile << "fileName , startX, startY, startZ, endX, endY, endZ, volume, radius, length, GrowthVolume, SegmentID, ParentSegmentID, BranchID, BranchOrder \n";
    std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = this->tree->getAllCylinders ();
    for ( std::vector<boost::shared_ptr<simpleTree::Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
        boost::shared_ptr<simpleTree::Cylinder> cylinder = *it;
        if ( cylinder->getSegment()->getParent() !=0 ) {
            myfile << fileName << " , " << cylinder->toString () << " , " << this->tree->getGrowthVolume ( cylinder )  << " , " << cylinder->getSegment()->segmentID
            << " , " << cylinder->getSegment()->getParent()->segmentID<< " , " << cylinder->getSegment()->branchID<< " , " << cylinder->getSegment()->branchOrder << "\n";
        } else {
            myfile << fileName << " , " << cylinder->toString () << " , " << this->tree->getGrowthVolume ( cylinder )  << " , " << cylinder->getSegment()->segmentID
            << " , " << -1 << " , " << cylinder->getSegment()->branchID<< " , " << cylinder->getSegment()->branchOrder << "\n";
        }
        
    }
    myfile.close ();
}


    void
    WriteCSV::exportCylinders () {
        std::ofstream myfile;

        std::ostringstream stream;
        stream << "../output/" << this->fileName << "_allCylinders.csv";

        std::string str = stream.str ();

        char *a = new char[str.size () + 1];
        a[str.size ()] = 0;
        memcpy ( a, str.c_str (), str.size () );

        myfile.open ( a );
        myfile << "name , startX, startY, startZ, endX, endY, endZ, volume, radius, length" << std::endl;
        std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = this->tree->getAllCylinders ();
        for ( std::vector<boost::shared_ptr<simpleTree::Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
            boost::shared_ptr<simpleTree::Cylinder> cylinder = *it;
            myfile << this->fileName << " ,  " << cylinder->toString () <<"\n";
        }
        myfile.close ();
    }

    void
    WriteCSV::exportStats () {
        std::ofstream myfile;

        std::ostringstream stream;
        stream << "../output/" << this->fileName << "_stats.csv";

        std::string str = stream.str ();

        char *a = new char[str.size () + 1];
        a[str.size ()] = 0;
        memcpy ( a, str.c_str (), str.size () );

        myfile.open ( a );
        myfile
        << "id, volume, solidVolume, height, length, BHD, BaseDiameter, volUntilBranch, volUntilCrown, crownHeight, crownVolume, crownArea, crownProjectionArea"
        << std::endl;
//         std::vector<boost::shared_ptr<Cylinder> > cylinders = getStemCylinders();
        myfile << fileName << " , " << this->tree->getVolume () << " , " << this->tree->getSolidVolume () << " , " << this->tree->getHeight () << " , "
        << this->tree->getLength () << " , " << this->tree->getBHD () << " , " << this->tree->getBaseDiameter () << " , " << this->tree->getRootSegmentVolume ()
        << " , " << this->tree->getVolumeToRoot ( this->tree->getFirstCrownSegment () ) << " , " << this->tree->getFirstCrownSegment ()->getEnd ().z << " , "
        << this->tree->crown->volume << " , " << this->tree->crown->area << " , " << this->tree->crown->crownProjectionArea << std::endl;
        myfile.close ();
    }

    void
    WriteCSV::exportSegments () {
        std::ofstream myfile;

        std::ostringstream stream;
        stream << "../output/" << this->fileName << "_segments.csv";

        std::string str = stream.str ();

        char *a = new char[str.size () + 1];
        a[str.size ()] = 0;
        memcpy ( a, str.c_str (), str.size () );

        myfile.open ( a );
        myfile << "startX, startY, startZ, endX, endY, endZ, volume, radius, length" << std::endl;

        std::vector<boost::shared_ptr<simpleTree::Segment> > allSegments = this->tree->getSegments ();
        for ( std::vector<boost::shared_ptr<simpleTree::Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
            boost::shared_ptr<simpleTree::Segment> segment = *it;
            myfile << segment->toString ();
        }
        myfile.close ();
    }

    void
    WriteCSV::exportGrowthVolume () {
        std::ofstream myfile;

        std::ostringstream stream;
        stream << "../output/" << this->fileName << "_GrowthVolume.csv";

        std::string str = stream.str ();

        char *a = new char[str.size () + 1];
        a[str.size ()] = 0;
        memcpy ( a, str.c_str (), str.size () );

        myfile.open ( a );
        myfile << "id , radius, volume" << std::endl;
        std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = this->tree->getAllCylinders ();
        for ( std::vector<boost::shared_ptr<simpleTree::Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
            boost::shared_ptr<simpleTree::Cylinder> cylinder = *it;
            myfile << fileName << " , " << cylinder->getRadius () << " , " << this->tree->getGrowthVolume ( cylinder ) << std::endl;
        }
        myfile.close ();
    }

    void
    WriteCSV::exportStem () {
        std::ofstream myfile;

        std::ostringstream stream;
        stream << "../output/" << this->fileName << "_stemCylinders.csv";

        std::string str = stream.str ();

        char *a = new char[str.size () + 1];
        a[str.size ()] = 0;
        memcpy ( a, str.c_str (), str.size () );

        myfile.open ( a );
        myfile << "id , startX, startY, startZ, endX, endY, endZ, volume, radius, length \n";
        std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = this->tree->getStemCylinders ();
        for ( std::vector<boost::shared_ptr<simpleTree::Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
            boost::shared_ptr<simpleTree::Cylinder> cylinder = *it;
            myfile << fileName << " , " << cylinder->toString ();
        }
        myfile.close ();
    }

