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

#include "writecsv.h"

WriteCSV::WriteCSV ( boost::shared_ptr<simpleTree::Tree> tree,
                     std::string fileName ) {
    this->fileName = fileName;
    this->tree = tree;
    exportSegments ();
    //exportCylinders ();
    exportStem ();
    exportStats ();
    exportDistances ();
    //exportGrowthVolume ();
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
        myfile << (distances[i] * 1000) << std::endl;
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
    myfile << "fileName , startX, startY, startZ, endX, endY, endZ, volume, radius, length, GrowthVolume, SegmentID, ParentSegmentID, BranchID, BranchOrder, median Radius Segment, volume to Root,  \n";
    std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = this->tree->getAllCylinders ();
    for ( std::vector<boost::shared_ptr<simpleTree::Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
        boost::shared_ptr<simpleTree::Cylinder> cylinder = *it;
        if ( cylinder->getSegment()->getParent() !=0 ) {
            QString str;
            str.append(QString::fromStdString(fileName)).append(QString(","));
            str.append(cylinder->toString()).append(QString(","));
            str.append(QString::number(this->tree->getGrowthVolume ( cylinder )).append(QString(",")));
            str.append(QString::number(cylinder->getSegment()->segmentID)).append(QString(","));
            str.append(QString::number(cylinder->getSegment()->getParent()->segmentID)).append(QString(","));
            str.append(QString::number(cylinder->getSegment()->branchID)).append(QString(","));
            str.append(QString::number(cylinder->getSegment()->branchOrder)).append(QString(","));
            str.append(QString::number(cylinder->getSegment()->medianRadius())).append(QString(","));
            str.append(QString::number(tree->getVolumeToRoot(cylinder->getSegment()))).append(QString("\n"));
            myfile << qPrintable(str);
//            << " , " << cylinder->toString ()
//            << " , " << this->tree->getGrowthVolume ( cylinder )
//            << " , " << cylinder->getSegment()->segmentID
//            << " , " << cylinder->getSegment()->getParent()->segmentID
//            << " , " << cylinder->getSegment()->branchID
//            << " , " << cylinder->getSegment()->branchOrder
//            << " , " << cylinder->getSegment()->medianRadius()
//            << " , " << tree->getVolumeToRoot(cylinder->getSegment())
//            << "\n";
        } else {
            QString str;
            str.append(QString::fromStdString(fileName)).append(QString(","));
            str.append(cylinder->toString()).append(QString(","));
            str.append(QString::number(this->tree->getGrowthVolume ( cylinder ))).append(QString(","));
            str.append(QString::number(cylinder->getSegment()->segmentID)).append(QString(","));
            str.append(QString::number(-1)).append(QString(","));
            str.append(QString::number(cylinder->getSegment()->branchID)).append(QString(","));
            str.append(QString::number(cylinder->getSegment()->branchOrder)).append(QString(","));
            str.append(QString::number(cylinder->getSegment()->medianRadius())).append(QString(","));
            str.append(QString::number(tree->getVolumeToRoot(cylinder->getSegment()))).append(QString("\n"));
            myfile << qPrintable(str);
//            myfile << fileName
//            << " , " << cylinder->toString ()
//            << " , " << this->tree->getGrowthVolume ( cylinder )
//            << " , " << cylinder->getSegment()->segmentID
//            << " , " << -1
//            << " , " << cylinder->getSegment()->branchID
//            << " , " << cylinder->getSegment()->branchOrder
//            << " , " << cylinder->getSegment()->medianRadius()
//            << " , " << tree->getVolumeToRoot(cylinder->getSegment())
//            << "\n";
        }
        
    }
    myfile.close ();
}


//    void
//    WriteCSV::exportCylinders () {
//        std::ofstream myfile;

//        std::ostringstream stream;
//        stream << "../output/" << this->fileName << "_allCylinders.csv";

//        std::string str = stream.str ();

//        char *a = new char[str.size () + 1];
//        a[str.size ()] = 0;
//        memcpy ( a, str.c_str (), str.size () );

//        myfile.open ( a );
//        myfile << "name , startX, startY, startZ, endX, endY, endZ, volume, radius, length" << std::endl;
//        std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = this->tree->getAllCylinders ();
//        for ( std::vector<boost::shared_ptr<simpleTree::Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++ ) {
//            boost::shared_ptr<simpleTree::Cylinder> cylinder = *it;
//            myfile << this->fileName << " ,  " << cylinder->toString () ;
//        }
//        myfile.close ();
//    }

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
        << "id, volume [m^3], solidVolume [m^3],stemVolume [m^3], height [m], length [m], DBH [cm], rootDiameter [cm], rootHeight [m], volUntilFirstBranch [m^3], volUntilCrown [m^3], crown base [m], crownVolume [m^3], crownArea [m^2], crownProjectionArea [m^2]"
        << std::endl;
//         std::vector<boost::shared_ptr<Cylinder> > cylinders = getStemCylinders();
        QString str2;
        str2.append(QString::fromStdString(fileName)).append(QString(","));
        str2.append(QString::number(this->tree->getVolume ())).append(QString(","));
        str2.append(QString::number(this->tree->getSolidVolume ())).append(QString(","));
        str2.append(QString::number(this->tree->getStemVolume() )).append(QString(","));
        str2.append(QString::number(this->tree->getHeight ())).append(QString(","));
        str2.append(QString::number(this->tree->getLength ())).append(QString(","));
        str2.append(QString::number(this->tree->getDBH ())).append(QString(","));
        str2.append(QString::number(this->tree->getBaseDiameter ())).append(QString(","));
        str2.append(QString::number(this->tree->getHeightAboveGround ())).append(QString(","));
        str2.append(QString::number(this->tree->getRootSegmentVolume ())).append(QString(","));
        str2.append(QString::number(this->tree->getVolumeToRoot ( this->tree->getFirstCrownSegment () ) )).append(QString(","));
        str2.append(QString::number(this->tree-> getFirstCrownSegment ()->getEnd ().z)).append(QString(","));
        str2.append(QString::number(this->tree->crown->volume )).append(QString(","));
        str2.append(QString::number(this->tree->crown->area)).append(QString(","));
        str2.append(QString::number(this->tree->crown->crownProjectionArea)).append("\n");
        myfile << str2.toStdString();
//        myfile << fileName << " , " << this->tree->getVolume () << " , " << this->tree->getSolidVolume () << " , " << this->tree->getHeight () << " , "
//        << this->tree->getLength () << " , " << this->tree->getDBH () << " , " << this->tree->getBaseDiameter () << " , " << this->tree->getHeightAboveGround()
//        << " , " << this->tree->getRootSegmentVolume ()
//        << " , " << this->tree->getVolumeToRoot ( this->tree->getFirstCrownSegment () ) << " , " << this->tree->getFirstCrownSegment ()->getEnd ().z << " , "
//        << this->tree->crown->volume << " , " << this->tree->crown->area << " , " << this->tree->crown->crownProjectionArea << std::endl;
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
        myfile << "startX, startY, startZ, endX, endY, endZ, volume, radius, length, growthVolume" << std::endl;

        std::vector<boost::shared_ptr<simpleTree::Segment> > allSegments = this->tree->getSegments ();
        for ( std::vector<boost::shared_ptr<simpleTree::Segment> >::iterator it = allSegments.begin (); it != allSegments.end (); it++ ) {
            boost::shared_ptr<simpleTree::Segment> segment = *it;
            QString str = segment->toString().append(",");
            str.append(QString::number( this->tree->getGrowthVolume( segment->getCylinders().at(0)))).append(QString("\n"));
            myfile << qPrintable( segment->toString());
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
            myfile << fileName << " , " << qPrintable(cylinder->toString ())<< "\n";
        }
        myfile.close ();
    }

