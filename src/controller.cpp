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
#include "controller.h"

Controller::Controller ()
{

}

Controller::~Controller ()
{

}

pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr
Controller::getCurvaturePtr ()
{
    return curvature_ptr;
}

void
Controller::init (int argc,
                  char *argv[])
{

    QApplication a (argc, argv);
    QRect screenres = QApplication::desktop ()->screenGeometry (1/*screenNumber*/);

    this->gui_ptr.reset (new PCLViewer);
    this->gui_ptr->move (QPoint (screenres.x (), screenres.y ()));
    this->gui_ptr->resize (screenres.width (), screenres.height ());
    this->gui_ptr->init();



    this->gui_ptr->show ();
    this->gui_ptr->connectToController (shared_from_this ());
    a.exec ();
}

std::string
Controller::getTreeID ()
{
    return treeID;
}

void
Controller::setTreeID (std::string treeID)
{
    this->treeID = treeID;
}

void
Controller::setTreeID (QString treeID)
{
    this->treeID = treeID.toStdString();
}

PointCloudI::Ptr
Controller::getCloudPtr ()
{
    return this->cloud_ptr;
}

void
Controller::setCloudPtr (pcl::PointCloud<PointI>::Ptr cloud_ptr, bool changeView )
{
    this->cloud_ptr = cloud_ptr;
    this->cloud_ptr->width = cloud_ptr->points.size();
    this->cloud_ptr->height = 1;
    this->tree_ptr = 0;
    this->e1.clear();
    this->e2.clear();
    this->e3.clear();
    this->isStem.clear();
    this->curvature_ptr.reset(new CurvatureCloud);
    if(changeView)
    {
        this->gui_ptr->setCloudPtr (cloud_ptr, false );
    } else {
        this->gui_ptr->setCloudPtr (cloud_ptr, false );
    }
}

void
Controller::setCloudPtr (PointCloudI::Ptr cloud_ptr,
                         CurvatureCloud::Ptr curvature_ptr)
{
    this->cloud_ptr = cloud_ptr;
    this->cloud_ptr->width = cloud_ptr->points.size();
    this->cloud_ptr->height = 1;
    this->curvature_ptr = curvature_ptr;
    this->tree_ptr = 0;
    this->e1.clear();
    this->e2.clear();
    this->e3.clear();
    this->isStem.clear();
    this->gui_ptr->setCloudPtr (cloud_ptr);
}


void
Controller::setCurvaturePtr (CurvatureCloud::Ptr curvature_ptr)
{
    this->curvature_ptr = curvature_ptr;
    this->gui_ptr->setCloudPtr (cloud_ptr);
}

boost::shared_ptr<PCLViewer>
Controller::getGuiPtr ()
{
    return this->gui_ptr;
}

boost::shared_ptr<simpleTree::Tree>
Controller::getTreePtr ()
{
    return this->tree_ptr;
}

void
Controller::setTreePtr (boost::shared_ptr<simpleTree::Tree> tree_ptr)
{
    this->tree_ptr = tree_ptr;
    this->gui_ptr->setCloudPtr(cloud_ptr);
    this->gui_ptr->setTreePtr (tree_ptr);
}

void
Controller::setE1 ( std::vector<float> e1)
{
    this->e1 = e1;
}

void
Controller::setE2 ( std::vector<float> e2)
{
    this->e2 = e2;
}

void
Controller::setE3 ( std::vector<float> e3)
{
    this->e3 = e3;
}

void
Controller::setIsStem(const std::vector<bool>& isStem) {
    this->isStem = isStem;
}
