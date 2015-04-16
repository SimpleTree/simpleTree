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

#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <QCleanlooksStyle>
#include <QApplication>

#include <QMainWindow>
#include <qdesktopwidget.h>

#include "Model/Tree.h"
#include "gui/pclviewer.h"
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#endif

typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;


class PCLViewer;
namespace simpleTree
{
	class Tree;

}

class Controller : public boost::enable_shared_from_this<Controller>
{
  private:
    boost::shared_ptr<simpleTree::Tree> tree_ptr;
    boost::shared_ptr<PCLViewer> gui_ptr;
    std::vector<float> e1,e2,e3;
    std::vector<bool> isStem;
    PointCloudI::Ptr cloud_ptr;
    CurvatureCloud::Ptr curvature_ptr;
    std::string treeID = "test cloud";
  public:
    Controller ();
    ~Controller ();
    void
    init (int argc,
          char *argv[]);
    boost::shared_ptr<simpleTree::Tree>
    getTreePtr ();
    void
    setTreePtr (boost::shared_ptr<simpleTree::Tree>);
    PointCloudI::Ptr
    getCloudPtr ();
    CurvatureCloud::Ptr
    getCurvaturePtr ();
    void
    setCloudPtr (PointCloudI::Ptr cloud_ptr,
                 CurvatureCloud::Ptr curvature_ptr);
    void
        setCurvaturePtr (CurvatureCloud::Ptr curvature_ptr);
    void
    setCloudPtr (PointCloudI::Ptr cloud_ptr, bool changeView = false);
    boost::shared_ptr<PCLViewer>
    getGuiPtr ();
    std::string
    getTreeID ();
    void
    setTreeID (std::string);

//   void setGuiPtr(boost::shared_ptr<PCLViewer>);

    std::vector<float>&
    getE1 ()
    {
      return e1;
    }

    void
    setE1 ( std::vector<float>& e1)
    {
      this->e1 = e1;
    }

    std::vector<float>&
    getE2 ()
    {
      return e2;
    }

    void
    setE2 ( std::vector<float>& e2)
    {
      this->e2 = e2;
    }

     std::vector<float>&
    getE3 ()
    {
      return e3;
    }

    void
    setE3 ( std::vector<float>& e3)
    {
      this->e3 = e3;
    }

	const std::vector<bool>& getIsStem() const {
		return isStem;
	}

	void setIsStem(const std::vector<bool>& isStem) {
		this->isStem = isStem;
// 		for(size_t i = 0 ; i < isStem.size(); i++)
// 		{
// 		  if(isStem.at(i)){
// 		  std::cout << isStem.at(i) << "\n";
// 		  }
// 		}
	}

	const std::string& getTreeId() const {
		return treeID;
	}

	void setTreeId(const std::string& treeId = "test cloud") {
		treeID = treeId;
	}
};

#endif // CONTROLLER_H
