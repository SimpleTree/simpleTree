cmake_minimum_required (VERSION 2.6 FATAL_ERROR)
project (simpleTree)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
set(CMAKE_BUILD_TYPE Release)
find_package (Qt4  COMPONENTS QtCore QtGui QtWebKit REQUIRED)
find_package (VTK REQUIRED) 
find_package (PCL 1.8.0 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories (${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})



set (project_SOURCES src/pointclouds/allignpointcloud.cpp
                     src/export/exportply.cpp
                     src/export/writecsv.cpp
                     src/main.cpp
		     src/controller.cpp 
		     src/gui/pclviewer.cpp
		     src/import/importpcd.cpp 
		     src/method/SphereFollowing.cpp 
		     src/method/eigenvalueestimator.cpp
		     src/method/StemPointDetection.cpp
		     src/method/set_coefficients.cpp
		     src/Model/crown.cpp 
		     src/Model/Cylinder.cpp 
		     src/Model/Segment.cpp 
                     src/Model/Tree.cpp
                     src/gui/allign.cpp)


set (project_HEADERS src/pointclouds/allignpointcloud.h
                     src/controller.h
                     src/gui/allign.h
                     src/export/writecsv.h
                     src/export/exportply.h
		     src/gui/pclviewer.h
		     src/import/importpcd.h 
		     src/method/SphereFollowing.h
		     src/method/set_coefficients.h
		     src/method/eigenvalueestimator.h
		     src/method/StemPointDetection.h 
		     src/Model/crown.h 
		     src/Model/Cylinder.h 
		     src/Model/Segment.h 
		     src/Model/Tree.h)

set (project_FORMS   src/gui/pclviewer.ui 
					 src/gui/allign_dialog.ui
					 src/gui/method_dialog.ui
					 src/gui/radius_dialog.ui
					 src/gui/intensity_dialog.ui
					 src/gui/statistical_dialog.ui
					 src/gui/voxel_grid_dialog.ui
					 src/gui/euclidean_dialog.ui
					 src/gui/crop_sphere_dialog.ui
					 src/gui/crop_box_dialog.ui
                                         src/gui/reference_dialog.ui
                                         src/gui/crown_dialog.ui)
set (VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE (simpleTree ${project_SOURCES}
			       ${project_FORMS_HEADERS}
			       ${project_HEADERS_MOC})
			       
TARGET_LINK_LIBRARIES (simpleTree ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
