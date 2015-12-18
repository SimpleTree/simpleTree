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



set (project_SOURCES gui/color_palette/color_factory.cpp
                     gui/publication/erythrophleum_fordii/ery_denoise.cpp
                     gui/publication/quercus_petraea/quercus_denoise.cpp
                     gui/publication/pinus_massoniana/pine_denoise.cpp
                     gui/publication/platanus/plane_folder.cpp
                     gui/publication/eucalyptus_spp/eucalypt_folder.cpp
                     gui/publication/erythrophleum_fordii/ery_folder.cpp
                     gui/publication/pinus_massoniana/pinus_folder.cpp
                     gui/publication/quercus_petraea/quercus_folder.cpp
                     method/optimization/workerspherefollowing.cpp
                     method/optimization/optimization.cpp
                     Model/childcylinderextraction.cpp
                     gui/cropbox/cropboxdialog.cpp
                     gui/cropsphere/cropspheredialog.cpp
                     gui/reference/referenceheightdialog.cpp
                     gui/statistics/visustatistics.cpp
                     Model/modelAdjustment/allometry.cpp
                     pointclouds/bufferpointcloud.cpp
                     gui/curvature/curvaturedialog.cpp
                     pointclouds/voxelgridfilter.cpp
                     pointclouds/allignpointcloud.cpp
                     export/exportply.cpp
                     export/writecsv.cpp
                     main.cpp
                     controller.cpp
                     gui/pclviewer.cpp
                     import/importpcd.cpp
                     method/SphereFollowing.cpp
                     method/eigenvalueestimator.cpp
                     method/StemPointDetection.cpp
                     method/set_coefficients.cpp
                     Model/crown.cpp
                     Model/Cylinder.cpp
                     Model/Segment.cpp
                     Model/Tree.cpp
                     gui/allign/allign.cpp
                     method/stem_fit.cpp
                     method/optimization/optimization_stem.cpp
                     method/optimization/workerstemfit.cpp)


set (project_HEADERS
                     gui/color_palette/color_factory.h
                     gui/color_palette/color_palette.h
                     gui/publication/erythrophleum_fordii/ery_denoise.h
                     gui/publication/quercus_petraea/quercus_denoise.h
                     gui/publication/pinus_massoniana/pine_denoise.h
                     gui/publication/platanus/plane_folder.h
                     gui/publication/eucalyptus_spp/eucalypt_folder.h
                     gui/publication/erythrophleum_fordii/ery_folder.h
                     gui/publication/pinus_massoniana/pinus_folder.h
                     includes.h
                     gui/publication/quercus_petraea/quercus_folder.h
                     method/optimization/workerspherefollowing.h
                     method/optimization/optimization.h
                     Model/childcylinderextraction.h
                     gui/cropbox/cropboxdialog.h
                     gui/cropsphere/cropspheredialog.h
                     gui/reference/referenceheightdialog.h
                     gui/guisubclass.h
                     gui/statistics/visustatistics.h
                     Model/modelAdjustment/allometry.h
                     pointclouds/bufferpointcloud.h
                     pointclouds/allignpointcloud.h
                     gui/curvature/curvaturedialog.h
                     pointclouds/voxelgridfilter.h
                     controller.h
                     gui/allign/allign.h
                     export/writecsv.h
                     export/exportply.h
                     gui/pclviewer.h
                     import/importpcd.h
                     method/SphereFollowing.h
                     method/set_coefficients.h
                     method/eigenvalueestimator.h
                     method/StemPointDetection.h
                     Model/crown.h
                     Model/Cylinder.h
                     Model/Segment.h
                     Model/Tree.h
                     method/stem_fit.h
                     method/optimization/optimization_stem.h
                     method/optimization/workerstemfit.h)

set (project_FORMS
                     gui/cropbox/crop_box_dialog.ui
                     gui/cropsphere/crop_sphere_dialog.ui
                     gui/reference/reference_dialog.ui
                     gui/curvature/curvature_dialog.ui
                     gui/pclviewer.ui
                     gui/eigen_dialog.ui
                     gui/allign/allign_dialog.ui
                     gui/method_dialog.ui
                     gui/radius_dialog.ui
                     gui/intensity_dialog.ui
                     gui/statistical_dialog.ui
                     gui/voxel_grid_dialog.ui
                     gui/euclidean_dialog.ui
                     gui/crop_sphere_dialog.ui
                     gui/crop_box_dialog.ui
                     gui/reference_dialog.ui
                     gui/crown_dialog.ui
		     gui/allometry_dialog.ui
		     gui/optimize_dialog.ui 

		     )
set (VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE (simpleTree ${project_SOURCES}
                               ${project_FORMS_HEADERS}
                               ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (simpleTree ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})
