#ifndef CROPSPHEREDIALOG_H
#define CROPSPHEREDIALOG_H

#include <QDialog>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include "../pclviewer.h"

#include "../../../build/ui_crop_sphere_dialog.h"
#include "../../pointclouds/allignpointcloud.h"

#include "../../../src/gui/guisubclass.h"

#include <QDir>
#include <QDialog>
class CropSphereDialog : public QDialog, public GuiSubClass
{
    Q_OBJECT
public:
    /** \brief Default contrstructor
     * \param parent: The parent QT class (main UI class)
     * */
    explicit CropSphereDialog(QWidget *parent = 0);

    /** \brief connects this with the main UI class
     * \param guiPtr: the main UI class
     * */
    void
    setViewer(boost::shared_ptr<PCLViewer> guiPtr);
    
    /** \brief returns a shared pointer of the main UI class
     * */
    boost::shared_ptr<PCLViewer>
    getViewer();
    
    /** \brief connects signals and slots
     * */
    void
    init();
    
    /** \brief a shared pointer of the UI dialog class
     * */
    boost::shared_ptr<Ui_crop_sphere_dialog> dialog;
private:
    /** \brief a shared pointer of the UI dialog class
     * */
    boost::weak_ptr<PCLViewer> viewer;
    
    /** \brief crop sphere center x
     * */
    float centerX = 0;
    
    /** \brief crop sphere center y
     * */
    float centerY = 0;
    
    /** \brief crop sphere center z
     * */
    float centerZ = 0;
    
    /** \brief crop sphere radius
     * */
    float r = 0.5f;
    
    /** \brief model coefficients of the crop sphere
     * */
    pcl::ModelCoefficients deleteSphere;
    
    /** \brief adds the crop sphere to the visualizer
     * */
    void
    addSphere();






signals:




public slots:
    /** \brief sets the crop sphere center x
     * \param x: the x-coordinate
     * */
    void
    set_crop_sphere_x ( double x );
    
    /** \brief sets the crop sphere center y
     * \param y: the y-coordinate
     * */
    void
    set_crop_sphere_y ( double y ) ;
    
    /** \brief sets the crop sphere center z
     * \param z: the z-coordinate
     * */
    void
    set_crop_sphere_z ( double z ) ;
    
    /** \brief sets the crop radius
     * \param r: the radius
     * */
    void
    set_crop_sphere_r ( double r ) ;
    
    /** \brief deletes inside the crop sphere all points from the point cloud stored in controller class
     * */
    void
    compute_crop_sphere () ;
        
    /** \brief cancels the radius
     * */
    void
    abort_crop_sphere () ;

//    void
//    cropsphere () ;

};

#endif // CROPSPHEREDIALOG_H
