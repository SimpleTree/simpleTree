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
#include "allign.h"

AllignPointCloudDialog::AllignPointCloudDialog(QWidget *parent)
    : QDialog(parent)
{
    allign_point_cloud.reset(new AllignPointCloud());
}

void
AllignPointCloudDialog::setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    this->viewer = viewer;
}

void
AllignPointCloudDialog::setUi(Ui::PCLViewer * ui)
{
    this->ui = ui;
}

boost::shared_ptr<PCLViewer>
AllignPointCloudDialog::getGuiPtr()
{
    return gui_ptr.lock();
}

void
AllignPointCloudDialog::saveFile(QString name, PointCloudI::Ptr cloud)
{
    if (cloud != 0 ) {
        QString dir ("../data/");
        dir.append(name);
        QString saveName = QFileDialog::getSaveFileName(&*getGuiPtr(), tr("Save File"),dir, tr ( "Point Cloud File(*pcd)" ));
        pcl::io::savePCDFileASCII<PointI> ( saveName.toStdString (), * cloud );
    }
}

void
AllignPointCloudDialog::resetVisualization()
{
    getGuiPtr()->viewer->removeShape("tree_text1");
    getGuiPtr()->viewer->removeShape("tree_text2");
    getGuiPtr()->getControl()->setTreeID(name_source.toStdString());
    if(allign_point_cloud->getFinal()!=0)
    {
    *getGuiPtr()->getControl()->getCloudPtr() = *(allign_point_cloud->getFinal());
    }
    else
    {
    *getGuiPtr()->getControl()->getCloudPtr() = *(allign_point_cloud->getSource());
    }
    getGuiPtr()->setCloudPtr(getGuiPtr()->getControl()->getCloudPtr());
}

void
AllignPointCloudDialog::abort()
{
    resetVisualization();
    this->accept();

}

void
AllignPointCloudDialog::save()
{
    saveFile(name_source, allign_point_cloud->getFinal());
    saveFile(name_target, allign_point_cloud->getTarget());
    abort();
}

void
AllignPointCloudDialog::init()
{

    boost::shared_ptr<PointCloudI> cloud_target (new PointCloudI);
    boost::shared_ptr<PointCloudI> cloud_source (new PointCloudI);        ;

    import(cloud_source,name_source);
    import(cloud_target,name_target);

    allign_point_cloud->setInputSource(cloud_source);
    allign_point_cloud->setInputTarget(cloud_target);
    allign_point_cloud->initialAllign();

    getGuiPtr()->getControl()->setCloudPtr(cloud_target);
    getGuiPtr()->computeBoundingBox();
    visualizeClouds();
    dialog.reset ( new Ui_dialog_init_allign );
    dialog->setupUi ( this );

    connect ( dialog->rotate, SIGNAL(sliderReleased()), this, SLOT (rotate_slider()));
    connect ( dialog->rotate_2, SIGNAL(valueChanged(double)), this, SLOT (rotate_spinbox()));
    connect ( dialog->x_2, SIGNAL(valueChanged(double)), this, SLOT (x_spinbox()));
    connect ( dialog->x, SIGNAL(sliderReleased()), this, SLOT (x_slider()));
    connect ( dialog->y_2, SIGNAL(valueChanged(double)), this, SLOT (y_spinbox()));
    connect ( dialog->y, SIGNAL(sliderReleased()), this, SLOT (y_slider()));
    connect ( dialog->z_2, SIGNAL(valueChanged(double)), this, SLOT (z_spinbox()));
    connect ( dialog->z, SIGNAL(sliderReleased()), this, SLOT (z_slider()));
    connect ( dialog->pushButton, SIGNAL(clicked()), this, SLOT (ICP()));
    connect ( dialog->save_button, SIGNAL(clicked()), this, SLOT (save()));
    connect ( dialog->abort_button, SIGNAL(clicked()), this, SLOT (abort()));

    this->setModal(false);
    this->show();
}

void
AllignPointCloudDialog::setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr)
{
    this->gui_ptr = guiPtr;
}


void
AllignPointCloudDialog::import(boost::shared_ptr<PointCloudI> & cloud, QString & name)
{
    cloud.reset(new PointCloudI);
    QString path;
    selectFile(name, path);
    getGuiPtr()->getControl()->setTreeID(name.toStdString());
    ImportPCD import ( path.toStdString(), getGuiPtr()->getControl() );
    cloud = import.getCloud();
}

void
AllignPointCloudDialog::selectFile (QString & name, QString & path) {
    QFileDialog dialog ( &*getGuiPtr(), tr ( "Open File" ), "../data/", tr ( "Point Cloud File(*pcd);;ASCII - File(*.asc);;txt - File(*.txt);;All Files(*)" ) );
    dialog.setOptions ( ( QFileDialog::DontUseNativeDialog ) );
    dialog.setViewMode ( QFileDialog::Detail );
    QStringList files;
    if ( dialog.exec () )
    {
        files = dialog.selectedFiles ();
    }
    if ( files.size () > 0 ) {
        path = files.at ( 0 );
        int index = path.lastIndexOf (QDir::separator() );
        int size = path.size ();
        int position = size - index - 1;
        name = path.right ( position );
    }
}


void
AllignPointCloudDialog::visualizeClouds(bool show_final)
{
    getGuiPtr()->viewer->removeAllPointClouds ();
    getGuiPtr()->viewer->removeAllShapes ();

    if(show_final)
    {
        boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
        visu_source = getGuiPtr()->convertPointCloud(allign_point_cloud->getSource(),128,0,0);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba1 ( visu_source );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_source, rgba1, "cloud1" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1" );
        getGuiPtr()->viewer->addText(name_source.toStdString(),10,50,20,1,0.8,0.2,"tree_text1");


        boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);
        visu_target = getGuiPtr()->convertPointCloud(allign_point_cloud->getTarget(),51,102,51);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_target );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_target, rgba2, "cloud2" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2" );
        getGuiPtr()->viewer->addText( name_target.toStdString(), 10, 20,20, 0.2, 0.4, 0.2, "tree_text2" );


        boost::shared_ptr<PointCloudD> visu_final (new PointCloudD);
        visu_final = getGuiPtr()->convertPointCloud(allign_point_cloud->getFinal(),255,204,54);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba3 ( visu_final );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_final, rgba3, "cloud3" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud3" );


    } else {
        boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
        visu_source = getGuiPtr()->convertPointCloud(allign_point_cloud->getSource(),128,0,0);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba1 ( visu_source );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_source, rgba1, "cloud1" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1" );
        getGuiPtr()->viewer->addText(name_source.toStdString(),10,50,20,0.5,0,0,"tree_text1");

        boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);
        visu_target = getGuiPtr()->convertPointCloud(allign_point_cloud->getTarget(),51,102,51);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_target );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_target, rgba2, "cloud2" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2" );
        getGuiPtr()->viewer->addText( name_target.toStdString(), 10, 20,20, 0.2, 0.4, 0.2, "tree_text2" );


        getGuiPtr()->xNegView ();
    }
    getGuiPtr()->ui->qvtkWidget->update ();
}

void
AllignPointCloudDialog::ICP()
{
    allign_point_cloud->ICP();
    getGuiPtr()->writeConsole(allign_point_cloud->result_str);
    visualizeClouds(true);
}



void
AllignPointCloudDialog::rotate_spinbox()
{
    int angle = dialog->rotate_2->value();
    dialog->rotate->setValue(angle*10);
    rotate_translate();
}

void
AllignPointCloudDialog::rotate_slider()
{
    float angle = dialog->rotate->value();
    dialog->rotate_2->setValue(angle/10.0f);
    rotate_translate();
}

void
AllignPointCloudDialog::z_spinbox()
{
    int z = dialog->z_2->value()*1000;
    dialog->z->setValue(z);
    rotate_translate();
}

void
AllignPointCloudDialog::z_slider()
{
    float z = dialog->z->value();
    dialog->z_2->setValue(z/1000.0f);
    rotate_translate();
}

void
AllignPointCloudDialog::y_spinbox()
{
    int y = dialog->y_2->value()*1000;
    dialog->y->setValue(y);
    rotate_translate();
}

void
AllignPointCloudDialog::y_slider()
{
    int y = dialog->y->value();
    dialog->y_2->setValue(y/1000.0f);
    rotate_translate();
}

void
AllignPointCloudDialog::x_spinbox()
{
    int x = dialog->x_2->value()*1000;
    dialog->x->setValue(x);
    rotate_translate();
}

void
AllignPointCloudDialog::x_slider()
{
    float x = dialog->x->value();
    dialog->x_2->setValue(x/1000.0f);
    rotate_translate();
}



void
AllignPointCloudDialog::rotate_translate()
{
    int angle = dialog->rotate->value();
    int x = dialog->x->value();
    int y = dialog->y->value();
    int z = dialog->z->value();
    allign_point_cloud->setInputFinal(allign_point_cloud->transform<PointI>(allign_point_cloud->getSource(),angle,x,y,z));
    visualizeClouds(true);

    getGuiPtr()->ui->qvtkWidget->update ();
}
