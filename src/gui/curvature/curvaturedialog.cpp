#include "curvaturedialog.h"

CurvatureDialog::CurvatureDialog(QWidget *parent) :
    QDialog(parent)
{

}

void
CurvatureDialog::setViewer(boost::shared_ptr<PCLViewer> viewer)
{
    this->viewer = viewer;
}

boost::shared_ptr<PCLViewer>
CurvatureDialog::getViewer()
{
    return viewer.lock();
}

void
CurvatureDialog::setGreen(PointD &p)
{
    p.r = 102;
    p.g = 102;
    p.b = 0;
    p.a = 255;
}

void
CurvatureDialog::setRed(PointD &p)
{
    p.r = 178;
    p.g = 10;
    p.b = 10;
    p.a = 50;
}

void
CurvatureDialog::resetViewer()
{
    getViewer()->viewer->removeAllPointClouds();
    getViewer()->viewer->removeAllShapes();

    visu_cloud.reset(new PointCloudD);
    boost::shared_ptr<PointCloudI> cloud = getViewer()->getControl()->getCloudPtr();

    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        PointD point;
        point.x = cloud->points.at(i).x;
        point.y = cloud->points.at(i).y;
        point.z = cloud->points.at(i).z;
        setGreen(point);
        visu_cloud->push_back(point);
    }
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( visu_cloud );
    getViewer()->viewer->addPointCloud<PointD> ( visu_cloud, rgba, "cloud" );
    getViewer()->viewer->addText (getViewer()->getControl()->getTreeID(), 10, 20,20, 1, 0, 0, "tree_text" );
    getViewer()->ui->qvtkWidget->update ();
}

void
CurvatureDialog::updateViewer()
{
    float min1b = dialog ->PCA1Min->value();
    float max1b = dialog ->PCA1Max->value();
    float min2b = dialog ->PCA2Min->value();
    float max2b = dialog ->PCA2Max->value();
    float min3b = dialog ->PCA3Min->value();
    float max3b = dialog ->PCA3Max->value();

    float min1 = min_e1 + (max_e1 - min_e1)/100*min1b;
    float max1 = min_e1 + (max_e1 - min_e1)/100*max1b;
    float min2 = min_e2 + (max_e2 - min_e2)/100*min2b;
    float max2 = min_e2 + (max_e2 - min_e2)/100*max2b;
    float min3 = min_e3 + (max_e3 - min_e3)/100*min3b;
    float max3 = min_e3 + (max_e3 - min_e3)/100*max3b;

    for(size_t i = 0; i < visu_cloud->points.size(); i++)
    {
        float pc1,pc2,pc3;
        pc1 = e1.at(i);
        pc2 = e2.at(i);
        pc3 = e3.at(i);

        if(pc1>=min1&&pc1<=max1&&pc2>=min2&&pc2<=max2&&pc3>=min3&&pc3<=max3)
        {
            setGreen((visu_cloud->points.at(i)));
        }
        else
        {
            setRed((visu_cloud->points.at(i)));
        }
    }
    getViewer()->viewer->updatePointCloud(visu_cloud, "cloud");
}

void
CurvatureDialog::minPC1()
{
    int max = dialog->PCA1Max->value();
    int min = dialog->PCA1Min->value();
    if(max<min)
    {
        dialog->PCA1Max->setValue(min);
    }
    updateViewer();
    getViewer()->ui->qvtkWidget->update ();
}

void
CurvatureDialog::maxPC1()
{
    int max = dialog->PCA1Max->value();
    int min = dialog->PCA1Min->value();
    if(min>max)
    {
        dialog->PCA1Min->setValue(max);
    }
    updateViewer();
    getViewer()->ui->qvtkWidget->update ();
}

void
CurvatureDialog::minPC2()
{
    int max = dialog->PCA2Max->value();
    int min = dialog->PCA2Min->value();
    if(max<min)
    {
        dialog->PCA2Max->setValue(min);
    }
    updateViewer();
    getViewer()->ui->qvtkWidget->update ();
}

void
CurvatureDialog::maxPC2()
{
    int max = dialog->PCA2Max->value();
    int min = dialog->PCA2Min->value();
    if(max<min)
    {
        dialog->PCA2Min->setValue(max);
    }
    updateViewer();
    getViewer()->ui->qvtkWidget->update ();
}

void
CurvatureDialog::minPC3()
{
    int max = dialog->PCA3Max->value();
    int min = dialog->PCA3Min->value();
    if(max<min)
    {
        dialog->PCA3Max->setValue(min);
    }
    updateViewer();
    getViewer()->ui->qvtkWidget->update ();
}



void
CurvatureDialog::maxPC3()
{
    int max = dialog->PCA3Max->value();
    int min = dialog->PCA3Min->value();
    if(max<min)
    {
        dialog->PCA1Min->setValue(max);
    }
    updateViewer();
    getViewer()->ui->qvtkWidget->update ();
}


void
CurvatureDialog::save()
{
    float min1b = dialog ->PCA1Min->value();
    float max1b = dialog ->PCA1Max->value();
    float min2b = dialog ->PCA2Min->value();
    float max2b = dialog ->PCA2Max->value();
    float min3b = dialog ->PCA3Min->value();
    float max3b = dialog ->PCA3Max->value();

    float min1 = min_e1 + (max_e1 - min_e1)/100*min1b;
    float max1 = min_e1 + (max_e1 - min_e1)/100*max1b;
    float min2 = min_e2 + (max_e2 - min_e2)/100*min2b;
    float max2 = min_e2 + (max_e2 - min_e2)/100*max2b;
    float min3 = min_e3 + (max_e3 - min_e3)/100*min3b;
    float max3 = min_e3 + (max_e3 - min_e3)/100*max3b;

    boost::shared_ptr<PointCloudI> new_cloud (new PointCloudI);
    std::vector<float> e1_new;
    std::vector<float> e2_new;
    std::vector<float> e3_new;

    for(size_t i = 0; i < visu_cloud->points.size(); i++)
    {
        float pc1,pc2,pc3;
        pc1 = e1.at(i);
        pc2 = e2.at(i);
        pc3 = e3.at(i);

        if(pc1>=min1&&pc1<=max1&&pc2>=min2&&pc2<=max2&&pc3>=min3&&pc3<=max3)
        {
            new_cloud->push_back(getViewer()->getControl()->getCloudPtr()->points.at(i));
            e1_new.push_back(e1.at(i));
            e2_new.push_back(e2.at(i));
            e3_new.push_back(e3.at(i));

        }
    }
    float size_before = getViewer()->getControl()->getCloudPtr()->points.size();
    float size_after  = new_cloud->points.size();
    float perc = 0;
    if(size_before!=0)
    {
        perc = size_after/size_before*100;
    }
    getViewer()->writeConsole(QString("\n"));
    getViewer()->writeLine();
    QString str;
    str.append(QString("By curvature thresholds clouod size was reduced to ")).append(QString::number(perc)).append(QString(" percent.\n"));
    str.append(QString("The new cloud has ")).append(QString::number(new_cloud->points.size())).append(QString(" points.\n"));
    getViewer()->writeConsole(str);
    getViewer()->getControl()->setCloudPtr(new_cloud);
    getViewer()->getControl()->setE1(e1_new);
    getViewer()->getControl()->setE2(e2_new);
    getViewer()->getControl()->setE3(e3_new);
    getViewer()->writeLine();

    this->accept();
}

void
CurvatureDialog::abort()
{
    getViewer()->getControl()->setCloudPtr(getViewer()->getControl()->getCloudPtr());
    getViewer()->getControl()->setE1(e1);
    getViewer()->getControl()->setE2(e2);
    getViewer()->getControl()->setE3(e3);


    this->accept();
}





void
CurvatureDialog::init()
{
    if(getViewer()->getControl()->getCloudPtr()!=0)
    {
        if(getViewer()->getControl()->getE1().size()!=getViewer()->getControl()->getCloudPtr()->points.size())
        {
            std::vector<bool> isStem;
            e1.clear();
            e2.clear();
            e3.clear();
            EigenValueEstimator es ( getViewer()->getControl ()->getCloudPtr (), e1, e2, e3, isStem, 0.035f );
            getViewer()->getControl ()->setE1 ( e1 );
            getViewer()->getControl ()->setE2 ( e2 );
            getViewer()->getControl ()->setE3 ( e3 );
        }
        e1 = getViewer()->getControl()->getE1();
        e2 = getViewer()->getControl()->getE2();
        e3 = getViewer()->getControl()->getE3();
        min_e1 = *(std::min_element(e1.begin(),e1.end()));
        max_e1 = *(std::max_element(e1.begin(),e1.end()));
        min_e2 = *(std::min_element(e2.begin(),e2.end()));
        max_e2 = *(std::max_element(e2.begin(),e2.end()));
        min_e3 = *(std::min_element(e3.begin(),e3.end()));
        max_e3 = *(std::max_element(e3.begin(),e3.end()));
        resetViewer();
        dialog.reset ( new Ui_Dialog_Eigen );
        dialog->setupUi ( this );
        connect( dialog->PCA1Min, SIGNAL(sliderMoved(int)),this,SLOT(minPC1()));
        connect( dialog->PCA1Max, SIGNAL(sliderMoved(int)),this,SLOT(minPC1()));
        connect( dialog->PCA2Min, SIGNAL(sliderMoved(int)),this,SLOT(minPC2()));
        connect( dialog->PCA2Max, SIGNAL(sliderMoved(int)),this,SLOT(maxPC2()));
        connect( dialog->PCA3Min, SIGNAL(sliderMoved(int)),this,SLOT(minPC3()));
        connect( dialog->PCA3Max, SIGNAL(sliderMoved(int)),this,SLOT(maxPC3()));
        connect( this, SIGNAL(rejected()),this,SLOT(abort()));
        connect( dialog->abort, SIGNAL(clicked()),this,SLOT(abort()));
        connect( dialog->compute, SIGNAL(clicked()),this,SLOT(save()));
        this->setModal(false);
        this->show();

//        if(getViewer()->getControl()->getE1().size()==getViewer()->getControl()->getCloudPtr()->points.size())
//        {
//            e1 = getViewer()->getControl()->getE1();
//            e2 = getViewer()->getControl()->getE2();
//            e3 = getViewer()->getControl()->getE3();
//            min_e1 = *(std::min_element(e1.begin(),e1.end()));
//            max_e1 = *(std::max_element(e1.begin(),e1.end()));
//            min_e2 = *(std::min_element(e2.begin(),e2.end()));
//            max_e2 = *(std::max_element(e2.begin(),e2.end()));
//            min_e3 = *(std::min_element(e3.begin(),e3.end()));
//            max_e3 = *(std::max_element(e3.begin(),e3.end()));
//            resetViewer();
//            dialog.reset ( new Ui_Dialog_Eigen );
//            dialog->setupUi ( this );
//            connect( dialog->PCA1Min, SIGNAL(sliderMoved(int)),this,SLOT(minPC1()));
//            connect( dialog->PCA1Max, SIGNAL(sliderMoved(int)),this,SLOT(minPC1()));
//            connect( dialog->PCA2Min, SIGNAL(sliderMoved(int)),this,SLOT(minPC2()));
//            connect( dialog->PCA2Max, SIGNAL(sliderMoved(int)),this,SLOT(maxPC2()));
//            connect( dialog->PCA3Min, SIGNAL(sliderMoved(int)),this,SLOT(minPC3()));
//            connect( dialog->PCA3Max, SIGNAL(sliderMoved(int)),this,SLOT(maxPC3()));
//            connect( this, SIGNAL(rejected()),this,SLOT(abort()));
//            connect( dialog->abort, SIGNAL(clicked()),this,SLOT(abort()));
//            connect( dialog->compute, SIGNAL(clicked()),this,SLOT(save()));
//            this->setModal(false);
//            this->show();
//        }
//        else
//        {
//            QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Curvature information found\n"
//                                                                    "Please compute PCA first" ),
//                                   QMessageBox::Ok );
//        }


    }
    else
    {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                                                "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }

}
