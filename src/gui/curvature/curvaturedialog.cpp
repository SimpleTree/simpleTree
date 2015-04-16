#include "curvaturedialog.h"

CurvatureDialog::CurvatureDialog(QWidget *parent, boost::shared_ptr<PCLViewer> ui) :
    QDialog(parent)
{
    setGuiPtr(ui);
}

void
CurvatureDialog::setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr)
{
    this->ui = guiPtr;
}

boost::shared_ptr<PCLViewer>
CurvatureDialog::getGuiPtr()
{
    return ui.lock();
}


void
CurvatureDialog::init()
{

}
