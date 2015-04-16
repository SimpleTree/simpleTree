#ifndef CURVATUREDIALOG_H
#define CURVATUREDIALOG_H

#include <QDialog>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include "../../../build/ui_curvature_dialog.h"
#include "../pclviewer.h"

class CurvatureDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CurvatureDialog(QWidget *parent = 0, boost::shared_ptr<PCLViewer> ui = 0);
    void
    setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr);
    boost::shared_ptr<PCLViewer>
    getGuiPtr();
    void
    init();
private:
    boost::shared_ptr<Ui_Dialog_Eigen> dialog;
    boost::weak_ptr<PCLViewer> ui;

signals:

public slots:

};

#endif // CURVATUREDIALOG_H
