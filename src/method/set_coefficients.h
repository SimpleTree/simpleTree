/*
 * set_coefficients.h
 *
 *  Created on: Feb 7, 2015
 *      Author: hackenberg
 */

#ifndef SET_COEFFICIENTS_H_
#define SET_COEFFICIENTS_H_
#include <QSettings>
#include <QCoreApplication>
#include <QApplication>
#include <QMessageBox>
#include "../gui/pclviewer.h"
#include "method_coefficients.h"
class PCLViewer;
class SetCoefficients
{
  public:
    SetCoefficients (boost::shared_ptr<PCLViewer> viewer_ptr);
    virtual
    ~SetCoefficients ();
    void
    set_coefficients_for_method(QString method_name);
    QStringList
    get_method_names();
    void
    struct_to_settings(Method_Coefficients coeff);
    QString
    struct_to_qstring(Method_Coefficients coeff);
    boost::shared_ptr<QSettings> settings;
  private:
    QString default_method = "Prunus_avium_Breisach_Germany";
    QStringList
    method_get_parameter_names(QString method = "Prunus_avium_Breisach_Germany");

    boost::weak_ptr<PCLViewer> viewer_ptr;
    boost::shared_ptr<PCLViewer> get_viewer();

    void
    init_settings();
    Method_Coefficients
    settings_to_struct(QString method = "Prunus_avium_Breisach_Germany");
    Method_Coefficients
    default_coeff();



};

#endif /* SET_COEFFICIENTS_H_ */
