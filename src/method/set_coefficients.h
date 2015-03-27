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
