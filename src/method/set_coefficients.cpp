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
#include "set_coefficients.h"

SetCoefficients::SetCoefficients (boost::shared_ptr<PCLViewer> viewer_ptr)
{
  this->viewer_ptr = viewer_ptr;
  this->settings.reset (new QSettings (QApplication::applicationDirPath ().append ("/../config/method.ini"), QSettings::IniFormat));
  init_settings ();
  this->settings->sync ();

}

SetCoefficients::~SetCoefficients ()
{
  // TODO Auto-generated destructor stub
}

void
SetCoefficients::set_coefficients_for_method (QString method_name)
{
  if (method_name == QString (""))
  {
    method_name = default_method;
  }
  Method_Coefficients coeff = settings_to_struct (method_name);
  get_viewer ()->setMethodCoefficients (coeff);
}

void
SetCoefficients::init_settings ()
{
  settings->beginGroup ("Prunus_avium_Breisach_Germany");
  settings->setValue ("sphere_radius_multiplier", 3);
  settings->setValue ("epsilon_cluster_stem", 0.02);
  settings->setValue ("epsilon_cluster_branch", 0.008);
  settings->setValue ("epsilon_sphere", 0.02);
  settings->setValue ("minPts_ransac_stem", 200);
  settings->setValue ("minPts_ransac_branch", 99999999);
  settings->setValue ("minPts_cluster_stem", 12);
  settings->setValue ("minPts_cluster_branch", 3);
  settings->setValue ("min_radius_sphere_stem", 0.035);
  settings->setValue ("min_radius_sphere_branch", 0.025);
  settings->endGroup ();

  settings->beginGroup ("Prunus_avium_Sinnsheim_Germany");
  settings->setValue ("sphere_radius_multiplier", 1.8);
  settings->setValue ("epsilon_cluster_stem", 1.02);
  settings->setValue ("epsilon_cluster_branch", 0.008);
  settings->setValue ("epsilon_sphere", 0.02);
  settings->setValue ("minPts_ransac_stem", 200);
  settings->setValue ("minPts_ransac_branch", 99999999);
  settings->setValue ("minPts_cluster_stem", 12);
  settings->setValue ("minPts_cluster_branch", 3);
  settings->setValue ("min_radius_sphere_stem", 0.035);
  settings->setValue ("min_radius_sphere_branch", 0.025);
  settings->endGroup ();
  
  settings->beginGroup ("Erytrophleum fordii");
  settings->setValue ("sphere_radius_multiplier", 3);
  settings->setValue ("epsilon_cluster_stem", 0.03);
  settings->setValue ("epsilon_cluster_branch", 0.02);
  settings->setValue ("epsilon_sphere", 0.02);
  settings->setValue ("minPts_ransac_stem", 200);
  settings->setValue ("minPts_ransac_branch", 99999999);
  settings->setValue ("minPts_cluster_stem", 12);
  settings->setValue ("minPts_cluster_branch", 3);
  settings->setValue ("min_radius_sphere_stem", 0.07);
  settings->setValue ("min_radius_sphere_branch", 0.025);
  settings->endGroup ();
  
  settings->beginGroup ("French AgroForestry Side");
  settings->setValue ("sphere_radius_multiplier", 3);
  settings->setValue ("epsilon_cluster_stem", 0.02);
  settings->setValue ("epsilon_cluster_branch", 0.012);
  settings->setValue ("epsilon_sphere", 0.02);
  settings->setValue ("minPts_ransac_stem", 100);
  settings->setValue ("minPts_ransac_branch", 99999999);
  settings->setValue ("minPts_cluster_stem", 12);
  settings->setValue ("minPts_cluster_branch", 3);
  settings->setValue ("min_radius_sphere_stem", 0.035);
  settings->setValue ("min_radius_sphere_branch", 0.035);
  settings->endGroup ();
  
  
}

Method_Coefficients
SetCoefficients::settings_to_struct (QString method)
{
  Method_Coefficients coeff;
  if (method == QString (""))
  {
    method = default_method;
  }
  QStringList methods = settings->childGroups ();

  if (methods.contains (method))
  {
    QStringList parameter_names = method_get_parameter_names (method);
    settings->beginGroup (method);

    coeff.name = method;
    bool is_well_formed = true;
    if (parameter_names.contains ("sphere_radius_multiplier"))
    {
      coeff.sphere_radius_multiplier = settings->value ("sphere_radius_multiplier").toFloat ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("epsilon_cluster_stem"))
    {
      coeff.epsilon_cluster_stem = settings->value ("epsilon_cluster_stem").toFloat ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("epsilon_cluster_branch"))
    {
      coeff.epsilon_cluster_branch = settings->value ("epsilon_cluster_branch").toFloat ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("epsilon_sphere"))
    {
      coeff.epsilon_sphere = settings->value ("epsilon_sphere").toFloat ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("minPts_ransac_stem"))
    {
      coeff.minPts_ransac_stem = settings->value ("minPts_ransac_stem").toInt ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("minPts_ransac_branch"))
    {
      coeff.minPts_ransac_branch = settings->value ("minPts_ransac_branch").toInt ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("minPts_cluster_stem"))
    {
      coeff.minPts_cluster_stem = settings->value ("minPts_cluster_stem").toInt ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("minPts_cluster_branch"))
    {
      coeff.minPts_cluster_branch = settings->value ("minPts_cluster_branch").toInt ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("min_radius_sphere_stem"))
    {
      coeff.min_radius_sphere_stem = settings->value ("min_radius_sphere_stem").toFloat ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("min_radius_sphere_branch"))
    {
      coeff.min_radius_sphere_branch = settings->value ("min_radius_sphere_branch").toFloat ();
    }
    else
    {
      is_well_formed = false;
    }

    if (parameter_names.contains ("sphere_radius_multiplier"))
    {
      coeff.sphere_radius_multiplier = settings->value ("sphere_radius_multiplier").toFloat ();
    }
    else
    {
      is_well_formed = false;
    }
    settings->endGroup ();
    if (is_well_formed)
    {
      return coeff;
    }
    else
    {
      QMessageBox::warning (&*get_viewer (), ("Simple Tree"), ("Coefficients are not wellformed. Please sent a short bug report."), QMessageBox::Ok);
    }

  }
  coeff = default_coeff ();
  return coeff;
}

boost::shared_ptr<PCLViewer>
SetCoefficients::get_viewer ()
{
  return this->viewer_ptr.lock ();
}

QStringList
SetCoefficients::method_get_parameter_names (QString method)
{
  if (method == QString (""))
  {
    method = QString (default_method);
  }
  QStringList methods = settings->childGroups ();
  QStringList parameter_names;
  if (methods.contains (method))
  {
    settings->beginGroup (method);
    parameter_names = settings->childKeys ();
    settings->endGroup ();
  }
//  for(int i = 0 ; i < parameter_names.size(); i++)
//  {
//    QString key = parameter_names.at(i);
//  }
  return parameter_names;
}

Method_Coefficients
SetCoefficients::default_coeff ()
{
  Method_Coefficients coeff;
  coeff.name = "Prunus_avium_Breisach_Germany";
  coeff.sphere_radius_multiplier = 1.8;
  coeff.epsilon_cluster_stem = 0.02;
  coeff.epsilon_cluster_branch = 0.008;
  coeff.epsilon_sphere = 0.02;
  coeff.minPts_ransac_stem = 200;
  coeff.minPts_ransac_branch = 99999999;
  coeff.minPts_cluster_stem = 12;
  coeff.minPts_cluster_branch = 3;
  coeff.min_radius_sphere_stem = 0.035;
  coeff.min_radius_sphere_branch = 0.025;
  return coeff;
}

QString
SetCoefficients::struct_to_qstring (Method_Coefficients coeff)
{
  QString str;
  str.append ("Name                     : ").append (QString (coeff.name)).append ("\n").append ("sphere_radius_multiplier : ").append (
      QString::number (coeff.sphere_radius_multiplier)).append ("\n").append ("epsilon_cluster_stem     : ").append (
      QString::number (coeff.epsilon_cluster_stem)).append ("\n").append ("epsilon_cluster_branch   : ").append (QString::number (coeff.epsilon_cluster_branch)).append (
      "\n").append ("epsilon_sphere           : ").append (QString::number (coeff.epsilon_sphere)).append ("\n").append ("minPts_ransac_stem       : ").append (
      QString::number (coeff.minPts_ransac_stem)).append ("\n").append ("minPts_ransac_branch     : ").append (QString::number (coeff.minPts_ransac_branch)).append (
      "\n").append ("minPts_cluster_stem      : ").append (QString::number (coeff.minPts_cluster_stem)).append ("\n").append ("minPts_cluster_branch    : ").append (
      QString::number (coeff.minPts_cluster_branch)).append ("\n").append ("min_radius_sphere_stem   : ").append (
      QString::number (coeff.min_radius_sphere_stem)).append ("\n").append ("min_radius_sphere_branch : ").append (
      QString::number (coeff.min_radius_sphere_branch)).append ("\n");
  return str;
}

QStringList
SetCoefficients::get_method_names ()
{
  return settings->childGroups ();
}

void
SetCoefficients::struct_to_settings (Method_Coefficients coeff)
{
  settings->beginGroup (coeff.name);
  settings->setValue ("sphere_radius_multiplier", coeff.sphere_radius_multiplier);
  settings->setValue ("epsilon_cluster_stem", coeff.epsilon_cluster_stem);
  settings->setValue ("epsilon_cluster_branch", coeff.epsilon_cluster_branch);
  settings->setValue ("epsilon_sphere", coeff.epsilon_sphere);
  settings->setValue ("minPts_ransac_stem", coeff.minPts_ransac_stem);
  settings->setValue ("minPts_ransac_branch", coeff.minPts_ransac_branch);
  settings->setValue ("minPts_cluster_stem", coeff.minPts_cluster_stem);
  settings->setValue ("minPts_cluster_branch", coeff.minPts_cluster_branch);
  settings->setValue ("min_radius_sphere_stem", coeff.min_radius_sphere_stem);
  settings->setValue ("min_radius_sphere_branch", coeff.min_radius_sphere_branch);
  settings->endGroup ();
  this->settings->sync ();
}
