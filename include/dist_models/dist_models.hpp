#pragma once

#include "dist_model_k1.hpp"
#include "dist_model_k1k2.hpp"
#include "dist_model_k1k2k3.hpp"
#include "dist_model_k1k2p1p2.hpp"
#include "dist_model_k1k2k3p1p2.hpp"
#include "dist_model_radtan_full.hpp"
#include "dist_model_kb.hpp"

// #include "ceres_costs.hpp"

// #include <Eigen/Core>
// #include <Eigen/Geometry>

// #include <ceres/ceres.h>

// #include <sophus/se3.hpp>

// class DistParam
// {
// public:

//   virtual ~DistParam() 
//   {}

//   virtual void displayParams() const = 0;

//   virtual Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const = 0;

//   virtual ceres::CostFunction*
//   createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const = 0;

//   virtual void
//   resetParameters(const std::vector<double> _v_dist_coefs) = 0;

//   int getNumberOfParameters() const
//   {
//     return m_nb_params;
//   }

// protected:
//   DistParam(const int _nb_params)
//     : m_nb_params(_nb_params)
//   {}

//   int m_nb_params = -1;
// };


// class Rad1DistParam : public DistParam
// {
// public:

//   Rad1DistParam()
//     : DistParam(1)
//   {}

//   Rad1DistParam(const double _k1)
//     : DistParam(1)
//     , m_k1(_k1)
//   {}

//   void displayParams() const override
//   {
//     std::cout << "\n Distortion k1 : " << m_k1 << "\n\n";
//   }

//   Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const override
//   {
//     const double inv_z = 1. / _cam_pt[2];
//     const double x = _cam_pt[0] * inv_z;
//     const double y = _cam_pt[1] * inv_z;

//     const double r2 = x*x + y*y;

//     const double D = 1. + r2*m_k1;
//     return Eigen::Vector2d(x*D, y*D);
//   }

//   ceres::CostFunction*
//   createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const override
//   {
//     return  new ceres::AutoDiffCostFunction<
//                 AutoDiffRad1Calib_Kernel, 2, 2, 2, 1, 7>(
//                     new AutoDiffRad1Calib_Kernel( _u, _v, _wpt));
//   }
  
//   void
//   resetParameters(const std::vector<double> _v_dist_coefs) override
//   {
//     assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
//     m_k1 = _v_dist_coefs[0];
//   }

// private:
//   double m_k1 = 0.;
// };


// class Rad2DistParam : public DistParam
// {
// public:

//   Rad2DistParam()
//     : DistParam(2)
//   {}

//   Rad2DistParam(const double _k1, const double _k2)
//     : DistParam(2)
//     , m_k1(_k1), m_k2(_k2)
//   {}

//   void displayParams() const override
//   {
//     std::cout << "\n Distortion k1 / k2 : " << m_k1 << " / " << m_k2 << "\n\n";
//   }

//   Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const override
//   {
//     const double inv_z = 1. / _cam_pt[2];
//     const double x = _cam_pt[0] * inv_z;
//     const double y = _cam_pt[1] * inv_z;

//     const double r2 = x*x + y*y;

//     const double D = 1. + r2*(m_k1 + m_k2*r2);
//     return Eigen::Vector2d(x*D, y*D);
//   }

//   ceres::CostFunction*
//   createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const override
//   {
//     return  new ceres::AutoDiffCostFunction<
//                 AutoDiffRad2Calib_Kernel, 2, 2, 2, 2, 7>(
//                     new AutoDiffRad2Calib_Kernel( _u, _v, _wpt));
//   }
  
//   void
//   resetParameters(const std::vector<double> _v_dist_coefs) override
//   {
//     assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
//     m_k1 = _v_dist_coefs[0];
//     m_k2 = _v_dist_coefs[1];
//   }


// private:
//   double m_k1=0., m_k2=0.;
// };


// class Rad3DistParam : public DistParam
// {
// public:

//   Rad3DistParam()
//     : DistParam(3)
//   {}

//   Rad3DistParam(const double _k1, const double _k2, const double _k3)
//     : DistParam(3)
//     , m_k1(_k1), m_k2(_k2), m_k3(_k3)
//   {}

//   void displayParams() const override
//   {
//     std::cout << "\n Distortion k1 / k2 / k3 : " << m_k1 << " / " << m_k2 << " / " << m_k3 << "\n\n";
//   }

//   Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const override
//   {
//     const double inv_z = 1. / _cam_pt[2];
//     const double x = _cam_pt[0] * inv_z;
//     const double y = _cam_pt[1] * inv_z;

//     const double r2 = x*x + y*y;

//     const double D = 1. + r2*(m_k1 + m_k2*r2 + m_k3*r2*r2);
//     return Eigen::Vector2d(x*D, y*D);
//   }

//   ceres::CostFunction*
//   createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const override
//   {
//     return  new ceres::AutoDiffCostFunction<
//                 AutoDiffRad3Calib_Kernel, 2, 2, 2, 3, 7>(
//                     new AutoDiffRad3Calib_Kernel( _u, _v, _wpt));
//   }

//   void
//   resetParameters(const std::vector<double> _v_dist_coefs) override
//   {
//     assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
//     m_k1 = _v_dist_coefs[0];
//     m_k2 = _v_dist_coefs[1];
//     m_k3 = _v_dist_coefs[2];
//   }

// private:
//   double m_k1=0., m_k2=0., m_k3=0.;
// };


// class RadTan4DistParam : public DistParam
// {
// public:

//   RadTan4DistParam()
//     : DistParam(4)
//   {}

//   RadTan4DistParam(const double _k1, const double _k2,
//                    const double _p1, const double _p2)
//     : DistParam(4)
//     , m_k1(_k1), m_k2(_k2), m_p1(_p1), m_p2(_p2)
//   {}

//   void displayParams() const override
//   {
//     std::cout << "\n Distortion k1 / k2 : " << m_k1 << " / " << m_k2;
//     std::cout << "\n Distortion p1 / p2 : " << m_p1 << " / " << m_p2 << "\n\n";
//   }

//   Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const override
//   {
//     const double inv_z = 1. / _cam_pt[2];
//     const double x = _cam_pt[0] * inv_z;
//     const double y = _cam_pt[1] * inv_z;

//     const double x2 = x*x;
//     const double y2 = y*y;

//     const double xy_2 = 2.*x*y;
    
//     const double r2 = x2 + y2;

//     const double D = 1. + r2*(m_k1 + m_k2*r2);

//     const double xd = x*D + m_p1*xy_2 + m_p2*(r2 +  2.*x2);
//     const double yd = y*D + m_p2*xy_2 + m_p1*(r2 +  2.*y2);

//     return Eigen::Vector2d(xd, yd);
//   }

//   ceres::CostFunction*
//   createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const override
//   {
//     return  new ceres::AutoDiffCostFunction<
//                 AutoDiffRadTan4Calib_Kernel, 2, 2, 2, 4, 7>(
//                     new AutoDiffRadTan4Calib_Kernel( _u, _v, _wpt));
//   }

//   void
//   resetParameters(const std::vector<double> _v_dist_coefs) override
//   {
//     assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
//     m_k1 = _v_dist_coefs[0];
//     m_k2 = _v_dist_coefs[1];
//     m_p1 = _v_dist_coefs[2];
//     m_p2 = _v_dist_coefs[3];
//   }

// private:
//   double m_k1=0., m_k2=0.;
//   double m_p1=0., m_p2=0.;
// };


// class RadTan5DistParam : public DistParam
// {
// public:

//   RadTan5DistParam()
//     : DistParam(5)
//   {}

//   RadTan5DistParam(const double _k1, const double _k2, const double _k3,
//                    const double _p1, const double _p2)
//     : DistParam(5)
//     , m_k1(_k1), m_k2(_k2), m_k3(_k3)
//     , m_p1(_p1), m_p2(_p2)
//   {}

//   void displayParams() const override
//   {
//     std::cout << "\n Distortion k1 / k2 / k3 : " << m_k1 << " / " << m_k2 << " / " << m_k3;
//     std::cout << "\n Distortion p1 / p2 : " << m_p1 << " / " << m_p2 << "\n\n";
//   }

//   Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const override
//   {
//     const double inv_z = 1. / _cam_pt[2];
//     const double x = _cam_pt[0] * inv_z;
//     const double y = _cam_pt[1] * inv_z;

//     const double x2 = x*x;
//     const double y2 = y*y;

//     const double xy_2 = 2.*x*y;
    
//     const double r2 = x2 + y2;

//     const double D = (1. + r2*(m_k1 + m_k2*r2 + m_k3*r2*r2));

//     const double xd = x*D + m_p1*xy_2 + m_p2*(r2 +  2.*x2);
//     const double yd = y*D + m_p2*xy_2 + m_p1*(r2 +  2.*y2);

//     return Eigen::Vector2d(xd, yd);
//   }

//   ceres::CostFunction*
//   createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const override
//   {
//     return  new ceres::AutoDiffCostFunction<
//                 AutoDiffRadTan5Calib_Kernel, 2, 2, 2, 5, 7>(
//                     new AutoDiffRadTan5Calib_Kernel( _u, _v, _wpt));
//   }

//   void
//   resetParameters(const std::vector<double> _v_dist_coefs) override
//   {
//     assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
//     m_k1 = _v_dist_coefs[0];
//     m_k2 = _v_dist_coefs[1];
//     m_k3 = _v_dist_coefs[2];
//     m_p1 = _v_dist_coefs[3];
//     m_p2 = _v_dist_coefs[4];
//   }

// private:
//   double m_k1=0., m_k2=0., m_k3=0.;
//   double m_p1=0., m_p2=0.;
// };


// class RadTan8DistParam : public DistParam
// {
// public:

//   RadTan8DistParam()
//     : DistParam(8)
//   {}

//   RadTan8DistParam(const double _k1, const double _k2, const double _k3,
//                    const double _k4, const double _k5, const double _k6,
//                    const double _p1, const double _p2)
//     : DistParam(8)
//     , m_k1(_k1), m_k2(_k2), m_k3(_k3)
//     , m_k4(_k4), m_k5(_k5), m_k6(_k6)
//     , m_p1(_p1), m_p2(_p2)
//   {}

//   void displayParams() const override
//   {
//     std::cout << "\n Distortion k1 / k2 / k3 : " << m_k1 << " / " << m_k2 << " / " << m_k3;
//     std::cout << "\n Distortion k4 / k5 / k6 : " << m_k4 << " / " << m_k5 << " / " << m_k6;
//     std::cout << "\n Distortion p1 / p2 : " << m_p1 << " / " << m_p2 << "\n\n";
//   }

//   Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const override
//   {
//     const double inv_z = 1. / _cam_pt[2];
//     const double x = _cam_pt[0] * inv_z;
//     const double y = _cam_pt[1] * inv_z;

//     const double x2 = x*x;
//     const double y2 = y*y;

//     const double xy_2 = 2.*x*y;
    
//     const double r2 = x2 + y2;

//     const double D = (1. + r2*(m_k1 + m_k2*r2 + m_k3*r2*r2)) 
//                     / (1. + r2*(m_k4 + m_k5*r2 + m_k6*r2*r2));

//     const double xd = x*D + m_p1*xy_2 + m_p2*(r2 +  2.*x2);
//     const double yd = y*D + m_p2*xy_2 + m_p1*(r2 +  2.*y2);

//     return Eigen::Vector2d(xd, yd);
//   }

//   ceres::CostFunction*
//   createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const override
//   {
//     return  new ceres::AutoDiffCostFunction<
//                 AutoDiffRadTan8Calib_Kernel, 2, 2, 2, 8, 7>(
//                     new AutoDiffRadTan8Calib_Kernel( _u, _v, _wpt));
//   }

//   void
//   resetParameters(const std::vector<double> _v_dist_coefs) override
//   {
//     assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
//     m_k1 = _v_dist_coefs[0];
//     m_k2 = _v_dist_coefs[1];
//     m_k3 = _v_dist_coefs[2];
//     m_k4 = _v_dist_coefs[3];
//     m_k5 = _v_dist_coefs[4];
//     m_k6 = _v_dist_coefs[5];
//     m_p1 = _v_dist_coefs[6];
//     m_p2 = _v_dist_coefs[7];
//   }

// private:
//   double m_k1=0., m_k2=0., m_k3=0.;
//   double m_k4=0., m_k5=0., m_k6=0.;
//   double m_p1=0., m_p2=0.;
// };


// class KB4DistParam : public DistParam
// {
// public:

//   KB4DistParam()
//     : DistParam(4)
//   {}

//   KB4DistParam(const double _k1, const double _k2, 
//                const double _k3, const double _k4)
//     : DistParam(4)
//     , m_k1(_k1), m_k2(_k2)
//     , m_k3(_k3), m_k4(_k4)
//   {}

//   void displayParams() const override
//   {
//     std::cout << "\n Distortion k1 / k2 / k3 / k4 : " << m_k1 << " / " << m_k2 << " / " << m_k3 << " / " << m_k4 << "\n\n";
//   }

//   Eigen::Vector2d distortCamPoint(const Eigen::Vector3d& _cam_pt) const override
//   {
//     const double inv_z = 1. / _cam_pt[2];
//     const double x = _cam_pt[0] * inv_z;
//     const double y = _cam_pt[1] * inv_z;

//     const double r2 = x*x + y*y;

//     const double r = std::sqrt(r2);

//     const double theta = std::atan(r);
//     const double theta2 = theta*theta;
//     const double theta4 = theta2*theta2;
//     const double theta6 = theta4*theta2;

//     const double D = theta*(1. + theta2*(m_k1 + theta2*m_k2 + theta4*m_k3 + theta6*m_k4));

//     const double D_r = D / r;

//     return Eigen::Vector2d(x*D_r, y*D_r);
//   }

//   ceres::CostFunction*
//   createCeresCostFunction(const double _u, const double _v, const Eigen::Vector3d& _wpt) const override
//   {
//     return  new ceres::AutoDiffCostFunction<
//                 AutoDiffKB4Calib_Kernel, 2, 2, 2, 4, 7>(
//                     new AutoDiffKB4Calib_Kernel( _u, _v, _wpt));
//   }
  
//   void
//   resetParameters(const std::vector<double> _v_dist_coefs) override
//   {
//     assert(static_cast<int>(_v_dist_coefs.size()) == m_nb_params);
//     m_k1 = _v_dist_coefs[0];
//     m_k2 = _v_dist_coefs[1];
//     m_k3 = _v_dist_coefs[2];
//     m_k4 = _v_dist_coefs[3];
//   }


// private:
//   double m_k1=0., m_k2=0.;
//   double m_k3=0., m_k4=0.;
// };
