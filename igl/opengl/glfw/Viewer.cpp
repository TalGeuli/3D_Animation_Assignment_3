// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  void Viewer::Init(const std::string config)
  {
     
  }

  //--------------------------------------------------------Assignment 3----------------------------

  void Viewer::Create_axis ()
  {
      data().tree.m_box.center();
      Eigen::MatrixXd V_box(6, 3);
      V_box <<
          1.6, 0, 0,
          -1.6, 0, 0,
          0, 1.6, 0,
          0, -1.6, 0,
          0, 0, 1.6,
          0, 0, -1.6;

      data().add_points(V_box, Eigen::RowVector3d(1, 0, 0));


      int x = 1;
      int y = 0;
      int z = 0;
      Eigen::MatrixXi E_box(3, 2);
      E_box <<
          0, 1,
          2, 3,
          4, 5;
      for (unsigned i = 0; i < E_box.rows(); i++) {
          data().add_edges
          (
              V_box.row(E_box(i, 0)),
              V_box.row(E_box(i, 1)),
              Eigen::RowVector3d(1, 0, 0)
          );
          z = y;
          y = x;
          x = 0;
      }
      
  }

  void Viewer::Create_Linkaixs(int index)
  {
      data_list[index].tree.m_box.center();
      Eigen::MatrixXd V_box(6, 3);
      V_box <<
          0.8, 0, 0.8,
          -0.8, 0, 0.8,
          0, 0.8, 0.8,
          0, -0.8, 0.8,
          0, 0, 0.8,
          0, 0, 2.4;

      data_list[index].add_points(V_box, Eigen::RowVector3d(0, 0, 1));

      Eigen::MatrixXi E_box(3, 2);
      E_box <<
          0, 1,
          2, 3,
          4, 5;
     
      int x = 1;
      int y = 0;
      int z = 0;
      int count = 0;
      for (unsigned i = 0; i < E_box.rows(); i++) {
          data_list[index].add_edges
          (
              V_box.row(E_box(i, 0)),
              V_box.row(E_box(i, 1)),
              Eigen::RowVector3d(x, y, z)
          );
          z = y;
          y = x;
          x = 0;
          
      }
      
  }



  void Viewer::Set_Tip() 
  {
      Eigen::Vector3d O = (data_list[1].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1)).head(3);
      Eigen::Matrix3d r;
      Eigen::Vector3d l;
      start_position = O + data_list[1].GetRotation() * Eigen::Vector3d(0, 0, -0.8);
      tips[0] = start_position;
      //std::cout << "start_position is: " << start_position << "\n";
      //tips.push_back(Eigen::Vector3d(-1, -1, -1));
      for (int i = 1; i < data_list.size(); i++)
      {
          r = data_list[i].GetRotation();
          for (int j = i-1; j>0; j--)
          {
              r = data_list[j].GetRotation() * r ;
          }
          if (i == 1)
          {
              l(0) = 0;
              l(1) = 0;
              l(2) = 0.8;
          }
          else
          {
              l(0) = 0;
              l(1) = 0;
              l(2) = 1.6;
          }
          O = O + r * l;
          tips[i] = O;
      }

      tip_position = O;
      
  }

  void Viewer::CCD(Eigen::Vector3d R, Eigen::Vector3d E, Eigen::Vector3d D, int R_index)
  {
          
          Eigen::Vector3d base = tips[link_number];
          Eigen::Vector3d baseToTip = R - base;
          Eigen::Vector3d baseToDestination = R - D;
          double almost_angle = baseToTip.normalized().dot(baseToDestination.normalized());
          if (almost_angle > 1)
              almost_angle = 1;
          if (almost_angle < -1)
              almost_angle = -1;
          double angle = acos(baseToTip.normalized().dot(baseToDestination.normalized()));
          Eigen::Vector3d almostrotVec = baseToTip.cross(baseToDestination);
          Eigen::Vector4d rotVec(almostrotVec(0), almostrotVec(1), almostrotVec(2), 0);
          data_list[R_index].MyRotate(((CalcParentsTrans(R_index) * data_list[R_index].MakeTransd()).inverse() * rotVec).head(3), angle / 10);
          Set_Tip();
     
     
      
      
  }

 
  void Viewer::backword_fabrik()
  {
     
      chain[link_number] = data_list[0].GetCenter();
      for (int i = link_number-1; i >= 0; i--)
      {
          Eigen::Vector3d curr = chain[i];
          Eigen::Vector3d next = chain[i + 1];
          double ri = (curr-next).norm();
          double offset = 1.6 / ri;
          chain[i] = (1 - offset) * next + curr * offset; 
          
      }
            

  }

  void Viewer::forward_fabrik()
  {
      chain[0] = start_position;
      for (int i = 0; i <link_number; i++)
      {
          Eigen::Vector3d curr = chain[i];
          Eigen::Vector3d prev = chain[i + 1];
          double ri = (curr - prev).norm();
          double offset = 1.6 / ri; 
          chain[i+1] = (1 - offset) * curr + prev * offset; 
      }
     
      
      
  }
  
  void Viewer::moveChain()
  {
      Eigen::Vector3d currVec;
      Eigen::Vector3d destVec;
      double angle;
      double tempAngle;
      for (int i = 0; i < link_number; i++)
      {
          currVec = tips[i] - tips[i + 1];
          destVec = chain[i] - chain[i + 1];
          tempAngle = currVec.normalized().dot(destVec.normalized());
          if (tempAngle > 1)
              tempAngle = 1;
          if (tempAngle < -1)
              tempAngle = -1;
          angle = acos(tempAngle);
          Eigen::Vector3d almostrotVec = currVec.cross(destVec);
          Eigen::Vector4d rotVec(almostrotVec(0), almostrotVec(1), almostrotVec(2), 0);
          data_list[i + 1].MyRotate(((CalcParentsTrans(i + 1) * data_list[i + 1].MakeTransd()).inverse() * rotVec).head(3), angle / 10);
          Set_Tip();
      }
      
  }

  void Viewer::Fix_rotarion()
  {
      Eigen::Vector3d V(0, 0, 1);
      for (int i = 1; i <= link_number; i++)
      {
          Eigen::Matrix3d rotMatrix = data_list[i].GetRotation();
          Eigen::Vector3d euler_angle = rotMatrix.eulerAngles(2, 0, 2);
          float z = euler_angle[2];
          data_list[i].MyRotate(V, -z);
          if (i<link_number)
            data_list[i+1].RotateInSystem(V, z);

      }
  }

  
  //--------------------------------------------------------Assignment 3----------------------------


  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;


    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {
	  Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

	  for (int i = indx; parents[i] >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  }

	  return prevTrans;
  }

} // end namespace
} // end namespace
}
