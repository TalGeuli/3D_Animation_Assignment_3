#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>



SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	link_number = 0;
	tips.resize(50);
	chain.resize(50);
	upRotateX = false;
	downRotateX = false;
	leftRotateY = false;
	rightRotateY = false;
	ccdMove = false;
	fabrikMove = false;

	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			Eigen::RowVector3d center(0, 0, -0.8);
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);
			if (selected_data_index != 0)
				data().SetCenterOfRotation(center.transpose());
			data().tree.init(data().V, data().F);
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	
	destination_position(0) = 5;
	destination_position(1) = 0;
	destination_position(2) = 0;

	// Intialize sphere
	data_list[0].MyTranslate(Eigen::Vector3d(5,0,0), true);
	Create_Linkaixs(0);
	
	// Intialize links
	for (int i = 1; i < data_list.size() - 1; i++)
	{
		parents[i+1] = i;
		data_list[i+1].MyTranslate(Eigen::Vector3d(0, 0, 1.6), true);
		link_number++;
		Create_Linkaixs(i);
	}
	Create_Linkaixs(data_list.size() - 1);
	link_number++;
	
	// Set tip_position
	Set_Tip();
}


SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{
		if (rightRotateY)
		{
			data().RotateInSystem(data().MakeTransd(), Eigen::Vector3d(0, 0, 1), 0.05);
			Set_Tip();
		}
		if (leftRotateY) 
		{
			data().RotateInSystem(data().MakeTransd(), Eigen::Vector3d(0, 0, 1), -0.05);
			Set_Tip();
		}
		if (upRotateX)
		{
			data().MyRotate(Eigen::Vector3d(1, 0, 0), 0.05);
			Set_Tip();
		}
		if (downRotateX)
		{
			data().MyRotate(Eigen::Vector3d(1, 0, 0), -0.05);
			Set_Tip();
		}
		if (ccdMove) {
			destination_position = data_list[0].GetCenter();
			if ((tips[link_number] - destination_position).norm() >= 0.1) {
				for (int i = link_number - 1; i > 0; i--)
				{
					Eigen::Vector3d E = tips[link_number];
					Eigen::Vector3d R = tips[i];
					Eigen::Vector3d D = destination_position;
					CCD(R, E, D, i);
					if ((tips[link_number] - destination_position).norm() < 0.1)
						break;
				}
			}
						
		}

		if (fabrikMove)
			if ((destination_position - tips[0]).norm() <= 1.6 * link_number)
			{
				for (int i = 0; i <= link_number; i++)
				{
					chain[i] = tips[i];
					//std::cout << "chain at " << i << " is: " << chain[i] << '\n';
				} 
				if ((destination_position - tips[link_number]).norm() >= 0.1)
				{
					forward_fabrik();
					backword_fabrik();
					moveChain();
				}
			}
	}
}


