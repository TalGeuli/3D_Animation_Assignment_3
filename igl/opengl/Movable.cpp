#include "Movable.h"
#include <iostream>
Movable::Movable()
{
	Tout = Eigen::Affine3d::Identity();
	Tin = Eigen::Affine3d::Identity();
}

Movable::Movable(const Movable& mov)
{
	Tout = mov.Tout;
	Tin = mov.Tin;
}

Eigen::Matrix4f Movable::MakeTransScale()
{
	return (Tout.matrix()*Tin.matrix()).cast<float>();
}

Eigen::Matrix4d Movable::MakeTransScaled()
{
	return (Tout.matrix() * Tin.matrix());
}

Eigen::Matrix4d Movable::MakeTransd()
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.col(3) << Tin.translation(), 1;

	return (Tout.matrix() * mat);
}

void Movable::MyTranslate(Eigen::Vector3d amt, bool preRotation)
{
	
	if(preRotation)
		Tout.pretranslate(amt);
	else
		Tout.translate(amt);
}

void Movable::TranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt)
{
	Tout.pretranslate(rot.transpose() * amt);
}


//angle in radians
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, rotAxis.normalized()));
}

void Movable::MyRotate(const Eigen::Matrix3d& rot)
{
	Tout.rotate(rot);
}

void Movable::RotateInSystem(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, Tout.rotation().transpose() * (rotAxis.normalized())));
}

void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle)
{
	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * rotAxis; //transpose instead of inverse
	MyRotate(v.normalized(), angle);
}

void Movable::MyScale(Eigen::Vector3d amt)
{
	Tin.scale(amt);
}

//---------------------------------------------------------Assignment 3-------------------------------------------------------

void Movable:: SetCenterOfRotation(Eigen::Vector3d amt)
{
	Tin.translate(-amt);
	Tout.pretranslate(amt);
}

Eigen::Vector3d Movable:: GetCenterOfRotation()
{
	return -Tin.translation();
}
void Movable::ResetRotation()
{
	Tout = Eigen::Affine3d::Identity();
}

Eigen::Vector3d Movable::GetCenter()
{
	Eigen::Vector3d amt;
	Eigen::Matrix4d temp = MakeTransd();
	amt(0) = temp(0, 3);
	amt(1) = temp(1, 3);
	amt(2) = temp(2, 3);
	return amt;
}


//---------------------------------------------------------Assignment 3-------------------------------------------------------























//void Movable::TranslateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d amt, bool preRotation)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * amt; //transpose instead of inverse
//	MyTranslate(v, preRotation);
//}
//
//void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * rotAxis; //transpose instead of inverse
//	MyRotate(v.normalized(), angle);
//}
//
//
//void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
//{
//	Tout.pretranslate(Tout.rotation().matrix().block<3, 3>(0, 0) * amt);
//	Tin.translate(-amt);
//}
//
//Eigen::Vector3d Movable::GetCenterOfRotation()
//{
//	return Tin.translation();
//}
