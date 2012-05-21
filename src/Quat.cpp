#include "Quat_Adapter.h"

namespace eigen {

void Quat_Adapter::getAngleAxis( eigen::Quat& instance, double& degrees, eigen::Vec3& axis )
{
	Eigen::AngleAxisd aa = Eigen::AngleAxisd( instance );
	degrees = aa.angle() * 180 / M_PI;
	axis = aa.axis();
}

double Quat_Adapter::getW( eigen::Quat& instance )
{
	return instance.w();
}

void Quat_Adapter::setW( eigen::Quat& instance, double w )
{
	instance.w() = w;
}

double Quat_Adapter::getX( eigen::Quat& instance )
{
	return instance.x();
}

void Quat_Adapter::setX( eigen::Quat& instance, double x )
{
	instance.x() = x;
}

double Quat_Adapter::getY( eigen::Quat& instance )
{
	return instance.y();
}

void Quat_Adapter::setY( eigen::Quat& instance, double y )
{
	instance.y() = y;
}

double Quat_Adapter::getZ( eigen::Quat& instance )
{
	return instance.z();
}

void Quat_Adapter::setZ( eigen::Quat& instance, double z )
{
	instance.z() = z;
}

void Quat_Adapter::conjugate( eigen::Quat& instance )
{
	instance = instance.conjugate();
}

void Quat_Adapter::copy( eigen::Quat& instance, const eigen::Quat& q )
{
	instance = q;
}

void Quat_Adapter::cross( eigen::Quat& instance, const eigen::Quat& q )
{
	instance = instance * q;
}

double Quat_Adapter::dot( eigen::Quat& instance, const eigen::Quat& q )
{
	return instance.dot( q );
}

void Quat_Adapter::fromMat4( eigen::Quat& instance, const eigen::Mat4& m )
{
	instance = instance.Identity();
	instance = m.topLeftCorner<3, 3>();
}

void Quat_Adapter::getWXYZ( eigen::Quat& instance, double& w, double& x, double& y, double& z )
{
	w = instance.w();
	x = instance.x();
	y = instance.y();
	z = instance.z();
}

void Quat_Adapter::inverse( eigen::Quat& instance )
{
	instance = instance.inverse();
}

void Quat_Adapter::mix( eigen::Quat& instance, const eigen::Quat& q, double factor )
{
	instance = instance.slerp( factor, q );
}

void Quat_Adapter::mul( eigen::Quat& instance, const eigen::Quat& q )
{
	instance = instance * q;
}
	
void Quat_Adapter::rotate( eigen::Quat& instance, double degrees, const eigen::Vec3& axis )
{
	double radians = degrees * M_PI / 180;

	Quat rot;
	rot = Eigen::AngleAxisd( radians, axis.normalized() ); //The axis vector must be normalized.
	
	instance = instance * rot;
}

void Quat_Adapter::rotationFromTo( eigen::Quat& instance, const eigen::Vec3& from, const eigen::Vec3& to  )
{	
	instance = instance.setFromTwoVectors( from, to );
}

void Quat_Adapter::setWXYZ( eigen::Quat& instance, double w, double x, double y, double z )
{
	instance.w() = w;
	instance.x() = x;
	instance.y() = y;
	instance.z() = z;
}

} // namespace eigen
