#include "Vec3_Adapter.h"

namespace eigen {

double Vec3_Adapter::getX( eigen::Vec3& instance  )
{
	return instance.x();
}

void Vec3_Adapter::setX( eigen::Vec3& instance, double x )
{
	instance.x() = x;
}

double Vec3_Adapter::getY( eigen::Vec3& instance  )
{
	return instance.y();
}

void Vec3_Adapter::setY( eigen::Vec3& instance, double y )
{
	instance.y() = y;
}

double Vec3_Adapter::getZ( eigen::Vec3& instance  )
{
	return instance.z();
}

void Vec3_Adapter::setZ( eigen::Vec3& instance, double z )
{
	instance.z() = z;
}

void Vec3_Adapter::getXYZ( eigen::Vec3& instance, double& x, double& y, double& z  )
{
	x = instance.x();
	y = instance.y();
	z = instance.z();
}

void Vec3_Adapter::setXYZ( eigen::Vec3& instance, double x, double y, double z  )
{
	instance.x() = x;
	instance.y() = y;
	instance.z() = z;
}

void Vec3_Adapter::add( eigen::Vec3& instance, const eigen::Vec3& v )
{
	instance += v;
}

void Vec3_Adapter::copy( eigen::Vec3& instance, const eigen::Vec3& v )
{
	instance = v;
}

void Vec3_Adapter::cross( eigen::Vec3& instance, const eigen::Vec3& v )
{
	instance = instance.cross( v );
}

double Vec3_Adapter::dot( eigen::Vec3& instance, const eigen::Vec3& v )
{
	return instance.dot( v );
}

bool Vec3_Adapter::equals( eigen::Vec3& instance, const eigen::Vec3& v )
{
	return instance == v;
}

double Vec3_Adapter::length( eigen::Vec3& instance )
{
	return instance.norm();
}

void Vec3_Adapter::mix( eigen::Vec3& instance, const eigen::Vec3& v, double factor )
{
	instance = instance * ( 1 - factor ) + v * factor;
}

void Vec3_Adapter::mul( eigen::Vec3& instance, double value )
{
	instance *= value;
}

void Vec3_Adapter::mulVecQuat( eigen::Vec3& instance, const eigen::Quat& q )
{	
	Quat quat = q.inverse();
	instance = quat * instance;
}

void Vec3_Adapter::mulQuatVec( eigen::Vec3& instance, const eigen::Quat& q )
{
	instance = q * instance;
}

void Vec3_Adapter::normalize( eigen::Vec3& instance )
{
	instance = instance.normalized();
}

void Vec3_Adapter::sub( eigen::Vec3& instance, const eigen::Vec3& v )
{
	instance -= v;
}

void Vec3_Adapter::transform( eigen::Vec3& instance, const eigen::Mat4& m )
{
	Eigen::Vector4d vec4 = Eigen::Vector4d( instance.x(), instance.y(), instance.z(), 1.0f );
	vec4 = m * vec4;
	instance.x() = vec4.x();
	instance.y() = vec4.y();
	instance.z() = vec4.z();
}

} // namespace eigen
