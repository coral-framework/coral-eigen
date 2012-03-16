#include "Mat4_Adapter.h"

namespace eigen {

static const double PI = 3.14159265358979323846;

void Mat4_Adapter::add( eigen::Mat4& instance, const eigen::Mat4& m )
{
	instance += m;
}	

void Mat4_Adapter::copy( eigen::Mat4& instance, const eigen::Mat4& m )
{
	instance = m;
}

void Mat4_Adapter::fromQuat( eigen::Mat4& instance, const eigen::Quat& q )
{
	instance = instance.Identity();
	instance.topLeftCorner<3, 3>() = q.toRotationMatrix();
}

void Mat4_Adapter::frustum( eigen::Mat4& instance, double left, double right, double bottom, double up, double nearVal, double farVal )
{
	instance = instance.Identity();

	instance( 0, 0 ) = (2 * nearVal) / (right - left);
	instance( 1, 1 ) = (2 * nearVal) / (up - bottom);
	instance( 0, 2 ) = (right + left) / (right - left);
	instance( 1, 2 ) = (up + bottom) / (up - bottom);
	instance( 2, 2 ) = -(farVal + nearVal) / (farVal - nearVal);
	instance( 3, 2 ) = -1;
	instance( 2, 3 ) = -(2 * farVal * nearVal) / (farVal - nearVal);
}

double Mat4_Adapter::getElement( eigen::Mat4& instance, co::int32 i, co::int32 j )
{
	return instance.coeff( i, j );
}

void Mat4_Adapter::identity( eigen::Mat4& instance )
{
	instance = Mat4::Identity();
}

void Mat4_Adapter::invert( eigen::Mat4& instance )
{
	instance = instance.inverse().eval();
}

void Mat4_Adapter::lookAt( eigen::Mat4& instance, const eigen::Vec3& eye, const eigen::Vec3& center, const eigen::Vec3& up )
{
	Vec3 forward = center - eye;
	forward = forward.normalized();
        
	Vec3 newUp = up.normalized();
	
	Vec3 side = forward.cross( newUp );
	side = side.normalized();

	newUp = side.cross( forward );

	instance = instance.Identity();

	instance( 0, 0 ) = side.x();
	instance( 0, 1 ) = side.y();
	instance( 0, 2 ) = side.z();
	instance( 1, 0 ) = newUp.x();
	instance( 1, 1 ) = newUp.y();
	instance( 1, 2 ) = newUp.z();
	instance( 2, 0 ) = -forward.x();	
	instance( 2, 1 ) = -forward.y(); 	
	instance( 2, 2 ) = -forward.z(); 

	Eigen::Transform<double, 3, Eigen::Affine> t;
    t = Eigen::Translation<double, 3>( -eye );

	instance = t * instance;
}

void Mat4_Adapter::mulScalar( eigen::Mat4& instance, double value )
{
	instance *= value;
}

void Mat4_Adapter::ortho( eigen::Mat4& instance, double left, double right, double bottom, double up )
{
	instance = instance.Identity();

	instance( 0, 0 ) = 2 / (right - left);
	instance( 1, 1 ) = 2 / (up - bottom);
	instance( 2, 2 ) = -1;

	instance( 0, 3 ) = - (right + left) / (right - left);
	instance( 1, 3 ) = - (up + bottom) / (up - bottom);	
}

void Mat4_Adapter::perspective( eigen::Mat4& instance, double fovy, double aspect, double zNear, double zFar )
{	
	double radians = (fovy / 2) * PI / 180;

	double range = std::tan(radians) * zNear;	
	double left = -range * aspect;
	double right = range * aspect;
	double bottom = -range;
	double top = range;

	instance = instance.Identity();

	instance( 0, 0 ) = (2 * zNear) / (right - left);
	instance( 1, 1 ) = (2 * zNear) / (top - bottom);
	instance( 2, 2 ) = - (zFar + zNear) / (zFar - zNear);
	instance( 3, 2 ) = -1;
	instance( 2, 3 ) = - (2 * zFar * zNear) / (zFar - zNear);
}

void Mat4_Adapter::posMul( eigen::Mat4& instance, const eigen::Mat4& m )
{
	instance = m * instance;
}

void Mat4_Adapter::preMul( eigen::Mat4& instance, const eigen::Mat4& m )
{
	instance *= m;
}

void Mat4_Adapter::rotate( eigen::Mat4& instance, double degrees, const eigen::Vec3& axis )
{
	double radians = degrees * PI / 180;	
	
	Eigen::Transform<double, 3, Eigen::Affine> t;
	t = Eigen::AngleAxisd( radians, axis.normalized() ); //The axis vector must be normalized.

	instance = t * instance;
}

void Mat4_Adapter::rotationFromTo( eigen::Mat4& instance, const eigen::Vec3& from, const eigen::Vec3& to )
{
	eigen::Vec3 nFrom = from.normalized();
	eigen::Vec3 nTo =  to.normalized();
	
	double angle = std::acos( nFrom.dot( nTo ) ); //radians
	eigen::Vec3 axis =  nFrom.cross( nTo );
	
	Eigen::Transform<double, 3, Eigen::Affine> t;
	t = Eigen::AngleAxisd( angle, axis.normalized() ); //The axis vector must be normalized.

	instance = t * instance;
}

void Mat4_Adapter::scale( eigen::Mat4& instance, const eigen::Vec3& scale )
{
	Eigen::Transform<double, 3, Eigen::Affine> t;
	t = Eigen::Scaling( scale );

	instance = t * instance;
}

void Mat4_Adapter::translate( eigen::Mat4& instance, const eigen::Vec3& position )
{
	Eigen::Transform<double, 3, Eigen::Affine> t;
    t = Eigen::Translation<double, 3>( position );

	instance = t * instance;
}

void Mat4_Adapter::transpose( eigen::Mat4& instance )
{
	instance.transposeInPlace();
}

} // namespace eigen