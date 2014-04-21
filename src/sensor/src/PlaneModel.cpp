#include <cmath>

#include "PlaneModel.h"


PlaneModel::PlaneModel()
{
}


PlaneModel::PlaneModel(pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers) : coefficients(coefficients), inliers(inliers)
{
	//normalize all the coefficients to positive if all of them are negative
	this->normalize();
}

PlaneModel::~PlaneModel()
{
}


bool PlaneModel::operator< (const PlaneModel & other) const
{
	return this->inliers->indices.size() < other.inliers->indices.size();
}

//hash value of this plane according to the properties of this plane
PlaneModel::operator size_t() const
{
	size_t prime = 31;
	size_t result = 1;
	size_t temp;

	//hash value of the coefficients
	for each (float f in this->coefficients->values)
	{
		unsigned long * pLong = (unsigned long *)(&f);
		temp = (*pLong);

		result = prime * result + (temp ^ (temp >> 32));
	}
	
	//hash value of the number of points in this plane
	temp = this->inliers->indices.size();
	result = prime * result + (temp ^ (temp >> 32));
	
	return result;
}

//decide whether if two planes are identical
//::return true if all the properties are the same.
bool PlaneModel::operator== (const PlaneModel & other) const
{
	bool equal = true;
	
	equal = equal && this->inliers->indices.size() == other.inliers->indices.size();
	if (!equal)
	{
		return equal;
	}

	for (size_t i = 0; i < other.coefficients->values.size(); i++)
	{
		equal = equal && (this->coefficients->values[i] == other.coefficients->values[i]);
	}
	return equal;
}

//the more similar the numbers of the two planes are, the less the distance is
//distance range : [0, 0.5]
float PlaneModel::distanceToByPointCount(const PlaneModel & other) const
{
	return std::fabs( (this->inliers->indices.size() / (float)(this->inliers->indices.size() + other.inliers->indices.size()) - 0.5) );
}

//the distance function to tell whether if two planes are similar enough.
float PlaneModel::distanceTo(const PlaneModel & other) const
{
	float distance = 0;

	for (size_t i = 0; i < other.coefficients->values.size(); i++)
	{
		distance += std::powf(this->coefficients->values[i] - other.coefficients->values[i],
								2);
	}

	distance += std::powf(this->distanceToByPointCount(other), 2);

	return distance;
}

//identical function to <
bool PlaneModel::comparePlaneByPointCount(PlaneModel & plane1, PlaneModel & plane2)
{
	return plane1.inliers->indices.size() > plane2.inliers->indices.size();
}

//normalize all the coefficients to positive if all of them are negative
void PlaneModel::normalize()
{
	bool allNegative = true;

	for (size_t i = 0; i < 3; i++)
	{
		allNegative = allNegative && (this->coefficients->values[i] < 0);
	}

	if (allNegative)
	{
		//std::cout << "all negative" << std::endl;
		for (size_t i = 0; i < 4; i++)
		{
			//std::cout << this->coefficients->values[i] << std::endl;

			this->coefficients->values[i] = 0 - (this->coefficients->values[i]);

			//std::cout << this->coefficients->values[i] << std::endl;
		}
	}

}

//decide which side this plane is, 0 for floor, 1 for left wall, 2 for ceiling, 3 for right wall
//when viewing along with the z-positive axis of the kinect

// the basic idea is to find the most similar norm vector with the one of this plane, then
// the plane of that norm vector is the result one.
int PlaneModel::whichSide(PlaneModel & plane)
{
	/*Eigen::Vector3f floor_1(0, 1, 0);
	Eigen::Vector3f floor_2(0, -1, 0);

	Eigen::Vector3f floor_1(1, 0, 0);
	Eigen::Vector3f floor_2(-1, 0, 0);*/

	//norm vectors for 0:floor, 1:left-wall, 2:ceiling, 3:right-wall;
	Eigen::Vector3f abcd[4] = { Eigen::Vector3f(0, 1, 0),
								Eigen::Vector3f(0, -1, 0),
								Eigen::Vector3f(1, 0, 0),
								Eigen::Vector3f(-1, 0, 0) };

	//plane.normalize();

	Eigen::Vector3f normVec;
	for (size_t i = 0; i < 3; i++)
	{
		normVec(i) = plane.coefficients->values[i];
	}

	//the threshold of rad, 
	//if the rad of angle of two planes is bigger than this threshold, then these two planes are not similar enough.
	float bound = M_PI_4 / 2;

	float rad[4];
	int idx = -1;

	//find the 1st side which is lower than the threshold.
	for (size_t i = 0; i < 4; i++)
	{
		rad[i] = std::acosf(normVec.dot(abcd[i]) / (normVec.norm() * abcd[i].norm()));

		if (rad[i] < bound)
		{
			idx = (int)i;
			break;
		}
	}


	if (idx < 0)
	{
		return idx;
	}

	//decide the which side the plane belongs to according to D in coefficient
	//after find the most similar nor vector.
	switch (idx)
	{
		case 0:
			if (plane.coefficients->values[3] > 0)
			{
				return 0;
			}
			else
			{
				return 2;
			}
			break;
		case 1:
			if (plane.coefficients->values[3] > 0)
			{
				return 2;
			}
			else
			{
				return 0;
			}
			break;
		case 2:
			if (plane.coefficients->values[3] > 0)
			{
				return 3;
			}
			else
			{
				return 1;
			}
			break;
		case 3:
			if (plane.coefficients->values[3] > 0)
			{
				return 1;
			}
			else
			{
				return 3;
			}
			break;
		default:
			return -1;
			break;
	}


	return -1;
	

}

//calculate the coordinates of the intersection point
Eigen::Vector3f PlaneModel::intersectionPoint(const PlaneModel & plane1, const PlaneModel & plane2, float x)
{

	float	a1 = plane1.coefficients->values[0],
			b1 = plane1.coefficients->values[1],
			c1 = plane1.coefficients->values[2],
			d1 = plane1.coefficients->values[3];

	float	a2 = plane2.coefficients->values[0],
			b2 = plane2.coefficients->values[1],
			c2 = plane2.coefficients->values[2],
			d2 = plane2.coefficients->values[3];


	float z = (-d2 + b2*(d1 + a1*x) / b1 - a2*x) / (c2 - b2*c1 / b1);

	float y = -((d1 + a1*x) / b1 + c1*z / b1);

	return Eigen::Vector3f(x, y, z);
}

//return the norm vector of this plane, obtained from coefficients A, B, C, D.
Eigen::Vector3f PlaneModel::normalVec()
{
	Eigen::Vector3f normVec;
	for (size_t i = 0; i < 3; i++)
	{
		normVec(i) = coefficients->values[i];
	}

	return normVec;
}

//get the rotation matrix through which this plane is rotated to another
Eigen::Matrix4f PlaneModel::rotationTo( PlaneModel & other)
{
	
	//Nom vectors of these two planes
	Eigen::Vector3f n1 = this->normalVec();
	Eigen::Vector3f n2 = other.normalVec();

	//make sure that the two norm vector point to the same direction.
	if (n1.dot(n2) < 0)
	{
		n2 = -n2;
	}
	
	if ((n1.dot(n2)) > 0.999999&&abs(this->coefficients->values[3]-other.coefficients->values[3])<0.03f)
		return Eigen::Matrix4f::Identity();
	//Obtain two intersection points P1 and P2
	Eigen::Vector3f P1 = PlaneModel::intersectionPoint(*this, other, 1.5);
	Eigen::Vector3f P2 = PlaneModel::intersectionPoint(*this, other, 2.5);

	
	

	//the rotation axis
	Eigen::Vector3f r(P2(0) - P1(0), P2(1) - P1(1), P2(2) - P1(2));
	r.normalize();

	n1.normalize(); n2.normalize();

	Eigen::Vector3f n1Xn2 = n1.cross(n2);/* (n1(1)*n2(2) - n1(2)*n2(1),
							n1(2)*n2(0) - n1(0)*n2(2),
							n1(0)*n2(1) - n1(1)*n2(0));*/
	n1Xn2.normalize();

	

	float signal = n1Xn2.dot(r);

	//calculate the sin and cos values used to in transformation matrix formulation
	//be aware of the signal of the sin value.
	float cos = n1.dot(n2);
	float sin = signal > 0 ? std::sqrtf(1 - cos * cos) : -std::sqrtf(1 - cos * cos);

	
	//calculate the transform matrix
	float u = r(0), v = r(1), w = r(2);
	float a = P1(0), b = P1(1), c = P1(2);

	Eigen::Matrix4f tm;

	tm(0, 0) = u * u + (v*v + w*w)*cos;
	tm(0, 1) = u*v*(1 - cos) - w*sin;
	tm(0, 2) = u*w*(1 - cos) + v*sin;
	tm(0, 3) = (a*(v*v + w*w) - u*(b*v + c*w))*(1 - cos) + (b*w - c*v)*sin;
	
	tm(1, 0) = u*v*(1 - cos) + w*sin;
	tm(1, 1) = v*v + (u*u + w*w)*cos;
	tm(1, 2) = v*w*(1 - cos) - u*sin;
	tm(1, 3) = (b*(u*u + w*w) - v*(a*u + c*w))*(1 - cos) + (c*u - a*w)*sin;

	tm(2, 0) = u*w*(1 - cos) - v*sin;
	tm(2, 1) = v*w*(1 - cos) + u*sin;
	tm(2, 2) = w*w + (u*u + v*v)*cos;
	tm(2, 3) = (c*(u*u + v*v) - w*(a*u + b*v))*(1 - cos) + (a*v - b*u)*sin;

	tm(3, 0) = 0;
	tm(3, 1) = 0;
	tm(3, 2) = 0;
	tm(3, 3) = 1;

	return tm;
}


