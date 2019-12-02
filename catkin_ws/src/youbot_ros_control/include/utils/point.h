#ifndef POINT_H
#define POINT_H

#include <eigen3/Eigen/Dense>

class Point
{
    public:
        Point(float x = 0, float y = 0, float z = 0, std::string unit = "None") : x(x), y(y), z(z), unit(unit){};
    
        Point operator + (const Point &val);
        Point& operator += (const Point &val);
        
        float x;
        float y;
        float z;
        
        std::string unit;  
};

class Pose
{
    public:
        Pose();        
        Pose(const Point p, const Point o);
        
        Pose &operator = (const Pose &val);
        Pose operator + (const Pose &val);
        Pose &operator += (const Pose &val);
        float &operator[](int i); // only allow access
        
        Point getPosition() {return position;};
        Point getOrientation() {return orientation;};
        Eigen::Matrix<float, 6, 1> getPoseVector() {return pose_vector;};
        
        void setPosition(const Point);
        void setPositionX(const float);
        void setPositionY(const float);
        void setPositionZ(const float);
        void setPositionUnit(const std::string u) {position.unit = u;};
        void setOrientation(const Point);
        void setOrientationX(const float);
        void setOrientationY(const float);
        void setOrientationZ(const float);
        void setOrientationUnit(const std::string u) {orientation.unit = u;};
        void setPoseVector(const Eigen::Matrix<float, 6, 1>);
        
        void setUnitPosition(std::string m) {position.unit = m;};
        
    private:
        Point position;
        Point orientation;
        Eigen::Matrix<float, 6, 1> pose_vector;
};

Point Point::operator + (const Point &val)
{
    Point tmp;
    tmp.x = x + val.x;
    tmp.y = y + val.y;
    tmp.z = z + val.z;
    
    if (unit != val.unit)
    {
        std::cout << "Warning: Units are not consistent" << '\n';
    }
    
    return tmp;
}

Point& Point::operator += (const Point &val)
{
    x += val.x;
    y += val.y;
    z += val.z;
    
    if (unit != val.unit)
    {
        std::cout << "Warning: Units are not consistent" << '\n';
    }
    
    return *this;
}

Pose::Pose()
{
    pose_vector(0,0) = position.x;
    pose_vector(1,0) = position.y;
    pose_vector(2,0) = position.z;
    pose_vector(3,0) = orientation.x;
    pose_vector(4,0) = orientation.y;
    pose_vector(5,0) = orientation.z;
}

Pose::Pose(const Point p, const Point o) : position(p), orientation(o) 
{
    pose_vector(0,0) = position.x;
    pose_vector(1,0) = position.y;
    pose_vector(2,0) = position.z;
    pose_vector(3,0) = orientation.x;
    pose_vector(4,0) = orientation.y;
    pose_vector(5,0) = orientation.z;
}

Pose& Pose::operator = (const Pose &val)
{
    position.x = val.position.x;
    position.y = val.position.y;
    position.z = val.position.z;
    orientation.x = val.orientation.x;
    orientation.y = val.orientation.y;
    orientation.z = val.orientation.z;
    
    pose_vector(0,0) = position.x;
    pose_vector(1,0) = position.y;
    pose_vector(2,0) = position.z;
    pose_vector(3,0) = orientation.x;
    pose_vector(4,0) = orientation.y;
    pose_vector(5,0) = orientation.z;    
    
    return *this;
}

Pose Pose::operator + (const Pose &val)
{
    Pose tmp;
    
    tmp.position = position + val.position;
    tmp.orientation = orientation + val.orientation;
    
    return tmp;
}

Pose& Pose::operator += (const Pose &val)
{
    position += val.position;
    orientation += val.orientation;

    pose_vector(0,0) += val.position.x;
    pose_vector(1,0) += val.position.y;
    pose_vector(2,0) += val.position.z;
    pose_vector(3,0) += val.orientation.x;
    pose_vector(4,0) += val.orientation.y;
    pose_vector(5,0) += val.orientation.z;  
    
    return *this;
}

float& Pose::operator[](int i)
{
    switch(i)
    {
        case 0:
            return position.x;
        case 1:
            return position.y;
        case 2:
            return position.z;
        case 3:
            return orientation.x;
        case 4:
            return orientation.y;
        case 5:
            return orientation.z;
        default:
            std::cout << "Out of bounds !\n";
            exit(0);
    }
}

void Pose::setPosition(const Point p)
{
    position = p;
    pose_vector(0,0) = p.x;
    pose_vector(1,0) = p.y;
    pose_vector(2,0) = p.z;
}

void Pose::setPositionX(const float x)
{
    position.x = x;
    pose_vector(0,0) = x;
}

void Pose::setPositionY(const float y)
{
    position.y = y;
    pose_vector(1,0) = y;
}

void Pose::setPositionZ(const float z)
{
    position.z = z;
    pose_vector(2,0) = z;
}

void Pose::setOrientation(const Point o)
{
    orientation = o;
    pose_vector(0,0) = o.x;
    pose_vector(1,0) = o.y;
    pose_vector(2,0) = o.z;    
}

void Pose::setOrientationX(const float x)
{
    orientation.x = x;
    pose_vector(3,0) = x;
}

void Pose::setOrientationY(const float y)
{
    orientation.y = y;
    pose_vector(4,0) = y;
}

void Pose::setOrientationZ(const float z)
{
    orientation.z = z;
    pose_vector(5,0) = z;
}

void Pose::setPoseVector(const Eigen::Matrix<float, 6, 1> p_v)
{
    pose_vector = p_v;
    position.x = p_v(0,0);
    position.y = p_v(1,0);
    position.z = p_v(2,0);
    orientation.x = p_v(3,0);
    orientation.y = p_v(4,0);
    orientation.z = p_v(5,0);
}

#endif
