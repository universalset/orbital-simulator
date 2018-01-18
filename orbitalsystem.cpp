#include <cmath>
#include <vector>
#include <map>

namespace orb
{
    class Vec3d
    {
    public:
	double x, y, z;
	Vec3d(double a, double b, double c)
	    {
		x=a;
		y=b;
		z=c;
	    }
	Vec3d()
	    {
		x=0;
		y=0;
		z=0;
	    }
	Vec3d(const Vec3d & vec)
	    {
		x = vec.x;
		y = vec.y;
		z = vec.z;
	    }
	Vec3d operator+(const Vec3d & vec) const
	    {
		return Vec3d(this->x+vec.x, this->y+vec.y, this->z+vec.z);
	    }
	Vec3d operator-() const
	    {
		return Vec3d(-(this->x), -(this->y), -(this->z));
	    }
	Vec3d operator-(const Vec3d & vec) const
	    {
		return Vec3d(this->x-vec.x, this->y-vec.y, this->z-vec.z);
	    }
	double length() const
	    {
		return std::sqrt((this->x)*(this->x)+(this->y)*(this->y)+(this->z)*(this->z));
	    }
    
    };
    Vec3d operator*(double c, const Vec3d & vec)
    {
	return Vec3d(c*vec.x, c*vec.y, c*vec.x);
    }
    double dotProduct(const Vec3d & a, Vec3d & b)
    {
	return a.x*b.x + a.y*b.y + a.z*b.z;
    }
    Vec3d crossProduct(const Vec3d & a, const Vec3d & b)
    {
	return Vec3d(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
    }
    double distance(const Vec3d & a, const Vec3d & b)
    {
	Vec3d diff = a-b;
	return diff.length();
    }
    double distanceSquared(const Vec3d & a, const Vec3d & b)
    {
	Vec3d diff = a-b;
	return dotProduct(diff, diff);
    }




    class Body
    {
    public:
	Vec3d pos, vel;
	double mass, radius;
	int color;
	Body(const Vec3d & myPos, const Vec3d & myVel, double myMass, double myRadius, int myColor)
	    {
		pos = myPos;
		vel = myVel;
		mass = myMass;
		radius = myRadius;
		color = myColor;
	    }
	Body()
	    {
		Body(Vec3d(0,0,0), Vec3d(0,0,0), 0, 0, 0);
	    }
	void update(const Vec3d & accel, double timeStep)
	    {
		pos = pos + (timeStep*vel);
		vel = vel + (timeStep*accel);
	    }
    };


    class OrbitalSystem
    {
    private:
	std::map<int, Body*> bodies;
	double timestep;
	double gravity;
	int counter;
	
    public:
	OrbitalSystem()
	    {
		counter = 0;
		timestep = 1;
		gravity = 1;
	    }
	OrbitalSystem(double ts, double g)
	    {
		counter = 0;
		timestep = ts;
		gravity = g;
	    }
	~OrbitalSystem()
	    {
		for(auto x : bodies)
		{
		    delete (x.second);
		}
	    }
	int addBody(Body b)
	    {
		bodies[counter] = new Body(b);
		return counter++;
	    }
	int removeBody(int key)
	    {
		bodies.erase(key);
	    }
	double getTimestep()
	    {
		return timestep;
	    }
	void setTimestep(double ts)
	    {
		timestep = ts;
	    }
	double getGravity()
	    {
		return gravity;
	    }
	void setGravity(double g)
	    {
		gravity = g;
	    }
	std::vector<std::pair<int, Body> > status()
	    {
		std::vector<std::pair<int, Body> > statusVector;
		for(auto z : bodies)
		    statusVector.push_back(std::pair<int, Body>(z.first, *(z.second)));
		return statusVector;
	    }
	void step()
	    {
		std::map<int, Vec3d> accels;
		for(auto x : bodies)
		{
		    accels[x.first]=Vec3d(0,0,0);
		    for(auto y : bodies)
		    {
			if(x.first != y.first)
			{
			    double d = distance(x.second->pos, y.second->pos);
			    accels[x.first] = accels[x.first] + (y.second->mass*gravity/(d*d*d))*(y.second->pos-x.second->pos);
			}
		    }
		}
		for(auto x : bodies)
		{
		    x.second->update(accels[x.first], timestep);
		}
	    }
    };


}
