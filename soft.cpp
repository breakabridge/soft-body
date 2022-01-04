#include <iostream>
#include <fstream>
#include <cmath>

#define SWAP(x0, x) {float * tmp = x0; x0 = x; x = tmp;}
#define IX(i, j) ((i) + (width * j))

const int frames = 900;
const int width = 10;
const int height = 10;
const float droppingHeight = 10;
const float stiffness = 100;
const float damping = 10;
const float grav = 1;
const float dt = 0.01;

const int size = width * height;
const int springSize = 4 * size - 3 * (width + height) + 2;
float scalarProduct2D(float * x, float * y)   {
    return x[0] * y[0] + x[1] * y[1];
}

float vectorNorm(float * x)	{
	return sqrt(scalarProduct2D(x, x));
}

class Point	{
	public:
		float pos[2];
		float vel[2]   = {0,  0};
		float force[2] = {0, -grav};

		Point()	{
			
		}

		Point(float x, float y)		{
			pos[0] = x; pos[1] = y;
		}

		void resetForces()	{
			force[0] = 0; force[1] = -grav;
		}

		void update()	{
			float newVel[2];
			newVel[0] = vel[0] + dt * force[0];
			newVel[1] = vel[1] + dt * force[1];
			pos[0] += (vel[0] + newVel[0]) * 0.5 * dt;
			pos[1] += (vel[1] + newVel[1]) * 0.5 * dt;
			vel[0] = newVel[0]; vel[1] = newVel[1];

			if (pos[1] < 0) {vel[1] -= 2 * vel[1]; force[0] = 0; force[1] = 1;}
		}
};

class Spring	{
	public:
		Point * p1;
		Point * p2;
		float l;

		Spring()	{

		}

		Spring(Point * point1, Point * point2)	{
			p1 = point1; p2 = point2;
			float pos1[2] = {(p1 -> pos)[0], (p1 -> pos)[1]};
			float pos2[2] = {(p2 -> pos)[0], (p2 -> pos)[1]};
			float relVector[2] = {pos2[0] - pos1[0], pos2[1] - pos1[1]};
			l = vectorNorm(relVector);
		}

		void calculateForce()	{
			float pos1[2] = {(p1 -> pos)[0], (p1 -> pos)[1]};
			float pos2[2] = {(p2 -> pos)[0], (p2 -> pos)[1]};
			float vel1[2] = {(p1 -> vel)[0], (p1 -> vel)[1]};
			float vel2[2] = {(p2 -> vel)[0], (p2 -> vel)[1]};
			float unitVector[2] = {pos2[0] - pos1[0], pos2[1] - pos1[1]};
			float vectorLength = vectorNorm(unitVector);

			if (vectorLength != 0)	{
				unitVector[0] /= vectorLength; unitVector[1] /= vectorLength;
				
				if (vectorLength < 0.1)	{
					float magnitude = scalarProduct2D((p1 -> vel), unitVector);
					(p1 -> vel)[0] -= 2 * magnitude * unitVector[0]; (p1 -> vel)[1] -= 2 * magnitude * unitVector[1];
					magnitude = scalarProduct2D((p2 -> vel), unitVector);
					(p2 -> vel)[0] += 2 * magnitude * unitVector[0]; (p2 -> vel)[1] += 2 * magnitude * unitVector[1];
				}	else	{
					float relVel[2] = {vel2[0] - vel1[0], vel2[1] - vel1[1]};
					float forceSize = (vectorLength - l) * stiffness + scalarProduct2D(unitVector, relVel) * damping;
					float force[2] = {forceSize * unitVector[0], forceSize * unitVector[1]};

					(p1 -> force)[0] += force[0]; (p1 -> force)[1] += force[1]; 
					(p2 -> force)[0] -= force[0]; (p2 -> force)[1] -= force[1];
				}
			}
		}
};

int main()  {
	int i, j;
	Point body[size];
	Spring springs[springSize];

	for (j = 0; j < height; j++)	{
		for (i = 0; i < width; i++)	{
			body[IX(i, j)] = Point(i, j + droppingHeight);
		}
	}
	
	int count = 0;
	float pos1[2], pos2[2];
	for (i = 0; i < size; i++)	{
		pos1[0] = body[i].pos[0]; pos1[1] = body[i].pos[1];
		for (j = i + 1; j < size; j++)	{
			pos2[0] = body[j].pos[0]; pos2[1] = body[j].pos[1];
			float relVector[2] = {pos2[0] - pos1[0], pos2[1] - pos1[1]};

			if (vectorNorm(relVector) < 1.5)	{
				springs[count] = Spring(&body[i], &body[j]);
				count++;
			}
		}
	}
	
	std::ofstream file;
    file.open("soft_sim.dat");
    srand(time(NULL));
    file << width << "," << height << "," << frames << "\n";

	for (i = 0; i < frames; i++)	{
		for (int k = 0; k < 10; k++)	{
			for (j = 0; j < springSize; j++)	{
				springs[j].calculateForce();
			}
			for (j = 0; j < size; j++)	{
				body[j].update();
				body[j].resetForces();
			}
		}

		for (j = 0; j < size; j++)	{
			file << body[j].pos[0] << "," << body[j].pos[1] << ",";
		}

		file << "\n";
	}

	file.close();

    return 0;
}
