#include "Point.h"
#include <iostream>
#include <vector>
#include <time.h>
#include <thread>
#include <Windows.h>
using namespace std;

///Prototypes
struct RGBbuffer;
struct Ray;
struct Camera;
struct Color;
struct AAPixel;
struct Source;
struct Light;
struct Object;
struct Sphere;
struct Plane;

//Global 
vector<Object*> scene_objects;
vector<Source*> light_sources;
//

void SaveBMP(const char *filename, int w, int h, RGBbuffer *data);
int HitObject(vector<double> & object_intersections);
Color CastRay(Point & intersect_loc, Point & intersect_ray_dir, vector<Object*> scene_objects, vector<Source*> light_sources, int hit_object_index, double ambientLight, double accuracy);
void CastRaysToPixels(int width, int height, int depth, RGBbuffer * pixels, Camera & scene_cam, int ambientLight, int lps, int lpe);
vector<int> bounds(int numNeeded, int size);
void RenderFrame(int width, int height, int aaFactor, RGBbuffer * pixels);
///

struct RGBbuffer {
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

struct Ray{
	Point Origin, Dir;

	Ray(){
		Origin = Point(0, 0, 0);
		Dir = Point(1, 0, 0);
	};

	Ray(Point Origin, Point Dir){
		this->Origin = Origin;
		this->Dir = Dir;
	};

};

struct Camera{
	Point camPos, camDir, camUp, camDown;

	Camera(){
		camPos = Point(0, 0, 0);
		camDir = Point(0, 0, 1);
		camUp = Point(0, 0, 0);
		camDown = Point(0, 0, 0);
	};

	Camera(Point camPos, Point camDir, Point camUp, Point camDown){
		this->camPos = camPos;
		this->camDir = camDir;
		this->camUp = camUp;
		this->camDown = camDown;
	};
};

struct Color {
	//red, green, blue, reflection factor [0-1]
	double red, green, blue, ref_factor;

	Color(){
		red = 0.5;
		green = 0.5;
		blue = 0.5;
	};
	
	Color(double red, double green, double blue, double ref_factor) {
		this->red = red;
		this->green = green;
		this->blue = blue;
		this->ref_factor = ref_factor;
	}

	Color operator *(double scalar) const{
		return Color(red*scalar, green*scalar, blue*scalar, ref_factor);
	}

	Color operator *(const Color & color) const {
			return Color(red*color.red, green*color.green, blue*color.blue, ref_factor);
	}
	
	Color operator +(const Color & color) const {
		return Color(red + color.red, green + color.green, blue + color.blue, ref_factor);
	}

	Color clip() {
		if (red > 1) { red = 1; }
		if (green > 1) { green = 1; }
		if (blue > 1) { blue = 1; }
		if (red < 0) { red = 0; }
		if (green < 0) { green = 0; }
		if (blue < 0) { blue = 0; }
		return Color(red, green, blue, ref_factor);
	}
};

struct AAPixel{
private:
	int depth;

public:
	double * red;
	double * blue;
	double * green;

	AAPixel(int depth){
		this->depth = depth;
		this->red = new double[depth*depth];
		this->blue = new double[depth*depth];
		this->green = new double[depth*depth];
	}

	~AAPixel(){
		delete red;
		delete blue;
		delete green;
	}

	Color AvgPixelsColor(){
		double totalRed = 0.0;
		double totalGreen = 0.0;
		double totalBlue = 0.0;

		for (int iColor = 0; iColor < depth * depth; iColor++){
			totalRed = totalRed + red[iColor];
			totalGreen = totalGreen + green[iColor];
			totalBlue = totalBlue + blue[iColor];
		}

		double avgRed = totalRed / (depth*depth);
		double avgGreen = totalGreen / (depth*depth);
		double avgBlue = totalBlue / (depth*depth);

		return Color(avgRed, avgGreen, avgBlue, -1);
	}
};

struct Source {
public:
	Source(){};
	virtual Point getLightPosition() { return Point(0, 0, 0); }
	virtual Color getLightColor() { return Color(1, 1, 1, 0); }
};

struct Light : public Source{
	Point position;
	Color color;
public:
	Light() {
		position = Point(0, 0, 0);
		color = Color(1, 1, 1, 0);
	}
	Light(Point position, Color color) {
		this->position = position;
		this->color = color;
	}

	virtual Point getLightPosition() { return position; }
	virtual Color getLightColor() { return color; }
};

struct Object {
public:
	Object(){};

	virtual Color getColor() { return Color(0.0, 0.0, 0.0, 0); }

	virtual Point getNormalAt(Point intersect_loc) {
		return Point(0, 0, 0);
	}

	virtual double findIntersection(Ray ray) {
		return 0;
	}
};

struct Sphere : public Object {
private:
	Point center;
	double radius;
	Color color;
public:
	Sphere(){
		center = Point(0, 0, 0);
		radius = 1.0;
		color = Color(0.5, 0.5, 0.5, 0);
	};

	Sphere(Point center, double radius, Color color) {
		this->center = center;
		this->radius = radius;
		this->color = color;
	}

	virtual Color getColor() { return color; }

	virtual Point getNormalAt(Point point) {
		Point normal_Vect = (point + (-center)).normalize();
		return normal_Vect;
	}

	virtual double findIntersection(Ray ray) {
		double ray_origin_x = ray.Origin.x();
		double ray_origin_y = ray.Origin.y();
		double ray_origin_z = ray.Origin.z();

		double ray_direction_x = ray.Dir.x();
		double ray_direction_y = ray.Dir.y();
		double ray_direction_z = ray.Dir.z();

		double sphere_center_x = center.x();
		double sphere_center_y = center.y();
		double sphere_center_z = center.z();

		double vx = ray_origin_x - sphere_center_x;
		double vy = ray_origin_y - sphere_center_y;
		double vz = ray_origin_z - sphere_center_z;

		double vB = (2 * vx * ray_direction_x) + (2 * vy * ray_direction_y) + (2 * vz * ray_direction_z);
		double sphere = pow(vx, 2) + pow(vy, 2) + pow(vz, 2) - (radius*radius);
		double vD = vB * vB - 4 * sphere;	//discriminant

		double err_Accuracy = 0.000001;

		if (vD <= 0) {
			//No Intersection
			return -1;
		}

		else {
			//Ray Intersects Sphere
			double rootA = ((-1 * vB - sqrt(vD)) / 2) - err_Accuracy;

			//Check for the smallest positive root
			if (rootA > 0) {
				return rootA;
			}
			else {
				double rootB = ((sqrt(vD) - vB) / 2) - err_Accuracy;
				return rootB;
			}
		}

	}
};

struct Plane : public Object {
private:
	Point normal;
	double distance;
	Color color;

public:
	Plane(){
		normal = Point(1, 0, 0);
		distance = 0;
		color = Color(0.5, 0.5, 0.5, 0);
	};

	Plane(Point normal, double distance, Color color) {
		this->normal = normal;
		this->distance = distance;
		this->color = color;
	}

	Point getPlaneNormal() { return normal; }

	double getPlaneDistance() { return distance; }

	virtual Color getColor() { return color; }

	virtual Point getNormalAt(Point point) {
		return normal;
	}

	virtual double findIntersection(Ray ray) {
		double v = ray.Dir.dot(normal);

		if (v == 0) {
			//Parallel Ray 
			return -1;
		}

		else {
			double v2 = normal.dot(ray.Origin + (-(distance * normal)));
			return (-1 * v2 / v);
		}

	}
};

void SaveBMP(const char *filename, int w, int h, RGBbuffer *data) {
	FILE *fp;

	int paddedsize = 1 * w * h + 1;

	// declare bmp structures 
	BITMAPFILEHEADER bmFileHeader;
	BITMAPINFOHEADER bmInfoHeader;

	memset(&bmFileHeader, 0, sizeof(BITMAPFILEHEADER));
	memset(&bmInfoHeader, 0, sizeof(BITMAPINFOHEADER));

	//Fileheader
	bmFileHeader.bfType = 0x4d42;       // 0x4d42 = 'BM'
	bmFileHeader.bfReserved1 = 0;
	bmFileHeader.bfReserved2 = 0;
	bmFileHeader.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + paddedsize;
	bmFileHeader.bfOffBits = 0x36;		// number of bytes to start of bitmap bits

	//Infoheader
	bmInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmInfoHeader.biWidth = w;
	bmInfoHeader.biHeight = h;
	bmInfoHeader.biPlanes = 1;	//One bitplane
	bmInfoHeader.biBitCount = 24;	//RGB mode is 24 bits
	bmInfoHeader.biCompression = BI_RGB;
	bmInfoHeader.biSizeImage = 0;
	bmInfoHeader.biXPelsPerMeter = 0x0ec4;	//value that paint uses
	bmInfoHeader.biYPelsPerMeter = 0x0ec4;
	bmInfoHeader.biClrUsed = 0;	//RGB mode/no palette
	bmInfoHeader.biClrImportant = 0;	//All colors are important

	fp = fopen(filename, "wb");
	fwrite(&bmFileHeader, 1, 14, fp);
	fwrite(&bmInfoHeader, 1, 40, fp);

	for (int i = 0; i < (w * h); i++) {
		RGBbuffer rgb = data[i];
		fwrite(&rgb, 1, 3, fp);
	}
	fclose(fp);
}

//Check if object is hit
int HitObject(vector<double> & object_intersections) {
	int minval_index;	//return the index of hit obj
	if (object_intersections.size() == 0) {
		//no intersections
		return -1;
	}
	else if (object_intersections.size() == 1) {
		if (object_intersections[0] > 0) {
			return 0;
		}
		else {
			return -1;
		}
	}
	else {
		//more than one intersection
		//first maximum value
		double max = 0;
		for (unsigned int i = 0; i < object_intersections.size(); i++) {
			if (max < object_intersections[i]) {
				max = object_intersections[i];
			}
		}
		//find the minimum positive value
		if (max > 0) {
			//positive intersections
			for (unsigned int index = 0; index < object_intersections.size(); index++) {
				if (object_intersections[index] > 0 && object_intersections[index] <= max) {
					max = object_intersections[index];
					minval_index = index;
				}
			}
			return minval_index;
		}
		else {
			//negative intersections
			return -1;
		}
	}
}

//Get the color at the hit location
Color CastRay(Point & intersect_loc, Point & intersect_ray_dir, vector<Object*> scene_objects, vector<Source*> light_sources, int hit_object_index, double ambientLight, double accuracy) {
	Color hit_obj_color = scene_objects[hit_object_index] -> getColor();
	Point hit_object_norm = scene_objects[hit_object_index] -> getNormalAt(intersect_loc);

	if (hit_obj_color.ref_factor == 2){
		//checkered/tile floor pattern
		int square = (int) floor(intersect_loc.x()) + (int) floor(intersect_loc.z());

		if ((square % 2) == 0) {
			//black tile
			hit_obj_color.red = 0;
			hit_obj_color.green = 0;
			hit_obj_color.blue = 0;
		}
		else {
			//white tile
			hit_obj_color.red = 1;
			hit_obj_color.green = 1;
			hit_obj_color.blue = 1;
		}
	}

	Color pixel_Color = hit_obj_color * ambientLight;

	if (hit_obj_color.ref_factor > 0 && hit_obj_color.ref_factor <= 1){
		//reflection from objects with specular
		Point scalar1 = hit_object_norm.dot(-intersect_ray_dir) * hit_object_norm;
		Point scalar2 = 2 * (scalar1 + intersect_ray_dir);
		Point delta = (-intersect_ray_dir) + (scalar2);
		Point reflection_dir = delta.normalize();
		Ray reflection_ray(intersect_loc, reflection_dir);

		//determine ray first intersect
		vector<double> reflection_intersections;

		for (int reflection_index = 0; reflection_index < scene_objects.size(); reflection_index++) {
			reflection_intersections.push_back(scene_objects[reflection_index] -> findIntersection(reflection_ray));
		}

		int reflection_hitbox_index = HitObject(reflection_intersections);

		if (reflection_hitbox_index != -1) {
			//reflect ray missed
			if (reflection_intersections[reflection_hitbox_index] > accuracy) {
				// determine the position and direction at the point of intersection with the reflection ray
				Point ref_intersect_loc = intersect_loc + (reflection_intersections[reflection_hitbox_index]*reflection_dir);
				Color ref_intersect_color = CastRay(ref_intersect_loc, reflection_dir, scene_objects, light_sources, reflection_hitbox_index, ambientLight, accuracy);
				pixel_Color = pixel_Color + (ref_intersect_color * hit_obj_color.ref_factor);
			}

		}

	}

	for (unsigned int light_index = 0; light_index < light_sources.size(); light_index++) {
		Point light_dir = (light_sources[light_index] -> getLightPosition() + (-intersect_loc) ).normalize();
		float cosine_angle = hit_object_norm.dot(light_dir);

		if (cosine_angle > 0) {
			// test for shadows
			bool shadowed = false;
			Point distance_to_light = (light_sources[light_index] -> getLightPosition() + (-intersect_loc) ).normalize();
			float distance_to_light_magnitude = distance_to_light.magnitude();
			Ray shadow_ray(intersect_loc, (light_sources[light_index] -> getLightPosition() + (-intersect_loc) ).normalize());
			
			vector<double> secondary_intersections;
			for (unsigned int object_index = 0; object_index < scene_objects.size() && shadowed == false; object_index++) {
				secondary_intersections.push_back(scene_objects[object_index] -> findIntersection(shadow_ray));
			}

			for (unsigned int j = 0; j < secondary_intersections.size(); j++) {
				if (secondary_intersections[j] > accuracy) {
					if (secondary_intersections[j] <= distance_to_light_magnitude) {
						shadowed = true;
					}
					break;
				}
			}

			if (!shadowed) {
				pixel_Color = pixel_Color + ( (hit_obj_color * light_sources[light_index]->getLightColor()) * cosine_angle);
				if (hit_obj_color.ref_factor > 0 && hit_obj_color.ref_factor <= 1) {
					Point scalar1 = (hit_object_norm.dot(-intersect_ray_dir)) * hit_object_norm;
					Point scalar2 = 2 * (scalar1 + intersect_ray_dir);
					Point delta = (-intersect_ray_dir) + scalar2;
					Point reflection_dir = delta.normalize();
					double specular = reflection_dir.dot(light_dir);

					if (specular > 0) {
						specular = pow(specular, 10);
						pixel_Color = pixel_Color + (light_sources[light_index]->getLightColor() * (specular*hit_obj_color.ref_factor) );
					} 
				}
			}
		}
	}
	return pixel_Color.clip();
}

//Compute the colors for all the pixels
void CastRaysToPixels(int width, int height, int depth, RGBbuffer * pixels, Camera & scene_cam, int ambientLight, int lps, int lpe){
	int flat_PixelIndex = 0;
	int aa_index = 0;
	double xchange = 0;
	double ychange = 0;
	double accuracy = 0.00000001;

	double aspectratio = (double)width / (double)height;

	for (int y = lps; y < lpe; y++){
		int temp = y * width;
		for (int x = 0; x < width; x++){
			flat_PixelIndex = temp + x;

			//Allocate our AApixel color 
			AAPixel * tempPixels = new AAPixel(depth);

			for (int aay = 0; aay < depth; aay++) {
				int temp1 = aay*depth;
				for (int aax = 0; aax < depth; aax++) {
					aa_index = temp1 + aax;

					if (depth == 1) {
						//No Anti-Aliasing
						if (width > height) {
							xchange = ((x + 0.5) / width)*aspectratio - (((width - height) / (double)height) / 2);
							ychange = ((height - y) + 0.5) / height;
						}
						else if (height > width) {
							xchange = (x + 0.5) / width;
							ychange = (((height - y) + 0.5) / height) / aspectratio - (((height - width) / (double)width) / 2);
						}
						else{
							//square
							xchange = (x + 0.5) / width;
							ychange = ((height - y) + 0.5) / height;
						}
					}
					else {
						//Anti-Aliasing
						if (width > height) {
							xchange = ((x + (double)aax / ((double)depth - 1)) / width)*aspectratio - (((width - height) / (double)height) / 2);
							ychange = ((height - y) + (double)aax / ((double)depth - 1)) / height;
						}
						else if (height > width) {
							xchange = (x + (double)aax / ((double)depth - 1)) / width;
							ychange = (((height - y) + (double)aax / ((double)depth - 1)) / height) / aspectratio - (((height - width) / (double)width) / 2);
						}
						else{
							//square image
							xchange = (x + (double)aax / ((double)depth - 1)) / width;
							ychange = ((height - y) + (double)aax / ((double)depth - 1)) / height;
						}
					}

					// cast ray from camera to pixel
					Point cam_ray_origin = scene_cam.camPos;
					Point cam_ray_direction = (scene_cam.camDir + (((xchange - 0.5)*scene_cam.camUp) + (((ychange - 0.5)*scene_cam.camDown)))).normalize();
					Ray cam_ray(cam_ray_origin, cam_ray_direction);

					vector<double> intersections;	//intersection between obj and ray
					//Fill the intersection array
					for (int index = 0; index < scene_objects.size(); index++) {
						intersections.push_back(scene_objects[index] -> findIntersection(cam_ray));
					}

					int first_hit_obj_index = HitObject(intersections);

					//Set the pixel to black if no intersection occurs
					if (first_hit_obj_index == -1) {
						tempPixels->red[aa_index] = 0;
						tempPixels->blue[aa_index] = 0;
						tempPixels->green[aa_index] = 0;
					}

					else{
						// index coresponds to an object in our scene
						if (intersections[first_hit_obj_index] > accuracy) {
							Point intersect_loc = cam_ray_origin + (intersections[first_hit_obj_index] * cam_ray_direction);
							Point intersect_ray_dir = cam_ray_direction;
							Color intersection_color = CastRay(intersect_loc, intersect_ray_dir, scene_objects, light_sources, first_hit_obj_index, ambientLight, accuracy);
							tempPixels->red[aa_index] = intersection_color.red;
							tempPixels->blue[aa_index] = intersection_color.blue;
							tempPixels->green[aa_index] = intersection_color.green;
						}
					}
				}
			}

			Color tempAvg = tempPixels -> AvgPixelsColor();
			pixels[flat_PixelIndex].r = (unsigned char) floor(tempAvg.red * 255.0);
			pixels[flat_PixelIndex].g = (unsigned char) floor(tempAvg.green * 255.0);
			pixels[flat_PixelIndex].b = (unsigned char) floor(tempAvg.blue * 255.0);
			delete tempPixels;
		}
	}
}

//Computes the bounds for multi-threading
vector<int> bounds(int numNeeded, int size){
	std::vector<int> b;
	int factor = floor(size / numNeeded);

	for (int i = 0; i <= size; i += factor){
		b.push_back(i);
	}

	return b;
}

//Renders the frame 
void RenderFrame(int width, int height, int aaFactor, RGBbuffer * pixels){

	//ID matrix
	Point X(1, 0, 0);
	Point Y(0, 1, 0);
	Point Z(0, 0, 1);
	//

	Point campos(3, 1.5, -4);
	Point look_at(0, 0, 0);
	Point diff_btw(campos - look_at);

	Point camdir = (-diff_btw).normalize();
	Point camright = Y.cross(camdir).normalize();
	Point camdown = camright.cross(camdir);

	Camera scene_cam(campos, camdir, camright, camdown);

	Color white_light(1.0, 1.0, 1.0, 0);
	Color green(0.5, 1.0, 0.5, 0.3);
	Color red(1.0, 0.5, 0.5, 0.4);
	Color blue(0.5, 0.5, 1.0, 0.5);
	Color tile_floor(1, 1, 1, 2);

	Point light_loc(-5, 10, -10);
	Light light(light_loc, white_light);

	light_sources.push_back(dynamic_cast<Source*>(&light));

	Point S1_loc(-2, 0, 0);
	Point S2_loc(1.75, -0.25, 0);
	Point S3_loc(0, 0.25, 5);

	Sphere scene_sphere(S1_loc, 1, green);
	Sphere scene_sphere2(S2_loc, 0.5, red);
	Sphere scene_sphere3(S3_loc, 1.5, blue);
	Plane scene_plane(Y, -1, tile_floor);

	scene_objects.push_back(dynamic_cast<Object*>(&scene_sphere));
	scene_objects.push_back(dynamic_cast<Object*>(&scene_sphere2));
	scene_objects.push_back(dynamic_cast<Object*>(&scene_sphere3));
	scene_objects.push_back(dynamic_cast<Object*>(&scene_plane));
	
	//Anti-Aliasing Factor
	int depth = aaFactor;
	
	double ambientLight = 0.1;

	//Multi Threading

	//On CPU with less threads the tasks will be scheduled
	int numThreads = 8;	//QuadCore

	//Vector of bounds for threads
	std::vector<int> vBounds = bounds(numThreads, height);

	std::thread * threads = new std::thread[numThreads - 1];

	//do numThreads - 1 threads of work
	for (int i = 0; i < numThreads - 1; ++i){
		threads[i] = std::thread(&CastRaysToPixels, width, height, depth, pixels, scene_cam, ambientLight, vBounds[i], vBounds[i+1]);
	}

	//use the main thread for 1 part
	for (int i = numThreads - 1; i < numThreads; ++i){
		CastRaysToPixels(width, height, depth, pixels, scene_cam, ambientLight, vBounds[i], vBounds[i + 1]);
	}

	//join all threads 
	for (int i = 0; i < numThreads - 1; ++i){
		threads[i].join();
	}

	delete[] threads;
	
	//End Threading

}

int main(){
	int width = 800;
	int height = 600;
	int nDim = width * height;

	std::cout << "Ray Tracer v1.0\n" << std::endl;
	std::cout << "Executing...\n" << std::endl;

	clock_t t1, t2;
	t1 = clock();

	RGBbuffer * rgbBuffer = new RGBbuffer[nDim];
	
	//The Higher the aafactor the slower the process but the more rays are cast
	RenderFrame(width, height, 1, rgbBuffer);
	SaveBMP("image.bmp", width, height, rgbBuffer);

	delete[] rgbBuffer;

	t2 = clock();
	float diff = (float)t2 - (float)t1;
	float seconds = diff / CLOCKS_PER_SEC;

	std::cout << "Execution Time: " << seconds << " sec" << std::endl;

	system("pause");

	return 0;
}