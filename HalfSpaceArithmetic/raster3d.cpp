//[header]
// A practical implementation of the rasterization algorithm.
//[/header]
//[compile]
// Download the raster3d.cpp, cow.h and geometry.h files to the same folder.
// Open a shell/terminal, and run the following command where the files are saved:
//
// c++ -o raster3d raster3d.cpp  -std=c++11 -O3
//
// Run with: ./raster3d. Open the file ./output.png in Photoshop or any program
// reading PPM files.
//[/compile]
//[ignore]
// Copyright (C) 2012  www.scratchapixel.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//[/ignore]

#include "pch.h"
#include "geometry.h"
#include <algorithm>
#include <fstream>
#include <chrono>

#include "cow.h"
#include <corecrt_math_defines.h>

static const float inchToMm = 25.4;
enum FitResolutionGate { kFill = 0, kOverscan };

using namespace std;

//[comment]
// Compute screen coordinates based on a physically-based camera model
// http://www.scratchapixel.com/lessons/3d-basic-rendering/3d-viewing-pinhole-camera
//[/comment]
void computeScreenCoordinates(
    const float &filmApertureWidth,
    const float &filmApertureHeight,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight,
    const FitResolutionGate &fitFilm,
    const float &nearClippingPLane,
    const float &focalLength,
    float &top, float &bottom, float &left, float &right
)
{
    float filmAspectRatio = filmApertureWidth / filmApertureHeight;
    float deviceAspectRatio = imageWidth / (float)imageHeight;
    
    top = ((filmApertureHeight * inchToMm / 2) / focalLength) * nearClippingPLane;
    right = ((filmApertureWidth * inchToMm / 2) / focalLength) * nearClippingPLane;

    // field of view (horizontal)
    float fov = 2 * 180 / M_PI * atan((filmApertureWidth * inchToMm / 2) / focalLength);
    std::cerr << "Field of view " << fov << std::endl;
    
    float xscale = 1;
    float yscale = 1;
    
    switch (fitFilm) {
        default:
        case kFill:
            if (filmAspectRatio > deviceAspectRatio) {
                xscale = deviceAspectRatio / filmAspectRatio;
            }
            else {
                yscale = filmAspectRatio / deviceAspectRatio;
            }
            break;
        case kOverscan:
            if (filmAspectRatio > deviceAspectRatio) {
                yscale = filmAspectRatio / deviceAspectRatio;
            }
            else {
                xscale = deviceAspectRatio / filmAspectRatio;
            }
            break;
    }
    
    right *= xscale;
    top *= yscale;
    
    bottom = -top;
    left = -right;
}

//[comment]
// Compute vertex raster screen coordinates.
// Vertices are defined in world space. They are then converted to camera space,
// then to NDC space (in the range [-1,1]) and then to raster space.
// The z-coordinates of the vertex in raster space is set with the z-coordinate
// of the vertex in camera space.
//[/comment]
void convertToRaster(
    const Vec3f &vertexWorld,
    const Matrix44f &worldToCamera,
    const float &l,
    const float &r,
    const float &t,
    const float &b,
    const float &near,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight,
    Vec3f &vertexRaster
)
{
    Vec3f vertexCamera;
	//将顶点从世界坐标转到相机坐标（视坐标）
    worldToCamera.multVecMatrix(vertexWorld, vertexCamera);

	//在把视锥体压缩成长方体的过程中，我们规定三个规则
	//1.近平面的所有点坐标不变
	//2.远平面的所有点坐标z值不变 都是f
	//3.远平面的中心点坐标值不变 为(0, 0, f)
	//正交矩阵和透视矩阵的推导，请参照https://zhuanlan.zhihu.com/p/122411512

    // convert to screen space，符合规则1，其实是将是椎体转化成长方体，其中一个面是近平面。
	// 假设相机和顶点在近平面的交点坐标为(Nx,Ny)，根据相似三角形, Near/Vz = Nx/Vx，Near/Vz = Ny/Vy
	// Nx = Near*Vx/Vz Ny = Near*Vy/Vz
	// 第一步将视椎体正交化
    Vec2f vertexScreen;
    vertexScreen.x = near * vertexCamera.x / -vertexCamera.z;
    vertexScreen.y = near * vertexCamera.y / -vertexCamera.z;
    
	// 透视矩阵，其实是将长方体映射到[-1,1]的正方体中，即现将视椎体正交化成长方体，在将长方体映射到正方体，注意是矩阵左乘屏幕坐标点。
    // now convert point from screen space to NDC space (in range [-1,1])
	// 第二步将正交化后的长方体在映射到[-1,1]的正方体
    Vec2f vertexNDC;
    vertexNDC.x = 2 * vertexScreen.x / (r - l) - (r + l) / (r - l);
    vertexNDC.y = 2 * vertexScreen.y / (t - b) - (t + b) / (t - b);

	// 映射到光栅化空间，即从[-1,1]映射为[0,1]，因为贴图等信息都是[0-1]
    // convert to raster space
    vertexRaster.x = (vertexNDC.x + 1) / 2 * imageWidth;
    // in raster space y is down so invert direction
    vertexRaster.y = (1 - vertexNDC.y) / 2 * imageHeight;
    vertexRaster.z = -vertexCamera.z;
}

float min3(const float &a, const float &b, const float &c)
{ return std::min(a, std::min(b, c)); }

float max3(const float &a, const float &b, const float &c)
{ return std::max(a, std::max(b, c)); }

float edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{ return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]); }

const uint32_t imageWidth = 640;
const uint32_t imageHeight = 480;
const Matrix44f worldToCamera = {0.707107, -0.331295, 0.624695, 0, 0, 0.883452, 0.468521, 0, -0.707107, -0.331295, 0.624695, 0, -1.63871, -5.747777, -40.400412, 1};

const uint32_t ntris = 3156;
const float nearClippingPLane = 1;
const float farClippingPLane = 1000;
float focalLength = 20; // in mm
// 35mm Full Aperture in inches
float filmApertureWidth = 0.980;
float filmApertureHeight = 0.735;

int main(int argc, char **argv)
{
    Matrix44f cameraToWorld = worldToCamera.inverse();

    // compute screen coordinates
    float t, b, l, r;
    
    computeScreenCoordinates(
        filmApertureWidth, filmApertureHeight,
        imageWidth, imageHeight,
        kOverscan,
        nearClippingPLane,
        focalLength,
        t, b, l, r);
    
    // define the frame-buffer and the depth-buffer. Initialize depth buffer
    // to far clipping plane.
    Vec3<unsigned char> *frameBuffer = new Vec3<unsigned char>[imageWidth * imageHeight];
    for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) frameBuffer[i] = Vec3<unsigned char>(255);
    float *depthBuffer = new float[imageWidth * imageHeight];
    for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) depthBuffer[i] = farClippingPLane;

    auto t_start = std::chrono::high_resolution_clock::now();
    
    // [comment]
    // Outer loop
    // [/comment]
	// cow一共有ntris=3156个顶点，对这3156个顶点进行光栅化
    for (uint32_t i = 0; i < ntris; ++i) {
        // 取出三角形顶点
		const Vec3f &v0 = vertices[nvertices[i * 3]];
        const Vec3f &v1 = vertices[nvertices[i * 3 + 1]];
        const Vec3f &v2 = vertices[nvertices[i * 3 + 2]];
        
        // [comment]
        // Convert the vertices of the triangle to raster space
		// 将顶点从物体坐标->世界坐标->视坐标（相机坐标）->屏幕坐标
        // [/comment]
        Vec3f v0Raster, v1Raster, v2Raster;
        convertToRaster(v0, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v0Raster);
        convertToRaster(v1, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v1Raster);
        convertToRaster(v2, worldToCamera, l, r, t, b, nearClippingPLane, imageWidth, imageHeight, v2Raster);
        
        // [comment]
        // Precompute reciprocal of vertex z-coordinate
        // [/comment]
        v0Raster.z = 1 / v0Raster.z,
        v1Raster.z = 1 / v1Raster.z,
        v2Raster.z = 1 / v2Raster.z;
        
        
        // [comment]
        // Prepare vertex attributes. Divde them by their vertex z-coordinate
        // (though we use a multiplication here because v.z = 1 / v.z)
		// 取出顶点属性
        // [/comment]
        Vec2f st0 = st[stindices[i * 3]];
        Vec2f st1 = st[stindices[i * 3 + 1]];
        Vec2f st2 = st[stindices[i * 3 + 2]];

        st0 *= v0Raster.z, st1 *= v1Raster.z, st2 *= v2Raster.z;
		
		// 计算三角形包围盒，为光栅化做准备
        float xmin = min3(v0Raster.x, v1Raster.x, v2Raster.x);
        float ymin = min3(v0Raster.y, v1Raster.y, v2Raster.y);
        float xmax = max3(v0Raster.x, v1Raster.x, v2Raster.x);
        float ymax = max3(v0Raster.y, v1Raster.y, v2Raster.y);
        
        // the triangle is out of screen，判断点不在屏幕范围内，因为v0Raster, v1Raster, v2Raster已经是屏幕坐标，所以通过和屏幕分辨率比较数值即可判断
        if (xmin > imageWidth - 1 || xmax < 0 || ymin > imageHeight - 1 || ymax < 0) continue;

		// 计算光栅化遍历起点和终点
        // be careful xmin/xmax/ymin/ymax can be negative. Don't cast to uint32_t
        uint32_t x0 = std::max(int32_t(0), (int32_t)(std::floor(xmin)));
        uint32_t x1 = std::min(int32_t(imageWidth) - 1, (int32_t)(std::floor(xmax)));
        uint32_t y0 = std::max(int32_t(0), (int32_t)(std::floor(ymin)));
        uint32_t y1 = std::min(int32_t(imageHeight) - 1, (int32_t)(std::floor(ymax)));

		// 使用半边函数(HalfFunction，相关论文参考REAMME.md)，注意叉乘在二维空间里表示的是平行四边形的面积
        float area = edgeFunction(v0Raster, v1Raster, v2Raster);
        
        // [comment]
        // Inner loop，开始光栅化
        // [/comment]
        for (uint32_t y = y0; y <= y1; ++y) {
            for (uint32_t x = x0; x <= x1; ++x) {
                Vec3f pixelSample(x + 0.5, y + 0.5, 0);
				// 分别计算光栅化过程中，像素点在不在三角形内部，由于使用的是最普通的划线算法，即遍历整个包围盒内的所有像素点，关于优化可以参考REAMME.md
                float w0 = edgeFunction(v1Raster, v2Raster, pixelSample);
                float w1 = edgeFunction(v2Raster, v0Raster, pixelSample);
                float w2 = edgeFunction(v0Raster, v1Raster, pixelSample);
				// 如果点在三条边内部，则处理该像素点
                if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
					// 计算三角形被中心点分割后的三个Child三角形的面积比率，即差值时的权重
                    w0 /= area;
                    w1 /= area;
                    w2 /= area;
					// 计算该像素点出的深度Z
                    float oneOverZ = v0Raster.z * w0 + v1Raster.z * w1 + v2Raster.z * w2;
					// 计算该像素点出的深度Z，因为在上面步骤中对屏幕空间的z已经取倒数，为了计算准确的深度值，此处又取了一次倒数
					float z = 1 / oneOverZ;
                    // [comment]
                    // Depth-buffer test
					// 深度测试，如果通过深度测试，则进行着色、灯光等响应计算
                    // [/comment]
                    if (z < depthBuffer[y * imageWidth + x]) {
						// 更新Z buffer
						depthBuffer[y * imageWidth + x] = z;
                        // 计算顶点属性差值
                        Vec2f st = st0 * w0 + st1 * w1 + st2 * w2;
                        
                        st *= z;
                        
                        // [comment]
                        // If you need to compute the actual position of the shaded
                        // point in camera space. Proceed like with the other vertex attribute.
                        // Divide the point coordinates by the vertex z-coordinate then
                        // interpolate using barycentric coordinates and finally multiply
                        // by sample depth.
                        // [/comment]
                        Vec3f v0Cam, v1Cam, v2Cam;
						// 将顶点从世界坐标转到视坐标（相机坐标）
                        worldToCamera.multVecMatrix(v0, v0Cam);
                        worldToCamera.multVecMatrix(v1, v1Cam);
                        worldToCamera.multVecMatrix(v2, v2Cam);
                        //计算屏幕近平面上交点坐标
                        float px = (v0Cam.x/-v0Cam.z) * w0 + (v1Cam.x/-v1Cam.z) * w1 + (v2Cam.x/-v2Cam.z) * w2;
                        float py = (v0Cam.y/-v0Cam.z) * w0 + (v1Cam.y/-v1Cam.z) * w1 + (v2Cam.y/-v2Cam.z) * w2;
						// 计算视空间下，点的位置
                        Vec3f pt(px * z, py * z, -z); // pt is in camera space
                        
                        // [comment]
                        // Compute the face normal which is used for a simple facing ratio.
                        // Keep in mind that we are doing all calculation in camera space.
                        // Thus the view direction can be computed as the point on the object
                        // in camera space minus Vec3f(0), the position of the camera in camera
                        // space.
						// 计算法线
                        // [/comment]
                        Vec3f n = (v1Cam - v0Cam).crossProduct(v2Cam - v0Cam);
                        n.normalize();
						// 计算入射方向
                        Vec3f viewDirection = -pt;
                        viewDirection.normalize();
                        
                        float nDotView =  std::max(0.f, n.dotProduct(viewDirection));
                        
                        // [comment]
                        // The final color is the reuslt of the faction ration multiplied by the
                        // checkerboard pattern.
                        // [/comment]
						// 设置棋盘格尺寸为10个格子
                        const int M = 10;
						// 绘制棋盘格，左上右下同色，左下右上同色，
                        float checker = (fmod(st.x * M, 1.0) > 0.5) ^ (fmod(st.y * M, 1.0) < 0.5);
                        float c = 0.3 * (1 - checker) + 0.7 * checker;
                        nDotView *= c;
						int val = std::floorf(nDotView * 255);
						// 将颜色值写入FrameBuffer
                        frameBuffer[y * imageWidth + x].x = val;
                        frameBuffer[y * imageWidth + x].y = val;
                        frameBuffer[y * imageWidth + x].z = val;
                    }
                }
            }
        }
    }
    
    auto t_end = std::chrono::high_resolution_clock::now();
	auto passedTime = std::chrono::duration<double, std::milli>(t_end - t_start).count();
	std::cerr << "Wall passed time:  " << passedTime << " ms" << std::endl;
    
    // [comment]
    // Store the result of the framebuffer to a PPM file (Photoshop reads PPM files).
    // [/comment]
    std::ofstream ofs;
    ofs.open("./output.ppm");
    ofs << "P6\n" << imageWidth << " " << imageHeight << "\n255\n";
    ofs.write((char*)frameBuffer, imageWidth * imageWidth * 3);
    ofs.close();
    
    delete [] frameBuffer;
    delete [] depthBuffer;
    
    return 0;
}