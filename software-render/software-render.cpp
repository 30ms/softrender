// software-render.cpp: 定义应用程序的入口点。
//

#include "software-render.h"

/*
 渲染线段
*/
void drawLine(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
    int dx = (x0 > x1) ? x0 - x1 : x1 - x0;
    int dy = (y0 > y1) ? y0 - y1 : y1 - y0;

    if (dx > dy) {//当斜率小于1时，使用X进行步进计算Y
        if (x0 > x1) {//交换坐标
            std::swap(x0, x1);
            std::swap(y0, y1);
        }
        //每步进一个单位的X要步进的Y值
        float step = dy / (float)dx;
        //Y步进的累积值
        float error = 0.f;
        //当前y
        int y = y0;
        //步进X，计算Y
        for (int x = x0; x < x1; x++) {
            image.set(x, y, color);
            //累积
            error += step;
            //当累积超过1时，Y步进1或-1
            if (error > 1.f) {
                y += (y1 > y0) ? 1 : -1;
                error -= 1.f;
            }
        }
    }
    else {//当斜率大于等于1，使用Y进行步进计算X
        if (y0 > y1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }
        float step = dx / (float)dy;
        float error = 0.f;
        int x = x0;
        for (int y = y0; y < y1; y++) {
            image.set(x, y, color);
            error += step;
            if (error > 1.f) {
                x += (x1 > x0) ? 1 : -1;
                error -= 1.f;
            }
        }
    }
}

/*
* 画三角面，扫描线法
*/
void drawTriangle_Scanline(int x0, int y0, int x1, int y1, int x2, int y2, TGAImage& image, TGAColor color) {
    //三角形的顶点坐标按y坐标进行排序
    if (y0 > y1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    if (y0 > y2) {
        std::swap(x0, x2);
        std::swap(y0, y2);
    }
    if (y1 > y2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }
    //从y0开始往y2步进，从y0开始往y2计算扫描线
    for (int y = y0; y <= y2; y++) {
        //相似系数
        float k;
        k = (float)(y - y0) / (y2 - y0);
        //扫描线与最长的边的交点的x
        float xa = x0 + (x2 - x0) * k;
        //扫描线与另一条边的交点的x
        float xb;

        if (y > y1) {//上半部分的交点
            k = (float)(y - y1) / (y2 - y1);
            xb = x1 + (x2 - x1) * k;
        }
        else {//下半部分的交点
            k = (float)(y - y0) / (y1 - y0);
            xb = x0 + (x1 - x0) * k;
        }

        if (xa > xb) {
            std::swap(xa, xb);
        }
        for (int x = xa; x < xb; x++) {
            image.set(x, y, color);
        }
    }
}

/*
 计算重心坐标
*/
vec3 barycentric(vec2 A, vec2 B, vec2 C, vec2 P) {
    //[u,v,1]和[AB,AC,PA]对应的x和y向量都垂直，所以叉乘
    vec3 u = cross(vec3(B.x - A.x, C.x - A.x, A.x - P.x), vec3(B.y - A.y, C.y - A.y, A.y - P.y));
    //三点共线时，会导致u.z为0，此时返回(-1,1,1)
    if (std::abs(u.z) < 1)return vec3(-1, 1, 1);
    return vec3(1.f - (u.x + u.y) / u.z, u.x / u.z, u.y / u.z);
}

//模型变换矩阵 模型坐标系的坐标变换到世界坐标系  Model -> World  右手系
mat<4, 4> modelTransforma(vec3 position, vec3 roate, vec3 scale) {
    return  translateXYZ(position) * scaleXYZ(scale) * rotateZ(roate.z) * rotateY(roate.y) * rotateX(roate.x);
}

//视图变换矩阵 世界坐标系的坐标变换到观察坐标系 World -> View   右手系
mat<4, 4> veiwTransforma(vec3 eye, vec3 roateEye) {
    return (translateXYZ(eye) * rotateZ(roateEye.z) * rotateY(roateEye.y) * rotateX(roateEye.x)).invert();
}

//正交投影变换矩阵 将观察坐标系的坐标变换到裁剪坐标系 View -> Clip. 裁剪空间中坐标轴的范围为 (-1,1). l,r,t,b,n,f 为视锥体的六个面矩形的中点坐标
mat<4, 4> orthoProjTransforma(double l, double r, double t, double b, double n, double f) {
    return { {{2 / (r - l),0,0,-(r + l) / (r - l)},{0,2 / (t - b),0,-(t + b) / (t - b)},{0,0, -2 / (n - f), (n + f) / (n - f)},{0,0,0,1}} };
}

//透视投影矩阵 观察坐标系的坐标变换到裁剪坐标系 View -> Clip. 此矩阵计算出来的齐次坐标的w分量不再为1，用于透视除法. l,r,t,b,n,f 为视锥体的六个面矩形的中点坐标
//此变换会改变坐标的手性(右手坐标系 -> 左手坐标系)
mat<4, 4> persProjTransforma(double r, double l, double t, double b, double n, double f) {
    mat<4, 4> mat = { {{(2 * n) / (r - l),0,0,0},{0,(2 * n) / (t - b),0,0},{0,0,(n + f) / (f - n),(-2 * n * f) / (f - n)},{0,0,1,0}} };
    mat = mat * -1;
    return mat;
}

//透视投影矩阵 使用宽高比，俯仰角， 远、近平面的距离
//此变换会改变坐标的手性(右手坐标系 -> 左手坐标系)
mat<4, 4> persProjTransforma(double aspect, double fov, double zNear, double zFar) {
    return { {{1 / (aspect * tan(fov / 2)),0,0,0},{0,1 / tan(fov / 2),0,0},{0,0,-(zFar + zNear) / (zFar - zNear),(-2 * zNear * zFar) / (zFar - zNear)},{0,0,-1,0}} };
}

//视口变换矩阵
mat<4, 4> viewportTransforma(int width, int height) {
    return { {{(double)width / 2,0,0,(double)width / 2},{0,(double)height / 2,0,(double)height / 2},{0,0,1,0},{0,0,0,1}} };
}

/*
 使用纹理渲染三角面，边界盒测试法
 有平行光照，使用平滑着色法的Gouraud着色法(即对每个像素点的光强进行插值计算)
 并且对像素点进行Zbuffer测试
*/
void drawTriangle_boundingbox_Gouraud(vec4 clip_verts[3], std::vector<double>& zbuffer, TGAImage& image, vec2 uvs[3], TGAImage& texture, vec3 norms[3], vec3 lightdir) {
    //齐次除法,转换为 NDC (标准化设备坐标)
    vec4 ndc_verts[3] = { clip_verts[0] / clip_verts[0][3], clip_verts[1] / clip_verts[1][3], clip_verts[2] / clip_verts[2][3] };
    //此处三角形的某个顶点超出视锥体则整个三角形将被裁剪。
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double tmp = ndc_verts[i][j];
            if (tmp < -1 || tmp > 1)return;
        }
    }
    //视口转换 转换为视口坐标
    mat<4, 4> invert = { {{1,0,0,0},{0,-1,0,0},{0,0,1,0},{0,0,0,1}} }; //图片坐标系y轴反向
    mat<4, 4> viewport = viewportTransforma(image.width(), image.height())* invert;
    vec4 viewport_verts[3] = { viewport * ndc_verts[0], viewport * ndc_verts[1],  viewport * ndc_verts[2] };
   
    //测试盒的边界
    vec2 boundingBoxMin(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    vec2 boundingBoxMax(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
    vec2 clamp(image.width() - 1, image.height() - 1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            boundingBoxMin[j] = std::max(0.,       std::min(boundingBoxMin[j], viewport_verts[i][j]));
            boundingBoxMax[j] = std::min(clamp[j], std::max(boundingBoxMax[j], viewport_verts[i][j]));
       }
    }

    //3个顶点光照强度
    vec3 intensity_vec(norms[0] * lightdir, norms[1] * lightdir, norms[2] * lightdir);
    intensity_vec = intensity_vec * -1;

    vec3 bc_viewport;
    for (int x = boundingBoxMin.x; x < boundingBoxMax.x; x++) {
        for (int y = boundingBoxMin.y; y < boundingBoxMax.y; y++) {
            //屏幕坐标的重心坐标
            bc_viewport = barycentric(proj<2>(viewport_verts[0]), proj<2>(viewport_verts[1]), proj<2>(viewport_verts[2]), vec2(x, y));
            //若点的重心坐标的某分量小于0，则表示该点不在三角形内
            if (bc_viewport.x < 0 || bc_viewport.y < 0 || bc_viewport.z < 0) {
                continue;
            }
            //当前点的Z坐标, 使用重心坐标乘以各个顶点的Z， 即使用各个顶点Z插值
            double depth = vec3(viewport_verts[0][2], viewport_verts[1][2], viewport_verts[2][2]) * bc_viewport;
            //进行深度测试，重合的像素点Z值大的(左手坐标系)不渲染
            int idx = x + y * image.width();
            if (zbuffer[idx] < depth) {
                continue;
            }
            zbuffer[idx] = depth;

            //插值计算当前像素的光照强度
            double intensity = std::max(0., bc_viewport * intensity_vec);

            //线性插值计算点的uv坐标
            mat<2, 3> m2 = {};
            m2.set_col(0, uvs[0]);
            m2.set_col(1, uvs[1]);
            m2.set_col(2, uvs[2]);
            vec2 bc_uv = m2 * bc_viewport;
            //材质uv坐标的像素颜色
            std::uint8_t* bgra = texture.get(bc_uv.x * texture.width(), bc_uv.y * texture.height()).bgra;
            //像素颜色分量乘光照强度
            image.set(x, y, TGAColor(bgra[2] * intensity, bgra[1] * intensity, bgra[0] * intensity, bgra[3]));

        }
    }

}

/*
 使用纹理渲染三角面，边界盒测试法
 有平行光照，使用平滑着色法的Phong着色法(即对每个像素点的法向量进行插值计算)
 并且对像素点进行Zbuffer测试
*/
void drawTriangle_boundingbox_Phong(vec4 clip_verts[3], std::vector<double>& zbuffer, TGAImage& image, vec2 uvs[3], TGAImage& texture, vec3 norms[3], vec3 lightdir) {

    //齐次除法,转换为 NDC (标准化设备坐标)
    vec4 ndc_verts[3] = { clip_verts[0] / clip_verts[0][3], clip_verts[1] / clip_verts[1][3], clip_verts[2] / clip_verts[2][3] };
    //此处三角形的某个顶点超出视锥体则整个三角形将被裁剪。
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double tmp = ndc_verts[i][j];
            if (tmp < -1 || tmp > 1)return;
        }
    }
    //视口转换 转换为视口坐标
    mat<4, 4> invert = { {{1,0,0,0},{0,-1,0,0},{0,0,1,0},{0,0,0,1}} }; //图片坐标系y轴反向
    mat<4, 4> viewport = viewportTransforma(image.width(), image.height()) * invert;
    vec4 viewport_verts[3] = { viewport * ndc_verts[0], viewport * ndc_verts[1],  viewport * ndc_verts[2] };

    //测试盒的边界
    vec2 boundingBoxMin(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    vec2 boundingBoxMax(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
    vec2 clamp(image.width() - 1, image.height() - 1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            boundingBoxMin[j] = std::max(0., std::min(boundingBoxMin[j], viewport_verts[i][j]));
            boundingBoxMax[j] = std::min(clamp[j], std::max(boundingBoxMax[j], viewport_verts[i][j]));
        }
    }

    vec3 bc_viewport;
    for (int x = boundingBoxMin.x; x < boundingBoxMax.x; x++) {
        for (int y = boundingBoxMin.y; y < boundingBoxMax.y; y++) {
            //屏幕坐标的重心坐标
            bc_viewport = barycentric(proj<2>(viewport_verts[0]), proj<2>(viewport_verts[1]), proj<2>(viewport_verts[2]), vec2(x, y));
            //若点的重心坐标的某分量小于0，则表示该点不在三角形内
            if (bc_viewport.x < 0 || bc_viewport.y < 0 || bc_viewport.z < 0) {
                continue;
            }
            //当前点的Z坐标, 使用重心坐标乘以各个顶点的Z， 即使用各个顶点Z插值
            double depth = vec3(viewport_verts[0][2], viewport_verts[1][2], viewport_verts[2][2]) * bc_viewport;
            //进行深度测试，重合的像素点Z值大的(左手坐标系)不渲染
            int idx = x + y * image.width();
            if (zbuffer[idx] < depth) {
                continue;
            }
            zbuffer[idx] = depth;

            //线性插值计算点的法向量
            mat<3, 3> m = {};
            m.set_col(0, norms[0]);
            m.set_col(1, norms[1]);
            m.set_col(2, norms[2]);
            vec3 n = m * bc_viewport;
            //计算光照强度
            float intensity = std::max(0., -1 * n * lightdir);
   
            //线性插值计算点的uv坐标
            mat<2, 3> m2 = {};
            m2.set_col(0, uvs[0]);
            m2.set_col(1, uvs[1]);
            m2.set_col(2, uvs[2]);
            vec2 bc_uv = m2 * bc_viewport;
            //材质uv坐标的像素颜色
            std::uint8_t* bgra = texture.get(bc_uv.x * texture.width(), bc_uv.y * texture.height()).bgra;
            //像素颜色分量乘光照强度
            image.set(x, y, TGAColor(bgra[2] * intensity, bgra[1] * intensity, bgra[0] * intensity, bgra[3]));

        }
    }

}

//模型的平面着色渲染
void FlatshadingRendering(Model* model, vec3 model_p, vec3 model_r, vec3 model_s, vec3 eye_p, vec3 eye_r, double fov, double near, double far, TGAImage& image, vec3 lightdir, TGAImage& texture) {
    //创建Z缓存用于深度测试
    std::vector<double> zbuffer(image.width() * image.height(), std::numeric_limits<double>::max());
    vec4 clip_coords[3]; //裁剪空间坐标 x,y,z => [-1,1]
    vec2 texture_coords[3];
    vec3 vert_norms[3];
    //渲染模型的每个平面
    mat<4, 4> modelMat=  modelTransforma(model_p, model_r, model_s);
    mat<4, 4> viewMat = veiwTransforma(eye_p, eye_r);
    //平行光方向的视图空间坐标
    vec3 uniform_light = proj<3>(viewMat * embed<4>(lightdir, 0.)).normalize();
    double aspect = (double)image.width() / image.height();
    double t = near * tan(fov / 2);
    double b = -t;
    double r = t * aspect;
    double l = -r;
    mat<4, 4> projectionMat = persProjTransforma(aspect, fov, near, far);
    for (int i = 0; i < model->nfaces(); i++) {
        for (int j = 0; j < 3; j++) {
            //顶点的裁剪空间坐标 模型，视图，投影变换 
            clip_coords[j] = projectionMat * viewMat * modelMat * embed<4>(model->vert(i, j));
            //顶点的纹理坐标
            texture_coords[j] = model->uv(i, j);
            //顶点的视图空间法向量， 模型，视图变换
            vert_norms[j] = proj<3>(viewMat * modelMat * embed<4>(model->normal(i, j), 0.));
        }
        drawTriangle_boundingbox_Phong(clip_coords, zbuffer, image, texture_coords, texture, vert_norms, uniform_light);
   
    }
}


int main(int argc, char** argv)
{
	cout << "Version" << VERSION_MAJOR << "." << VERSION_MINOR << endl;
    char* fileName;
    const char * configFilePath = "config";
    string modelPath, texturePath, outputWidth, outputHeight, modelPosition, modelRotation, lightdirStr, cameraPosition, cameraRotation, fov, near, far;
    readConfigFile(configFilePath, "model-path", modelPath);
    readConfigFile(configFilePath, "texture-path", texturePath);
    readConfigFile(configFilePath, "output-image-width", outputWidth);
    readConfigFile(configFilePath, "output-image-height", outputHeight);
    readConfigFile(configFilePath, "model-position", modelPosition);
    readConfigFile(configFilePath, "model-rotation", modelRotation);
    readConfigFile(configFilePath, "light-direction", lightdirStr);
    readConfigFile(configFilePath, "camera-position", cameraPosition);
    readConfigFile(configFilePath, "camera-rotation", cameraRotation);
    readConfigFile(configFilePath, "frustum-fov", fov);
    readConfigFile(configFilePath, "frustum-near", near);
    readConfigFile(configFilePath, "frustum-far", far);

    //载入模型
	Model* model = new Model(modelPath.c_str());
    TGAImage texture{};
    //载入模型材质
    texture.read_tga_file(texturePath.c_str());
    //设置输出图片
    TGAImage image(atoi(outputWidth.c_str()), atoi(outputHeight.c_str()), TGAImage::RGB);
    //渲染
    FlatshadingRendering(model, getVec(modelPosition), getVec(modelRotation), vec3(1, 1, 1), getVec(cameraPosition), getVec(cameraRotation), atof(fov.c_str()), atof(near.c_str()), atof(far.c_str()), image, getVec(lightdirStr), texture);
    image.flip_vertically();
    //导出帧
    image.write_tga_file("output.tga");
    delete model;
	return 0;
}
