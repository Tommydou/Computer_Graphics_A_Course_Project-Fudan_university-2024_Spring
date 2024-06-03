#include "surf.h"
#include "vertexrecorder.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;
namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }
}

// DEBUG HELPER
Surface quad() { 
	Surface ret;
	ret.VV.push_back(Vector3f(-1, -1, 0));
	ret.VV.push_back(Vector3f(+1, -1, 0));
	ret.VV.push_back(Vector3f(+1, +1, 0));
	ret.VV.push_back(Vector3f(-1, +1, 0));

	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));

	ret.VF.push_back(Tup3u(0, 1, 2));
	ret.VF.push_back(Tup3u(0, 2, 3));
	return ret;
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    surface = quad();
    std::cout << "SurfRev" << std::endl;
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.

    //my code
    vector<Vector3f> VV;
    vector<Vector3f> VN;
    vector<Tup3u> VF;

    for(unsigned i=0; i<steps; i++) // 遍历生成曲面
    {
        float t = (float)i / (float)steps * 2*c_pi;  // 将循环变量 i 归一化到 [0, 2π] 范围内，以进行旋转操作。
        Matrix4f M(cos(t),0,sin(t),0,0,1,0,0,-sin(t),0,cos(t),0,0,0,0,1);

        for(unsigned j=0; j<profile.size(); j++) // 遍历曲线
        {
            Vector4f V(profile[j].V[0],profile[j].V[1],profile[j].V[2],1);
            V = M*V;
            Vector4f N(profile[j].N[0],profile[j].N[1],profile[j].N[2],1);
            N = ( M.inverse().transposed()* N *(-1));
            N = N.normalized();

            VV.push_back(Vector3f(V[0],V[1],V[2])); // 将旋转后的顶点坐标加入到顶点向量 VV 中
            VN.push_back(Vector3f(N[0],N[1],N[2])); // 将旋转后的法向量加入到法向量向量 VN 中
        }
    }

    for(unsigned i=0; i<steps; i++) // 生成曲面的三角形面片
    {
    	for(unsigned j=0; j < profile.size()-1; j++)
            {
                VF.push_back(Tup3u(i*profile.size()+(j+1)%profile.size(), ((i+1)%steps)*profile.size()+j, i*profile.size()+j )); // 左上三角形
                VF.push_back(Tup3u(((i+1)%steps)*profile.size()+(j+1)%profile.size(), ((i+1)%steps)*profile.size()+j, i*profile.size()+(j+1)%profile.size())); // 右下三角形
            }
    }

    surface.VV = VV;
    surface.VN = VN;
    surface.VF = VF;

    return surface;

    cerr << "\t>>> makeSurfRev called (but not implemented).\n\t>>> Returning empty surface." << endl;
 
    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;
	surface = quad();

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.

    //my code
    vector<Vector3f> VV;
    vector<Vector3f> VN;
    vector<Tup3u> VF;
    Curve new_sweep = sweep;

    if(sweep[0].N != sweep[sweep.size()-1].N) // 如果不是闭合曲线，使用插值法
    {  
   		float alpha = acos(sweep[0].N.dot(sweep[0].N,sweep[sweep.size()-1].N) / (sweep[0].N.abs() * sweep[sweep.size()-1].N.abs())); // 计算首尾法线夹角
   		float dir = sweep[0].N.dot(sweep[sweep.size()-1].N,sweep[0].B);

   		for(unsigned i=0; i < sweep.size(); i++)
        {	
         	float surf_size = i;
         	float theta = alpha * surf_size / (sweep.size()-1);
         	Vector3f N_ ;
         	Vector3f B_ ;
         		
         	if(dir < 0){
         		N_ = cos(theta)*sweep[i].N + sin(theta)*sweep[i].B;
         		B_ = cos(theta)*sweep[i].B - sin(theta)*sweep[i].N;
         	}
         	else{
         		N_ = cos(theta)*sweep[i].N - sin(theta)*sweep[i].B;
         		B_ = cos(theta)*sweep[i].B + sin(theta)*sweep[i].N;	
         	}
         	
         	new_sweep[i].N = N_;
         	new_sweep[i].B = B_;
         	
        }
   	}


    unsigned steps = profile.size();

    for(unsigned i=0; i<new_sweep.size(); i++) // 遍历曲线
    {
	    Matrix4f M(new_sweep[i].N[0], new_sweep[i].B[0], new_sweep[i].T[0], new_sweep[i].V[0], 
                    new_sweep[i].N[1], new_sweep[i].B[1], new_sweep[i].T[1], new_sweep[i].V[1],
                    new_sweep[i].N[2], new_sweep[i].B[2], new_sweep[i].T[2], new_sweep[i].V[2], 0, 0, 0, 1);

        for(unsigned j=0; j < steps; j++)
        {
            Vector4f V(profile[j].V[0],profile[j].V[1],profile[j].V[2],1);
            V = M*V;
            Vector4f N(profile[j].N[0],profile[j].N[1],profile[j].N[2],1);
             N = ( M.inverse().transposed()* N *(-1));
            N = N.normalized();

            VV.push_back(Vector3f(V[0],V[1],V[2]));
            VN.push_back(Vector3f(N[0],N[1],N[2]));
        }
    }

    for(unsigned i=0; i < sweep.size(); i++) // 生成曲面的三角形面片
    {
    	for(unsigned j=0; j < steps; j++)
            {
                VF.push_back(Tup3u(i*steps+(j+1)%steps, ((i+1)%sweep.size())*steps+j, i*steps+j )); // 左上
                VF.push_back(Tup3u(((i+1)%sweep.size())*steps+(j+1)%steps, ((i+1)%sweep.size())*steps+j, i*steps+(j+1)%steps)); // 右下
            }
    }

    surface.VV = VV;
    surface.VN = VN;
    surface.VF = VF;
    return surface;
    

    cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." <<endl;

    return surface;
}

void recordSurface(const Surface &surface, VertexRecorder* recorder) {
	const Vector3f WIRECOLOR(0.4f, 0.4f, 0.4f);
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
		recorder->record(surface.VV[surface.VF[i][0]], surface.VN[surface.VF[i][0]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][1]], surface.VN[surface.VF[i][1]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][2]], surface.VN[surface.VF[i][2]], WIRECOLOR);
    }
}

void recordNormals(const Surface &surface, VertexRecorder* recorder, float len)
{
	const Vector3f NORMALCOLOR(0, 1, 1);
    for (int i=0; i<(int)surface.VV.size(); i++)
    {
		recorder->record_poscolor(surface.VV[i], NORMALCOLOR);
		recorder->record_poscolor(surface.VV[i] + surface.VN[i] * len, NORMALCOLOR);
    }
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (int i=0; i<(int)surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (int i=0; i<(int)surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
