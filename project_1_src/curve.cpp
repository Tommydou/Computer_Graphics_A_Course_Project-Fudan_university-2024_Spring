#include "curve.h"
#include "vertexrecorder.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(const Vector3f& lhs, const Vector3f& rhs)
{
	const float eps = 1e-8f;
	return (lhs - rhs).absSquared() < eps;
}


}


Curve evalBezier(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		std::cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	// TODO:
	// You should implement this function so that it returns a Curve
	// (e.g., a vector< CurvePoint >).  The variable "steps" tells you
	// the number of points to generate on each piece of the spline.
	// At least, that's how the sample solution is implemented and how
	// the SWP files are written.  But you are free to interpret this
	// variable however you want, so long as you can control the
	// "resolution" of the discretized spline curve with it.

	// Make sure that this function computes all the appropriate
	// Vector3fs for each CurvePoint: V,T,N,B.
	// [NBT] should be unit and orthogonal.

	// Also note that you may assume that all Bezier curves that you
	// receive have G1 continuity.  Otherwise, the TNB will not be
	// be defined at points where this does not hold.
	cerr << "\t>>> evalBezier has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;


	cerr << "\t>>> Steps (type steps): " << steps << endl;

	const Matrix4f M_BEZ(
		1,-3,3,-1,
		0,3,-6,3,
		0,0,3,-3,
		0,0,0,1
	);
	// requires refresh every iteration
	Vector4f T(0,0,0,0);
	Matrix4f G_BEZ;
	// // output 
	Vector3f B= Vector3f(0.0,0.0,1.0).normalized();

	int sector_count = (P.size()-1)/3;
	Curve R(steps*sector_count + 1);
	float t;
	for(unsigned sector = 0 ; sector < sector_count; sector++){

		for (unsigned i = 0; i <= steps; ++i)
		{
			if (i == steps && sector!= (sector_count-1)){
				continue;
			}
			// closed interval on the left, open interval on the right
			t = i *(1.0f/steps);
			

			G_BEZ= Matrix4f(
				Vector4f(P[3*sector],0),
				Vector4f(P[3*sector+1],0),
				Vector4f(P[3*sector+2],0),
				Vector4f(P[3*sector+3],0)
			);
			T = Vector4f(
				1.0f,t,t*t,t*t*t
			);
			R[sector*steps+i].V = (G_BEZ*(M_BEZ*T)).xyz();
			T = Vector4f(
				0,1.0f,2.0f*t,3.0f*t*t
			);
			R[sector*steps+i].T = (G_BEZ*(M_BEZ*T)).xyz().normalized();
			// B= Vector3f::cross(Vector3f(0,0,1),R[sector*steps+i].T).normalized();
			if(sector == 0&& i == 0){				
				// B= Vector3f::cross(Vector3f(0,0,1),R[0].T).normalized();

				R[sector*steps+i].N = Vector3f::cross(B,R[sector*steps+i].T).normalized();
			}
			else{
				R[sector*steps+i].N = (Vector3f::cross(R[sector*steps+i-1].B,R[sector*steps+i].T)).normalized();
			}
			R[sector*steps+i].B =( Vector3f::cross(R[sector*steps+i].T,R[sector*steps+i].N)).normalized();


		}

	}

	for (int i = 0; i < (int)R.size(); ++i)
	{
		cerr << "\t>>> " << R[i].V.z() << endl;
	}
	cerr << "\t>>> Steps_bezier (type steps): " << steps << endl;
	return R;

	// Right now this will just return this empty curve.
	return Curve();
}


Curve evalBezier_Bsp(const vector< Vector3f >& P, unsigned steps,Vector3f B= Vector3f(0.0,0.0,1.0))
{
	// Check
	
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		std::cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	// TODO:
	// You should implement this function so that it returns a Curve
	// (e.g., a vector< CurvePoint >).  The variable "steps" tells you
	// the number of points to generate on each piece of the spline.
	// At least, that's how the sample solution is implemented and how
	// the SWP files are written.  But you are free to interpret this
	// variable however you want, so long as you can control the
	// "resolution" of the discretized spline curve with it.

	// Make sure that this function computes all the appropriate
	// Vector3fs for each CurvePoint: V,T,N,B.
	// [NBT] should be unit and orthogonal.

	// Also note that you may assume that all Bezier curves that you
	// receive have G1 continuity.  Otherwise, the TNB will not be
	// be defined at points where this does not hold.
	cerr << "\t>>> evalBezier has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;


	cerr << "\t>>> Steps (type steps): " << steps << endl;

	const Matrix4f M_BEZ(
		1,-3,3,-1,
		0,3,-6,3,
		0,0,3,-3,
		0,0,0,1
	);
	// requires refresh every iteration
	Vector4f T(0,0,0,0);
	Matrix4f G_BEZ;
	// // output 
	// Vector3f B= Vector3f(0.0,0.0,1.0).normalized();

	int sector_count = (P.size()-1)/3;
	Curve R(steps*sector_count + 1);
	float t;
	for(unsigned sector = 0 ; sector < sector_count; sector++){

		for (unsigned i = 0; i <= steps; ++i)
		{
			if (i == steps && sector!= (sector_count-1)){
				continue;
			}
			// closed interval on the left, open interval on the right
			t = i *(1.0f/steps);
			

			G_BEZ= Matrix4f(
				Vector4f(P[3*sector],0),
				Vector4f(P[3*sector+1],0),
				Vector4f(P[3*sector+2],0),
				Vector4f(P[3*sector+3],0)
			);
			T = Vector4f(
				1.0f,t,t*t,t*t*t
			);
			R[sector*steps+i].V = (G_BEZ*(M_BEZ*T)).xyz();
			T = Vector4f(
				0,1.0f,2.0f*t,3.0f*t*t
			);
			R[sector*steps+i].T = (G_BEZ*(M_BEZ*T)).xyz().normalized();
			// B= Vector3f::cross(Vector3f(0,0,1),R[sector*steps+i].T).normalized();
			if(sector == 0&& i == 0){				
				// B= Vector3f::cross(Vector3f(0,0,1),R[0].T).normalized();

				R[sector*steps+i].N = Vector3f::cross(B,R[sector*steps+i].T).normalized();
			}
			else{
				R[sector*steps+i].N = (Vector3f::cross(R[sector*steps+i-1].B,R[sector*steps+i].T)).normalized();
			}
			R[sector*steps+i].B =( Vector3f::cross(R[sector*steps+i].T,R[sector*steps+i].N)).normalized();


		}

	}

	for (int i = 0; i < (int)R.size(); ++i)
	{
		cerr << "\t>>> " << R[i].V.z() << endl;
	}
	cerr << "\t>>> Steps_bezier (type steps): " << steps << endl;
	return R;

	// Right now this will just return this empty curve.
	return Curve();
}



Curve evalBspline(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4)
	{
		cerr << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
	}

	// TODO:
	// It is suggested that you implement this function by changing
	// basis from B-spline to Bezier.  That way, you can just call
	// your evalBezier function.


	cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	for (int i = 0; i < (int)P.size(); ++i)
	{
		cerr << "\t>>> " << P[i].x() << endl;
	}


	int sector_count = (P.size()-1)/3;
	Matrix4f G_BEZ;
	const Matrix4f M_B = (1.0f/6.0f) *Matrix4f(
								1.0f,-3.0f,3.0f,-1.0f,
								4.0f,0.0f,-6.0f,3.0f,
								1.0f,3.0f,3.0f,-3.0f,
								0.0f,0.0f,0.0f,1.0f
								);
	const Matrix4f M_B_IN = Matrix4f(
								1.0f,1.0f,1.0f,1.0f,
								0,1.0f/3.0f,2.0f/3.0f,1.0f,
								0,0,1.0f/3.0f,1,
								0,0,0,1
								);
	vector<Vector3f> buffer;

	Curve R(steps*sector_count+1);
	R.clear();
	Curve R_buf(steps+1);
	Vector3f B =Vector3f(0.0,0.0,1.0);
	
	for(unsigned sector = 0 ; sector < P.size()-3; sector++){
		G_BEZ= Matrix4f(
				Vector4f(P[sector],0),
				Vector4f(P[sector+1],0),
				Vector4f(P[sector+2],0),
				Vector4f(P[sector+3],0)
			);
		G_BEZ = G_BEZ*M_B*M_B_IN;
		cerr << "\t>>> G_BEZ content" << G_BEZ.getRow(0).x() << endl;
		
		cerr << "\t>>> G_BEZ content" << G_BEZ.getRow(0).y() << endl;
		
		cerr << "\t>>> G_BEZ content" << G_BEZ.getRow(0).z() << endl;
		buffer.clear();
		for(int i = 0; i < 4;i++){
			buffer.push_back(G_BEZ.getCol(i).xyz());
		}
		if(sector!=0){
			B = R[sector*steps-1].B;
		}
		
		R_buf = evalBezier_Bsp(buffer,steps,B);
		

		cerr<<"\t>>> R_buf"<<R_buf[1].B.x()<<endl;
		
		cerr<<"\t>>> R_buf"<<R_buf[1].B.y()<<endl;
		
		cerr<<"\t>>> R_buf_Z"<<R_buf[1].B.z()<<endl;
		// if(sector == P.size()-4){
		if(sector == P.size()-4){
			for( int i =0 ; i <= steps;i++){
				R.push_back( R_buf[i]);
			}
		}
		else{
		
			for( int i =0 ; i < steps;i++){
				R.push_back( R_buf[i]);
			}
		}
		// R[0{
		
		// 	for( int i =0 ; i < steps;i++){
		// 		R.push_back( R_buf[i]);
		// 	}
		// }
		// R[0].N = (R[R.size()-1].B*R[0].T).normalized();
		// else{
		// 	for( int i =0 ; i < steps;i++){
		// 		R.push_back( R_buf[i]);
		// 	}
		// }

	}
	cerr << "\t>>> R size" << R.size()<< endl;
	cerr << "\t>>> Returning empty curve." << endl;
	return R;


	return Curve();
}

Curve evalCircle(float radius, unsigned steps)
{
	// This is a sample function on how to properly initialize a Curve
	// (which is a vector< CurvePoint >).

	// Preallocate a curve with steps+1 CurvePoints
	Curve R(steps + 1);

	// Fill it in counterclockwise
	for (unsigned i = 0; i <= steps; ++i)
	{
		// step from 0 to 2pi
		float t = 2.0f * c_pi * float(i) / steps;

		// Initialize position
		// We're pivoting counterclockwise around the y-axis
		R[i].V = radius * Vector3f(cos(t), sin(t), 0);

		// Tangent vector is first derivative
		R[i].T = Vector3f(-sin(t), cos(t), 0);

		// Normal vector is second derivative
		R[i].N = Vector3f(-cos(t), -sin(t), 0);

		// Finally, binormal is facing up.
		R[i].B = Vector3f(0, 0, 1);
	}

	return R;
}

void recordCurve(const Curve& curve, VertexRecorder* recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve& curve, VertexRecorder* recorder, float framesize)
{  
	Matrix4f T;
	const Vector3f RED(1, 0, 0);
	const Vector3f GREEN(0, 1, 0);
	const Vector3f BLUE(0, 0, 1);
	
	const Vector4f ORGN(0, 0, 0, 1);
	const Vector4f AXISX(framesize, 0, 0, 1);
	const Vector4f AXISY(0, framesize, 0, 1);
	const Vector4f AXISZ(0, 0, framesize, 1);

	for (int i = 0; i < (int)curve.size(); ++i)
	{
		T.setCol(0, Vector4f(curve[i].N, 0));
		T.setCol(1, Vector4f(curve[i].B, 0));
		T.setCol(2, Vector4f(curve[i].T, 0));
		T.setCol(3, Vector4f(curve[i].V, 1));
 
		// Transform orthogonal frames into model space
		Vector4f MORGN  = T * ORGN;
		Vector4f MAXISX = T * AXISX;
		Vector4f MAXISY = T * AXISY;
		Vector4f MAXISZ = T * AXISZ;

		// Record in model space
		recorder->record_poscolor(MORGN.xyz(), RED);
		recorder->record_poscolor(MAXISX.xyz(), RED);

		recorder->record_poscolor(MORGN.xyz(), GREEN);
		recorder->record_poscolor(MAXISY.xyz(), GREEN);

		recorder->record_poscolor(MORGN.xyz(), BLUE);
		recorder->record_poscolor(MAXISZ.xyz(), BLUE);
	}
}
