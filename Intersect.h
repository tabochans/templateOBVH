#pragma once

#include"Vec3.h"
#include"Vec2.h"
#include<vector>
#include<algorithm>

namespace Primitive{


	struct Sphere{

		PTUtility::Vec3 m_Pos;
		float m_Radius;

		static bool compPX(const Sphere* left, const Sphere* right){
			return left->m_Pos.x() - right->m_Pos.x() < 0;
		}
		static bool compPY(const Sphere* left, const Sphere* right){
			return left->m_Pos.y() - right->m_Pos.y() < 0;
		}
		static bool compPZ(const Sphere* left, const Sphere* right){
			return left->m_Pos.z() - right->m_Pos.z() < 0;
		}

		Sphere(const PTUtility::Vec3& Center, float R) : m_Pos(Center), m_Radius(R){
		}
		Sphere() : m_Pos(PTUtility::Vec3::Zero()), m_Radius(0.5f){
		}
		virtual ~Sphere(){
		}
	};

	struct Ray{

	public:

		PTUtility::Vec3 m_Org;
		PTUtility::Vec3 m_Dir;
		float m_Index;

		Ray operator-()const{
			return Ray(this->m_Org - 0.0001 * this->m_Dir, -this->m_Dir, this->m_Index);
		}

		static PTUtility::Vec3 ReflectRay(const Ray& in, const PTUtility::Vec3& normal){
			return (in.m_Dir - 2.0f * (normal.dot(in.m_Dir)) * normal).normalized();
		}

		Ray(const PTUtility::Vec3& Origin, const PTUtility::Vec3& Direction) : m_Org(Origin), m_Dir(Direction.normalized()){

		}
		Ray(const PTUtility::Vec3& Origin, const PTUtility::Vec3& Direction, float Index) : m_Org(Origin), m_Dir(Direction.normalized()), m_Index(Index){

		}
		~Ray(){

		}

	};

	struct Vertex{
		PTUtility::Vec3 m_Pos;
		PTUtility::Vec3 m_Normal;
		PTUtility::Vec2 m_UV;


		static bool compPX(const Vertex* left, const Vertex* right){
			return (left->m_Pos.x() - right->m_Pos.x()) < 0;
		}
		static bool compPY(const Vertex* left, const Vertex* right){
			return (left->m_Pos.y() - right->m_Pos.y()) < 0;
		}
		static bool compPZ(const Vertex* left, const Vertex* right){
			return (left->m_Pos.z() - right->m_Pos.z()) < 0;
		}

		
		explicit Vertex(){

		}
		explicit Vertex(const PTUtility::Vec3& Pos) : m_Pos(Pos), m_Normal(0,1,0),m_UV(0,0){

		}
		Vertex(const PTUtility::Vec3& Pos, const PTUtility::Vec3& Normal) : m_Pos(Pos), m_Normal(Normal.normalized()), m_UV(0, 0){

		}
		Vertex(const PTUtility::Vec3& Pos, const PTUtility::Vec3& Normal, const PTUtility::Vec2& UV) : m_Pos(Pos), m_Normal(Normal.normalized()), m_UV(UV){

		}

	};


	struct Polygon{
		std::vector<Vertex> m_V;

		PTUtility::Vec3 GetCenter()const{
			return 0.33333333 * (m_V[0].m_Pos + m_V[1].m_Pos + m_V[2].m_Pos);
		}

		static bool compPX(const Polygon* left, const Polygon* right){
			return (left->GetCenter().x() - right->GetCenter().x()) < 0;
		}
		static bool compPY(const Polygon* left, const Polygon* right){
			return (left->GetCenter().y() - right->GetCenter().y()) < 0;
		}
		static bool compPZ(const Polygon* left, const Polygon* right){
			return (left->GetCenter().z() - right->GetCenter().z()) < 0;
		}

		Polygon(const Vertex& v1, const Vertex& v2, const Vertex& v3) : m_V(3){
			m_V[0] = v1;
			m_V[1] = v2;
			m_V[2] = v3;
		}
		void CalNormal(){
			PTUtility::Vec3 N(0, 1, 0);
			PTUtility::Vec3 u = (m_V[2].m_Pos - m_V[0].m_Pos).normalized();
			PTUtility::Vec3 s = (m_V[1].m_Pos - m_V[0].m_Pos).normalized();

			N = u.cross(s).normalized();
			for (auto v : m_V){
				v.m_Normal = N;
			}

		}
		~Polygon(){

		}

	};

	struct AABB{

		PTUtility::Vec3 m_MinPos;
		PTUtility::Vec3 m_MaxPos;

		const PTUtility::Vec3 GetWidth(){
			return m_MaxPos - m_MinPos;
		}
		const PTUtility::Vec3 GetCenter(){
			return 0.500 * (m_MaxPos + m_MinPos);
		}

		//無効なAABBをつくる
		AABB() : m_MinPos(1000.001, 1000.001, 1000.001), m_MaxPos(-1000.001, -1000.001, -1000.001){}

		//最大と最小位置を指定
		AABB(const PTUtility::Vec3& max, const PTUtility::Vec3& min) : m_MinPos(min), m_MaxPos(max){}

		~AABB(){}



		template<typename T>
		void Expand(const T& obj);
		
		template<>
		void Expand<Polygon>(const Polygon& obj){
			
			float max[3], min[3];

			PTUtility::Vec3 p1(obj.m_V[0].m_Pos);
			PTUtility::Vec3 p2(obj.m_V[1].m_Pos);
			PTUtility::Vec3 p3(obj.m_V[2].m_Pos);

			//よくわからんがたぶん？あってる
			for (int i = 0; i < 3; i++){
				max[i] = (p1(i) > p2(i) ? p1(i) : p2(i)) > p3(i) ? (p1(i) > p2(i) ? p1(i) : p2(i)) : p3(i);
			}
			for (int i = 0; i < 3; i++){
				min[i] = (p1(i) < p2(i) ? p1(i) : p2(i)) < p3(i) ? (p1(i) < p2(i) ? p1(i) : p2(i)) : p3(i);
			}
			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = (max[i] + 0.001f > m_MaxPos[i] ? max[i] + 0.001f : m_MaxPos[i]);
				m_MinPos(i) = (min[i] - 0.001f < m_MinPos[i] ? min[i] - 0.001f : m_MinPos[i]);
			}
		}

		template<>
		void Expand<PTUtility::Vec3>(const PTUtility::Vec3& p){
			PTUtility::Vec3 max = m_MaxPos;
			PTUtility::Vec3 min = m_MinPos;

			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = max(i) > p(i) ? max(i) : p(i);
				m_MinPos(i) = min(i) < p(i) ? min(i) : p(i);
			}
		}

		template<>
		void Expand<Primitive::Vertex>(const Primitive::Vertex& p){
			Expand<PTUtility::Vec3>(p.m_Pos);
		}

		template<>
		void Expand<AABB>(const AABB& ref){
			PTUtility::Vec3 max = m_MaxPos;
			PTUtility::Vec3 min = m_MinPos;

			for (int i = 0; i < 3; i++){
				m_MaxPos(i) = max(i) > ref.m_MaxPos(i) ? max(i) : ref.m_MaxPos(i);
				m_MinPos(i) = min(i) < ref.m_MinPos(i) ? min(i) : ref.m_MinPos(i);
			}
		}

		//無効化する
		void Reset(){
			m_MaxPos = PTUtility::Vec3(-1000.001, -1000.001, -1000.001);
			m_MinPos = PTUtility::Vec3(1000.001, 1000.001, 1000.001);
		}

		int MaxDimension()const{
			PTUtility::Vec3 x = m_MaxPos - m_MinPos;
			if (x(0) > x(1)){
				if (x(0) > x(2)){ return 0; }
				else{ return 2; }
			}
			else{
				if (x(1) > x(2)){ return 1; }
				else{ return 2; }
			}
		}

		//表面積を得ます
		float GetArea()const{
			PTUtility::Vec3 x = m_MaxPos - m_MinPos;
			return abs(2.0f * (x(0)*x(1) + x(1)*x(2) + x(2)*x(0)));
		}


	};



}




namespace Isc{


	template<typename T, typename S>
	bool Intersect(const T&, const S&);

	template<typename T, typename S>
	bool Intersect(const T&, const S&, float& d);

	template<typename T, typename S>
	bool Intersect(const T&, const S&, float& d, float& u, float& v);

	template<typename T, typename S>
	bool Intersect(const T&, const S&, float& d, PTUtility::Vec3& Normal);



	template<>
	bool Intersect(const Primitive::Sphere& A, const Primitive::Sphere& B){
		if ((A.m_Pos- B.m_Pos).norm2() < (A.m_Radius *B.m_Radius)*(A.m_Radius*B.m_Radius)){
			return true;
		}
		return false;
	}

	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::Sphere& s, float& d){
		PTUtility::Vec3 ss = ray.m_Org - s.m_Pos;

		double A = 1.0f;
		double B = 2.0 * ss.dot(ray.m_Dir);
		double C = ss.norm2() - s.m_Radius*s.m_Radius;

		double D = B*B - 4.0 * A*C;

		if (D > 0.00001){

			if ((s.m_Pos - ray.m_Org).norm2() < s.m_Radius*s.m_Radius){
				d = 0.5 * (-B + sqrt(D));
			}
			else{
				d = 0.5 * (-B - sqrt(D));
			}
			return true;
		}


		return false;
	}

	template<>
	bool Intersect<Primitive::Sphere, Primitive::Ray>(const Primitive::Sphere& s, const Primitive::Ray& ray, float& d){
		return Intersect(ray, s, d);
	}


	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::Polygon& p, float& d, float& u, float& v){
		

		const PTUtility::Vec3& dd = ray.m_Dir;
		const PTUtility::Vec3& e1 = (p.m_V[1].m_Pos - p.m_V[0].m_Pos);
		const PTUtility::Vec3& e2 = (p.m_V[2].m_Pos - p.m_V[0].m_Pos);
		const PTUtility::Vec3& ps = dd.cross(e2);

		float a = e1.dot(ps);
		if (a > -FLT_MIN && a < FLT_MIN) { u = v = d = 0.0; return false; }
		float f = 1.0f / a;
		PTUtility::Vec3 s = ray.m_Org - p.m_V[0].m_Pos;
		u = f * s.dot(ps);
		if (u < -0.01 || u > 1.0 + 0.01) { u = v = d = 0.0; return false; }

		PTUtility::Vec3 q = s.cross(e1);
		v = f * dd.dot(q);
		if (v < -0.01 || u + v > 1.0 + 0.01) { u = v = d = 0.0; return false; }

		d = f * e2.dot(q);

		return true;
	}

	template<>
	bool Intersect(const Primitive::Polygon& p, const Primitive::Ray& ray, float& d, float& u, float& v){
		return Intersect(ray, p, d, u, v);
	}

	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Ray& ray){

		float tmax, tmin;
		const PTUtility::Vec3& max = aabb.m_MaxPos;
		const PTUtility::Vec3& min = aabb.m_MinPos;

		if (abs(ray.m_Dir.x()) < 0.0001){
			tmax = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (max.x() - ray.m_Org.x());
			tmin = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (min.x() - ray.m_Org.x());
		}
		else{
			tmax = (max.x() - ray.m_Org.x()) / ray.m_Dir.x();
			tmin = (min.x() - ray.m_Org.x()) / ray.m_Dir.x();
		}
		if (tmax < tmin){
			std::swap(tmax, tmin);
		}

		float tymin, tymax;
		if (abs(ray.m_Dir.y()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (max.y() - ray.m_Org.y());
			tymin = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (min.y() - ray.m_Org.y());
		}
		else{
			tymax = (max.y() - ray.m_Org.y()) / ray.m_Dir.y();
			tymin = (min.y() - ray.m_Org.y()) / ray.m_Dir.y();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;

		if (abs(ray.m_Dir.z()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (max.z() - ray.m_Org.z());
			tymin = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (min.z() - ray.m_Org.z());
		}
		else{
			tymax = (max.z() - ray.m_Org.z()) / ray.m_Dir.z();
			tymin = (min.z() - ray.m_Org.z()) / ray.m_Dir.z();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;

		//if (tmin < FLT_MIN){ return false; }

		return true;
	}
	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::AABB& aabb){
		return Intersect(aabb, ray);
	}

	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Ray& ray, float& t, PTUtility::Vec3& Normal){

		float mm[3];
		float MM[3];


		float tmax, tmin;
		const PTUtility::Vec3& max = aabb.m_MaxPos;
		const PTUtility::Vec3& min = aabb.m_MinPos;

		if (abs(ray.m_Dir.x()) < 0.0001){
			tmax = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (max.x() - ray.m_Org.x());
			tmin = (2.0f * (ray.m_Dir.x() > 0) - 1) * 10000.0f * (min.x() - ray.m_Org.x());
		}
		else{
			tmax = (max.x() - ray.m_Org.x()) / ray.m_Dir.x();
			tmin = (min.x() - ray.m_Org.x()) / ray.m_Dir.x();
		}
		if (tmax < tmin){
			std::swap(tmax, tmin);
		}
		MM[0] = tmax;
		mm[0] = tmin;



		float tymin, tymax;
		if (abs(ray.m_Dir.y()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (max.y() - ray.m_Org.y());
			tymin = (2.0f * (ray.m_Dir.y() > 0) - 1) * 10000.0f * (min.y() - ray.m_Org.y());
		}
		else{
			tymax = (max.y() - ray.m_Org.y()) / ray.m_Dir.y();
			tymin = (min.y() - ray.m_Org.y()) / ray.m_Dir.y();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}
		MM[1] = tmax;
		mm[1] = tmin;

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;

		if (abs(ray.m_Dir.z()) < 0.0001){
			tymax = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (max.z() - ray.m_Org.z());
			tymin = (2.0f * (ray.m_Dir.z() > 0) - 1) * 10000.0f * (min.z() - ray.m_Org.z());
		}
		else{
			tymax = (max.z() - ray.m_Org.z()) / ray.m_Dir.z();
			tymin = (min.z() - ray.m_Org.z()) / ray.m_Dir.z();
		}
		if (tymax < tymin){
			std::swap(tymax, tymin);
		}
		MM[2] = tmax;
		mm[2] = tmin;

		if (tmin > tymax || tymin > tmax){
			return false;
		}
		tmin = tmin > tymin ? tmin : tymin;
		tmax = tmax < tymax ? tmax : tymax;


		if (tmin < 0){
			t = tmax;
		}
		else{
			t = tmin;
		}


		PTUtility::Vec3 ppp = ray.m_Org + t*ray.m_Dir;

		int ii = 0;
		int jj = 0;
		float dd = 10000;
		for (int i = 0; i < 3; i++){
			if (abs(ppp[i] - max[i]) < dd){
				ii = i;
				jj = 1;
				dd = abs(ppp[ii] - max[ii]);
			}
			if (abs(ppp[i] - min[i]) < dd){
				ii = i;
				jj = -1;
				dd = abs(ppp[ii] - min[ii]);
			}
			Normal = PTUtility::Vec3(0, 0, 0);
			Normal[ii] = jj;
		}

		return true;
	}
	template<>
	bool Intersect(const Primitive::Ray& ray, const Primitive::AABB& aabb, float& t, PTUtility::Vec3& Normal){
		return Intersect(aabb, ray, t, Normal);
	}

	template<>
	bool Intersect(const Primitive::AABB& A, const Primitive::AABB& B){

		for (int i = 0; i < 3; i++){
			if (A.m_MinPos[i] > B.m_MaxPos[i] || A.m_MaxPos[i] < B.m_MinPos[i]){
				return false;
			}

		}

		return true;
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::AABB& aabb){
		Primitive::AABB aa(s.m_Pos + s.m_Radius * PTUtility::Vec3::Ones(), s.m_Pos - s.m_Radius * PTUtility::Vec3::Ones());
		return Intersect(aa, aabb);
	}
	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Sphere& s){
		return Intersect(s, aabb);
	}

	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::Vertex& v){
		if ((s.m_Pos - v.m_Pos).norm2() < s.m_Radius*s.m_Radius){
			return true;
		}
		else{
			return false;
		}
	}
	template<>
	bool Intersect(const Primitive::Vertex& v, const Primitive::Sphere& s){
		return Intersect(s, v);
	}

	template<>
	bool Intersect(const Primitive::AABB& aabb, const Primitive::Vertex& v){
		for (int i = 0; i < 3; i++){
			if (aabb.m_MaxPos[i] < v.m_Pos[i] || aabb.m_MinPos[i] > v.m_Pos[i]){
				return false;
			}
		}
		return true;
	}

	template<>
	bool Intersect(const Primitive::Vertex& v, const Primitive::AABB& aabb){
		return Intersect(aabb, v);
	}




	template<>
	bool Intersect(const Primitive::Sphere& s, const Primitive::Polygon& p, float& d, float& u, float& v){

		//Vec3 N = (0.3333 * (facet.GetNormal(0) + facet.GetNormal(1) + facet.GetNormal(2))).normalized();
		//Vec3 p = facet.GetPos(1) - facet.GetPos(0);
		//Vec3 q = facet.GetPos(2) - facet.GetPos(0);

		//Eigen::Matrix3f mat;

		//mat << -N.x(), -N.y(), -N.z(), p.x(), p.y(), p.z(), q.x(), q.y(), q.z();

		//mat.inverse();
		//Eigen::Vector3f pq = Eigen::Vector3f(
		//	CPos[0] - facet.GetPos(0)[0],
		//	CPos[1] - facet.GetPos(0)[1],
		//	CPos[2] - facet.GetPos(0)[2]);

		//Eigen::Vector3f x = mat * pq;

		//if (x[0] <= R && x[1] >= 0.0f && x[2] >= 0.0f && x[1] + x[2] <= 1.0f){
		//	return true;
		//}
		//else if ((CPos - facet.GetPos(2)).norm() <= R){
		//	return true;
		//}
		//else if ((CPos - facet.GetPos(1)).norm() <= R){
		//	return true;
		//}
		//else if ((CPos - facet.GetPos(0)).norm() <= R){
		//	return true;
		//}
		//else if (x[1] < 0.0f){
		//	if (Intersect::Ray_Sphere(Ray(facet.GetPos(0), p), CPos, R, t)){
		//		return true;
		//	}
		//}
		//else if (x[2] < 0.0f){
		//	if (Intersect::Ray_Sphere(Ray(facet.GetPos(0), q), CPos, R, t)){
		//		return true;
		//	}
		//}
		//else if (x[1] + x[2] < 1.0f){
		//	if (Intersect::Ray_Sphere(Ray(facet.GetPos(1), facet.GetPos(2) - facet.GetPos(1)), CPos, R, t)){
		//		return true;
		//	}
		//}

		///*else if (x[1] <= 0.0f && x[2] >= 1.0f && abs(x[1]) + abs(x[2]) >= 1.0f && (CPos - facet.GetPos(2)).norm() <= R){
		//return true;
		//}
		//else if (x[1] >= 1.0f && x[2] <= 0.0f && abs(x[1]) + abs(x[2]) >= 1.0f && (CPos - facet.GetPos(1)).norm() <= R){
		//return true;
		//}
		//else if (x[1] <= 0.0f && x[2] <= 0.0f && abs(x[1]) + abs(x[2]) <= 1.0f && (CPos - facet.GetPos(0)).norm() <= R){
		//return true;
		//}*/


		//return false;


		return false;
	}


	bool AABB_RAY_SIMD(
		const __m256 value[2][3],
		const __m256* org,
		const __m256* idir,
		const int* sign,
		int& mask
		)
	{

		__m256 tmin = _mm256_set1_ps(-100000.0f);
		__m256 tmax = _mm256_set1_ps(100000.0f);


		int idx0, idx1;

		idx0 = sign[1];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][1], org[1]), idir[1]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][1], org[1]), idir[1]));

		idx0 = sign[2];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][2], org[2]), idir[2]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][2], org[2]), idir[2]));

		idx0 = sign[0];
		idx1 = 1 - idx0;
		tmin = _mm256_max_ps(tmin, _mm256_mul_ps(_mm256_sub_ps(value[idx0][0], org[0]), idir[0]));
		tmax = _mm256_min_ps(tmax, _mm256_mul_ps(_mm256_sub_ps(value[idx1][0], org[0]), idir[0]));

		mask = _mm256_movemask_ps(_mm256_cmp_ps(tmax, tmin, _CMP_GT_OS));
		return (mask > 0);

	}


}