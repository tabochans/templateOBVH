#include"OBVH.h"

namespace mSec{

	template<>
	int OBVH<Primitive::Polygon>::PGetElementFromRay(const Primitive::Ray& ray, ::std::vector<IscData>& result, int PID)const{

		float u, v, t;
		u = v = t = 0.0f;
		if (m_NArray[PID]._NumChildren == 0){
			int nc = m_NArray[PID]._Elements.size();

			for (int i = 0; i < nc; i++){
				if (Isc::Intersect(ray, *m_NArray[PID]._Elements[i], t, u, v)){
					if (t > FLT_MIN){
						result.push_back(IscData(m_NArray[PID]._Elements[i], t, u, v));
					}
				}
			}
			return result.size();
		}

		__m256 org[3];
		__m256 idir[3];
		int sign[3];
		int mask = 0;

		for (int i = 0; i < 3; i++){

			sign[i] = ray.m_Dir[i] < 0;
			org[i] = _mm256_set1_ps(ray.m_Org[i]);

			if (abs(ray.m_Dir[i]) < 1.0f / 100000.0f){
				idir[i] = _mm256_set1_ps(100000.0f - 200000.0f * sign[i]);
			}
			else{
				idir[i] = _mm256_set1_ps(1.0f / ray.m_Dir[i]);
			}

		}


		if (Isc::AABB_RAY_SIMD(m_NArray[PID].CAABB, org, idir, sign, mask)){
			for (int i = 0; i < 8; i++){
				int fg = mask & (1 << (i));
				if ((fg) > 0){
					int iid = m_NArray[PID]._Children[i];
					if (iid > 0){ PGetElementFromRay(ray, result, iid); }
				}
			}
		}

		return result.size();
	}


	template<>
	int OBVH<Primitive::Polygon>::GetElementFromRay(const Primitive::Ray& ray, ::std::vector<IscData>& result)const{
		result.clear();

		int n = PGetElementFromRay(ray, result, 1);
		::std::sort(result.begin(), result.end());
		return n;
	}




}