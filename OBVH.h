#pragma once

#include<deque>
#include <immintrin.h>
#include"Intersect.h"
#include<vector>


namespace mSec{

	template<typename E>
	class OBVH{

	public:


		struct IscData{
			float _d;
			float _u;
			float _v;
			E* _pElement;

			bool operator<(const IscData& ref){
				return this->_d < ref._d;
			}
			bool operator>(const IscData& ref){
				return this->_d > ref._d;
			}
			bool operator<=(const IscData& ref){
				return this->_td <= ref._d;
			}
			bool operator>=(const IscData& ref){
				return this->_d >= ref._d;
			}
			IscData& operator=(const IscData& ref){
				_pElement = ref._pElement;
				_u = ref._u;
				_v = ref._v;
				_d = ref._d;
				return *this;
			}

			IscData(E* e, float d, float u, float v) : _u(u), _v(v), _d(d), _pElement(e){
			}

		};

	private:

		bool m_IfCreated;

		const float COST_ELEMTNT;
		const float COST_AABB;
		const int MAX_DEPTH;
		const int MIN_ELEMENT;
		const float ZEROP;

		

		PTUtility::Vec3 m_Max;
		PTUtility::Vec3 m_Min;
		::std::vector<E> m_data;

		struct Node{
			::std::vector<E*> _Elements;
			::std::vector<Primitive::AABB> _AABBs;
			int _Children[8];
			int _NumChildren;

			__m256 CAABB[2][3];

			Node(){
				_Children[0] = 0;
				_Children[1] = 0;
				_Children[2] = 0;
				_Children[3] = 0;
				_Children[4] = 0;
				_Children[5] = 0;
				_Children[6] = 0;
				_Children[7] = 0;
				_NumChildren = 0;
			}
			~Node(){
			}

		};


		::std::deque<Node> m_NArray;

		void Divide(Node& node, int GID, int mode, int Depth);

		float CostAABB(
			const Primitive::AABB& Parent,
			float sA, int NumElementsA,
			float SB, int NumElementsB)const;

		float ZeroCost(const Primitive::AABB& Parent, int NumElements)const;

		mutable ::std::vector<::std::vector<int>> _FList;
		mutable ::std::vector<PTUtility::Vec3> _VList;
		mutable int _NF;
		mutable int _SS;
		mutable int _SSS;
		
		int PGetElementFromRay(const Primitive::Ray& ray, ::std::vector<IscData>& result, int PID)const;

		template<typename K>
		int PGetElement(const K& key, ::std::vector<E>& result, int PID)const;

		int CreateAABBTree(int ID);

		void OBVH::PWriteObj(::std::ostream& ost, int PID)const;

	protected:

	public:

		int GetElementFromRay(const Primitive::Ray& ray, ::std::vector<IscData>& result)const;

		template<typename K>
		int GetElement(const K& key, ::std::vector<E>& result)const;

		const ::std::string Print() const{
			return ::std::string("OBVH");
		}
		float GetSceneWidth();
		PTUtility::Vec3 GetSceneCenter();
		void CreateGraph(const ::std::vector<E>& Elements);
		void ClearGraph();
		void WriteObj(::std::ostream& ost) const;

		OBVH();
		OBVH(
			float cost_AABB,
			float cost_Element,
			float Max_Depth,
			float MIN_Element);

		~OBVH();



		//return true if graph is created
		bool IfCreated(){
			return m_IfCreated;
		}
	};







	template<typename E>
	void  OBVH<E>::Divide(Node& node, int GID, int Mode, int Depth){

		int NElm = node._Elements.size();

		//分割軸
		int Axis = -1;
		//分割場所leftが含むポリゴン数
		int Dpos = -1;
		//分割コスト
		float Cost = COST_ELEMTNT * NElm;

		::std::vector<E*> FA[3];
		FA[0] = node._Elements;
		FA[1] = node._Elements;
		FA[2] = node._Elements;

		::std::sort(FA[0].begin(), FA[0].end(), &E::compPX);
		::std::sort(FA[1].begin(), FA[1].end(), &E::compPY);
		::std::sort(FA[2].begin(), FA[2].end(), &E::compPZ);



		//三軸について
		for (int ax = 0; ax < 3; ax++){

			//表面積の控え
			//SufArray[i] には、i個のポリゴンが含まれた場合の表面積ね
			::std::vector<float> SufArray(NElm + 2);

			Primitive::AABB Bright, Bleft;

			//rightの表面積を計算しとく
			SufArray[0] = 1000000;
			Bleft.Reset();
			for (int i = NElm - 1; i >= 0; i--){
				Bright.Expand(*FA[ax][i]);
				SufArray[NElm - i] = Bright.GetArea();
			}
			Bright.Reset();
			Bleft.Reset();

			//SAHの計算
			for (int i = 0; i < NElm; i++){
				Bleft.Expand(*FA[ax][i]);
				float tCost = CostAABB(node._AABBs[0], Bleft.GetArea(), i + 1, SufArray[NElm - i - 1], NElm - i - 1);
				if (Cost > tCost){
					Axis = ax;
					Dpos = i + 1;
					Cost = tCost;
				}
			}
		}


		if (Axis != -1 && Dpos > 0 && Dpos < NElm && NElm > MIN_ELEMENT){

			//分割先が見つかった

			Node Nright, Nleft;
			Nleft._Elements.resize(Dpos);
			Nright._Elements.resize(NElm - Dpos);
			Primitive::AABB Bright, Bleft;
			for (int i = 0; i < Dpos; i++){
				Bleft.Expand(*FA[Axis][i]);
				Nleft._Elements[i] = FA[Axis][i];
			}
			for (int i = Dpos; i < NElm; i++){
				Bright.Expand(*FA[Axis][i]);
				Nright._Elements[i - Dpos] = FA[Axis][i];
			}
			Nleft._AABBs.push_back(Bleft);
			Nright._AABBs.push_back(Bright);


			//OBVHだから、さらに二回分割をする
			if (Mode == 0){
				Divide(Nleft, GID, 1, Depth++);
				Divide(Nright, GID, 1, Depth++);
				::std::vector<E*>().swap(node._Elements);
			}
			if (Mode == 1){
				Divide(Nleft, GID, 2, Depth++);
				Divide(Nright, GID, 2, Depth++);
				::std::vector<E*>().swap(node._Elements);
				return;
			}
			if (Mode == 2){
				int cid = m_NArray.size();
				m_NArray.push_back(Nleft);
				m_NArray.push_back(Nright);

				m_NArray[GID]._Children[m_NArray[GID]._NumChildren] = cid;
				m_NArray[GID]._NumChildren++;

				m_NArray[GID]._Children[m_NArray[GID]._NumChildren] = cid + 1;
				m_NArray[GID]._NumChildren++;

				::std::vector<E*>().swap(node._Elements);

				return;
			}

			//そして時は動き出す
			for (int i = 0; i < 8; i++){
				int cd = m_NArray[GID]._Children[i];
				if (cd > 0){ Divide(m_NArray[cd], cd, 0, Depth++); }
			}


		}
		else{
			//分割しない

			if (Mode == 1 || Mode == 2){
				int cid = m_NArray.size();
				m_NArray.push_back(node);
				m_NArray[GID]._Children[m_NArray[GID]._NumChildren] = cid;
				m_NArray[GID]._NumChildren++;
			}

		}

	}

	template<typename E>
	float OBVH<E>::CostAABB(
		const Primitive::AABB& Parent,
		float sA, int NumTriangleA,
		float sB, int NumTriangleB) const{

		return 2.0f * COST_AABB +
			NumTriangleA * COST_ELEMTNT * (sA) / (Parent.GetArea()) +
			NumTriangleB * COST_ELEMTNT * (sB) / (Parent.GetArea());

	}
	template<typename E>
	float OBVH<E>::ZeroCost(const Primitive::AABB& Parent, int NumElements) const{
		return CostAABB(Parent, Parent.GetArea(), NumElements, 10000000000000, 0);
	}
	template<typename E>
	int OBVH<E>::CreateAABBTree(int ID){
		//親ノードが子のAABB８個を一括で持つよーにする


		if (m_NArray[ID]._NumChildren == 0){
			return 1;
		}
		else{

			float AABBMax[3][8];
			float AABBMin[3][8];

			for (int i = 0; i < 8; i++){

				if (m_NArray[ID]._Children[i] > 0){
					const Node& cn = m_NArray[m_NArray[ID]._Children[i]];
					for (int j = 0; j < 3; j++){
						AABBMax[j][i] = cn._AABBs[0].m_MaxPos[j];
						AABBMin[j][i] = cn._AABBs[0].m_MinPos[j];
					}
				}
				else{
					for (int j = 0; j < 3; j++){
						AABBMax[j][i] = -1000000;
						AABBMin[j][i] = 1000000;
					}
				}
			}

			for (int i = 0; i < 3; i++){
				m_NArray[ID].CAABB[0][i] = _mm256_loadu_ps(AABBMin[i]);
				m_NArray[ID].CAABB[1][i] = _mm256_loadu_ps(AABBMax[i]);
			}
			//m_NArray[ID]._AABBs.clear();

			m_NArray[ID]._AABBs.resize(8);
			for (int i = 0; i < 8; i++){
				m_NArray[ID]._AABBs[i] = Primitive::AABB(
					PTUtility::Vec3(
					AABBMax[0][i], AABBMax[1][i], AABBMax[2][i]
					),
					PTUtility::Vec3(
					AABBMin[0][i], AABBMin[1][i], AABBMin[2][i]
					)
					);
			}

			for (int i = 0; i < 8; i++){
				if (m_NArray[ID]._Children[i] > 0){
					CreateAABBTree(m_NArray[ID]._Children[i]);
				}
			}

		}

		return 1;
	}

	template<typename E>
	void OBVH<E>::CreateGraph(const ::std::vector<E>& FacetArray){

		m_NArray.clear();

		Node parent;
		Primitive::AABB pAB;

		parent._AABBs.push_back(pAB);
		m_NArray.push_back(parent);
		parent._AABBs.clear();

		m_data = FacetArray;

		for (int i = 0; i < m_data.size(); i++){
			pAB.Expand(m_data[i]);
		}
		m_Max = pAB.m_MaxPos;
		m_Min = pAB.m_MinPos;
		parent._AABBs.push_back(pAB);
		parent._Elements.resize(m_data.size());
		for (int i = 0; i < m_data.size(); i++){
			parent._Elements[i] = &m_data[i];
		}

		m_NArray.push_back(parent);

		Divide(m_NArray[1], 1, 0, 0);

		CreateAABBTree(1);

		return;

	}

	template<typename E>
	void OBVH<E>::ClearGraph(){
	}

	template<typename E>
	void OBVH<E>::PWriteObj(::std::ostream& ost, int PID) const{

		if (m_NArray[PID]._Elements.size() > 0){

			Primitive::AABB A = m_NArray[PID]._AABBs[0];
			PTUtility::Vec3 c = A.GetCenter();
			float dx, dy, dz;
			dx = A.GetWidth()[0] * 0.5f;
			dy = A.GetWidth()[1] * 0.5f;
			dz = A.GetWidth()[2] * 0.5f;

			_VList.push_back(c + PTUtility::Vec3(-dx, -dy, dz));
			_VList.push_back(c + PTUtility::Vec3(-dx, dy, dz));
			_VList.push_back(c + PTUtility::Vec3(dx, dy, dz));
			_VList.push_back(c + PTUtility::Vec3(dx, -dy, dz));
			_VList.push_back(c + PTUtility::Vec3(-dx, -dy, -dz));
			_VList.push_back(c + PTUtility::Vec3(-dx, dy, -dz));
			_VList.push_back(c + PTUtility::Vec3(dx, dy, -dz));
			_VList.push_back(c + PTUtility::Vec3(dx, -dy, -dz));

			::std::vector<int> tt;

			tt.push_back(1 + _NF * 8); tt.push_back(4 + _NF * 8); tt.push_back(3 + _NF * 8); tt.push_back(2 + _NF * 8);
			_FList.push_back(tt);
			tt.clear();

			tt.push_back(1 + _NF * 8); tt.push_back(5 + _NF * 8); tt.push_back(8 + _NF * 8); tt.push_back(4 + _NF * 8);
			_FList.push_back(tt);
			tt.clear();

			tt.push_back(2 + _NF * 8); tt.push_back(6 + _NF * 8); tt.push_back(5 + _NF * 8); tt.push_back(1 + _NF * 8);
			_FList.push_back(tt);
			tt.clear();

			tt.push_back(3 + _NF * 8); tt.push_back(7 + _NF * 8); tt.push_back(6 + _NF * 8); tt.push_back(2 + _NF * 8);
			_FList.push_back(tt);
			tt.clear();

			tt.push_back(4 + _NF * 8); tt.push_back(8 + _NF * 8); tt.push_back(7 + _NF * 8); tt.push_back(3 + _NF * 8);
			_FList.push_back(tt);
			tt.clear();

			tt.push_back(6 + _NF * 8); tt.push_back(7 + _NF * 8); tt.push_back(8 + _NF * 8); tt.push_back(5 + _NF * 8);
			_FList.push_back(tt);
			tt.clear();

			_NF++;
			_SSS += m_NArray[PID]._Elements.size();
		}


		for (int i = 0; i < 8; i++){
			if (m_NArray[PID]._Children[i] > 0){ PWriteObj(ost, m_NArray[PID]._Children[i]); }
		}



	}

	template<typename E>
	void OBVH<E>::WriteObj(::std::ostream& ost) const{

		_VList.clear();
		_FList.clear();
		_NF = 0;
		_SSS = 0;

		ost << " o BVH" << ::std::endl;

		PWriteObj(ost, 1);

		for (int v = 0; v < _VList.size(); v++){
			ost << "v " << _VList[v][0] << " " << _VList[v][1] << " " << _VList[v][2] << ::std::endl;
		}
		for (int f = 0; f < _FList.size(); f++){
			ost << "f ";
			for (int ff = 0; ff < _FList[0].size(); ff++){
				ost << _FList[f][ff] << " ";
			}
			ost << ::std::endl;
		}

	}

	/*template<typename E>
	int OBVH<E>::PGetElements(const PTUtility::Ray& ray, ::std::vector<RF>& Facets, int ID) const{

	float u, v, t;
	u = v = t = 0.0f;
	if (m_NArray[ID]._NumChildren == 0){
	int nc = m_NArray[ID]._facets.size();

	for (int i = 0; i < nc; i++){
	if (Intersect::Ray_Triangle(ray, *m_NArray[ID]._facets[i], u, v, t)){
	if (t > FLT_MIN){
	Facets.push_back(AbsG::RF(m_NArray[ID]._facets[i], u, v, t));
	}
	}
	}
	return Facets.size();
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


	if (Intersect::IsHit(m_NArray[ID].CAABB, org, idir, sign, mask)){
	for (int i = 0; i < 8; i++){
	int fg = mask & (1 << (i));
	if ((fg) > 0){
	int iid = m_NArray[ID]._Children[i];
	if (iid > 0){ PGetElements(ray, Facets, iid); }
	}
	}
	}

	return Facets.size();
	}

	template<typename E>
	int OBVH<E>::GetElements(const PTUtility::Ray& ray, ::std::vector<RF>& Facets) const{

	Facets.clear();

	int n = PGetElements(ray, Facets, 1);
	::std::sort(Facets.begin(), Facets.end());
	return n;
	}

	template<typename E>
	int OBVH<E>::PGetElements(const Vec3& Center, float R, ::std::vector<RF>& Facets, int ID) const{

	float u, v, t;
	u = v = t = 0.0f;
	if (m_NArray[ID]._NumChildren == 0){
	int nc = m_NArray[ID]._facets.size();

	for (int i = 0; i < nc; i++){
	if (Intersect::Sphere_Triangle(Center, R, *m_NArray[ID]._facets[i], u, v, t)){
	Facets.push_back(AbsG::RF(m_NArray[ID]._facets[i], u, v, t));
	}
	}
	return Facets.size();
	}
	else{

	__m256 Pos[3];
	__m256 Rd[1];

	int mask = 0;

	for (int i = 0; i < 3; i++){
	Pos[i] = _mm256_set1_ps(Center[i]);
	}
	Rd[0] = _mm256_set1_ps(R);

	if (Intersect::Sphere_AABB_Fast_S(Pos, Rd, m_NArray[ID].CAABB, mask)){
	for (int i = 0; i < 8; i++){
	int fg = mask & (1 << (i));
	if ((fg) > 0){
	int iid = m_NArray[ID]._Children[i];
	if (iid > 0){ PGetElements(Center, R, Facets, iid); }
	}
	}
	}

	return Facets.size();

	}


	}

	template<typename E>
	int OBVH<E>::GetElements(const Vec3& Center, float R, ::std::vector<RF>& Facets){
	Facets.clear();
	int n = PGetElements(Center, R, Facets, 1);
	::std::sort(Facets.begin(), Facets.end());
	return n;
	}*/

	template<typename E>
	float OBVH<E>::GetSceneWidth(){

		Vec3 w = m_Max - m_Min;
		return ::std::max(w[0], ::std::max(w[1], w[2]));
	}

	template<typename E>
	PTUtility::Vec3 OBVH<E>::GetSceneCenter(){
		return 0.5 * (m_Max + m_Min);
	}

	template<typename E>
	OBVH<E>::OBVH(
		float cost_AABB,
		float cost_Element,
		float Max_Depth,
		float MIN_Element
		) : COST_AABB(cost_AABB), COST_ELEMTNT(cost_Element), MAX_DEPTH(Max_Depth), MIN_ELEMENT(MIN_Element), ZEROP(FLT_MIN){
	}

	template<typename E>
	OBVH<E>::OBVH(
		) : COST_AABB(1.0f), COST_ELEMTNT(1.0f), MAX_DEPTH(10), MIN_ELEMENT(1), ZEROP(FLT_MIN){
	}


	template<typename E>
	OBVH<E>::~OBVH(){
	}


	template<>
	int OBVH<Primitive::Polygon>::GetElementFromRay(const Primitive::Ray& ray, ::std::vector<IscData>& result)const{
		result.clear();

		int n = PGetElementFromRay(ray, result, 1);
		::std::sort(result.begin(), result.end());
		return n;
	}

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

	//template<typename E>
	//int OBVH<E>::GetElements(const Primitive::Ray& ray, ::std::vector<IscData>& result) const{

	//	result.clear();

	//	int n = PGetElements(ray, result, 1);
	//	return n;
	//}

	//template<typename E>
	//int OBVH<E>::PGetElement(const Primitive::Vec3& Center, float R, ::std::vector<IscData>& result, int PID) const{

	//	float u, v, t;
	//	u = v = t = 0.0f;
	//	if (m_NArray[ID]._NumChildren == 0){
	//		int nc = m_NArray[ID]._facets.size();

	//		for (int i = 0; i < nc; i++){
	//			if (Intersect::Sphere_Triangle(Center, R, *m_NArray[ID]._facets[i], u, v, t)){
	//				Facets.push_back(AbsG::RF(m_NArray[ID]._facets[i], u, v, t));
	//			}
	//		}
	//		return Facets.size();
	//	}
	//	else{

	//		__m256 Pos[3];
	//		__m256 Rd[1];

	//		int mask = 0;

	//		for (int i = 0; i < 3; i++){
	//			Pos[i] = _mm256_set1_ps(Center[i]);
	//		}
	//		Rd[0] = _mm256_set1_ps(R);

	//		if (Intersect::Sphere_AABB_Fast_S(Pos, Rd, m_NArray[ID].CAABB, mask)){
	//			for (int i = 0; i < 8; i++){
	//				int fg = mask & (1 << (i));
	//				if ((fg) > 0){
	//					int iid = m_NArray[ID]._Children[i];
	//					if (iid > 0){ PGetElements(Center, R, Facets, iid); }
	//				}
	//			}
	//		}

	//		return Facets.size();

	//}




	template<typename E>template<typename K>
	int OBVH<E>::GetElement(const K& key, ::std::vector<E>& result)const{
		result.clear();
		int n = PGetElement(key, result, 1);
		//::std::sort(result.begin(), result.end());
		return n;
	}

	template<typename E>template<typename K>
	int OBVH<E>::PGetElement(const K& key, ::std::vector<E>& result, int PID)const{



		if (m_NArray[PID]._NumChildren == 0){

			for (int i = 0; i < m_NArray[PID]._Elements.size(); i++){

				Vertex* vv = m_NArray[PID]._Elements[i];
				if (Isc::Intersect(key, *(vv))){
					result.push_back(*vv);
				}
			}
			return Facets.size();
		}
		else{

			const AABB* tab;
			for (int i = 0; i < m_NArray[PID]._NumChildren; i++){

				if (Isc::Intersect(m_NArray[PID]._AABBs[i], key)){
					PGetElement(key, result, m_NArray[PID]._Children[i]);
				}
			}
			return result.size();
		}

		return result.size();
	}

}