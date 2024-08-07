#ifndef slic3r_TriangleMesh_hpp_
#define slic3r_TriangleMesh_hpp_

#include "libslic3r.h"
#include <stl.h>
#include <vector>
#include <map>
#include <list>
#include <boost/thread.hpp>
#include "BoundingBox.hpp"
#include "Line.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "ExPolygon.hpp"
#include "ClipperUtils.hpp"
#include <float.h>
#include <io.h>
#include <unordered_set>
#include <stack>
#include <vector>

#ifdef OLD_TEST
#define SIGN3(A) (((A).x < 0) ? 4 : 0 | ((A).y < 0) ? 2 \
								: 0 | ((A).z < 0)	? 1 \
													: 0)
#else
#define EPS 10e-5
#define SIGN3(A)                                 \
	(((A).x < EPS) ? 4 : 0 | ((A).x > -EPS) ? 32 \
					 : 0 | ((A).y < EPS)	? 2  \
					 : 0 | ((A).y > -EPS)	? 16 \
					 : 0 | ((A).z < EPS)	? 1  \
					 : 0 | ((A).z > -EPS)	? 8  \
											: 0)
#endif

#define CROSS(A, B, C)                          \
	{                                           \
		(C).x = (A).y * (B).z - (A).z * (B).y;  \
		(C).y = -(A).x * (B).z + (A).z * (B).x; \
		(C).z = (A).x * (B).y - (A).y * (B).x;  \
	}
#define SUB(A, B, C)           \
	{                          \
		(C).x = (A).x - (B).x; \
		(C).y = (A).y - (B).y; \
		(C).z = (A).z - (B).z; \
	}
#define LERP(A, B, C) ((B) + (A) * ((C) - (B)))
#define MIN3(a, b, c) ((((a) < (b)) && ((a) < (c))) ? (a) : (((b) < (c)) ? (b) : (c)))
#define MAX3(a, b, c) ((((a) > (b)) && ((a) > (c))) ? (a) : (((b) > (c)) ? (b) : (c)))
#define INSIDE 0
#define OUTSIDE 1

#ifndef M_PI
#define M_PI 3.14159265358979323846 // pi
#endif

namespace Slic3r
{

	enum Ray_type
	{
		axis_None,
		axis_X,
		axis_Y,
		axis_Z
	};

	class TriangleMesh;
	template <Axis A>
	class TriangleMeshSlicer;
	typedef std::vector<TriangleMesh *> TriangleMeshPtrs;

	typedef std::vector<Linef3> BoundLines_vec;
	typedef std::vector<Pointf3> BoundLoop;
	typedef std::vector<BoundLoop> BoundLoops_vec;

	/////////////////////////////////////////////////////////
	//  Lattice 支撑点结构
	////////////////////////////////////////////////////////
	class LatticeSPT_Pointf3;
	class Pillar;
	class PillarNode;
	class LatticeSupportMesh;

	typedef Pillar *PillarPtr;
	typedef PillarNode *PillarNodePtr;
	typedef LatticeSupportMesh *LatticeSupportMeshPtr;
	typedef std::vector<LatticeSPT_Pointf3> LatticeSPT_Pointf3s;

	typedef std::map<coord_t, PillarNode> PillarNodeMap;
	typedef std::map<coord_t, Pillar> Vertical_Pillar;		// <y,Pillar>
	typedef std::map<coord_t, Vertical_Pillar> Grid_Pillar; // <x,<y,Pillar>>

	typedef std::pair<Pointf3, Pointf3> CrossbarPointPair;
	typedef std::vector<CrossbarPointPair> CrossbarPointPairs;

	/////////////////////////////////////////////////////////
	//  标记面片边界线段
	////////////////////////////////////////////////////////

	union Facet_Index
	{
		long long union_id;
		struct
		{
			int Mark_facetIndex;   // 标记侧的面片ID
			int NoMark_facetIndex; //  非标记侧的面片ID
		};
	};

	struct BoundSegment
	{
		Facet_Index FacetID;
		double angle;		 // 角度阈值
		Linef3 Segment_line; // 边界线段
		int weight;			 // 边界权重
	};

	typedef std::vector<BoundSegment> BoundSegments_vec;		 // 悬垂线和边界线集合
	typedef BoundSegments_vec BoundSegments_loop;				 // 悬垂线和边界线集合
	typedef std::vector<BoundSegments_loop> BoundSegments_loops; // 悬垂线和边界线集合
	typedef std::map<long long, BoundSegments_vec> FacetSegments_map;
	/////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////
	//  横杆数据结构
	////////////////////////////////////////////////////////
	typedef std::pair<Pointf3, Pointf3> CrossbarPointPair;	   // 横杆双点信息
	typedef std::vector<CrossbarPointPair> CrossbarPointPairs; // 横杆集合
	struct PlaneZ_CrossBars									   // 横杆主结构体
	{
		double slice_z;								  // 主横杆所在切片平面的高度
		CrossbarPointPairs Major_CrossBars;			  // 主横杆集
		BoundSegments_loops Slice_loops;			  // 切片边界线集
		CrossbarPointPairs Slice_Assist_CrossBars;	  // 辅助横杆集
		std::vector<Pointf3> Boundary_Points;		  //  边缘边界线集
		CrossbarPointPairs Boundary_Assist_CrossBars; // 辅助横杆集
	};
	/////////////////////////////////////////////////////////////////////////////////////////////

	// 支撑点信息结构体
	typedef std::map<double, int> MapCollsionInfo;
	struct SPT_pointf
	{
		bool IsValid;		// 是否有效
		Pointf hitpoint;	// 射线在xy平面的投影位置
		Pointf basepoint;	// 下基本点位置
		double basepoint_z; // 下基本点z轴坐标

		coordf_t dirangle;			   // 支撑旋转的角度
		size_t region_id;			   // 所属的区域id
		MapCollsionInfo CollsionZ_map; // 基本点Z轴碰撞点信息  z值-->碰撞facet_id

		SPT_pointf *treebasep;				   // 树支撑专属 树基支撑指针
		std::vector<SPT_pointf *> branch_vecp; // 树支撑专属 树的分支支撑指针
	public:
		// 构造函数
		SPT_pointf(Pointf _hit, coordf_t _dir, size_t _id)
		{
			this->hitpoint = _hit;
			this->dirangle = _dir;
			this->region_id = _id;
			// 默认值  树形支撑可能会修改
			this->basepoint = _hit;
			this->basepoint_z = 0;
			this->CollsionZ_map.clear();
			this->branch_vecp.clear();
			treebasep = NULL;
			IsValid = true;
		};
		SPT_pointf(Pointf3 _hit3, coordf_t _dir, size_t _id, size_t _face_id)
		{
			Pointf _hit = Pointf(_hit3.x, _hit3.y);
			this->hitpoint = _hit;
			this->dirangle = _dir;
			this->region_id = _id;
			// 默认值  树形支撑可能会修改
			this->basepoint = _hit;
			this->basepoint_z = 0;
			this->CollsionZ_map.clear();
			this->setCollsionZ(_hit3.z, _face_id);
			this->branch_vecp.clear();
			treebasep = NULL;
			IsValid = true;
		};
		// 接口函数
		bool setCollsionZ(double _z, size_t _face_id) // 供晶格树支撑使用，直接设置了支撑点的Z值
		{
			this->CollsionZ_map.insert(MapCollsionInfo::value_type(_z, _face_id));
			return true;
		}
		bool IsCollsion()
		{
			return (this->CollsionZ_map.size() != 0);
		};

		double get_mincollsionZ()
		{
			if (this->IsCollsion())
				return this->CollsionZ_map.begin()->first;
			return 0.0;
		}

		int get_mincollsionFaceID()
		{
			if (this->IsCollsion())
				return this->CollsionZ_map.begin()->second;
			return -1;
		}

		double get_collsionZ(unsigned int hit_index)
		{
			if (hit_index < this->CollsionZ_map.size())
			{
				MapCollsionInfo::iterator p_It = this->CollsionZ_map.begin();
				int start_index = 0;
				while (start_index != hit_index)
				{
					start_index++;
					p_It++;
				}
				return p_It->first;
			}
			return 0.0;
		}

		Pointf3 Get_upPoint()
		{
			return Pointf3(hitpoint.x, hitpoint.y, get_mincollsionZ());
		}

		Pointf3 Get_dwPoint()
		{
			return Pointf3(basepoint.x, basepoint.y, basepoint_z);
		}

		bool IsTreeBase()
		{
			LOGINFO("1020 branch_vecp size = [%d]", this->branch_vecp.size());
			return (this->branch_vecp.size() > 0);
		}

		bool IsTreeBranch()
		{
			if (this->treebasep != NULL)
			{
				int branch_base_branch_num = this->treebasep->branch_vecp.size();
				LOGINFO("1020 branch_base_branch_num = [%d]", branch_base_branch_num);
				return true;
			}
			else
			{
				return false;
			}
		}

		bool Is_Valid()
		{
			return IsValid;
		}

		void delete_BranchInfo()
		{
			if (IsTreeBranch())
			{
				std::vector<SPT_pointf *>::iterator branch_It = this->treebasep->branch_vecp.begin();
				for (; branch_It != this->treebasep->branch_vecp.end();)
				{
					if (*branch_It == this)
					{
						branch_It = this->treebasep->branch_vecp.erase(branch_It);
					}
					else
					{
						branch_It++;
					}
				}
				LOGINFO(" this->treebasep->branch_vecp.size() = %d", this->treebasep->branch_vecp.size());
			}
		}
	};

	// 3D vector
	class Vector3
	{
	public:
		Vector3() : x(0.0), y(0.0), z(0.0) {}

		Vector3(double fx, double fy, double fz)
			: x(fx), y(fy), z(fz) {}

		// Subtract
		Vector3 operator-(const Vector3 &v) const
		{
			return Vector3(x - v.x, y - v.y, z - v.z);
		}
		// add
		Vector3 operator+(const Vector3 &v) const
		{
			return Vector3(x + v.x, y + v.y, z + v.z);
		}

		// add
		Vector3 operator*(const double &v) const
		{
			return Vector3(v * x, v * y, v * z);
		}

		// Dot product
		double Dot(const Vector3 &v) const
		{
			return x * v.x + y * v.y + z * v.z;
		}

		// 获取与z轴正向的夹角，返回rad
		double AngleWithZAxis() const
		{
			// normalized
			double x1 = x, y1 = y, z1 = z;
			double d = x * x + y * y + z * z;
			if (d != 1)
			{
				double m = sqrt(d);
				x1 /= m;
				y1 /= m;
				z1 /= m;
			}
		}

		// Cross product
		Vector3 Cross(const Vector3 &v) const
		{
			return Vector3(
				y * v.z - z * v.y,
				z * v.x - x * v.z,
				x * v.y - y * v.x);
		}

		// normal
		void normalize()
		{
			double m = sqrt(x * x + y * y + z * z);
			x /= m;
			y /= m;
			z /= m;
		}

		bool normalized()
		{
			return x * x + y * y + z * z == 1;
		}

		//
		double distance_to(Vector3 other)
		{
			return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y) + (z - other.z) * (z - other.z));
		}

		/* Which of the six face-plane(s) is point P outside of? */
		long face_plane(double side_length)
		{
			long outcode;

			outcode = 0;
			if (x > side_length / 2)
				outcode |= 0x01;
			if (x < -side_length / 2)
				outcode |= 0x02;
			if (y > side_length / 2)
				outcode |= 0x04;
			if (y < -side_length / 2)
				outcode |= 0x08;
			if (z > side_length / 2)
				outcode |= 0x10;
			if (z < -side_length / 2)
				outcode |= 0x20;
			return (outcode);
		}

		/* Which of the twelve edge plane(s) is point P outside of? */
		long bevel_2d(double side_length)
		{
			long outcode;

			outcode = 0;
			if (x + y > side_length)
				outcode |= 0x001;
			if (x - y > side_length)
				outcode |= 0x002;
			if (-x + y > side_length)
				outcode |= 0x004;
			if (-x - y > side_length)
				outcode |= 0x008;
			if (x + z > side_length)
				outcode |= 0x010;
			if (x - z > side_length)
				outcode |= 0x020;
			if (-x + z > side_length)
				outcode |= 0x040;
			if (-x - z > side_length)
				outcode |= 0x080;
			if (y + z > side_length)
				outcode |= 0x100;
			if (y - z > side_length)
				outcode |= 0x200;
			if (-y + z > side_length)
				outcode |= 0x400;
			if (-y - z > side_length)
				outcode |= 0x800;
			return (outcode);
		}

		/* Which of the eight corner plane(s) is point P outside of? */
		long bevel_3d(double side_length)
		{
			long outcode;

			outcode = 0;
			if ((x + y + z) > side_length * 1.5)
				outcode |= 0x01;
			if ((x + y - z) > side_length * 1.5)
				outcode |= 0x02;
			if ((x - y + z) > side_length * 1.5)
				outcode |= 0x04;
			if ((x - y - z) > side_length * 1.5)
				outcode |= 0x08;
			if ((-x + y + z) > side_length * 1.5)
				outcode |= 0x10;
			if ((-x + y - z) > side_length * 1.5)
				outcode |= 0x20;
			if ((-x - y + z) > side_length * 1.5)
				outcode |= 0x40;
			if ((-x - y - z) > side_length * 1.5)
				outcode |= 0x80;
			return (outcode);
		}

		/* Test the point "alpha" of the way from this to p */
		/* See if it is on a face of the cube              */
		/* Consider only faces in "mask"                   */
		long check_point(Vector3 p, float alpha, long mask, double side_length)
		{
			Vector3 plane_point;

			plane_point.x = LERP(alpha, x, p.x);
			plane_point.y = LERP(alpha, y, p.y);
			plane_point.z = LERP(alpha, z, p.z);
			return (plane_point.face_plane(side_length) & mask);
		}

		/*. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */

		/* Compute intersection of this --> P line segment with face planes */
		/* Then test intersection point to see if it is on cube face       */
		/* Consider only face planes in "outcode_diff"                     */
		/* Note: Zero bits in "outcode_diff" means face line is outside of */
		long check_line(Vector3 p, long outcode_diff, double side_length)
		{

			if ((0x01 & outcode_diff) != 0)
				if (this->check_point(p, (side_length / 2 - x) / (p.x - x), 0x3e, side_length) == INSIDE)
					return (INSIDE);
			if ((0x02 & outcode_diff) != 0)
				if (this->check_point(p, (-side_length / 2 - x) / (p.x - x), 0x3d, side_length) == INSIDE)
					return (INSIDE);
			if ((0x04 & outcode_diff) != 0)
				if (this->check_point(p, (side_length / 2 - y) / (p.y - y), 0x3b, side_length) == INSIDE)
					return (INSIDE);
			if ((0x08 & outcode_diff) != 0)
				if (this->check_point(p, (-side_length / 2 - y) / (p.y - y), 0x37, side_length) == INSIDE)
					return (INSIDE);
			if ((0x10 & outcode_diff) != 0)
				if (this->check_point(p, (side_length / 2 - z) / (p.z - z), 0x2f, side_length) == INSIDE)
					return (INSIDE);
			if ((0x20 & outcode_diff) != 0)
				if (this->check_point(p, (-side_length / 2 - z) / (p.z - z), 0x1f, side_length) == INSIDE)
					return (INSIDE);
			return (OUTSIDE);
		}

	public:
		double x, y, z;
	};

	class TriangleFace3
	{
	public:
		TriangleFace3(Vector3 _a, Vector3 _b, Vector3 _c)
			: p1(_a), p2(_b), p3(_c) {};
		~TriangleFace3() {};

		Vector3 normal()
		{
			Vector3 vp12 = p2 - p1;
			Vector3 vp13 = p3 - p1;
			Vector3 normal_rev = vp12.Cross(vp13);
			normal_rev.normalize();

			return normal_rev;
		}

		void translate(double x, double y, double z)
		{
			p1.x += x;
			p1.y += y;
			p1.z += z;
			p2.x += x;
			p2.y += y;
			p2.z += z;
			p3.x += x;
			p3.y += y;
			p3.z += z;
		}

		static double area(Vector3 _p1, Vector3 _p2, Vector3 _p3)
		{
			double d12 = _p1.distance_to(_p2);
			double d13 = _p1.distance_to(_p3);
			double d32 = _p3.distance_to(_p2);
			double s = (d12 + d13 + d32) / 2;

			return sqrt(s * (s - d12) * (s - d13) * (s - d32));
		}

		bool IsPointIn(Vector3 _Point)
		{
			double areaD = area(p1, p2, p3);
			double area1 = area(_Point, p2, p3);
			double area2 = area(p1, _Point, p3);
			double area3 = area(p1, p2, _Point);

			if (fabs(area1 + area2 + area3 - areaD) > 0.001)
				return false;

			return true;
		}

		Vector3 cale_hitPoint(Vector3 line_a, Vector3 line_b)
		{
			Vector3 face_normal = this->normal();
			Vector3 line_v = line_b - line_a;
			double TriD = -(face_normal.Dot(p1));
			double tempU = face_normal.Dot(line_a) + TriD;
			double tempD = face_normal.Dot(line_v);
			if (tempD == 0.0)
			{
				return this->p1;
			}
			double t = -tempU / tempD;

			return line_v * t + line_a;
		}

		/* Test if 3D point is inside 3D triangle */
		long point_triangle_intersection(Vector3 p)
		{
			long sign12, sign23, sign31;
			Vector3 vect12, vect23, vect31, vect1h, vect2h, vect3h;
			Vector3 cross12_1p, cross23_2p, cross31_3p;

			/* First, a quick bounding-box test:                               */
			/* If P is outside triangle bbox, there cannot be an intersection. */

			if (p.x > MAX3(p1.x, p2.x, p3.x))
				return (OUTSIDE);
			if (p.y > MAX3(p1.y, p2.y, p3.y))
				return (OUTSIDE);
			if (p.z > MAX3(p1.z, p2.z, p3.z))
				return (OUTSIDE);
			if (p.x < MIN3(p1.x, p2.x, p3.x))
				return (OUTSIDE);
			if (p.y < MIN3(p1.y, p2.y, p3.y))
				return (OUTSIDE);
			if (p.z < MIN3(p1.z, p2.z, p3.z))
				return (OUTSIDE);

			/* For each triangle side, make a vector out of it by subtracting vertexes; */
			/* make another vector from one vertex to point P.                          */
			/* The crossproduct of these two vectors is orthogonal to both and the      */
			/* signs of its X,Y,Z components indicate whether P was to the inside or    */
			/* to the outside of this triangle side.                                    */

			SUB(p1, p2, vect12)
			SUB(p1, p, vect1h);
			CROSS(vect12, vect1h, cross12_1p)
			sign12 = SIGN3(cross12_1p); /* Extract X,Y,Z signs as 0..7 or 0...63 integer */

			SUB(p2, p3, vect23)
			SUB(p2, p, vect2h);
			CROSS(vect23, vect2h, cross23_2p)
			sign23 = SIGN3(cross23_2p);

			SUB(p3, p1, vect31)
			SUB(p3, p, vect3h);
			CROSS(vect31, vect3h, cross31_3p)
			sign31 = SIGN3(cross31_3p);

			/* If all three crossproduct vectors agree in their component signs,  */
			/* then the point must be inside all three.                           */
			/* P cannot be OUTSIDE all three sides simultaneously.                */

			/* this is the old test; with the revised SIGN3() macro, the test
			 * needs to be revised. */
#ifdef OLD_TEST
			if ((sign12 == sign23) && (sign23 == sign31))
				return (INSIDE);
			else
				return (OUTSIDE);
#else
			return ((sign12 & sign23 & sign31) == 0) ? OUTSIDE : INSIDE;
#endif
		}

		/**********************************************/
		/* This is the main algorithm procedure.      */
		/* Triangle t is compared with a cube,        */
		/* side_length centered on the origin.        */
		/* It returns INSIDE (0) or OUTSIDE(1) if t   */
		/* intersects or does not intersect the cube. */
		/**********************************************/
		long t_c_intersection(double side_length)
		{
			long v1_test, v2_test, v3_test;
			float d, denom;
			Vector3 vect12, vect13, norm;
			Vector3 hitpp, hitpn, hitnp, hitnn;

			/* First compare all three vertexes with all six face-planes */
			/* If any vertex is inside the cube, return immediately!     */

			if ((v1_test = p1.face_plane(side_length)) == INSIDE)
				return (INSIDE);
			if ((v2_test = p2.face_plane(side_length)) == INSIDE)
				return (INSIDE);
			if ((v3_test = p3.face_plane(side_length)) == INSIDE)
				return (INSIDE);

			/* If all three vertexes were outside of one or more face-planes, */
			/* return immediately with a trivial rejection!                   */

			if ((v1_test & v2_test & v3_test) != 0)
				return (OUTSIDE);

			/* Now do the same trivial rejection test for the 12 edge planes */

			v1_test |= p1.bevel_2d(side_length) << 8;
			v2_test |= p2.bevel_2d(side_length) << 8;
			v3_test |= p3.bevel_2d(side_length) << 8;
			if ((v1_test & v2_test & v3_test) != 0)
				return (OUTSIDE);

			/* Now do the same trivial rejection test for the 8 corner planes */

			v1_test |= p1.bevel_3d(side_length) << 24;
			v2_test |= p2.bevel_3d(side_length) << 24;
			v3_test |= p3.bevel_3d(side_length) << 24;
			if ((v1_test & v2_test & v3_test) != 0)
				return (OUTSIDE);

			/* If vertex 1 and 2, as a pair, cannot be trivially rejected */
			/* by the above tests, then see if the v1-->v2 triangle edge  */
			/* intersects the cube.  Do the same for v1-->v3 and v2-->v3. */
			/* Pass to the intersection algorithm the "OR" of the outcode */
			/* bits, so that only those cube faces which are spanned by   */
			/* each triangle edge need be tested.                         */

			if ((v1_test & v2_test) == 0)
				if (p1.check_line(p2, v1_test | v2_test, side_length) == INSIDE)
					return (INSIDE);
			if ((v1_test & v3_test) == 0)
				if (p1.check_line(p3, v1_test | v3_test, side_length) == INSIDE)
					return (INSIDE);
			if ((v2_test & v3_test) == 0)
				if (p2.check_line(p3, v2_test | v3_test, side_length) == INSIDE)
					return (INSIDE);

			/* By now, we know that the triangle is not off to any side,     */
			/* and that its sides do not penetrate the cube.  We must now    */
			/* test for the cube intersecting the interior of the triangle.  */
			/* We do this by looking for intersections between the cube      */
			/* diagonals and the triangle...first finding the intersection   */
			/* of the four diagonals with the plane of the triangle, and     */
			/* then if that intersection is inside the cube, pursuing        */
			/* whether the intersection point is inside the triangle itself. */

			/* To find plane of the triangle, first perform crossproduct on  */
			/* two triangle side vectors to compute the normal vector.       */

			SUB(p1, p2, vect12);
			SUB(p1, p3, vect13);
			CROSS(vect12, vect13, norm)

			/* The normal vector "norm" X,Y,Z components are the coefficients                         */
			/* of the triangles AX + BY + CZ + D = 0 plane equation.                                  */
			/* If we solve the plane equation for X=Y=Z (a diagonal), we get                          */
			/* -D/(A+B+C) as a metric of the distance from cube center to the                         */
			/* diagonal/plane intersection.                                                           */
			/* If this is between -side_length / 2 and side_length / 2,                               */
			/* 	the intersection is inside the cube.                                                  */
			/* If so, we continue by doing a point/triangle intersection.                             */
			/* Do this for all four diagonals.                                                        */

			d = norm.x * p1.x + norm.y * p1.y + norm.z * p1.z;

			/* if one of the diagonals is parallel to the plane, the other will intersect the plane */
			if (fabs(denom = (norm.x + norm.y + norm.z)) > EPS)
			/* skip parallel diagonals to the plane; division by 0 can occur */
			{
				hitpp.x = hitpp.y = hitpp.z = d / denom;
				if (fabs(hitpp.x) <= side_length / 2)
					if (this->point_triangle_intersection(hitpp) == INSIDE)
						return (INSIDE);
			}
			if (fabs(denom = (norm.x + norm.y - norm.z)) > EPS)
			{
				hitpn.z = -(hitpn.x = hitpn.y = d / denom);
				if (fabs(hitpn.x) <= side_length / 2)
					if (this->point_triangle_intersection(hitpn) == INSIDE)
						return (INSIDE);
			}
			if (fabs(denom = (norm.x - norm.y + norm.z)) > EPS)
			{
				hitnp.y = -(hitnp.x = hitnp.z = d / denom);
				if (fabs(hitnp.x) <= side_length / 2)
					if (this->point_triangle_intersection(hitnp) == INSIDE)
						return (INSIDE);
			}
			if (fabs(denom = (norm.x - norm.y - norm.z)) > EPS)
			{
				hitnn.y = hitnn.z = -(hitnn.x = d / denom);
				if (fabs(hitnn.x) <= side_length / 2)
					if (this->point_triangle_intersection(hitnn) == INSIDE)
						return (INSIDE);
			}

			/* No edge touched the cube; no cube diagonal touched the triangle. */
			/* We're done...there was no intersection.                          */

			return (OUTSIDE);
		}

	public:
		Vector3 p1, p2, p3;
	};

	enum ShapeType
	{
		Shape_Middle = 1,
		Shape_LineA = 2,
		Shape_LineB = 3
	};

// 面片标记类型  stl_extra
#ifndef Fact_Mark
#define Fact_Mark
#define Norm_Fact (0b00000000)
#define Bound_Fact (0b00000001)
#define Area_Fact (0b00000010)
#define Tuqi_Fact (0b00000100)
#define SSLine_Fact (0b00001000)
#define SSFace_Fact (0b00010000)
#define Selected_Fact (0b10000000)
#define RPDStrong_Fact (0b01000000)
#define RPDWeak_Fact (0b00100000)
#endif // Fact_Mark

// 支架部件标记类型 stl_rpd_type
#ifndef Rpd_Side_Mark
#define Rpd_Side_Mark
#define None_Face 0
#define Lower_Face 1
#define Upper_Face 2
#define Side_Face 3
#endif // Rpd_Mark

#ifndef Rpd_Face_Mark
#define Rpd_Face_Mark
#define None_Part 0
#define RetentionMesh_Part 1
#define MajorConnector_Part 2
#define MinorConnector_Part 3
#define OcclusalRest_Part 4
#define Clasp_Part 5
#define FinishingLine_Part 6
#define Fulcrum_Part 7
#define Nail_Part 8
#define StippledWax_Part 9
#define LingualBar_Part 10
#define TextLabel_Part 11
#define Crossbar_Part 12
#endif // Rpd_Face_Mark

	union XoY_Index
	{
		long long union_id;
		struct
		{
			int X_id;
			int Y_id;
		};
		bool operator<(const XoY_Index &rt) const { return this->union_id < rt.union_id; };
		bool operator>(const XoY_Index &rt) const { return this->union_id > rt.union_id; };
		bool operator==(const XoY_Index &rt) const { return this->union_id == rt.union_id; };
		XoY_Index operator=(const XoY_Index &rt)
		{
			this->union_id = rt.union_id;
			return *this;
		};
	};

	union XYZ_Index
	{
		long long union_id;
		struct
		{
			short int X_id;
			short int Y_id;
			short int Z_id;
		};
		bool operator<(const XYZ_Index &rt) const { return this->union_id < rt.union_id; };
		bool operator>(const XYZ_Index &rt) const { return this->union_id > rt.union_id; };
		bool operator==(const XYZ_Index &rt) const { return this->X_id == rt.X_id && this->Y_id == rt.Y_id && this->Z_id == rt.Z_id; };
		XYZ_Index operator=(const XYZ_Index &rt)
		{
			this->X_id = rt.X_id;
			this->Y_id = rt.Y_id;
			this->Z_id = rt.Z_id;
			return *this;
		};
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////
	// TriangleMesh
	/////////////////////////////////////////////////////////////////////////////////////////////////
	class TriangleMesh
	{
	public:
		TriangleMesh();
		~TriangleMesh();
		TriangleMesh(const Pointf3s &points, const std::vector<Point3> &facets);
		TriangleMesh(const Pointf3s &points, const std::vector<Point3> &facets, const std::vector<int> &marks);
		TriangleMesh(const TriangleMesh &other);
		TriangleMesh &operator=(TriangleMesh other);
		void swap(TriangleMesh &other);

		bool InteractionWithOtherTriangleMesh(TriangleMesh &otherMesh, double dis = 0.2);

		void ClearMesh();
		void ReadSTLFile(const std::string &input_file);
		void write_ascii(const std::string &output_file);
		void write_binary(const std::string &output_file);
		void repair(bool auto_repair = true);
		void check_topology(bool auto_repair = true);
		float volume();
		float surface_area();
		bool is_manifold() const;
		void WriteOBJFile(const std::string &output_file);
		void transform(float *trafo3x4); // glm模型变换
		void scale(float factor);
		void scale(const Pointf3 &versor);
		void translate(float x, float y, float z);
		void rotate(float angle, const Axis &axis);
		void rotate_self(Eigen::Matrix3d rotation_matrix);
		void rotate_x(float angle);
		void rotate_y(float angle);
		void rotate_z(float angle);
		TriangleMesh rotate(Vector3 from, Vector3 to, Pointf3 &p1, Pointf3 &p2);
		TriangleMesh rotate(Eigen::Matrix3d rotation_matrix);
		void mirror(const Axis &axis);
		void mirror_x();
		void mirror_y();
		void mirror_z();
		void align_to_origin();
		void center_around_origin();
		void rotate(double angle, Point *center);
		void rotate(double angle);
		TriangleMeshPtrs split() const;
		TriangleMeshPtrs cut_by_grid(const Pointf &grid) const;
		void merge(const TriangleMesh &mesh);
		void merge_SptPoints(TriangleMesh mesh, std::string _type);
		ExPolygons horizontal_projection() const;
		ExPolygons horizontal_projection_slm(double angle);
		BoundLoops_vec GetMeshBoundLoops(double angle);
		ExPolygons Get_SelectFacets_Ex();
		Polygon convex_hull();
		BoundingBoxf3 bounding_box() const;
		void reset_repair_stats();
		bool needed_repair() const;
		size_t facets_count() const;
		void extrude_tin(float offset);
		void require_shared_vertices();
		size_t shells_count() const;
		bool UnableRepair() const;
		bool is_normal_hole() const;
		bool check_needed_repair() const;

		Pointf3 lowest_point();

		TriangleMesh GetSubMesh(std::vector<int> facets);
		TriangleMesh GetSubMesh(char Mark);

		stl_file stl;
		bool repaired;
		bool checkonly;

		static bool fact_is_mark(char extrainfo, char Mark);
		static void fact_set_mark(char &extrainfo, char Mark);
		static void fact_remove_mark(char &extrainfo, char Mark);
		static void fact_reset(char &extrainfo);

		static bool is_face_rpd_mark(char extrainfo, char side);
		static void set_face_rpd_mark(char &extrainfo, char type);
		static void reset_face_rpd_mark(char &extrainfo);
		static void parse_face_mark(int mark, int &side, int &type, int &type_id);

		std::vector<int> MarkFacts_SelectedPoint(const Pointf3 pickPos);
		std::vector<int> RemoveFacts_SelectedPoint(const Pointf3 pickPos);
		// 选择种植孔面片
		std::vector<int> MarkHole_SelectedPoint(const Pointf3 pickPos, const std::set<int> &LimitedFaces);
		// 选择种植孔上下约束圈
		std::vector<int> HoleFaceMarkFacts_SelectedPoint(const Pointf3 pickPos);
		void Clear_SelectFacts();

		int parse_mesh_type(bool upper);
		///////////////////////////////////////////////////////////////////////////////
		// 建立面片hash抽屉  加速查找
	public:
		double X_stepLen = 0.0;
		double Y_stepLen = 0.0;
		double Z_stepLen = 0.0;
		std::map<XYZ_Index, std::vector<int>> hash_facets;
		void set_step_length(double xlen, double ylen, double zlen)
		{
			X_stepLen = xlen;
			Y_stepLen = ylen;
			Z_stepLen = zlen;
			if (X_stepLen <= 0.1)
				X_stepLen = 0.1;
			if (Y_stepLen <= 0.1)
				Y_stepLen = 0.1;
			if (Z_stepLen <= 0.1)
				Z_stepLen = 0.1;
		};
		std::vector<int> get_Pointf3_gridfactes(Pointf3 _pointf3, size_t expand_ring = 0);
		std::vector<XYZ_Index> get_box_grids(BoundingBoxf3 box, double step = 0.0);
		std::vector<XYZ_Index> get_facet_grids(int tid, double step = 0.0);
		std::vector<XYZ_Index> get_line_grids(Linef3 line);
		std::vector<XYZ_Index> get_zline_grids(double x, double y);

		bool facet_is_in_XYZ_Index(TriangleFace3 facet, XYZ_Index index, double step = 0.0);

		void init_hash_facets();															  // 建立hash抽屉
		void translate_hash_facets(int x, int y, int z);									  // 平移hash抽屉
		std::map<XYZ_Index, std::vector<int>> translate_hash_facets_new(int x, int y, int z); // 不改变原有的hash抽屉，平移后得到新的hash抽屉
		std::vector<int> get_grids_facets(std::vector<XYZ_Index> grid_ids);
		std::vector<int> GetFacetIdxByZaxisFast(double x, double y);
		std::vector<int> GetFacetIdxByZaxisFast(Pointf3 p1, Pointf3 p2);

		std::set<XYZ_Index> build_mesh_voxels(double step);

		// 不加支撑的面
		std::set<int> NoSupportFaces;
		// 不加支撑是选择面片的约束圈
		std::set<int> NoSupportLimitFaces;
		////////////////////////////////////////////////////////////////////////////////////////////////

	private:
		friend class TriangleMeshSlicer<X>;
		friend class TriangleMeshSlicer<Y>;
		friend class TriangleMeshSlicer<Z>;

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// zhijia Adjust
		/////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		bool Normal_PCA(std::map<double, Vectorf3> &ClaspNor_map, bool IsReverseTuqi);

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// Spt Common
		/////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		size_t get_tree_region_idx(size_t spted_volume_idx, size_t contor_idx = 0, size_t hole_idx = 0);
		// 获取面片的法向量
		Pointf3 GetFacetNormal(int Fact_Idx);
		// 获取面片的特殊字段
		char GetFacetExtra(int Fact_Idx);
		// 计算经过点xpos，ypos的平行Z轴射线与Mesh面片相交的面片索引集合
		std::vector<int> GetFacetIdxByZaxis(double xpos, double ypos);
		// 计算经过点xpos，ypos的平行Z轴射线与面片的交点z值
		double GetPointZByZaxis(int Fact_Idx, double xpos, double ypos);
		// 通过拾取点得到该拾取点所在的面片索引号
		int GetFacetIdxByPickPoint(Pointf3 pickPos);

		// sawtooth
		bool Cale_CollsionZ_byXY(const Pointf basePos, Pointf3 &hitPos, Pointf3 &hitNor);
		// auto spt step
		// 对某个支撑点计算交线上的交点
		bool Cale_CollsionZ_bythreads(std::string _type, unsigned int _threads = boost::thread::hardware_concurrency() / 2);
		bool Cale_CollsionZ(unsigned int _index, std::string _type);
		// 获取数组大小
		int Get_PointsSize(std::string _type);
		int Get_WallsSize(std::string _type);
		int Get_WallPartSize(std::string _type, int wall_index);
		// 获取支撑线交点顶点
		Pointf3 Get_HitPos(std::string _type, unsigned int spt_index, unsigned int hit_index);
		// 获取支撑线交点法相
		Pointf3 Get_HitNol(std::string _type, unsigned int spt_index, unsigned int hit_index);
		// 获取支撑角度
		double Get_Angle(std::string _type, unsigned int spt_index);
		// 获取支撑基点顶点
		Pointf3 Get_BasePos(std::string _type, unsigned int spt_index);
		// 检测该点是否是树枝
		bool IsTreeLeaf(std::string _type, unsigned int spt_index);
		bool IsTreeBaseOne(std::string _type, unsigned int spt_index);
		// 获取该点是否命中
		size_t Get_HitPointNum(std::string _type, unsigned int spt_index);
		size_t Get_TreeBaseBranchNum(unsigned int spt_index);
		bool Get_Valid(std::string _type, unsigned int spt_index);
		// 获取支撑点是否在自动支撑区域内
		bool IsHitPoint_InThreshold(std::string _type, unsigned int spt_index, unsigned int hit_index);
		bool IsHitPoint_InRPDRetenion(std::string _type, unsigned int spt_index, unsigned int hit_index);

		// 清空缓存
		bool ClearBufferDate(std::string _type);
		// 计算三角面片的面积
		static double Calculate_Facet_Area(stl_facet facet);
		// 计算三角面片的周长
		static double Calculate_Sides_Length(stl_facet facet);
		// 计算三角面片的最长边和最短边比值
		static double Calculate_MaxMinSide_Scale(stl_facet facet);
		//
		static void filter_ex_holes(double fliter_hole_area, ExPolygons &exs);
		static void filter_ex_conters(double fliter_area, ExPolygons &exs);
		static void make_expolygons(const Polygons &loops, ExPolygons *slices, bool abandon_hole_loops = false);
		static void make_expolygons2(const Polygons &loops, ExPolygons *slices, double _offset);
		static ExPolygons Offset_ExboundPoints(ExPolygons exps, double offset_distance);

		double get_min_distance_point_to_facets(std::vector<int> facets_index, Pointf3 point);
		// 对某个支撑点计算交线上的交点
		bool Cale_CollsionZ(SPT_pointf &sptP);

	private:
		// 判断点是否在三角面片上
		// Ray_type::None  精确判断
		// Ray_type::X Y Z  沿X，Y，Z射线投影成平面后判断
		bool PointinTriangle(int Fact_Idx, stl_vertex P, Ray_type ray_type);
		bool Pre_PointinTriangle(int Fact_Idx, stl_vertex Zaixs, Ray_type ray_type);
		// 计算经过点xpos，ypos的平行Z轴射线与Mesh面片相交的交点z值集合(从小到大排序)
		std::vector<stl_point_in_facet> GetZvecByZaxis(double xpos, double ypos);
		// 类型分拣
		std::vector<SPT_pointf> *get_points(std::string _type);
		// 从一个边界多边形上提取边界采样点
		unsigned int make_PolylinePoints(std::string _type, Polyline polyline, double Points_distance, size_t Loop_Id);
		// 计算两个邻接面片法向量夹角
		double facet_angle(const stl_facet &facet, const stl_facet &facet_neighbor);
		// 特定类型的面片进行退化处理
		bool Area_FilterFacets(char facet_type, std::vector<int> facets_index);
		// 特定类型的面片进行反向退化处理
		bool reArea_FilterFacets(char facet_type, std::vector<int> facets_index);
		// 特定类型的面片进行角度过滤
		bool Angle_FilterFacets(char facet_type, std::vector<int> facets_index, double filter_angle);
		// 初始化面片集合
		std::vector<int> Get_facets(char facet_type, bool Iscomplementaryset);

		int Get_rpd_facet_mark_count(std::vector<int> facets_index, int facet_mark, int facet_side);
		int Get_facet_mark_count(std::vector<int> facets_index, char facet_type);
		bool ExpandMarkFacets(char facet_type);
		bool ExpandMarkFacets_Anyway(char facet_type);
		bool ExpandMarkFacets_Special(char facet_type);
		bool ContractMarkFacets(char facet_type);
		std::vector<int> ExpandBaseFacets(std::vector<int> baseFace_IDs);
		std::vector<int> ExpandSegLine2Facets(BoundSegments_loop Seg_loop, int Expend_Num);
		std::vector<int> ExpandSegLine2Facets_Region(BoundSegments_loop Seg_loop, int Expend_Num);
		BoundSegments_loops ExpandSegLine_by_Facets(std::vector<int> facets, int Expend_Num);
		std::vector<int> ExpandSegLine2Facets(BoundSegments_loops Seg_loops, int Expend_Num);
		BoundSegments_loops ExpandedFacets2SegLine(std::vector<int> ExpandedFacets);
		std::vector<int> ExpandFacetbyRing(int fact_index, int LoopNum);
		std::vector<int> ExpandFacetbyEdge(int fact_index, int LoopNum);
		bool FaceIndexValid(int fact_index);

	public:
		std::vector<int> ExpandFacetbyRing_BFS(int fact_index, int LoopNum);
		//////////////////////////////////////////////////////////////////////
		// 支撑点简化
		//////////////////////////////////////////////////////////////////////

	public:
		bool support_points_simplify(
			double boundary_distance,
			double grid_distance,
			double tuqi_distance,
			double overhang_distance,
			int min_percent,
			int marknum);

	private:
		void support_points_simplify_byCircle(std::string s_type, std::string d_type, double min_distance);
		void support_points_simplify_byBox(std::string s_type, std::string d_type, double X_distance, double Y_distance);
		void simplify_TreeBase();

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// Tree
		/////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		bool Pre_GeneralTreePoint(bool IsMuliti, int branch_Num, double _offset_z, double _min_angel = 45, double _min_z = 0.1, double _max_dis = 3.0);

	public:
		std::vector<SPT_pointf> TreePoints; // 树基点
	private:
		double offset_z;
		double min_angel;
		double min_z;
		double max_dis;

	private:
		bool Pre_GeneralTreePoint(std::string _type, int branch_Num);
		double Get_treebaseZ(Pointf treeBase, Pointf BanchPoint, double BanchZ, double offset_z, double min_angel = 45);
		bool Cale_treebaseZ(SPT_pointf &sptP);
		bool CheckRegion(std::string _type, std::map<unsigned int, std::vector<SPT_pointf *>> &map_pPoints);
		SPT_pointf *find_onepoint_Nearest(Pointf3 treebasePoint, std::list<SPT_pointf *> &raw_points, double max_dis = 3.0);
		Pointf3 Cale_treebase(std::vector<SPT_pointf *> &banch_points);
		bool find_onetree(int branch_Num,
						  std::list<SPT_pointf *> &raw_points,
						  std::vector<SPT_pointf *> &tree_points,
						  Pointf &treebasePoint);

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// Gird
		/////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		// 网格点生成预处理
		bool Pre_GeneralGridPoint(
			size_t spted_volume_id,
			double angel,
			double step,
			int X_num,
			int Y_num,
			bool notIntuqi);
		bool Pre_CustomGridPoint(
			size_t spted_volume_id,
			double angel,
			double step,
			int X_num,
			int Y_num,
			bool notIntuqi);

	private:
		std::vector<SPT_pointf> GirdPoints; // 网格点集合

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// Overhang
		/////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		// 获取悬垂点, angle_threshold噪声点角度阈值，min_percent支撑点简化间距阈值
		bool Mark_OverhangPoints(size_t spted_volume_id, double angle_threshold);
		// 获取悬垂线
		bool Mark_OverhangLines(double angel_throld);
		// 生成悬垂线间隔采样点
		bool Make_OverhangLinesPoints(size_t spted_volume_id, double overhang_distance);
		// 获取悬垂面
		bool Mark_OverhangFacet(double angle_throld, double step);

	public:
		BoundLines_vec OverhangLines;
		Polylines Overhang_polylines; // 悬垂线多段线，放大处理

	private:
		std::vector<SPT_pointf> OverhangPoints;		 // 悬垂点支撑点集合
		std::vector<SPT_pointf> OverhangLinesPoints; // 悬垂线支撑点集合
	private:
		// 获取该面片上z值最小的顶点
		int get_zmin_vertex_of_facet(const stl_facet facet);
		// 获取三角面片上某一条边与xoy平面的夹角
		double get_angle_to_xoy(const Linef3 edge);
		// 悬垂线连接成为多段线
		bool find_onePolyline(BoundLines_vec &Orgin_Lines, Polyline &onePolyline);
		// 面片是否包含某个顶点
		bool facet_contain_vertex(const stl_facet facet, const stl_vertex vertex);
		// 顶点是否是面片上的z轴最低点
		bool is_zmin_vertex_of_facet(const stl_facet facet, const stl_vertex vertex);
		// 找到共顶点的邻接面片
		int index_of_neighbor_with_same_vertex(const int index, const stl_vertex vertex1, const stl_vertex vertex2);

	public:
		Pointf3 center_on_mesh();	   // 计算mesh的中心点
		Pointf3 mesh_average_normal(); // 计算mesh的平均法相

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// selected
		////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		bool Make_SelectedFacets_ThinLines(size_t spted_volume_id, double Points_distance);
		bool Make_SelectedFacets_ThinBounds(size_t spted_volume_id, double Points_distance);
		ExPolygons custom_simply_boundary;
		bool Make_Occlusa_ThinLines(
			size_t spted_volume_id,
			double Points_distance,
			double Offset_distance,
			std::string type = "selected",
			double tree_distance = 1.0);
		bool Make_SelectedFacets_Center_and_Normal();

		bool MergeCorrespondingMesh(TriangleMesh *Meshtomerge); // 将两个相同的面片但之间有一定距离的面片合并成一个trianglemesh

	public:
		Pointf3 SelectedCenter;
		Pointf3 SelectedNormal;
		std::vector<SPT_pointf> SelectedPoints;
		std::vector<SPT_pointf> custom_GridPoints;
		std::vector<SPT_pointf> RetionPart_Points;
		std::vector<SPT_pointf> LatticeMesh_Points;	 // 极简支撑晶格取点，不包括固位网。
		std::vector<SPT_pointf> CrossBar_TreePoints; // 极简支撑横杆取点

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// Lattice Support
		////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		LatticeSPT_Pointf3s Lspt_Points;		  // 晶体支撑的接触点集合
		CrossbarPointPairs Crossbar_Points;		  // 横杆的生成点集合
		CrossbarPointPairs Assit_Crossbar_Points; // 辅助横杆的生成点集合
	public:
		// 获取某个角度下的边界线集合
		bool Get_BoundSegments(char facet_type, double angle, FacetSegments_map &boundsegs_map);
		// 通过获取不同角度的边界，提取一定重复数以上的线段集合
		int Get_SuspendPoints(
			const int angel_step,	// 角度步长
			const int boundNumMin,	// 识别为悬垂边的重复数阈值
			const int suspend_angel // 识别为悬垂面的角度阈值
		);
		// 对支撑结果进行简化
		bool Simply_SuspendPoints(
			const double Xcell_Length,		 // 简化晶格X轴向长度
			const double Ycell_Length,		 // 简化晶格Y轴向长度
			const double Zcell_Length = 0.0, // 简化晶格Z轴向长度 可选
			const double reten_factor = 1.0, // 固位网区域支撑简化加强系数
			const double tuqi_factor = 1.0,	 // 花纹区域支撑简化加强系数
			const bool IsShowCell = false	 // 显示晶格
		);
		// 提取某个类型的晶格LatticeSPT
		LatticeSPT_Pointf3s Filter_LatticeSPT_byRPDPart(
			stl_face_type _type,
			bool remove = false);
		// 对支撑结果进行修正
		bool Modify_SuspendPoints(
			const double Xcell_Length,
			const double Ycell_Length,
			const double Zcell_Length);
		void _Modify_SuspendPoints(
			size_t spt_id,
			const double Xcell_Length,
			const double Ycell_Length,
			const double Zcell_Length);

		//
		int Lspt2SPT(std::string _type);
		int SPT2Lspt(std::string _type);
		int Get_LsptPoints();
		int Make_CorssBar_SPT();
		int Make_Suspend_SPT();
		/////////////////////////////////////////////////////////////////////////////////////////////////
		// boundary
		////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		// 用法相角度阈值对三角面片进行分类标记 对标记的三角面片进行退化处理 生成边界线段
		bool Mark_FilterFacets(double angel_throld);
		std::vector<int> MarkFacetsByAngle(const double angel_throld, const char fact_mark, const bool all_reset = true);
		// 生成边界间隔采样点
		bool Make_BoundLoops(
			size_t spted_volume_id,
			double angle_facts,
			double Points_distance,
			double boundary_offset,
			double boundgrid_distance,
			bool make_outline,
			bool make_boundary,
			bool make_thinwalls);
		ExPolygons Make_ThinWalls(ExPolygons exps, size_t spted_volume_id, std::string _type, double _wedth, double _step);
		bool Make_ExCenterLine(ExPolygons thinLoops, std::string _type, size_t spted_volume_id, double _step);
		bool Make_WallLoops(std::string _type);

	public:
		// 存边界线段的容器
		BoundLines_vec boundLines_vec; // 边界线段集合
		BoundLoops_vec boundLoops_vec; //  边界环集合
		BoundLoops_vec wallLoops_vec;  // 墙顶点集合

		Polygons boundLoops;	 // 边界环集合 数据经过放大处理  float->int
		ExPolygons exboundLoops; // 缩小的边界环集合 网格生成时用，数据经过放大处理  float->int
	private:
		std::vector<SPT_pointf> BoundaryPoints; // 边界点集合

	private:
		Linef3 GetFacets_OneEdge(const stl_facet &Selected_facet, const stl_facet &unSelected_facet);
		BoundLoop find_oneLoop(BoundLines_vec &Orgin_Lines);
		Polygon find_oneLoop_polygon(BoundLines_vec &Orgin_Lines);
		bool General_BoundLoop_Line(char facet_type, BoundLines_vec &boundlines);
		bool make_ExboundPoints(ExPolygons exps, double Points_distance, size_t spted_volume_id);
		int PointIn_exboundLoops(Slic3r::Pointf checkPoint);
		int PointIn_customboundLoops(Slic3r::Pointf checkPoint);

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// RPD Part
		////////////////////////////////////////////////////////////////////////////////////////////////
	private:
		// 从给定的线段开始深度优先搜索以查找环byGuoXiao
		bool isAdjacent(const Linef3 &l1, const Linef3 &l2);
		bool dfs(BoundLines_vec &Orgin_Lines, int current, int start, std::unordered_set<int> &visited, std::stack<int> &path);
		bool hasCycle(BoundLines_vec &Orgin_Lines, std::vector<int> &cyclePath, int start);
		void removeCycleLines(BoundLines_vec &Orgin_Lines, const std::vector<int> &cyclePath);

	public:
		BoundLoops_vec ShowLoops;

		int Mark_RPDPart(
			stl_face_side _side,
			stl_face_type _type,
			stl_face_type_id _id);
		int Mark_RPDPart_Selected(
			stl_face_side _side,
			stl_face_type _type,
			stl_face_type_id _id);
		int UnMark_RPDPart_Selected(
			stl_face_side _side,
			stl_face_type _type,
			stl_face_type_id _id);
		int Make_RPDPart_BoundWall(double Points_distance, double filter_angle);
		std::vector<std::vector<int>> RPD_selected_facets;
		bool Get_RPDRegion_By_Facets(double filter_angle, int expand_degree, int back_expand_degree);
		bool General_BoundWall_By_SelectedFacets(double Points_distance);
		int Make_Selected_Region_GridPoints(double filter_angle = 90.0, bool is_filter_angle = false);
		void draw_svg_byfacets(std::vector<int> facets, std::string name, int color = 1);
		void draw_svg_byBoundSegments_loops(BoundSegments_loops loops, std::string name, int color = 1);

		bool make_auto_crossbar(
			double distance,
			double minLength,
			double minDistanceToRpd,
			double Crossbar_raduis, // 半径
			double slice_assist_dis,
			double bound_assist_dis,
			int bound_constrat_loop);
		bool Is_CrossBar_CanUse(
			CrossbarPointPair cb_pointpair,
			std::vector<int> facets_index,
			double minLength,
			double maxLength,
			double minDistanceToRpd,
			double Crossbar_raduis);
		BoundSegments_loops Get_auto_crossbar_boundloops();
		std::vector<int> Get_auto_crossbar_facets();
		std::vector<int> Get_special_crossbar_facets(std::vector<int> facets_index, int Max_Contract_Count);
		PlaneZ_CrossBars Get_PlaneZ_CrossBars_ByZ(BoundSegments_loops bound_loops, double z);
		bool General_Slice_Assist_CrossBar(PlaneZ_CrossBars &_planeZ, double Point_distance);
		bool Link_Assist_CrossBar(Pointf3 start_p, CrossbarPointPair major_pointpair, CrossbarPointPair &assit_pointpair);
		bool Add_SepcialSegLoops(BoundSegments_loops bound_loops, std::map<double, PlaneZ_CrossBars> &zBars, double Point_distance);
		bool Add_ZSliceSegs(std::vector<int> facets_index, PlaneZ_CrossBars &_zBars);
		bool is_intersect(Pointf3 p1, Pointf3 p2, double r, int ringSize); // 检测两点之间的连线半径为r的范围内，是否与mesh有交点
		bool is_intersect(Pointf3 p1, Pointf3 p2, Pointf3 &p3);			   // 检测两点之间的连线是否与mesh有交点
		bool is_intersect_new(Pointf3 p1, Pointf3 p2, Pointf3 &p3);		   // P1, P2要有相同的x, y值，否则失效
		Pointf3 rotate_pointf3(Eigen::Matrix3d rotation_matrix, Pointf3 p);
		void Set_facets_Selected(std::vector<int> facets, bool remove_selected = false);
		void Set_2dimfacets_Selected(int dim, bool remove_selected = false, double filter_angle = 90.0);

	private:
		// 将支架转换为face mark；
		std::vector<int> MarkFacetsByRPDPart(
			const stl_face_side _side,	// 支架上下表面标记
			const stl_face_type _type,	// 支架的各部件标记
			const stl_face_type_id _id, // 支架的各部件id
			const char target_fact_mark,
			bool IsRemoveOther = true);
		// 获取某一个边界合集
		BoundSegments_vec Get_BoundSegments(char facet_type);
		BoundSegments_loop find_oneLoop(BoundSegments_vec &Orgin_Segs);
		BoundLoop Sampling_SegmentsLoop(BoundSegments_loop Seg_loop, double Points_distance);
		BoundLoop Sampling_SegmentsLoop2(BoundSegments_loop Seg_loop, double Points_distance);
		BoundLoop Sampling_SegmentsLoop3(BoundLoop Seg_loop, double Points_distance);

	public:
		void ResetMark_RPDStrong();
		void ResetMark_Selected();
		void ResetMark_By_Type(const char _type);
		void ResetMark_RPDWeak();

		// 各种过滤
		BoundSegments_loops CutLoopsBySize(BoundSegments_loops Seg_loops, int min_size);
		BoundSegments_loops CutLoopsByFaceAngle(BoundSegments_loops Seg_loops, double filter_angle);
		BoundSegments_loops CutLoopByFaceAngle(BoundSegments_loop Seg_loop, double filter_angle);

		BoundSegments_loops CutLoopsByRPDType(BoundSegments_loops Seg_loops, const stl_face_type _type);
		BoundSegments_loops CutLoopsByFaceMark(BoundSegments_loops Seg_loops, const char _type);
		BoundSegments_loops CutLoopByRPDType(BoundSegments_loop Seg_loop, const stl_face_type _type);
		BoundSegments_loops CutLoopByFaceMark(BoundSegments_loop Seg_loop, const char _type);
		bool IsSegRPDType(BoundSegment loopSeg, const stl_face_type _type);
		bool IsSegFaceMark(BoundSegment loopSeg, const char _type);
		bool IsFaceNerRPDType(int Mark_faceIndex, const stl_face_type _type);
		bool IsFaceNerFaceMark(int Mark_faceIndex, const char _type);

		BoundSegments_loops CutLoopsByLoop(BoundSegments_loops Seg_loops);
		BoundSegments_loops CutLoop(BoundSegments_loop Seg_loop);
		BoundSegments_loops CutLoopByMajorLoopAndHole(BoundSegments_loop Seg_loop, Linef3 midLinef3, bool IsLoop, bool IsFront);
		BoundSegments_loops CutLoopByMajorLoopAndHole_oldversion(BoundSegments_loop Seg_loop, Linef3 midLinef3, bool IsLoop);
		BoundSegments_loops CutLoopsByMajorLoopAndHole(BoundSegments_loops Seg_loops, Linef3 midLinef3);
		BoundSegments_loops CutLoopsByMajorLoopAndHole_oldversion(BoundSegments_loops Seg_loops, Linef3 midLinef3);
		double Get_BoundLoops_Max_Y(BoundSegments_loops Seg_loops);
		void FilterLoopsByIPD(BoundSegments_loops &bound_loops);

		bool GetMaxLoopAndHole(BoundSegments_loops Seg_loops, int &MaxLoop_index, int &MaxHole_index);
		std::pair<int, int> CutBoundLoop_by_MidPoint(BoundSegments_loop Seg_loop, Linef3 midLinef3, bool IsLoop);
		std::pair<int, int> CutBoundLoop_by_LeftRight(BoundSegments_loop Seg_loop, Linef3 midLinef3);
		bool Is_ClockWise(BoundSegments_loop Seg_loop);

		BoundingBoxf3 GetBoundingBox(BoundSegments_loops loops);
		BoundSegments_vec getLoops(BoundSegments_loop loop, double z);
		double DistanceBetween2Line(Linef3 line1, Linef3 line2);

	public:
		void FilterFacetsByIPD(std::vector<int> Selected_Facets, double filter_angle = 90.0);

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// tuqi
		////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		// 标记面积小于阈值的面片，挑出边缘
		bool Area_Filter(double area, bool _in, bool _out, bool order);
		// 支架流程
		bool make_tuqiLoops(size_t spted_volume_id, int threshold, double tuqiGridDistance, double tuqiDistance, double tuqi_width, double min_length, double resolution, bool donot_makepoints);

	public:
		BoundLines_vec tuqiLines_vec; // 突起线段集合
		BoundLoops_vec tuqiLoops_vec; // 突起环集合
		Polygons tuqiLoops;			  // 突起环集合，数据经过放大处理，float->int
		ExPolygons extuqiLoops;		  // 扩大的突起环集合，该区域单独添加支撑，网格生成时需要排除该区域

	private:
		std::vector<SPT_pointf> tuqiPoints;	 // 突起间隔点集合
		std::vector<int> tuqi_facets_index;	 // 凸起面片集合，面片退化用
		std::vector<int> other_facets_index; // 非突起面片集合，面片反向退化用
	private:
		// 突起区域支撑点计算
		bool make_tuqiPoints(size_t spted_volume_id, double tuqiDistance, double tuqi_width, double min_length, double resolution);
		bool find_onetuqiLoop(BoundLines_vec &Orgin_Lines, Polygon &oneLoop_polygon);
		int PointIn_tuqiLoops(Slic3r::Pointf checkPoint);

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// Num_mark
		////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		bool make_numPoints(size_t spted_volume_id);
		static ExPolygon get_maxEx(ExPolygons exs);	  // 取得ExPolygons面积最大那个ExPolygon
		static Pointf get_ExMidPoint(ExPolygons exs); // 取得ExPolygons中的近似中心点
		static std::vector<Pointf> get_ExPoints(ExPolygons exs, double offset_val);

	private:
		std::vector<SPT_pointf> numPoints;

		/////////////////////////////////////////////////////////////////////////////////////////////////
		// ZhongZhi
		////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		std::vector<Pointf3> centers;
		std::vector<stl_file> loopmeshs;
		std::map<std::pair<size_t, size_t>, double> edge_normal;
		std::set<size_t> Selected_Triangles;
		void draw_Plane(size_t select_idx, const std::set<int> &draw_Plane);
		void convex_Hull();
		bool IsSelected(size_t tid)
		{
			return Selected_Triangles.find(tid) != Selected_Triangles.end();
		}
		bool triFilterF(size_t tid, size_t ner_tid);
		void FloodFill(size_t Seeds, const std::set<int> &LimitedFaces);

	private:
		typedef void (*pCommandForLibigl)();
	};

	enum SphereType
	{
		SpWhole,
		SpTop,
		SpBottom
	};
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TriangleMeshGeneral 定义
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class TriangleMeshGeneral
	{
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// static
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	public:
		static TriangleMesh make_sphere(double rho, double fa = 36, SphereType stype = SpWhole);
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	public:
		TriangleMeshGeneral();
		~TriangleMeshGeneral();
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// TriangleMeshSlicer 定义
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum FacetEdgeType
	{
		feNone,
		feTop,
		feBottom,
		feHorizontal
	};

	class IntersectionPoint : public Point
	{
	public:
		int point_id;
		int edge_id;
		IntersectionPoint() : point_id(-1), edge_id(-1) {};
	};

	class IntersectionLine : public Line
	{
	public:
		int a_id;
		int b_id;
		int edge_a_id;
		int edge_b_id;
		FacetEdgeType edge_type;
		bool skip;
		IntersectionLine() : a_id(-1), b_id(-1), edge_a_id(-1), edge_b_id(-1), edge_type(feNone), skip(false) {};
		void reverse() // 翻转交线方向  慎用
		{
			this->Line::reverse();
			std::swap(a_id, b_id);
			std::swap(edge_a_id, edge_a_id);
		};
	};
	typedef std::vector<IntersectionLine> IntersectionLines;
	typedef std::vector<IntersectionLine *> IntersectionLinePtrs;

	template <Axis A>
	class TriangleMeshSlicer
	{
	public:
		TriangleMesh *mesh;
		TriangleMeshSlicer(TriangleMesh *_mesh);
		~TriangleMeshSlicer();
		boost::thread_group **worker_pp; // 记录线程组管理器
		boost::thread_group *worker_p;	 // 用于worker_pp 的初始化，无实际意义
		// 非实体的切片函数
		void slice(const std::vector<float> &z, std::vector<Polylines> *layers, int threads = boost::thread::hardware_concurrency() / 2) const;
		// 实体的切片函数
		void slice(const std::vector<float> &z, std::vector<Polygons> *layers, int threads = boost::thread::hardware_concurrency() / 2) const;
		void slice(const std::vector<float> &z, std::vector<ExPolygons> *layers, int threads = boost::thread::hardware_concurrency() / 2) const;
		void slice(float z, ExPolygons *slices, int threads = boost::thread::hardware_concurrency() / 2) const;
		void slice_facet(float slice_z, const stl_facet &facet, const int &facet_idx,
						 const float &min_z, const float &max_z, std::vector<IntersectionLine> *lines,
						 boost::mutex *lines_mutex = NULL) const;
		void slice_facet_nonsolid(float slice_z, const stl_facet &facet, const int &facet_idx,
								  const float &min_z, const float &max_z, std::vector<IntersectionLine> *lines,
								  boost::mutex *lines_mutex = NULL) const;

		void cut(float z, TriangleMesh *upper, TriangleMesh *lower) const;

	private:
		typedef std::vector<std::vector<int>> t_facets_edges;
		t_facets_edges facets_edges; // facet_id --> edge_id
		stl_vertex *v_scaled_shared; // point_id --> point
		void _slice_do(size_t facet_idx, std::vector<IntersectionLines> *lines, boost::mutex *lines_mutex, const std::vector<float> &z) const;
		void _make_loops_do(size_t i, std::vector<IntersectionLines> *lines, std::vector<Polygons> *layers) const;
		void make_loops(std::vector<IntersectionLine> &lines, Polygons *loops) const;
		void make_loops_force(std::vector<IntersectionLinePtrs> &noloop_lines, Polygons *loops) const;
		void make_expolygons(const Polygons &loops, ExPolygons *slices) const;
		void make_expolygons_simple(std::vector<IntersectionLine> &lines, ExPolygons *slices) const;
		void make_expolygons(std::vector<IntersectionLine> &lines, ExPolygons *slices) const;
		void _make_expolygons_do(size_t i, std::vector<Polygons> *layers_p, std::vector<ExPolygons> *layers_ex) const;

		float &_x(stl_vertex &vertex) const;
		float &_y(stl_vertex &vertex) const;
		float &_z(stl_vertex &vertex) const;
		const float &_x(stl_vertex const &vertex) const;
		const float &_y(stl_vertex const &vertex) const;
		const float &_z(stl_vertex const &vertex) const;

		bool is_nonsolid;
	};

	template <>
	inline float &TriangleMeshSlicer<X>::_x(stl_vertex &vertex) const { return vertex.y; }
	template <>
	inline float &TriangleMeshSlicer<X>::_y(stl_vertex &vertex) const { return vertex.z; }
	template <>
	inline float &TriangleMeshSlicer<X>::_z(stl_vertex &vertex) const { return vertex.x; }
	template <>
	inline float const &TriangleMeshSlicer<X>::_x(stl_vertex const &vertex) const { return vertex.y; }
	template <>
	inline float const &TriangleMeshSlicer<X>::_y(stl_vertex const &vertex) const { return vertex.z; }
	template <>
	inline float const &TriangleMeshSlicer<X>::_z(stl_vertex const &vertex) const { return vertex.x; }

	template <>
	inline float &TriangleMeshSlicer<Y>::_x(stl_vertex &vertex) const { return vertex.z; }
	template <>
	inline float &TriangleMeshSlicer<Y>::_y(stl_vertex &vertex) const { return vertex.x; }
	template <>
	inline float &TriangleMeshSlicer<Y>::_z(stl_vertex &vertex) const { return vertex.y; }
	template <>
	inline float const &TriangleMeshSlicer<Y>::_x(stl_vertex const &vertex) const { return vertex.z; }
	template <>
	inline float const &TriangleMeshSlicer<Y>::_y(stl_vertex const &vertex) const { return vertex.x; }
	template <>
	inline float const &TriangleMeshSlicer<Y>::_z(stl_vertex const &vertex) const { return vertex.y; }

	template <>
	inline float &TriangleMeshSlicer<Z>::_x(stl_vertex &vertex) const { return vertex.x; }
	template <>
	inline float &TriangleMeshSlicer<Z>::_y(stl_vertex &vertex) const { return vertex.y; }
	template <>
	inline float &TriangleMeshSlicer<Z>::_z(stl_vertex &vertex) const { return vertex.z; }
	template <>
	inline float const &TriangleMeshSlicer<Z>::_x(stl_vertex const &vertex) const { return vertex.x; }
	template <>
	inline float const &TriangleMeshSlicer<Z>::_y(stl_vertex const &vertex) const { return vertex.y; }
	template <>
	inline float const &TriangleMeshSlicer<Z>::_z(stl_vertex const &vertex) const { return vertex.z; }

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////
	// 晶格支撑点简化器
	/////////////////////////////////////////////////////////////////////////////////////////////////

	class LatticeSPT_Simply
	{
	public:
		LatticeSPT_Simply(double _xstep, double _ystep) : X_stepLen(_xstep), Y_stepLen(_ystep), Z_stepLen(0.0), has_Z_step(false) {};
		LatticeSPT_Simply(double _xstep, double _ystep, double _zstep) : X_stepLen(_xstep), Y_stepLen(_ystep), Z_stepLen(_zstep), has_Z_step(true) {};
		~LatticeSPT_Simply() {};
		// 获取简化后的点集
		std::vector<LatticeSPT_Pointf3> GetSimplyVec(std::vector<LatticeSPT_Pointf3> points_vec, bool nearMid = false);
		BoundLines_vec GetSimplyCellLines(std::vector<LatticeSPT_Pointf3> points_vec);
		bool Set_Zstep(double _zstep)
		{
			Z_stepLen = _zstep;
			has_Z_step = true;
			return true;
		};

	private:
		double X_stepLen;
		double Y_stepLen;
		double Z_stepLen;
		bool has_Z_step;
		std::map<XoY_Index, std::vector<LatticeSPT_Pointf3>> Hash_Pointf3s;
		XoY_Index GetPointID(Pointf3 _p);									  // 获取该点XOY坐标
		int GetPointZID(Pointf3 _p);										  // 获取该点Z坐标
		LatticeSPT_Pointf3 GetOnePoint(std::vector<LatticeSPT_Pointf3> _vec); // 取出权重最重的那个点
		LatticeSPT_Pointf3 GetOnePoint_MidPf3(std::vector<LatticeSPT_Pointf3> _vec, Pointf3 MidPointf3);
		LatticeSPT_Pointf3 GetOnePoint_MidP(std::vector<LatticeSPT_Pointf3> _vec, Pointf MidPointf);
		BoundLines_vec ShowCell(int Xcell_id, int Ycell_id, int Zcell_id);
		BoundLines_vec ShowCell(int Xcell_id, int Ycell_id);
		Pointf3 CellMidPoint(int Xcell_id, int Ycell_id, int Zcell_id);
		Pointf CellMidPoint(int Xcell_id, int Ycell_id);
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////
	// 晶格支撑点生成器
	/////////////////////////////////////////////////////////////////////////////////////////////////
	struct LatticeSPT_SliceLayer
	{
		size_t Layer_ID;				// 该层所在的ID
		ExPolygons Obj_Slices;			// 物体的切片轮廓
		ExPolygons Suspend_Areas;		// 悬垂区域
		std::vector<Pointf> spt_points; // 支撑点集合
	};

	class LatticeSPT_PointsGenerate
	{
	public:
		LatticeSPT_PointsGenerate(TriangleMesh _obj, double _Slice_Z, double _spt_angle, double _spt_radius) : Slice_Z(_Slice_Z), spt_angle(_spt_angle), spt_radius(_spt_radius)
		{
			OBJ_copy = _obj;
			obj_min_z = OBJ_copy.stl.stats.min.z;
			obj_max_z = OBJ_copy.stl.stats.max.z;
		}
		~LatticeSPT_PointsGenerate() {};
		size_t Slice_Obj();
		std::vector<LatticeSPT_Pointf3> GetSPT_PointsVec(); // 获取所有的支撑点集合
	private:
		TriangleMesh OBJ_copy; // 被切片的物体
		double Slice_Z;		   // 切片层厚
		double obj_min_z;
		double obj_max_z;
		std::vector<float> z_Vec;						 // 切片层高度值值集合
		std::vector<LatticeSPT_SliceLayer> slice_Layers; // 切片层集合
		double spt_angle;								 // 支撑角度阈值
		double spt_radius;								 // 支撑密度半径
	private:
		bool GetSPT_zVec();					// 从切片参数获取切片平面集合
		float GetSPT_zVal(size_t Layer_ID); // 获取某一层的切片厚度

		void _Make_SliceLayer(size_t i, std::vector<ExPolygons> *p_exs);
		ExPolygons GetDownObjArea(size_t this_layer_id, size_t down_layer_Num = 1);
		ExPolygons GetDownSptArea(size_t this_layer_id, size_t down_layer_Num = 1);
		void _preMake_SuspendAreas(size_t i, coord_t offset_slice);
		void generate_oneLayer_spt_points(size_t Ly_ID);
		void simply_oneLayer_spt_points(size_t Ly_ID);
		std::vector<Pointf> generate_spt_points(ExPolygon suspend_area);
		coord_t adjust_solid_spacing(const coord_t width, const coord_t distance, const coordf_t factor_max = 1.2);
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 支撑基类
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 抽象基类  支撑体
	class Base_Mesh
	{
	public:
		Base_Mesh() {};
		~Base_Mesh() {};
		// 纯虚函数
		virtual TriangleMesh GeneralMesh() = 0; // 构建Mesh
		virtual bool GeneralExPoints() = 0;		// 构建拓展顶点轮廓集
		virtual bool GeneralShapePoints() = 0;	// 构建基本顶点轮廓集
	};

	// 面墙基类
	class LineWall_BaseMesh : public Base_Mesh
	{
	public:
		LineWall_BaseMesh(
			Pointf3s _wallps,
			bool _IsLoop,
			double _insert_depth,
			double _tooth_depth,
			unsigned int _tooth_step,
			unsigned int _split_step,
			unsigned int _split_wedth);
		~LineWall_BaseMesh();

		TriangleMesh GeneralMesh();				  // 构建Mesh
		TriangleMesh GeneralMesh(int part_index); // 构建部分Mesh
		bool GeneralShapePoints();
		// 构造锯齿
		static bool make_tooth(
			Pointf3s &ploop_points,
			double _tooth_depth, // 有正负 正向下挖  负向上凸
			unsigned int _tooth_step,
			bool Isstagger);
		static bool make_tooth_top( // 只为上接触点进行优化使用，造型从锯齿变为梯形。
			Pointf3s &ploop_points,
			double _tooth_depth, // 有正负 正向下挖  负向上凸
			unsigned int _tooth_step,
			unsigned int _tooth_width = 0);
		static bool make_tooth_limit(
			Pointf3s &ploop_points,
			const Pointf3s plimit_points_up,
			const Pointf3s plimit_points_dw,
			double _tooth_depth, // 有正负 正向下挖  负向上凸
			unsigned int _tooth_step,
			bool Isstagger);
		static bool make_tooth_limit( // 重载版本，为六边形孔造型
			Pointf3s &ploop_points,
			Pointf3s &RecordHoleLine,
			const Pointf3s plimit_points_up,
			const Pointf3s plimit_points_dw,
			double _tooth_depth, // 有正负 正向下挖  负向上凸
			unsigned int _tooth_step,
			bool Isstagger);
		// 构造界线
		static Pointf3s make_wallline(
			const Pointf3s base_wallline,
			double wall_Z);

	public:
		double splitwall_maxz = DBL_MIN;
		double splitwall_minz = DBL_MAX;

	private:
		void clear_date();

	protected:
		bool IsLoop;

		unsigned int split_step;  // 打断间隔
		unsigned int split_wedth; // 打断宽度

		unsigned int tooth_step; // 齿间隔
		double tooth_depth;		 // 齿高度
		double insert_depth;	 // 插入高度

		Pointf3s wallbase_points;  // 墙面主干线接触顶点
		Pointf3s wallshape_points; // 墙面造型顶点

		std::list<Pointf3s> Walls_UpShape; // 墙体上半边
		std::list<Pointf3s> Walls_DwShape; // 墙体下半边
		// 构建加强筋Mesh
		TriangleMesh GeneralMesh_CrossBeam(double _wedth, double _depth);
		TriangleMesh GeneralMesh_CrossBeam(double _wedth, double _depth, int part_index);
	};

	// 面墙类
	class LineWall_Mesh : public LineWall_BaseMesh
	{
	public:
		LineWall_Mesh(
			Pointf3s _wallps,
			bool _IsLoop,
			double _insert_depth,
			double _tooth_depth,
			unsigned int _tooth_step,
			double _hole_height_step,
			unsigned int _hole_length_step,
			double _hole_depth,
			unsigned int _split_step,
			unsigned int _split_wedth) : hole_height_step(_hole_height_step),
										 hole_length_step(_hole_length_step),
										 hole_depth(_hole_depth),
										 LineWall_BaseMesh(_wallps, _IsLoop, _insert_depth, _tooth_depth, _tooth_step, _split_step, _split_wedth)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("LineWall_Mesh GeneralShapePoints or GeneralExPoints Failed");
		}
		~LineWall_Mesh() {};

	private:
		bool GeneralExPoints();																			  // 构建墙体上下半边
		bool GeneralWallLines(Pointf3s wall_top, Pointf3s wall_base, double WallHeight, bool &Isstagger); // 构建某一层墙体上下半边
	private:
		double hole_height_step;	   // 中空齿排间隔
		unsigned int hole_length_step; // 中空齿列间隔
		double hole_depth;			   // 中空齿高度
	};

	// 带六边形孔的面墙类
	class Hexhole_LineWall_Mesh : public LineWall_BaseMesh
	{
	public:
		Hexhole_LineWall_Mesh(
			Pointf3s _wallps,
			bool _IsLoop,
			double _thickness_hl,
			double _insert_depth,
			double _tooth_depth,
			double _down_tooth_depth_hl,
			unsigned int _tooth_step,
			unsigned int _tooth_width,
			double _hole_height_step,
			unsigned int _hole_length_step,
			double _hole_depth,
			unsigned int _split_step,
			unsigned int _split_wedth,
			double _hexhole_length,
			bool _If_addBeam,
			double _Beam_Width_hl) : thickness_hl(_thickness_hl),
									 tooth_width_hl(_tooth_width),
									 down_tooth_depth_hl(_down_tooth_depth_hl),
									 hole_height_step(_hole_height_step),
									 hole_length_step(_hole_length_step),
									 hole_depth(_hole_depth),
									 hexhole_length(_hexhole_length),
									 If_addBeam_hl(_If_addBeam),
									 Beam_Width_hl(_Beam_Width_hl),
									 tooth_step_hl(_tooth_step),
									 LineWall_BaseMesh(_wallps, _IsLoop, _insert_depth, _tooth_depth, _tooth_step, _split_step, _split_wedth)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Hexhole_LineWall_Mesh GeneralShapePoints or GeneralExPoints Failed");
		}
		~Hexhole_LineWall_Mesh() {};

	public:
		double hole_height_step;	   // 中空齿排间隔
		unsigned int hole_length_step; // 中空齿列间隔
		double hole_depth;			   // 中空齿高度
		bool If_addBeam_hl;
		double Beam_Width_hl;
		unsigned int tooth_step_hl;
		unsigned int tooth_width_hl;

	public:
		TriangleMesh GeneralMesh();
		TriangleMesh GeneralMesh(int part_index); // 构建部分Mesh
	private:
		bool GeneralShapePoints();
		bool GeneralExPoints();
		bool GeneralWallLines(Pointf3s wall_top, Pointf3s wall_base, double WallHeight, bool &Isstagger);

	private:
		double thickness_hl = 0.0;
		double down_tooth_depth_hl = 0.3;
		double hexhole_length = 0.5; // 六边形侧边长度
		Pointf3s line_Tooth_Top_Up;
		Pointf3s line_Tooth_Top_Dw;
	};

	// 实体墙基类
	class EntityWall_BaseMesh : public Base_Mesh
	{
	public:
		EntityWall_BaseMesh(
			Pointf3s _wallps,
			bool _IsLoop,
			double _insert_depth,
			double _tooth_depth,
			double _thickness,
			double _contact_thickness,
			unsigned int _tooth_step,
			unsigned int _hole_step,
			unsigned int _split_step,
			unsigned int _split_wedth,
			double _cross_width);
		~EntityWall_BaseMesh();

		bool CheckShapePoint();
		TriangleMesh GeneralMesh();						   // 构建Mesh
		TriangleMesh GeneralMesh(unsigned int part_index); // 构建部分Mesh
		bool GeneralShapePoints();
		// 构造锯齿
		static bool make_tooth(
			Pointf3s &ploop_points,
			double _tooth_depth, // 有正负 正向下挖  负向上凸
			unsigned int _tooth_step,
			unsigned int _hole_step);
		// 构造界线
		static Pointf3s make_wallline(
			const Pointf3s base_wallline,
			double wall_Z,
			double distance // 偏移距离
		);

	private:
		void clear_date();

	protected:
		bool IsLoop;

		unsigned int split_step;  // 打断间隔
		unsigned int split_wedth; // 打断宽度

		unsigned int tooth_step;  // 齿间隔
		unsigned int hole_step;	  // 洞间隔
		double tooth_depth;		  // 齿高度
		double insert_depth;	  // 插入高度
		double thickness;		  // 厚度
		double contact_thickness; // 接触点厚度
		double cross_width;		  // 加强筋宽度

		Pointf3s wallbase_points;	// 墙面主干线接触顶点
		Pointf3s wallshape_points;	// 墙面造型顶点
		Pointf3s wallmiddle_points; // 墙面造型中间顶点

		Pointf3s Walls_UpLeftShape;	 // 墙体上半左边
		Pointf3s Walls_UpRightShape; // 墙体上半右边
		Pointf3s Walls_CtLeftShape;	 // 墙体中半左边
		Pointf3s Walls_CtRightShape; // 墙体中半右边
		Pointf3s Walls_DwLeftShape;	 // 墙体下半左边
		Pointf3s Walls_DwRightShape; // 墙体下半右边

		// 构建加强筋Mesh
		TriangleMesh GeneralMesh_CrossBeam(double _wedth, double _depth);
		TriangleMesh GeneralMesh_CrossBeam(unsigned part_id, double _wedth, double _depth);
	};

	// 实体墙类
	class EntityWall_Mesh : public EntityWall_BaseMesh
	{
	public:
		EntityWall_Mesh(
			Pointf3s _wallps,
			bool _IsLoop,
			double _insert_depth,
			double _tooth_depth,
			double _thisckness,
			double _contact_thisckness,
			unsigned int _tooth_step,
			unsigned int _hole_step,
			double _hole_height_step,
			unsigned int _hole_length_step,
			double _hole_depth,
			unsigned int _split_step,
			unsigned int _split_wedth,
			double _cross_width) : hole_height_step(_hole_height_step),
								   hole_length_step(_hole_length_step),
								   hole_depth(_hole_depth),
								   EntityWall_BaseMesh(_wallps, _IsLoop, _insert_depth, _tooth_depth, _thisckness, _contact_thisckness, _tooth_step, _hole_step, _split_step, _split_wedth, _cross_width)
		{
			if ((GeneralShapePoints() && GeneralPoints()) == false)
				LOGINFO("EntityWall_Mesh GeneralShapePoints or GeneralExPoints Failed");
		}
		~EntityWall_Mesh() {};

	private:
		bool GeneralPoints();	// 构建墙体上下半边
		bool GeneralExPoints(); // 构建墙体上下半边
	private:
		double hole_height_step;	   // 中空齿排间隔
		unsigned int hole_length_step; // 中空齿列间隔
		double hole_depth;			   // 中空齿高度
	};

	// 单个体基类
	class Single_BaseMesh : public Base_Mesh
	{
	public:
		Single_BaseMesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg);
		~Single_BaseMesh();

		TriangleMesh GeneralMesh(); // 构建Mesh
		bool GeneralExPoints();		// 构建轮廓环

		double GetMeshHeigt() { return (upBasePoint.z - dwBasePoint.z); }; // 获取实体高度
		bool OffsetMeshFlat(bool IsDw, const Vectorf3 offsetVec);		   // 偏移实体上下表面
		bool OffsetMesh(const Vectorf3 offsetVec);						   // 偏移实体
		bool SetMeshHollow(double hollow_dentity);						   // 设置中空形状
		bool SetMeshFlatAutoFix(bool IsDw, Vectorf3 normalVec);			   // 平面自适应法相
		bool SetMeshAutoFix(Vectorf3 normalVec);						   // 平面自适应法相
		// 顶点集工具函数
		static bool ScaleLoopPoints(Pointf3s *ploop_points, const Pointf3 basePoint, double sacle_dentity); // 缩放轮廓环  顶点集  中心点  缩放比例
		static bool TranslateLoopPoints(Pointf3s *ploop_points, const Vectorf3 offsetvector);				// 平移轮廓环  顶点集  平移向量
		static bool RotatePointfs(Pointfs *shape_points, const double _reg);								// 旋转二维点集
		static bool translate_bynormal(Pointf3 &orgin_point, const Pointf3 basePoint, const Vectorf3 normalVec);

	protected:
		double shape_rotate;
		// 所有顶点必须按序 顺时针方向
		Pointfs shape_points; // 截面形状轮廓点
		Pointf3 dwBasePoint;  // 上表面接触点
		Pointf3 upBasePoint;  // 下表面接触点
		// 由基本轮廓点产生以下顶点集
		Pointf3s dwouter_points; // 下表面外轮廓顶点链表
		Pointf3s dwinner_points; // 下表面内轮廓顶点链表
		Pointf3s upouter_points; // 上表面外轮廓顶点链表
		Pointf3s upinner_points; // 上表面内轮廓顶点链表
	};

	// 实体台柱类
	class Contact_Mesh : public Single_BaseMesh
	{
	public:
		Contact_Mesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg, double _w, double _l) : w(_w), l(_l), Single_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Contact_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~Contact_Mesh() {};

	private:
		double w;
		double l;
		bool GeneralShapePoints();
	};

	// 实体锥体类
	class Pyramid_Mesh : public Single_BaseMesh
	{
	public:
		Pyramid_Mesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg, Pointfs *_pshapePoints, double _Scale_density, bool _IsDw) : pshapePoints(_pshapePoints), IsDw(_IsDw), Scale_density(_Scale_density), Single_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Pyramid_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~Pyramid_Mesh() {};

	private:
		Pointfs *pshapePoints;
		bool IsDw;
		double Scale_density;
		bool GeneralShapePoints();
		bool GeneralExPoints();
	};

	// 单个支撑基类类
	class SingleSPT_BaseMesh : public Single_BaseMesh
	{
	public:
		SingleSPT_BaseMesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg) : Single_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			pUpContact = NULL;
			pUpPyramid = NULL;
			pDwContact = NULL;
			pDwPyramid = NULL;
		};

		~SingleSPT_BaseMesh()
		{
			if (pUpContact)
				delete pUpContact;
			if (pDwContact)
				delete pDwContact;
			if (pDwPyramid)
				delete pDwPyramid;
			if (pUpPyramid)
				delete pUpPyramid;
		};

		TriangleMesh GeneralMesh(); // 构建Mesh
		// 台柱相关
		bool Add_Contact(bool IsDw, double _w, double _l, double _h);
		void Del_Contact(bool IsDw);
		// 锥体相关
		bool Add_Pyramid(bool IsDw, double _h, double _Scale_density);
		void Del_Pyramid(bool IsDw);
		//
		double GetTotalMeshHeigt();								  // 获取实体高度
		bool OffsetMeshFlat(bool IsDw, const Vectorf3 offsetVec); // 偏移实体上下表面
		bool OffsetMesh(const Vectorf3 offsetVec);				  // 偏移实体
		bool SetMeshHollow(double hollow_dentity);				  // 设置中空形状
		bool SetMeshAutoFix(bool IsDw, Vectorf3 normalVec);		  // 平面自适应法相

	private:
		Contact_Mesh *pDwContact; // 下台柱
		Contact_Mesh *pUpContact; // 上台柱
		Pyramid_Mesh *pDwPyramid; // 下锥体
		Pyramid_Mesh *pUpPyramid; // 上锥体
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 支撑具体类
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 圆柱类
	class Cylinder_Mesh : public SingleSPT_BaseMesh
	{
	public:
		// 不带上下半球的圆柱
		Cylinder_Mesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg, double _ra, int _fa) : ra(_ra), fa(_fa), Topsp(false), Btmsp(false), SingleSPT_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Cylinder_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		// 带上下半球的圆柱
		Cylinder_Mesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg, double _ra, int _fa, bool _sTop, bool _sBtm) : ra(_ra), fa(_fa), Topsp(_sTop), Btmsp(_sBtm), SingleSPT_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Cylinder_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~Cylinder_Mesh() {};
		TriangleMesh GeneralMesh();

	private:
		double ra;
		int fa;
		bool Topsp; // 是否是上圆球
		bool Btmsp; // 是否是下圆球
		bool GeneralShapePoints();
	};

	// 直方柱类
	class Cuber_Mesh : public SingleSPT_BaseMesh
	{
	public:
		Cuber_Mesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg, double _w, double _l) : w(_w), l(_l), SingleSPT_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Cuber_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~Cuber_Mesh() {};

	private:
		double w;
		double l;
		bool GeneralShapePoints();
	};

	// 数字支持单个数码管条状
	class NumMarkBar_Mesh : public SingleSPT_BaseMesh
	{
	public:
		NumMarkBar_Mesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg, double _w, double _l) : w(_w), l(_l), SingleSPT_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Cuber_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~NumMarkBar_Mesh() {};

	private:
		double w;
		double l;
		bool GeneralShapePoints();
	};

	// 十字棱柱类
	class CrossPrism_Mesh : public SingleSPT_BaseMesh
	{
	public:
		CrossPrism_Mesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg, double _w, double _l) : w(_w), l(_l), SingleSPT_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("CrossPrism_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~CrossPrism_Mesh() {};

	private:
		double w;
		double l;
		bool GeneralShapePoints();
	};

	// 非实体单片类
	enum SliceType
	{
		sTypeMiddle,
		sTypeRight,
		sTypeLeft
	};
	class SliceSingle_Mesh : public SingleSPT_BaseMesh
	{
	public:
		SliceSingle_Mesh(const Pointf3 &_dwPoint, const Pointf3 &_upPoint, double _reg, double _l, SliceType _stype) : l(_l), sType(_stype), SingleSPT_BaseMesh(_dwPoint, _upPoint, _reg)
		{
			if (sType != sTypeMiddle) // Left Right 都是半片
				l /= 2;
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("SliceSingle_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~SliceSingle_Mesh() {};

	private:
		double l;
		SliceType sType;
		bool GeneralShapePoints();
	};

	enum NumSingleType
	{
		Num_0 = 0,
		Num_1,
		Num_2,
		Num_3,
		Num_4,
		Num_5,
		Num_6,
		Num_7,
		Num_8,
		Num_9,
		Alp_A,
		Alp_B,
		Alp_C,
		Alp_D,
		Alp_E,
		Alp_F,
		Alp_G,
		Alp_H,
		Alp_I,
		Alp_J,
		Alp_K,
		Alp_L,
		Alp_M,
		Alp_N,
		Alp_O,
		Alp_P,
		Alp_Q,
		Alp_R,
		Alp_S,
		Alp_T,
		Alp_U,
		Alp_V,
		Alp_W,
		Alp_X,
		Alp_Y,
		Alp_Z,
		Alp_a,
		Alp_b,
		Alp_c,
		Alp_d,
		Alp_e,
		Alp_f,
		Alp_g,
		Alp_h,
		Alp_i,
		Alp_j,
		Alp_k,
		Alp_l,
		Alp_m,
		Alp_n,
		Alp_o,
		Alp_p,
		Alp_q,
		Alp_r,
		Alp_s,
		Alp_t,
		Alp_u,
		Alp_v,
		Alp_w,
		Alp_x,
		Alp_y,
		Alp_z,
		Sym_Min, /*-*/
		Sym_Sharp /*#*/,
		Sym_Star, /***/
		Sym_Exc /*!*/,
		Sym_Per /*%*/,
		Sym_Que /*?*/,
		Sym_At /*@*/,
		Sym_Dol /*$*/,
		Sym_Pow /*^*/,
		Sym_And /*&*/,
		Sym_Colon /*:*/,
		Sym_Plus /*+*/
	};

	enum NumPartType
	{
		NumPart_Mid = 0,
		NumPart_UpMid,
		NumPart_DwMid,
		NumPart_UnderLine,
		NumPart_UpL,
		NumPart_DwL,
		NumPart_UpR,
		NumPart_DwR
	};

	struct NumMarkPart
	{
		NumPartType part_type; // 部件类型
		Pointf3 part_point;	   // 部件顶点
		Pointf3 part_normal;   //  顶点法相

		NumMarkPart(Pointf3 _MidPoint,
					NumPartType _Type,
					double NumMark_Length,
					double NumMark_Width,
					double NumMark_gap)
		{
			part_type = _Type;
			part_point = _MidPoint;
			switch (_Type)
			{
			case Slic3r::NumPart_Mid:
				break;
			case Slic3r::NumPart_UpMid:
				part_point.y += (NumMark_Length + NumMark_Width + 2 * NumMark_gap);
				break;
			case Slic3r::NumPart_DwMid:
				part_point.y -= (NumMark_Length + NumMark_Width + 2 * NumMark_gap);
				break;
			case Slic3r::NumPart_UpL:
				part_point.x += (NumMark_Length / 2 + NumMark_Width / 2 + NumMark_gap);
				part_point.y += (NumMark_Length / 2 + NumMark_Width / 2 + NumMark_gap);
				break;
			case Slic3r::NumPart_DwL:
				part_point.x += (NumMark_Length / 2 + NumMark_Width / 2 + NumMark_gap);
				part_point.y -= (NumMark_Length / 2 + NumMark_Width / 2 + NumMark_gap);
				break;
			case Slic3r::NumPart_UpR:
				part_point.x -= (NumMark_Length / 2 + NumMark_Width / 2 + NumMark_gap);
				part_point.y += (NumMark_Length / 2 + NumMark_Width / 2 + NumMark_gap);
				break;
			case Slic3r::NumPart_DwR:
				part_point.x -= (NumMark_Length / 2 + NumMark_Width / 2 + NumMark_gap);
				part_point.y -= (NumMark_Length / 2 + NumMark_Width / 2 + NumMark_gap);
				break;
			case Slic3r::NumPart_UnderLine:
				part_point.y -= (NumMark_Length + NumMark_Width * 3 / 2 + 3 * NumMark_gap + NumMark_Width / 2);
				break;
			default:
				break;
			}
		}
	};

	// 单个数字标记类
	class NumSingle_Mesh : public Base_Mesh
	{
	public:
		NumSingle_Mesh(
			TriangleMesh *_pObj,
			Pointf3 _MidPoint,
			NumSingleType _NumType,
			double _mark_thickness,
			double _NumMark_Length,
			double _NumMark_Width,
			double _NumMark_gap,
			double _NumMark_size,
			bool _hasUnderLine)
			: MidPoint(_MidPoint),
			  NumType(_NumType),
			  pObj(_pObj),
			  mark_thickness(_mark_thickness),
			  insert_thickness(0.05),
			  NumMark_Length(_NumMark_Length * _NumMark_size),
			  NumMark_Width(_NumMark_Width * _NumMark_size),
			  NumMark_gap(_NumMark_gap * _NumMark_size),
			  NumMark_size(_NumMark_size),
			  hasUnderLine(_hasUnderLine)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("NumSingle_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};

		NumSingle_Mesh(
			TriangleMesh *_pObj,
			Pointf3 _MidPoint,
			NumSingleType _NumType,
			double _mark_thickness,
			double _insert_thickness,
			double _NumMark_Length,
			double _NumMark_Width,
			double _NumMark_gap,
			double _NumMark_size,
			bool _hasUnderLine)
			: MidPoint(_MidPoint),
			  NumType(_NumType),
			  pObj(_pObj),
			  mark_thickness(_mark_thickness),
			  insert_thickness(_insert_thickness),
			  NumMark_Length(_NumMark_Length * _NumMark_size),
			  NumMark_Width(_NumMark_Width * _NumMark_size),
			  NumMark_gap(_NumMark_gap * _NumMark_size),
			  NumMark_size(_NumMark_size),
			  hasUnderLine(_hasUnderLine)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("NumSingle_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~NumSingle_Mesh() {};
		TriangleMesh GeneralMesh();
		double GetHitOnPlat();

	public:
		Pointf3 MidPoint;				  // 中心点位置
		std::list<NumMarkPart> Num_Parts; // 部件
		NumSingleType NumType;			  // 数字标号
		TriangleMesh *pObj;				  //  实体指针
		double mark_thickness;			  // 标记的厚度  针对侧壁有效
		double insert_thickness;		  // 嵌入的厚度
		double NumMark_Length;
		double NumMark_Width;
		double NumMark_gap;
		double NumMark_size;
		bool hasUnderLine;

	private:
		TriangleMesh GeneralMesh(const NumMarkPart &_PartPoint);
		bool GeneralShapePoints();
		bool GeneralExPoints();
		bool CalePartsHit();
	};

	struct NumSingleInfo
	{
		Pointf3 MidPoint;		  // 单个数字的中心点位置
		NumSingleType SingleType; // 单个数字类型
		// 兼容字符0-9，a-z，A-Z，-
		NumSingleInfo(Pointf3 _MidPoint, char _c)
		{
			if (_c <= '9' && _c >= '0')
			{
				MidPoint = _MidPoint;
				size_t _num = (size_t)_c - (size_t)('0');
				SingleType = (NumSingleType)_num;
				LOGINFO("MidPoint = %s, _num = %d", MidPoint.dump_perl().c_str(), _num);
			}
			else if (_c <= 'z' && _c >= 'a')
			{
				MidPoint = _MidPoint;
				size_t _num = (size_t)_c - (size_t)('a') + 36;
				SingleType = (NumSingleType)_num;
				LOGINFO("MidPoint = %s, _char = %d", MidPoint.dump_perl().c_str(), _num);
			}
			else if (_c <= 'Z' && _c >= 'A')
			{
				MidPoint = _MidPoint;
				size_t _num = (size_t)_c - (size_t)('A') + 10;
				SingleType = (NumSingleType)_num;
				LOGINFO("MidPoint = %s, _char = %d", MidPoint.dump_perl().c_str(), _num);
			}
			else if (_c == '-')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Min;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '+')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Plus;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '!')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Exc;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '#')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Sharp;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '*')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Star;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '$')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Dol;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '%')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Per;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '&')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_And;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '@')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_At;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == ':')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Colon;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '^')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Pow;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else if (_c == '?')
			{
				MidPoint = _MidPoint;
				SingleType = Sym_Que;
				LOGINFO("MidPoint = %s, SingleType = [%c]", MidPoint.dump_perl().c_str(), _c);
			}
			else
			{
				LOGINFO("_num error");
			}
			// x_min = Char_Meshs[SingleType]->cy_mesh.bounding_box.min.x;
			// y_min = Char_Meshs[SingleType]->cy_mesh.bounding_box.min.y;
			// x_max = Char_Meshs[SingleType]->cy_mesh.bounding_box.max.x;
			// y_max = Char_Meshs[SingleType]->cy_mesh.bounding_box.max.y;
			// double x_Dif = x_max - x_min;
			// double y_Dif = y_max - y_min;
			// LOGINFO("_c = %c STLCharMark_size is %f,x_min is %f,x_max is %f,y_min is %f,y_max is %f", _c, STLCharMark_size, x_min, x_max, y_min, y_max);
			// LOGINFO("x_Dif is %f,y_Dif is %f", x_Dif, y_Dif);
		}
	};

	// 数字标记类
	class NumMark_Mesh : public Base_Mesh
	{
	public:
		NumMark_Mesh(TriangleMesh *_pObj,
					 Pointf3 _MidPoint,
					 size_t _Num,
					 double _mark_thickness,
					 double _NumMark_Length,
					 double _NumMark_Width,
					 double _NumMark_gap,
					 double _NumMark_Size = 1.0,
					 bool _hasUnderLine = true)
			: MidPoint(_MidPoint),
			  Num(_Num),
			  pObj(_pObj),
			  mark_thickness(_mark_thickness),
			  NumMark_Length(_NumMark_Length),
			  NumMark_Width(_NumMark_Width),
			  NumMark_gap(_NumMark_gap),
			  NumMark_Size(_NumMark_Size),
			  hasUnderLine(_hasUnderLine)
		{
			// NumMark_Length = _NumMark_Length;
			// NumMark_Width = _NumMark_Width;
			// NumMark_gap = _NumMark_gap;
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("NumMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~NumMark_Mesh() {};
		TriangleMesh GeneralMesh();
		double GetNumOnPlat();

	public:
		size_t Num;								  // 数字值
		Pointf3 MidPoint;						  // 中心点
		TriangleMesh *pObj;						  //  实体指针
		std::vector<NumSingleInfo> SingleNum_vec; // 拆解的单个数字信息集合
		double mark_Length;						  // 总体长度
		double mark_thickness;					  // 标记的厚度  针对侧壁有效
		double NumMark_Length;
		double NumMark_Width;
		double NumMark_gap;
		double NumMark_Size;
		bool hasUnderLine;

		double all_mark_Width;	// 总体长度
		double all_mark_Length; // 总体长度
		double one_mark_Length; // 单个长度
		double gap_mark_Length; // 单个长度
	private:
		bool GeneralShapePoints();
		bool GeneralExPoints();
	};

	// 侧壁支撑
	class Side_NumMark_Mesh : public Base_Mesh
	{
	public:
		Side_NumMark_Mesh(TriangleMesh _obj,
						  size_t _Num,
						  double _mark_thickness,
						  double _NumMark_Length,
						  double _NumMark_Width,
						  double _NumMark_gap,
						  double _NumMark_Size = 1.0)
			: autoGeneral(true),
			  Num(_Num),
			  obj_copy(_obj),
			  mark_thickness(_mark_thickness),
			  NumMark_Length(_NumMark_Length),
			  NumMark_Width(_NumMark_Width),
			  NumMark_gap(_NumMark_gap),
			  NumMark_Size(_NumMark_Size)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Side_NumMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};

		Side_NumMark_Mesh(TriangleMesh _obj,
						  Pointf3 _MidPoint,
						  Pointf3 _MidNormal,
						  size_t _Num,
						  double _mark_thickness,
						  double _NumMark_Length,
						  double _NumMark_Width,
						  double _NumMark_gap,
						  double _NumMark_Size = 1.0)
			: autoGeneral(false),
			  MidPoint(_MidPoint),
			  MidNormal(_MidNormal),
			  Num(_Num),
			  obj_copy(_obj),
			  mark_thickness(_mark_thickness),
			  NumMark_Length(_NumMark_Length),
			  NumMark_Width(_NumMark_Width),
			  NumMark_gap(_NumMark_gap),
			  NumMark_Size(_NumMark_Size)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Side_NumMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~Side_NumMark_Mesh() {};
		TriangleMesh GeneralMesh();

	public:
		bool autoGeneral;
		TriangleMesh obj_copy; //  实体mesh拷贝
		size_t Num;			   // 数字值
		Pointf3 MidPoint;	   // 中心点
		Pointf3 MidNormal;	   // 中心点法向

		double mark_thickness; // 标记的厚度  针对侧壁有效
		double NumMark_Length;
		double NumMark_Width;
		double NumMark_gap;
		double NumMark_Size;

	private:
		bool GeneralShapePoints() { return true; };
		bool GeneralExPoints() { return true; };

		TriangleMesh GeneralMeshAuto();
		TriangleMesh GeneralMeshManual();
	};

	// STL数字标记
	enum STL_NumType
	{
		SNum_0 = 0,
		SNum_1,
		SNum_2,
		SNum_3,
		SNum_4,
		SNum_5,
		SNum_6,
		SNum_7,
		SNum_8,
		SNum_9
	};
	// STL 单个数字标记
	class STL_NumSingle_Mesh : public Base_Mesh
	{
	public:
		STL_NumSingle_Mesh(
			TriangleMesh *_pobj,
			Pointf3 _STL_MidPoint,
			STL_NumType _NumType,
			double _STLNumbermark_thickness,
			float _STLNUmberMark_size)
			: STL_Midpoint(_STL_MidPoint),
			  NumType(_NumType),
			  pObj(_pobj),
			  insert_thickness(0.05), // 插入深度
			  STLMark_thickness(_STLNumbermark_thickness),
			  STLMark_Length(0.8 * _STLNUmberMark_size),
			  STLMark_Width(0.4 * _STLNUmberMark_size),
			  STLMark_size(_STLNUmberMark_size),
			  NumbercomZMax(-DBL_MAX),
			  NumbercomZMin(DBL_MAX),
			  NumbercomDif(0.0),
			  hit_radio(0.0)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("NumSingle_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~STL_NumSingle_Mesh() {};
		TriangleMesh GeneralMesh();

	public:
		Pointf3 STL_Midpoint;
		STL_NumType NumType;
		TriangleMesh *pObj;
		TriangleMesh mesh_STL_num;

		double STLMark_thickness;
		double insert_thickness;
		double STLMark_Length;
		double STLMark_Width;
		float STLMark_size;
		double NumbercomZMax;
		double NumbercomZMin;
		double NumbercomDif;
		double hit_radio;

	private:
		TriangleMesh GetSTLNumber(); // 读取指定文件夹下单个数字的STL模型
		bool GeneralShapePoints() { return true; };
		bool GeneralExPoints() { return true; };
	};

	struct STL_NumSingleInfo
	{
		Pointf3 MidPoint;		// 单个数字的中心点位置
		STL_NumType SingleType; // 单个数字类型
		STL_NumSingleInfo(Pointf3 _MidPoint, size_t _num)
		{
			if (_num < 10 && _num >= 0)
			{
				MidPoint = _MidPoint;
				SingleType = (STL_NumType)_num;
				LOGINFO("MidPoint = %s, _num = %d", MidPoint.dump_perl().c_str(), _num);
			}
			else
			{
				LOGINFO("_num error [%d]", _num);
			}
		}
	};
	struct finalSTLcal
	{
		unsigned int F_rotate_Num = 0;
		TriangleMesh F_mesh;
		double F_HitRadio = 0.0;
		double F_STLzDifSUM = 0.0;
		bool F_meshValid = true;
		Pointf3 F_MidPoint;
		Pointf3 F_MidPointNormal;
	};
	// STL 数字标记类
	class STL_NumMark_Mesh : public Base_Mesh
	{
	public:
		STL_NumMark_Mesh(
			TriangleMesh *_pObj,
			Pointf3 _STL_MidPoint,
			size_t _Num,
			double _STLNumbermark_thickness,
			float _STLNumberMark_size)
			: pObj(_pObj),
			  STL_MidPoint(_STL_MidPoint),
			  Num(_Num),
			  STLmark_thickness(_STLNumbermark_thickness),
			  STLNumMark_Length(0.8),
			  STLNumMark_Width(0.4),
			  STLNumMark_Size(_STLNumberMark_size),
			  STLzDifSUM(0.0),
			  HitRadioSum(0.0)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Side_NumMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~STL_NumMark_Mesh() {};
		TriangleMesh GeneralMesh();
		bool RSearch_point(size_t rotate_num, TriangleMesh ObjMesh, std::vector<finalSTLcal> *FinalCal, size_t Num, double thickness, double Size);
		double GetNumOnPlat();

	public:
		TriangleMesh *pObj;							  //  实体指针
		Pointf3 STL_MidPoint;						  // 中心点
		size_t Num;									  // 数字值
		std::vector<STL_NumSingleInfo> SingleNum_vec; // 拆解的单个数字信息集合
		double mark_Length;							  // 总体长度
		double STLmark_thickness;					  // 标记的厚度  针对侧壁有效
		double STLNumMark_Length;
		double STLNumMark_Width;
		float STLNumMark_Size;
		double STLzDifSUM;
		double HitRadioSum;

	private:
		bool GeneralShapePoints(); // 拆分数字
		bool GeneralExPoints();
	};

	// STL侧壁支撑
	class Side_STL_NumMark_Mesh : public Base_Mesh
	{
	public:
		Side_STL_NumMark_Mesh(
			TriangleMesh _pObj,
			size_t _Num,
			double _STLNumbermark_thickness,
			float _STLNumberMark_size = 1.0)
			: autoGeneral(true),
			  obj_copy(_pObj),
			  Num(_Num),
			  STL_mark_thickness(_STLNumbermark_thickness),
			  STL_NumMark_Length(0.8), // 其实没啥用，stl的中心点在构造函数确定了，然后大小在单字里放大过了。
			  STL_NumMark_Width(0.4),
			  STL_NumMark_Size(_STLNumberMark_size)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Side_STL_NumMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};

	public:
		Side_STL_NumMark_Mesh(
			TriangleMesh _obj,
			Pointf3 _MidPoint,
			Pointf3 _MidNormal,
			size_t _STL_Num,
			double _mark_thickness,
			double _STL_NumMark_Size = 1.0)
			: autoGeneral(false),
			  MidPoint(_MidPoint),
			  MidNormal(_MidNormal),
			  Num(_STL_Num),
			  obj_copy(_obj),
			  STL_mark_thickness(_mark_thickness),
			  STL_NumMark_Length(0.8),
			  STL_NumMark_Width(0.4),
			  STL_NumMark_Size(_STL_NumMark_Size)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Side_STL_NumMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~Side_STL_NumMark_Mesh() {};
		TriangleMesh GeneralMesh();

	public:
		bool autoGeneral;
		TriangleMesh obj_copy; //  实体mesh拷贝
		size_t Num;			   // 数字值
		Pointf3 MidPoint;	   // 中心点
		Pointf3 MidNormal;	   // 中心点法向

		double STL_mark_thickness; // 标记的厚度  针对侧壁有效
		double STL_NumMark_Length;
		double STL_NumMark_Width;
		double STL_NumMark_Size;

	private:
		bool GeneralShapePoints() { return true; };
		bool GeneralExPoints() { return true; };

		TriangleMesh STL_GeneralMeshAuto();
		TriangleMesh STL_GeneralMeshManual();
	};
	struct Charmark
	{
		double x = 2.0;
		double y = 3.5;
		double z = 1.0;
		double scale_x = 1.0;
		double scale_y = 1.0;
		double scale_z = 1.0;

		TriangleMesh cy_mesh;	   // mesh结构
		std::string FilePath = ""; // 读取文件路径
		// 初始化
		Charmark(std::string _filepath);
		void Print_Info();
	};

	// 特殊造型的标签存储集合
	static std::map<NumSingleType, Charmark *> Char_Meshs = std::map<NumSingleType, Charmark *>();
	typedef std::vector<NumSingleInfo> NumSingleInfos;
	// 侧壁STL支撑--支持字符和数字
	//  STL 单个数字标记
	class STL_Char_Single_Mesh : public Base_Mesh
	{
	public:
		STL_Char_Single_Mesh(
			TriangleMesh *_pobj,
			Pointf3 _STL_MidPoint,
			NumSingleType _NumType,
			double _STLCharmark_thickness,
			double _STLinsert_thickness, // insert_thickness
			double _STLCharMark_size,
			double _STLCharMark_gap,
			bool _autoGeneral = false)
			: autoGeneral(_autoGeneral),
			  STL_Midpoint(_STL_MidPoint),
			  NumType(_NumType),
			  pObj(_pobj),
			  insert_thickness(_STLinsert_thickness), // 插入深度
			  STLCharMark_thickness(_STLCharmark_thickness),
			  STLCharMark_size(_STLCharMark_size),
			  STLCharMark_gap(_STLCharMark_gap),
			  NumbercomZMax(-DBL_MAX),
			  NumbercomZMin(DBL_MAX),
			  NumbercomDif(0.0),
			  hit_radio(0.0)
		{
			LOGINFO("0208 STL_Char_Single_Mesh start!");
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("NumSingle_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~STL_Char_Single_Mesh() {};
		TriangleMesh GeneralMesh();

	public:
		Pointf3 STL_Midpoint;
		NumSingleType NumType;
		TriangleMesh *pObj;
		double STLCharMark_thickness;
		double insert_thickness;
		double STLCharMark_size;
		double NumbercomZMax;
		double NumbercomZMin;
		double NumbercomDif;
		double hit_radio;
		double STLCharMark_gap;
		bool autoGeneral;
		bool singeCharvaild = true;

	private:
		// NumSingleType getTypeFromChar(char s);
		// static bool Init_Special_Charmark_Mesh();
		TriangleMesh GetSTLChar(); // 读取指定文件夹下单个数字的STL模型
		bool GeneralShapePoints() { return true; };
		bool GeneralExPoints() { return true; };
	};

	// STL字符标记类
	class STL_CharMark_Mesh : public Base_Mesh
	{
	public:
		STL_CharMark_Mesh(
			TriangleMesh *_pObj,
			Pointf3 _STL_MidPoint,
			std::string _STL_Str,
			double _STL_Charmark_thickness,
			double _STLInsert_thickness,
			double _STL_CharMark_Length, // 字符高度
			double _STL_CharMark_Width,	 // 字符宽度
			double _STL_CharMark_Size,
			double _STL_CharMark_gap,
			double _autoGeneral = false)
			: autoGeneral(_autoGeneral),
			  pObj(_pObj),
			  STL_MidPoint(_STL_MidPoint),
			  STL_Str(_STL_Str),
			  STLCharmark_thickness(_STL_Charmark_thickness),
			  STLInsert_thickness(_STLInsert_thickness),
			  STLCharMark_Length(_STL_CharMark_Length),
			  STLCharMark_Width(_STL_CharMark_Width),
			  STLCharMark_Size(_STL_CharMark_Size),
			  STLCharMark_gap(_STL_CharMark_gap),
			  STLzDifSUM(0.0),
			  HitRadioSum(0.0)
		{
			LOGINFO("0208 STL_CharMark_Mesh start!");
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Side_NumMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~STL_CharMark_Mesh() {};
		TriangleMesh GeneralMesh();
		bool RSearch_point(size_t rotate_num, TriangleMesh ObjMesh, std::vector<finalSTLcal> *FinalCal, std::string STL_Str, double thickness, double insert_thickness, double Size, double gap);
		double GetNumOnPlat();

	public:
		TriangleMesh *pObj;	  //  实体指针
		Pointf3 STL_MidPoint; // 中心点
		std::string STL_Str;
		std::vector<NumSingleInfos> SingleChar_vec; // 拆解的字符信息集合
		double mark_Length;							// 总体长度
		double STLCharmark_thickness;				// 标记的厚度  针对侧壁有效
		double STLInsert_thickness;
		double STLCharMark_Length;
		double STLCharMark_Width;
		double STLCharMark_Size;
		double STLzDifSUM;
		double HitRadioSum;
		double STLCharMark_gap;
		bool autoGeneral;
		bool charMeshIsValid = true;

	private:
		bool GeneralShapePoints(); // 拆分字符
		bool GeneralExPoints();
	};

	// STL字符侧壁支撑
	class Side_STL_CharMark_Mesh : public Base_Mesh
	{
	public:
		Side_STL_CharMark_Mesh(
			TriangleMesh _pObj,
			std::string _STL_Str,
			double _STL_Charmark_thickness, // 标记的厚度  针对侧壁有效
			double _STL_Insert_thickness,	// 标记的插入深度  针对侧壁有效
			double _STL_CharMark_Length,	// 字符高度
			double _STL_CharMark_Width,		// 字符宽度
			double _STL_CharMark_size,
			double _STL_CharMark_gap)
			: autoGeneral(true),
			  obj_copy(_pObj),
			  STL_Str(_STL_Str),
			  STL_Charmark_thickness(_STL_Charmark_thickness),
			  STL_Insert_thickness(_STL_Insert_thickness),
			  STL_CharMark_Length(_STL_CharMark_Length),
			  STL_CharMark_Width(_STL_CharMark_Width),
			  STL_CharMark_Size(_STL_CharMark_size),
			  STL_CharMark_gap(_STL_CharMark_gap)
		{
			LOGINFO("0208 Side_STL_CharMark_Mesh start");
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Side_STL_CharMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};

	public:
		Side_STL_CharMark_Mesh(
			TriangleMesh _obj,
			Pointf3 _MidPoint,
			Pointf3 _MidNormal,
			std::string _STL_Str,
			double _STL_Charmark_thickness, // 标记的厚度  针对侧壁有效
			double _STL_Insert_thickness,	// 标记的插入深度  针对侧壁有效
			double _CharMark_Length,		// 字符高度
			double _CharMark_Width,			// 字符宽度
			double _CharMark_size,
			double _STL_CharMark_gap)
			: autoGeneral(false),
			  obj_copy(_obj),
			  MidPoint(_MidPoint),
			  MidNormal(_MidNormal),
			  STL_Str(_STL_Str),
			  STL_Charmark_thickness(_STL_Charmark_thickness),
			  STL_Insert_thickness(_STL_Insert_thickness),
			  STL_CharMark_Length(_CharMark_Length),
			  STL_CharMark_Width(_CharMark_Width),
			  STL_CharMark_Size(_CharMark_size),
			  STL_CharMark_gap(_STL_CharMark_gap)
		{
			if ((GeneralShapePoints() && GeneralExPoints()) == false)
				LOGINFO("Side_STL_CharMark_Mesh GeneralShapePoints or GeneralExPoints Failed");
		};
		~Side_STL_CharMark_Mesh() {};
		TriangleMesh GeneralMesh();
		static bool Init_Charmark_Mesh();
		static NumSingleType getTypeFromChar(char s);

	public:
		bool autoGeneral;
		TriangleMesh obj_copy;		   // 实体mesh拷贝
		std::string STL_Str;		   // 数字值
		Pointf3 MidPoint;			   // 中心点
		Pointf3 MidNormal;			   // 中心点法向
		double STL_Charmark_thickness; // 标记的厚度  针对侧壁有效
		double STL_Insert_thickness;   // 标记的插入深度  针对侧壁有效
		double STL_CharMark_Length;	   // 字符高度
		double STL_CharMark_Width;	   // 字符宽度
		double STL_CharMark_Size;
		double STL_CharMark_gap;
		double STL_CharMark_zDifSUM;	  // 高低差
		double STL_CharMark_HitRadio;	  // 命中率,为了保证所有字符都必须在模型上,不然就提示
		bool STL_CharMark_IsValid = true; // 衡量字符生成的质量

	private:
		bool GeneralShapePoints() { return true; };
		bool GeneralExPoints() { return true; };

		TriangleMesh STL_GeneralMeshAuto();
		TriangleMesh STL_GeneralMeshManual();
	};
	//////////////////////////////////////////////////////////////////////////
	////晶格支撑
	//////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////
	//  Lattice 支撑点结构
	////////////////////////////////////////////////////////
	enum LatticeSPT_Type
	{
		Suspend_Bound, // 悬垂边
		Suspend_Face,  // 悬垂面
		Angle_Bound,   // 角度阈值边
		Angle_Face,	   // 角度阈值面
		LSPT_Crossbar, // 横杆
		LSPT_Other
	};

	enum PillarNode_Type
	{
		Odd,  // 奇数点
		Even, // 偶数点
		None
	};

	// 晶格顶部支撑点结构体
	class LatticeSPT_Pointf3
	{
	public:
		LatticeSPT_Pointf3(coordf_t insert_z, coordf_t up_contact_height, coordf_t up_contact_width, coordf_t down_contact_width) : up_insert_z(insert_z),
																																	up_connection_height(up_contact_height),
																																	up_contact_width(up_contact_width),
																																	down_contact_width(down_contact_width),
																																	UpPointNormal(Pointf3(0, 0, -1)),
																																	angleD(0.0)
		{
			GenerateValid = true;
		};
		LatticeSPT_Pointf3() {};
		~LatticeSPT_Pointf3() {};

		void set_parameter(const LatticeSupportMesh &latticeSupportMesh);
		void set_grid_position(coord_t x, coord_t y)
		{
			x_num = x;
			y_num = y;
		};
		void set_pillar_position(coord_t z) { z_num = z; };
		void check_support_length(LatticeSupportMesh &latticeSupportMesh);
		double get_spt_height() { return UpPoint.z - DwPoint.z; };

	public:
		bool GenerateValid;

		Pointf3 UpPoint;		  // 上接触点坐标
		Pointf3 DwPoint;		  // 下接触点坐标
		LatticeSPT_Type spt_type; // 支撑点来源类型
		coord_t Form_FaceID;	  // 接触面片所在的ID
		int Weight;				  // 支撑点权重
		Pointf3 UpPointNormal;	  // 支撑点法相

		coord_t x_num; // 台柱的X_Num
		coord_t y_num; // 台柱的Y_Num
		coord_t z_num; // 台柱的Z_Num

		double maximum_length = DBL_MAX; // 最大长度
		double angleD = 0.0;
		TriangleMesh GenerateMesh();
		bool is_node_selected(Pointf3 select_point, Pillar &pillar, LatticeSupportMesh &latticeSupportMesh);

	private:
		Pointf3 insert_up_point() const
		{
			return UpPoint - up_insert_z * UpPointNormal; // Pointf3(UpPoint.x, UpPoint.y, UpPoint.z + up_insert_z);		// 插入顶点
		};

		double lattice_spt_length() const
		{ // 顶部支撑长度
			coordf_t z = UpPoint.z - up_insert_z - up_contact_height - up_connection_height;

			Pointf3 down_1 = Pointf3(UpPoint.x, UpPoint.y, z);
			return down_1.distance_to(UpPoint) + down_1.distance_to(DwPoint);
		};

		double get_lattice_spt_length(Pointf3 up, Pointf3 down) const
		{ // 获取顶部支撑长度
			coordf_t z = up.z - up_insert_z - up_contact_height - up_connection_height;

			Pointf3 down_1 = Pointf3(up.x, up.y, z);
			return down_1.distance_to(up) + down_1.distance_to(down);
		};

		coordf_t up_insert_z = 0.0;			 // 插入深度
		coordf_t up_contact_height = 0.2;	 // 支撑顶部支撑高度
		coordf_t up_contact_width = 0.3;	 // 支撑顶部宽度
		coordf_t up_connection_height = 0.4; // 连接高度
		coordf_t down_contact_width = 0.6;	 // 支撑底部宽度

		double current_length = DBL_MAX;	  // 当前顶部支撑长度
		Pointf3 minimum_current_length_point; // 最短顶部支撑最低点
		coord_t pillar_z_num;				  // 最短顶部支撑最低点节点的 Z Num
	};

	/// <summary>
	/// 晶格支撑
	/// </summary>
	class LatticeSupportMesh : public Base_Mesh
	{
	public:
		LatticeSupportMesh(
			TriangleMesh _pObj,
			std::vector<Linef3> _crossbar_midLine,
			coordf_t _angel_step,
			coord_t _bound_num_min,
			coordf_t _exclude_tuqi_area,
			coordf_t _suspend_angel,
			coordf_t _x_cell_length,
			coordf_t _y_cell_length,
			coordf_t _z_cell_length,
			coordf_t _lattice_slice_thickness,
			coordf_t _lattice_spt_distance,
			coordf_t _crossbar_spt_distance,
			coordf_t _up_insert_z,
			coordf_t _up_contact_height,
			coordf_t _up_contact_width,
			coordf_t _up_connection_height,
			coordf_t _down_contact_width,
			coord_t _down_contact_angle,
			coordf_t _lattice_width,
			coordf_t _lattice_height,
			coordf_t _pillar_width,
			coordf_t _lattice_grid_angle,
			coord_t _extand_factor,
			bool _bottom_strengthen,
			bool _lattice_merge,
			coordf_t _arrange_distance);
		~LatticeSupportMesh();

		TriangleMesh GeneralMesh();
		bool GeneralExPoints() { return true; };
		bool GeneralShapePoints() { return true; };

	public:
		TriangleMesh Obj;
		TriangleMesh *pObj;			   // 需要加支撑的mesh
		ExPolygons support_thumbnails; // 支撑区域二维投影
		Polygon bed_type;			   // 基台形状，用于检测支撑是否超出盘外

		LatticeSPT_Pointf3s contact_support_points;			// 顶部的支撑接触点
		LatticeSPT_Pointf3s Speical_contact_support_points; // RPD特殊区域的支撑接触点
		std::vector<Linef3> crossbar_midLines;				// 横杆中轴线

		//////////////////////////////////////////////////////////////////////////
		// 支撑点生成的参数
		//////////////////////////////////////////////////////////////////////////
		coordf_t angel_step;			  // 角度步长
		coord_t bound_num_min;			  // 识别为悬垂边的重复数阈值
		coordf_t exclude_tuqi_area;		  // 识别为花纹的面片大小阈值
		coordf_t suspend_angel;			  // 识别为悬垂面的角度阈值
		coordf_t x_cell_length;			  // 简化晶格X轴向长度
		coordf_t y_cell_length;			  // 简化晶格Y轴向长度
		coordf_t z_cell_length;			  // 简化晶格Z轴向长度 可选
		coordf_t tuqi_factor;			  // 花纹区域简化缩放因子
		coordf_t reten_factor;			  // 固位网区域简化缩放因子
		coordf_t lattice_slice_thickness; // 切片层厚
		coordf_t lattice_spt_distance;	  //  支撑间距
		coordf_t crossbar_spt_distance;	  //  横杆支撑间距

		//////////////////////////////////////////////////////////////////////////
		// 顶部支撑参数
		//////////////////////////////////////////////////////////////////////////
		coordf_t up_insert_z;		   // 插入深度
		coordf_t up_contact_height;	   // 顶部支撑顶部支撑高度
		coordf_t up_contact_width;	   // 顶部支撑顶部接触宽度
		coordf_t up_connection_width;  // 顶部支撑连接宽度
		coordf_t up_connection_height; // 顶部支撑连接高度
		coordf_t down_contact_width;   // 顶部支撑底端接触宽度
		coord_t down_contact_angle;	   // 顶部支撑底端支撑角度
		//////////////////////////////////////////////////////////////////////////
		// 晶格支撑参数
		//////////////////////////////////////////////////////////////////////////
		coordf_t lattice_width;		   // 晶格宽度， X、Y方向节点步长
		coordf_t lattice_height;	   // 晶格高度, Z方向节点步长
		coordf_t lattice_min_distance; // 晶格柱顶部到物体最小距离
		coordf_t pillar_width;		   // 台柱宽度
		coordf_t lattice_grid_angle;   // 网格旋转角度, 弧度表示
		coord_t extand_factor;		   // 带支撑点台柱延伸因子
		bool bottom_strengthen;		   // 底部加强
		bool lattice_merge;			   // 面片重组，是否进行二维布尔
		bool avoid_tuqi_area;		   // 避开花纹区域
		bool avoid_reten_area;		   // 避开固位网区域

		bool tuqi_area_pillar_connect;	// 花纹区域晶格柱之间的连接
		bool reten_area_pillar_connect; // 固位网区域晶格柱之间的连接

		coordf_t min_pillar_connect_height_reten; // 固位网区域添加台柱连接的最低高度
		coordf_t min_pillar_connect_height_tuqi;  // 花纹区域添加台柱连接的最低高度
		coordf_t min_pillar_connect_height;		  // 其他区域添加台柱连接的最低高度

		bool pillar_connect_jump;
		bool pillar_simplify;

		bool pillar_connect_jump_reten;
		bool pillar_simplify_reten;

		bool pillar_connect_jump_tuqi;
		bool pillar_simplify_tuqi;

		coordf_t arrange_distance;
		coordf_t bottom_height; // 底座高度

		Pointf start_location;	// 二维网格起始点
		Pointf center_location; // 二维网格中心点

		coord_t x_num_max; // X方向二维网格数
		coord_t y_num_max; // Y方向二维网格数

		Grid_Pillar pillars;

		std::map<int, Polygons> map_x_polygons;
		std::map<int, Polygons> map_y_polygons;

		bool has_generte_shape_points() const { return _has_generte_shape_points; };
		bool has_generte_ex_points() const { return _has_generte_ex_points; };

		bool init_contact_support_points(); // 生成需要加支撑的点
		bool generate_support_thumbnails(); // 生成支撑面的二维投影
		bool init_lattice_grid();			// 晶格网格初始化

		// 顶部支撑点聚类, reten晶格避开固位网区域， sw晶格避开花纹区域
		bool cluster_contact_support_points(bool reten = false, bool sw = false);
		bool generate_pillars(); // 建立台柱的拓扑关系

		PillarNode_Type generate_one_support_pillar_nodes(Pillar &pillar); // 生成台柱节点
		bool generate_all_support_pillar_nodes_bythreads(unsigned int _threads = boost::thread::hardware_concurrency() / 2);
		PillarNode_Type generate_all_support_pillar_nodes();

		void check_all_support_length();

		bool extand_pillars();
		bool extand_a_pillar(Pillar &center_pillar);

		bool generate_pillars_topology(PillarNode_Type type); // 生成台柱之间的拓扑结构

		bool check_pillars(); // 去除多余的晶格连接

		void set_lattice_min_distance(coordf_t min_distance)
		{
			lattice_min_distance = min_distance;
		}

		void set_reten_pillar_para(bool _avoid_area, bool _pillar_connected, coordf_t _factor)
		{
			avoid_reten_area = _avoid_area;
			reten_area_pillar_connect = _pillar_connected;
			reten_factor = _factor;
		}

		void set_tuqi_pillar_para(bool _avoid_area, bool _pillar_connected, coordf_t _factor)
		{
			avoid_tuqi_area = _avoid_area;
			tuqi_area_pillar_connect = _pillar_connected;
			tuqi_factor = _factor;
		}

		void set_min_pillar_connect_height(coordf_t _height, coordf_t _reten_height, coordf_t _tuqi_height)
		{
			min_pillar_connect_height = _height;
			min_pillar_connect_height_reten = _reten_height;
			min_pillar_connect_height_tuqi = _tuqi_height;
		}

		void set_pillar_connect_jump(bool _jump, bool _simplify)
		{
			pillar_connect_jump = _jump;
			pillar_simplify = _simplify;
		}

		void set_pillar_connect_jump_reten(bool _jump, bool _simplify)
		{
			pillar_connect_jump_reten = _jump;
			pillar_simplify_reten = _simplify;
		}

		void set_pillar_connect_jump_tuqi(bool _jump, bool _simplify)
		{
			pillar_connect_jump_tuqi = _jump;
			pillar_simplify_tuqi = _simplify;
		}

		bool is_lattice_spt_intersect(Pointf3 p1, Pointf3 p2, Pointf3 &p3)
		{
			return this->is_intersect(p1, p2, p3);
		}

		Pointf3 get_pillar_node_position(coord_t x, coord_t y, coord_t z)
		{
			return this->get_pillarnode_position(x, y, z);
		}

		bool get_nearest_pillar_node_position(Pointf3 select_point, Pointf3 &target_point);
		bool is_intersect_new(TriangleMesh *mesh, Pointf3 p1, Pointf3 p2, Pointf3 &p3); // P1,P2要有相同的Z值，如果相交，返回线段最高的交点

	protected:
		bool _has_generte_shape_points = false;
		bool _has_generte_ex_points = false;

	private:
		bool _is_in_thumbnails(Point p);

		Pointf get_pillar_position(coord_t x, coord_t y);
		Pointf3 get_pillarnode_position(coord_t x, coord_t y, coord_t z);

		bool is_intersect(Pointf3 p1, Pointf3 p2, Pointf3 &p3);							// 针对顶部支撑检查是否干涉，如果相交，返回线段最高的交点
		bool is_intersect(PillarNode node1, PillarNode node2, int type, Pointf3 &pout); // 检查晶格支撑是否干涉，如果相交，返回线段最高的交点
		TriangleMesh merge_pillar(TriangleMesh *latticeSupportMesh);

		// 针对晶格支撑检查是否干涉
		TriangleMesh mesh0;
		TriangleMesh mesh1;
		TriangleMesh mesh2;
		TriangleMesh mesh3;

		Eigen::Matrix3d rotation_matrix0;
		Eigen::Matrix3d rotation_matrix1;
		Eigen::Matrix3d rotation_matrix2;
		Eigen::Matrix3d rotation_matrix3;

		coordf_t hash_step_length = 0.5;
		bool init_mesh();
		Pointf3 rotate_pointf3(Eigen::Matrix3d rotation_matrix, Pointf3 p);
	};

	/// <summary>
	/// 十字台柱
	/// </summary>
	class Pillar
	{
	public:
		Pillar() : is_activated(false), x_Num(-1), y_Num(-1)
		{
			face_type = None_Part;
			face_side = None_Face;
		};
		~Pillar()
		{
			Pointf3s().swap(intersect_points);
			LatticeSPT_Pointf3s().swap(support_points);
			PillarNodeMap().swap(pillar_nodes);
		};

		void set_parameter(const LatticeSupportMesh &latticeSupportMesh);
		void set_grid_position(coord_t x, coord_t y)
		{
			x_Num = x;
			y_Num = y;
		};
		bool is_valid() { return x_Num >= 0 && y_Num >= 0; };
		coord_t get_top_node_id();
		bool set_pillar_type(stl_face_type _type, stl_face_side _side)
		{
			face_type = _type;
			face_side = _side;
			return true;
		};

	public:
		stl_face_type face_type;
		stl_face_side face_side;

		coord_t x_Num;
		coord_t y_Num;

		Pointf3s intersect_points;	   // 与支架交点坐标集， 按照z轴方向由下到上排列
		MapCollsionInfo CollsionZ_map; // 基本点Z轴碰撞点信息  z值-->碰撞facet_id

		bool is_activated;					// 该网格点的台柱是否激活
		LatticeSPT_Pointf3s support_points; // 台柱的支撑点集
		PillarNodeMap pillar_nodes;			// 台柱节点集

		std::map<int, Polygons> map_x_polygons;
		std::map<int, Polygons> map_y_polygons;

		bool IsCollsion()
		{
			return (this->CollsionZ_map.size() != 0);
		};

		double get_mincollsionZ()
		{
			if (this->IsCollsion())
				return this->CollsionZ_map.begin()->first;
			return 0.0;
		}

		double get_maxcollsionZ()
		{
			if (this->IsCollsion())
				return this->CollsionZ_map.rbegin()->first;
			return 0.0;
		}

		double get_second_maxcollsionZ()
		{
			if (this->IsCollsion())
			{
				MapCollsionInfo::reverse_iterator it = this->CollsionZ_map.rbegin();
				it++;

				return it->first;
			}
			return 0.0;
		}

		TriangleMesh GenerateMesh(Pointf3 top_p, Pointf3 bottom_p, bool is_intersect);

	private:
		coordf_t pillar_width;		  // 台柱支撑宽度
		coordf_t down_contact_width;  // 台柱下支撑接触宽度
		coordf_t down_contact_height; // 台柱下支撑接触高度
		coordf_t down_connect_height; // 台柱下支撑连接高度
		coordf_t down_insert_z;		  // 台柱下支撑的插入深度
	};

	/// <summary>
	/// 十字台柱节点
	/// </summary>
	class PillarNode
	{
	public:
		PillarNode() : connect_with_next(true),
					   is_intersect(false),
					   pillar_node_0_ptr(NULL),
					   pillar_node_1_ptr(NULL),
					   pillar_node_2_ptr(NULL),
					   pillar_node_3_ptr(NULL),
					   x_Num(-1), y_Num(-1), z_Num(-1),
					   up_connect_num(0),
					   down_connect_num(0),
					   support_num(0) {};
		~PillarNode() {};

	public:
		coord_t x_Num;
		coord_t y_Num;
		coord_t z_Num;

		int up_connect_num;	  // 上连接计数
		int down_connect_num; // 下连接计数
		int support_num;	  // 支撑连接计数

		// bool is_support_point;//节点上是否有支撑
		bool connect_with_next;	 // 是否需要向下连接
		bool is_intersect;		 // 该节点的向下台柱是否与物体相交
		Pointf3 intersect_point; // 记录向下连接的台柱与物体的交点

		PillarNode_Type node_type; // 节点类型

		PillarNodePtr pillar_node_0_ptr; // 与（x_Num-1，y_Num）台柱的链接节点
		PillarNodePtr pillar_node_1_ptr; // 与（x_Num+1，y_Num）台柱的链接节点
		PillarNodePtr pillar_node_2_ptr; // 与（x_Num，y_Num-1）台柱的链接节点
		PillarNodePtr pillar_node_3_ptr; // 与（x_Num，y_Num+1）台柱的链接节点

		std::map<int, Polygons> map_x_polygons;
		std::map<int, Polygons> map_y_polygons;
		TriangleMesh GenerateConnectMesh(Pointf3 p_1, Pointf3 p_2, Axis dir);

		void set_parameter(const LatticeSupportMesh &latticeSupportMesh);
		void set_grid_position(coord_t x, coord_t y, coord_t z);
		bool is_valid() { return x_Num >= 0 && y_Num >= 0 && z_Num >= 0; };

	private:
		double node_width;
		double bottom_height;
		bool is_connect_bottom;
	};

	//////////////////////////////////////////////////////////////////////////
	////晶格支撑 end
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	////  特殊结构横杆
	//////////////////////////////////////////////////////////////////////////
	struct Special_Cylinder
	{
		double raduis = 0.0; // 半径
		int raduis_round = 0;
		double length = 0.0; // 长度
		int length_round = 0;
		TriangleMesh cy_mesh;	   // mesh结构
		std::string FilePath = ""; // 读取文件路径
		// 初始化
		Special_Cylinder(std::string _filepath);
		void Print_Info();
	};

	// 特殊结构的横杆存储集合
	static std::map<int, Special_Cylinder *> SpCy_Meshs = std::map<int, Special_Cylinder *>();

	class CrossBarGenerator
	{
	public:
		CrossBarGenerator(Pointf3 start_point, double radius, double contact_width, double contact_length, bool Is_Special);
		~CrossBarGenerator() {};

	public:
		TriangleMesh Generate(Pointf3 end_point);
		TriangleMesh GenerateStartBall();
		// 初始化静态变量
		static bool Init_Special_CyMesh();

	private:
		Pointf3 start_point; // 起始点
		Pointf3 end_point;	 // 终止点

		double radius = 0.5;			// 杆半径
		double start_ball_radius = 0.3; // 球半径

		double contact_width;  // 两端台柱宽度
		double contact_length; // 两端台柱长度

		bool Is_Special = false; // 是否是特殊结构

		// 特殊结构的横杆
		TriangleMesh Generate_SpecialBarMesh(Pointf3 start_p, Pointf3 end_p, double raduis);
		TriangleMesh Generate_CylinderBarMesh(Pointf3 start_p, Pointf3 end_p, double raduis);
		TriangleMesh Generate_CrossfaceBarMesh(Pointf3 start_p, Pointf3 end_p, double raduis);
	};
}

#endif
