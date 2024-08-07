#include "TriangleMesh.hpp"
#include <float.h>
#include <assert.h>
#include "SVG.hpp"
#include "ClipperUtils.hpp"

namespace Slic3r
{
	LatticeSupportMesh::LatticeSupportMesh(
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
		coordf_t _arrange_distance) : angel_step(_angel_step),
									  bound_num_min(_bound_num_min),
									  exclude_tuqi_area(_exclude_tuqi_area),
									  suspend_angel(_suspend_angel),
									  x_cell_length(_x_cell_length),
									  y_cell_length(_y_cell_length),
									  z_cell_length(_z_cell_length),
									  lattice_slice_thickness(_lattice_slice_thickness),
									  lattice_spt_distance(_lattice_spt_distance),
									  crossbar_spt_distance(_crossbar_spt_distance),
									  up_insert_z(_up_insert_z),
									  up_contact_height(_up_contact_height),
									  up_contact_width(_up_contact_width),
									  up_connection_height(_up_connection_height),
									  down_contact_width(_down_contact_width),
									  down_contact_angle(_down_contact_angle),
									  lattice_width(_lattice_width),
									  lattice_height(_lattice_height),
									  pillar_width(_pillar_width),
									  extand_factor(_extand_factor),
									  bottom_strengthen(_bottom_strengthen),
									  lattice_merge(_lattice_merge),
									  arrange_distance(_arrange_distance)
	{
		avoid_tuqi_area = false;
		avoid_reten_area = false;
		tuqi_area_pillar_connect = true;
		reten_area_pillar_connect = true;
		pillar_connect_jump = false;
		pillar_simplify = false;
		min_pillar_connect_height_reten = 0.0;
		min_pillar_connect_height_tuqi = 0.0;
		min_pillar_connect_height = 0.0;

		tuqi_factor = 1.0;
		reten_factor = 1.0;

		lattice_grid_angle = _lattice_grid_angle;
		Obj = TriangleMesh(_pObj);
		crossbar_midLines = _crossbar_midLine;

		if (lattice_grid_angle != 0)
		{
			DWORD timecount = GetTickCount();
			LOGINFO("start rotate !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			Obj.rotate(-lattice_grid_angle);
			// for (int i = 0; i < crossbar_midLines.size(); i++)
			//	crossbar_midLines.at(i).rotate_z(-lattice_grid_angle / 180.0 * PI);
			LOGINFO("end rotate !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! times =%d", GetTickCount() - timecount);
		}

		this->pObj = &this->Obj;
		bottom_height = 0;
		pObj->set_step_length(1, 1, 1);
		pObj->init_hash_facets();
		init_mesh();
	}

	LatticeSupportMesh::~LatticeSupportMesh()
	{
		LatticeSPT_Pointf3s().swap(contact_support_points);
		mesh0.ClearMesh();
		mesh1.ClearMesh();
		mesh2.ClearMesh();
		mesh3.ClearMesh();
		Obj.ClearMesh();
		ExPolygons().swap(support_thumbnails);
		LatticeSPT_Pointf3s().swap(Speical_contact_support_points);
		std::vector<Linef3>().swap(crossbar_midLines);
		std::map<int, Polygons>().swap(map_x_polygons);
		std::map<int, Polygons>().swap(map_y_polygons);
		Grid_Pillar().swap(pillars);
	}

	TriangleMesh LatticeSupportMesh::GeneralMesh()
	{
		DWORD timecount = GetTickCount();
		// add by DuYH 清空被支撑物体的晶格容器
		int Lspt_Points_count = this->pObj->Lspt_Points.size();
		this->pObj->Lspt_Points.clear();
		this->pObj->Lspt_Points.reserve(Lspt_Points_count);

		TriangleMesh latticeSupportMesh;
		latticeSupportMesh.checkonly = true;

		for (size_t x = 0; x < this->x_num_max; x++)
		{
			for (size_t y = 0; y < this->y_num_max; y++)
			{
				Pillar &pillar = this->pillars[x][y];

				LOGINFO("x = [%d], y = [%d], pillar.support_points.size() = [%d]", x, y, pillar.support_points.size());
				// 顶部支撑
				for (size_t i = 0; i < pillar.support_points.size(); i++)
				{
					// add by DuYH 顶部支撑不再直接生成，保存在被支撑物体中
					// latticeSupportMesh.merge(pillar.support_points[i].GenerateMesh());
					this->pObj->Lspt_Points.push_back(pillar.support_points[i]);
					// LOGINFO("lattice top, top = [%s], bottom = [%s]", pillar.support_points[i].UpPoint.dump_perl().c_str(), pillar.support_points[i].DwPoint.dump_perl().c_str());
				}

				// 十字台柱
				if (pillar.is_activated == false)
				{
					continue;
				}
				// 连接台柱
				// LOGINFO("pillar.pillar_nodes.size() = [%d], x = [%d], y = [%d]", pillar.pillar_nodes.size(), x, y);
				coord_t nodenum = pillar.get_top_node_id();
				for (coord_t ikk = nodenum; ikk >= 0; ikk--)
				{
					// LOGINFO("pillar.get_top_node_id() = [%d], x = [%d], y = [%d], ikk = [%d]", nodenum, x, y, ikk);

					PillarNode &node = pillar.pillar_nodes[ikk];
					if (node.is_valid() == false)
					{
						// LOGINFO("node.is_valid() == false!!!node.x = [%d], node.y = [%d], node.z = [%d]", node.x_Num, node.y_Num, node.z_Num);
						// LOGINFO("pillar.pillar_nodes.size() = [%d], x = [%d], y = [%d]", pillar.pillar_nodes.size(), x, y);

						continue;
					}
					Pointf3 p_0 = this->get_pillarnode_position(node.x_Num, node.y_Num, node.z_Num);
					// LOGINFO("p_0 = [%s], x_Num = [%d], y_Num = [%d], z_Num = [%d]", p_0.dump_perl().c_str(), node.x_Num, node.y_Num, node.z_Num);

					node.set_parameter(*this);
					if (node.pillar_node_0_ptr != nullptr)
					{
						PillarNodePtr nodeptr = node.pillar_node_0_ptr;
						Pillar &p0_pillar = this->pillars[nodeptr->x_Num][nodeptr->y_Num];
						if (nodeptr->is_valid())
						{
							Pointf3 p_1 = this->get_pillarnode_position(nodeptr->x_Num, nodeptr->y_Num, nodeptr->z_Num);
							// LOGINFO("pillar_node_0_ptr, p1 = [%s], x_Num = [%d], y_Num = [%d], z_Num = [%d]", p_1.dump_perl().c_str(), nodeptr->x_Num, nodeptr->y_Num, nodeptr->z_Num);
							TriangleMesh node_mesh;
							if (p_0.z == 0.0 && p_1.z == 0.0)
							{
								// 底部加强，nodeptr上垂直台柱是否存在
								if (p0_pillar.pillar_nodes[1].is_valid() && p0_pillar.pillar_nodes[1].connect_with_next == false)
								{
								}
								else if (nodeptr->up_connect_num < 2)
								{
								}
								else
									node_mesh = node.GenerateConnectMesh(p_0, p_1, Axis::X);
							}
							else
							{
								node_mesh = node.GenerateConnectMesh(p_1, p_0, Axis::X);
							}

							if (this->lattice_merge == true)
							{
								this->map_x_polygons[x] += node.map_x_polygons[x];
								this->map_y_polygons[y] += node.map_y_polygons[y];
							}
							else
							{
								latticeSupportMesh.merge(node_mesh);
							}
						}
					}
					if (node.pillar_node_1_ptr != nullptr)
					{
						PillarNodePtr nodeptr = node.pillar_node_1_ptr;
						Pillar &p1_pillar = this->pillars[nodeptr->x_Num][nodeptr->y_Num];
						if (nodeptr->is_valid())
						{

							Pointf3 p_2 = this->get_pillarnode_position(nodeptr->x_Num, nodeptr->y_Num, nodeptr->z_Num);
							// LOGINFO("pillar_node_1_ptr, p2 = [%s], x_Num = [%d], y_Num = [%d], z_Num = [%d]", p_2.dump_perl().c_str(), nodeptr->x_Num, nodeptr->y_Num, nodeptr->z_Num);

							TriangleMesh node_mesh;
							// 底部加强，nodeptr上垂直台柱是否存在
							if (node.z_Num == 0 && nodeptr->z_Num == 0 &&
								p1_pillar.pillar_nodes[1].is_valid() &&
								p1_pillar.pillar_nodes[1].connect_with_next == false)
							{
							}
							else if (node.z_Num == 0 && nodeptr->z_Num == 0 &&
									 nodeptr->up_connect_num < 2)
							{
							}
							else
								node_mesh = node.GenerateConnectMesh(p_2, p_0, Axis::X);

							if (this->lattice_merge == true)
							{
								this->map_x_polygons[x] += node.map_x_polygons[x];
								this->map_y_polygons[y] += node.map_y_polygons[y];
							}
							else
							{
								latticeSupportMesh.merge(node_mesh);
							}
						}
					}
					if (node.pillar_node_2_ptr != nullptr)
					{
						PillarNodePtr nodeptr = node.pillar_node_2_ptr;
						Pillar &p2_pillar = this->pillars[nodeptr->x_Num][nodeptr->y_Num];
						if (nodeptr->is_valid())
						{
							Pointf3 p_3 = this->get_pillarnode_position(nodeptr->x_Num, nodeptr->y_Num, nodeptr->z_Num);
							// LOGINFO("pillar_node_2_ptr, p3 = [%s], x_Num = [%d], y_Num = [%d], z_Num = [%d]", p_3.dump_perl().c_str(), nodeptr->x_Num, nodeptr->y_Num, nodeptr->z_Num);

							TriangleMesh node_mesh;
							if (p_0.z == 0.0 && p_3.z == 0.0)
							{
								// 底部加强，nodeptr上垂直台柱是否存在
								if (p2_pillar.pillar_nodes[1].is_valid() && p2_pillar.pillar_nodes[1].connect_with_next == false)
								{
								}
								else if (nodeptr->up_connect_num < 2)
								{
								}
								else
									node_mesh = node.GenerateConnectMesh(p_0, p_3, Axis::Y);
							}
							else
								node_mesh = node.GenerateConnectMesh(p_3, p_0, Axis::Y);

							if (this->lattice_merge == true)
							{
								this->map_x_polygons[x] += node.map_x_polygons[x];
								this->map_y_polygons[y] += node.map_y_polygons[y];
							}
							else
							{
								latticeSupportMesh.merge(node_mesh);
							}
						}
					}
					if (node.pillar_node_3_ptr != nullptr)
					{
						PillarNodePtr nodeptr = node.pillar_node_3_ptr;
						Pillar &p3_pillar = this->pillars[nodeptr->x_Num][nodeptr->y_Num];
						if (nodeptr->is_valid())
						{
							Pointf3 p_4 = this->get_pillarnode_position(nodeptr->x_Num, nodeptr->y_Num, nodeptr->z_Num);
							// LOGINFO("pillar_node_3_ptr, p4 = [%s], x_Num = [%d], y_Num = [%d], z_Num = [%d]", p_4.dump_perl().c_str(), nodeptr->x_Num, nodeptr->y_Num, nodeptr->z_Num);

							TriangleMesh node_mesh;
							// 底部加强，nodeptr上垂直台柱是否存在
							if (node.z_Num == 0 && nodeptr->z_Num == 0 &&
								p3_pillar.pillar_nodes[1].is_valid() &&
								p3_pillar.pillar_nodes[1].connect_with_next == false)
							{
							}
							else if (node.z_Num == 0 && nodeptr->z_Num == 0 &&
									 nodeptr->up_connect_num < 2)
							{
							}
							else
							{
								node_mesh = node.GenerateConnectMesh(p_4, p_0, Axis::Y);
							}

							if (this->lattice_merge == true)
							{
								this->map_x_polygons[x] += node.map_x_polygons[x];
								this->map_y_polygons[y] += node.map_y_polygons[y];
							}
							else
							{
								latticeSupportMesh.merge(node_mesh);
							}
						}
					}

					// if (node.connect_with_next == false && node.support_num > 0)
					//{
					//	LOGINFO("node.connect_with_next == false && node.support_num > 0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
					//	LOGINFO("node.x = [%d], y = [%d], z = [%d]", node.x_Num, node.y_Num, node.z_Num);
					//	LOGINFO("node.support = [%d], up = [%d], down = [%d]", node.support_num, node.up_connect_num, node.down_connect_num);
					// }
					// 十字台柱
					if (node.connect_with_next == true)
					{
						Pointf3 top_p = p_0; // this->get_pillarnode_position(pillar.x_Num, pillar.y_Num, pillar.pillar_nodes.size() - 1);
						Pointf3 bottom_p = this->get_pillarnode_position(pillar.x_Num, pillar.y_Num, node.z_Num - 1);
						// LOGINFO("lattice top_p = [%s], bottom_p = [%s]", top_p.dump_perl().c_str(), bottom_p.dump_perl().c_str());

						if (node.is_intersect)
							bottom_p = node.intersect_point;

						pillar.set_parameter(*this);
						TriangleMesh pillar_mesh = pillar.GenerateMesh(top_p, bottom_p, node.is_intersect);
						if (this->lattice_merge == true)
						{
							this->map_x_polygons[x] += pillar.map_x_polygons[x];
							this->map_y_polygons[y] += pillar.map_y_polygons[y];
						}
						else
						{
							latticeSupportMesh.merge(pillar_mesh);
						}
					}
				}
			}
		}
		LOGINFO("GeneralMesh  foreach  cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();
		if (this->lattice_merge == true)
		{
			this->merge_pillar(&latticeSupportMesh);
		}
		LOGINFO("GeneralMesh  merge_pillar  cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// this->pObj->write_binary("latticeSupportMesh_rpd.stl");
		if (lattice_grid_angle != 0)
		{
			latticeSupportMesh.rotate(lattice_grid_angle);
		}

		LOGINFO("this->pObj->Lspt_Points.size() = [%d]", this->pObj->Lspt_Points.size());

		latticeSupportMesh.repair();
		if (latticeSupportMesh.UnableRepair())
		{
			LOGINFO("LatticeSupportMesh::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		LOGINFO("GeneralMesh  latticeSupportMesh  cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		return latticeSupportMesh;
	}

	TriangleMesh LatticeSupportMesh::merge_pillar(TriangleMesh *latticeSupportMesh)
	{
		for (size_t x = 0; x < this->x_num_max; x++)
		{
			ExPolygons exPolygons_x = union_ex(this->map_x_polygons[x]);

			Polygons triangles;
			for (ExPolygons::const_iterator expolygon = exPolygons_x.begin(); expolygon != exPolygons_x.end(); ++expolygon)
				expolygon->triangulate_p2t(&triangles);

			// convert triangles to facets and append them to mesh
			for (Polygons::const_iterator polygon = triangles.begin(); polygon != triangles.end(); ++polygon)
			{
				Polygon p = *polygon;
				p.reverse();
				stl_facet facet;
				for (size_t i = 0; i <= 2; ++i)
				{
					facet.vertex[i].x = this->get_pillarnode_position(x, 0, 0).x;
					facet.vertex[i].y = unscale(p.points[i].x);
					facet.vertex[i].z = unscale(p.points[i].y);
				}
				stl_add_facet(&latticeSupportMesh->stl, &facet);
			}
		}

		for (size_t y = 0; y < this->y_num_max; y++)
		{
			ExPolygons exPolygons_y = union_ex(this->map_y_polygons[y]);

			Polygons triangles;
			for (ExPolygons::const_iterator expolygon = exPolygons_y.begin(); expolygon != exPolygons_y.end(); ++expolygon)
				expolygon->triangulate_p2t(&triangles);

			// convert triangles to facets and append them to mesh
			for (Polygons::const_iterator polygon = triangles.begin(); polygon != triangles.end(); ++polygon)
			{
				Polygon p = *polygon;
				p.reverse();
				stl_facet facet;
				for (size_t i = 0; i <= 2; ++i)
				{
					facet.vertex[i].x = unscale(p.points[i].x);
					facet.vertex[i].y = this->get_pillarnode_position(0, y, 0).y;
					facet.vertex[i].z = unscale(p.points[i].y);
				}
				stl_add_facet(&latticeSupportMesh->stl, &facet);
			}
		}
		return *latticeSupportMesh;
	}

	TriangleMesh PillarNode::GenerateConnectMesh(Pointf3 p_1, Pointf3 p_2, Axis dir)
	{
		// 三角面片顶点索引
		std::vector<Point3> facets;
		// 顶点数组
		Pointf3s vertices;

		Points x_plane_points;
		Points y_plane_points;

		double width = this->node_width;

		if (p_1.z == 0 && p_2.z == 0)
		{
			if (this->is_connect_bottom == false)
			{
				return TriangleMesh();
			}

			vertices.push_back(Pointf3(p_2.x, p_2.y, p_2.z + this->bottom_height));
			vertices.push_back(Pointf3(p_1.x, p_1.y, p_1.z + this->bottom_height));
			vertices.push_back(Pointf3(p_1.x, p_1.y, p_1.z));
			vertices.push_back(Pointf3(p_2.x, p_2.y, p_2.z));

			switch (dir)
			{
			case Slic3r::Axis::X:
				y_plane_points.push_back(Point(scale_(p_2.x), scale_(p_2.z + this->bottom_height)));
				y_plane_points.push_back(Point(scale_(p_1.x), scale_(p_1.z + this->bottom_height)));
				y_plane_points.push_back(Point(scale_(p_1.x), scale_(p_1.z)));
				y_plane_points.push_back(Point(scale_(p_2.x), scale_(p_2.z)));
				break;
			case Slic3r::Axis::Y:
				x_plane_points.push_back(Point(scale_(p_2.y), scale_(p_2.z + this->bottom_height)));
				x_plane_points.push_back(Point(scale_(p_1.y), scale_(p_1.z + this->bottom_height)));
				x_plane_points.push_back(Point(scale_(p_1.y), scale_(p_1.z)));
				x_plane_points.push_back(Point(scale_(p_2.y), scale_(p_2.z)));
				break;
			case Slic3r::Axis::Z:
				break;
			default:
				break;
			}
		}
		else
		{
			switch (dir)
			{
			case Slic3r::Axis::X:
				vertices.push_back(Pointf3(p_1.x - width / 2, p_1.y, p_1.z));
				vertices.push_back(Pointf3(p_2.x - width / 2, p_2.y, p_2.z));
				vertices.push_back(Pointf3(p_2.x + width / 2, p_2.y, p_2.z));
				vertices.push_back(Pointf3(p_1.x + width / 2, p_1.y, p_1.z));

				y_plane_points.push_back(Point(scale_(p_1.x - width / 2), scale_(p_1.z)));
				y_plane_points.push_back(Point(scale_(p_2.x - width / 2), scale_(p_2.z)));
				y_plane_points.push_back(Point(scale_(p_2.x + width / 2), scale_(p_2.z)));
				y_plane_points.push_back(Point(scale_(p_1.x + width / 2), scale_(p_1.z)));

				break;
			case Slic3r::Axis::Y:
				vertices.push_back(Pointf3(p_1.x, p_1.y - width / 2, p_1.z));
				vertices.push_back(Pointf3(p_2.x, p_2.y - width / 2, p_2.z));
				vertices.push_back(Pointf3(p_2.x, p_2.y + width / 2, p_2.z));
				vertices.push_back(Pointf3(p_1.x, p_1.y + width / 2, p_1.z));

				x_plane_points.push_back(Point(scale_(p_1.y - width / 2), scale_(p_1.z)));
				x_plane_points.push_back(Point(scale_(p_2.y - width / 2), scale_(p_2.z)));
				x_plane_points.push_back(Point(scale_(p_2.y + width / 2), scale_(p_2.z)));
				x_plane_points.push_back(Point(scale_(p_1.y + width / 2), scale_(p_1.z)));

				break;
			case Slic3r::Axis::Z:
				break;
			default:
				break;
			}
		}

		if (x_plane_points.size() != 0)
		{
			this->map_x_polygons[this->x_Num].push_back(Polygon(x_plane_points));
		}
		if (y_plane_points.size() != 0)
		{
			this->map_y_polygons[this->y_Num].push_back(Polygon(y_plane_points));
		}

		facets.push_back(Point3(0, 1, 3));
		facets.push_back(Point3(3, 1, 2));

		TriangleMesh mesh(vertices, facets);
		mesh.checkonly = true;
		return mesh;
	}

	void Pillar::set_parameter(const LatticeSupportMesh &latticeSupportMesh)
	{
		this->pillar_width = latticeSupportMesh.pillar_width;
		this->down_contact_width = latticeSupportMesh.up_contact_width;
		this->down_contact_height = latticeSupportMesh.up_contact_height;
		this->down_connect_height = latticeSupportMesh.up_connection_height;
		this->down_insert_z = latticeSupportMesh.up_insert_z;
	}

	coord_t Pillar::get_top_node_id()
	{
		coord_t id = this->pillar_nodes.size() - 1;
		while (id >= 0)
		{
			if (this->pillar_nodes[id].is_valid())
				break;
			id--;
		}
		return id;
	}

	// 生成十字台柱
	// 如果有干涉交点，使用下接触点造型
	TriangleMesh Pillar::GenerateMesh(Pointf3 top_p, Pointf3 bottom_p, bool is_intersect)
	{
		// 三角面片顶点索引
		std::vector<Point3> facets;
		// 顶点数组
		Pointf3s vertices;

		TriangleMesh mesh_lspt;
		mesh_lspt.checkonly = true;

		Points x_plane_points;
		Points y_plane_points;

		if (is_intersect)
		{
			// 顶点和底点距离过近，直接用down_contact_width连接
			if (top_p.z - bottom_p.z <= this->down_connect_height + this->down_contact_height + 0.2)
			{
				Pointf3 px_0(top_p.x + this->down_contact_width / 2, top_p.y, top_p.z);
				Pointf3 px_1(px_0.x, px_0.y, bottom_p.z);
				Pointf3 px_2(px_1.x - this->down_contact_width, px_1.y, bottom_p.z);
				Pointf3 px_3(top_p.x - this->down_contact_width / 2, top_p.y, top_p.z);

				y_plane_points.push_back(Point(scale_(px_0.x), scale_(px_0.z)));
				y_plane_points.push_back(Point(scale_(px_1.x), scale_(px_1.z)));
				y_plane_points.push_back(Point(scale_(px_2.x), scale_(px_2.z)));
				y_plane_points.push_back(Point(scale_(px_3.x), scale_(px_3.z)));

				Pointf3 px_4(top_p.x, top_p.y + this->down_contact_width / 2, top_p.z);
				Pointf3 px_5(px_4.x, px_4.y, bottom_p.z);
				Pointf3 px_6(px_5.x, px_5.y - this->down_contact_width, bottom_p.z);
				Pointf3 px_7(top_p.x, top_p.y - this->down_contact_width / 2, top_p.z);

				x_plane_points.push_back(Point(scale_(px_4.y), scale_(px_4.z)));
				x_plane_points.push_back(Point(scale_(px_5.y), scale_(px_5.z)));
				x_plane_points.push_back(Point(scale_(px_6.y), scale_(px_6.z)));
				x_plane_points.push_back(Point(scale_(px_7.y), scale_(px_7.z)));

				if (x_plane_points.size() != 0)
				{
					this->map_x_polygons[this->x_Num].push_back(Polygon(x_plane_points));
				}
				if (y_plane_points.size() != 0)
				{
					this->map_y_polygons[this->y_Num].push_back(Polygon(y_plane_points));
				}

				vertices.push_back(px_0);
				vertices.push_back(px_1);
				vertices.push_back(px_2);
				vertices.push_back(px_3);
				vertices.push_back(px_4);
				vertices.push_back(px_5);
				vertices.push_back(px_6);
				vertices.push_back(px_7);

				facets.push_back(Point3(0, 1, 3));
				facets.push_back(Point3(3, 1, 2));
				facets.push_back(Point3(0 + 4, 1 + 4, 3 + 4));
				facets.push_back(Point3(3 + 4, 1 + 4, 2 + 4));
			}
			else // 使用下支撑方式连接
			{
				Pointf3 px_0_c = bottom_p + this->down_insert_z * Pointf3(0, 0, -1);
				Pointf3 px_1_c = bottom_p + this->down_contact_height * Pointf3(0, 0, 1);
				Pointf3 px_2_c = bottom_p + (this->down_contact_height + this->down_connect_height) * Pointf3(0, 0, 1);
				Pointf3 px_3_c = top_p;

				Pointf3 px_0(px_3_c.x + this->pillar_width / 2, px_3_c.y, px_3_c.z);
				Pointf3 px_1(px_2_c.x + this->pillar_width / 2, px_2_c.y, px_2_c.z);
				Pointf3 px_2(px_1_c.x + this->down_contact_width / 2, px_1_c.y, px_1_c.z);
				Pointf3 px_3(px_0_c.x + this->down_contact_width / 2, px_0_c.y, px_0_c.z);
				Pointf3 px_4(px_0_c.x - this->down_contact_width / 2, px_0_c.y, px_0_c.z);
				Pointf3 px_5(px_1_c.x - this->down_contact_width / 2, px_1_c.y, px_1_c.z);
				Pointf3 px_6(px_2_c.x - this->pillar_width / 2, px_2_c.y, px_2_c.z);
				Pointf3 px_7(px_3_c.x - this->pillar_width / 2, px_3_c.y, px_3_c.z);

				y_plane_points.push_back(Point(scale_(px_0.x), scale_(px_0.z)));
				y_plane_points.push_back(Point(scale_(px_1.x), scale_(px_1.z)));
				y_plane_points.push_back(Point(scale_(px_2.x), scale_(px_2.z)));
				y_plane_points.push_back(Point(scale_(px_3.x), scale_(px_3.z)));
				y_plane_points.push_back(Point(scale_(px_4.x), scale_(px_4.z)));
				y_plane_points.push_back(Point(scale_(px_5.x), scale_(px_5.z)));
				y_plane_points.push_back(Point(scale_(px_6.x), scale_(px_6.z)));
				y_plane_points.push_back(Point(scale_(px_7.x), scale_(px_7.z)));

				Pointf3 px_8(px_3_c.x, px_3_c.y + this->pillar_width / 2, px_3_c.z);
				Pointf3 px_9(px_2_c.x, px_2_c.y + this->pillar_width / 2, px_2_c.z);
				Pointf3 px_10(px_1_c.x, px_1_c.y + this->down_contact_width / 2, px_1_c.z);
				Pointf3 px_11(px_0_c.x, px_0_c.y + this->down_contact_width / 2, px_0_c.z);
				Pointf3 px_12(px_0_c.x, px_0_c.y - this->down_contact_width / 2, px_0_c.z);
				Pointf3 px_13(px_1_c.x, px_1_c.y - this->down_contact_width / 2, px_1_c.z);
				Pointf3 px_14(px_2_c.x, px_2_c.y - this->pillar_width / 2, px_2_c.z);
				Pointf3 px_15(px_3_c.x, px_3_c.y - this->pillar_width / 2, px_3_c.z);

				x_plane_points.push_back(Point(scale_(px_8.y), scale_(px_8.z)));
				x_plane_points.push_back(Point(scale_(px_9.y), scale_(px_9.z)));
				x_plane_points.push_back(Point(scale_(px_10.y), scale_(px_10.z)));
				x_plane_points.push_back(Point(scale_(px_11.y), scale_(px_11.z)));
				x_plane_points.push_back(Point(scale_(px_12.y), scale_(px_12.z)));
				x_plane_points.push_back(Point(scale_(px_13.y), scale_(px_13.z)));
				x_plane_points.push_back(Point(scale_(px_14.y), scale_(px_14.z)));
				x_plane_points.push_back(Point(scale_(px_15.y), scale_(px_15.z)));

				if (x_plane_points.size() != 0)
				{
					this->map_x_polygons[this->x_Num].push_back(Polygon(x_plane_points));
				}
				if (y_plane_points.size() != 0)
				{
					this->map_y_polygons[this->y_Num].push_back(Polygon(y_plane_points));
				}

				vertices.push_back(px_0);
				vertices.push_back(px_1);
				vertices.push_back(px_2);
				vertices.push_back(px_3);
				vertices.push_back(px_4);
				vertices.push_back(px_5);
				vertices.push_back(px_6);
				vertices.push_back(px_7);

				vertices.push_back(px_8);
				vertices.push_back(px_9);
				vertices.push_back(px_10);
				vertices.push_back(px_11);
				vertices.push_back(px_12);
				vertices.push_back(px_13);
				vertices.push_back(px_14);
				vertices.push_back(px_15);

				facets.push_back(Point3(0, 1, 7));
				facets.push_back(Point3(7, 1, 6));
				facets.push_back(Point3(6, 1, 5));
				facets.push_back(Point3(2, 5, 1));
				facets.push_back(Point3(5, 2, 3));
				facets.push_back(Point3(5, 3, 4));

				facets.push_back(Point3(0 + 8, 1 + 8, 7 + 8));
				facets.push_back(Point3(7 + 8, 1 + 8, 6 + 8));
				facets.push_back(Point3(6 + 8, 1 + 8, 5 + 8));
				facets.push_back(Point3(2 + 8, 5 + 8, 1 + 8));
				facets.push_back(Point3(5 + 8, 2 + 8, 3 + 8));
				facets.push_back(Point3(5 + 8, 3 + 8, 4 + 8));
			}

			TriangleMesh mesh_temp(vertices, facets);

			std::vector<Point3>().swap(facets);
			Pointf3s().swap(vertices);

			mesh_lspt.merge(mesh_temp);
		}
		else
		{
			Pointf3 px_0(top_p.x + this->pillar_width / 2, top_p.y, top_p.z);
			Pointf3 px_1(px_0.x, px_0.y, bottom_p.z);
			Pointf3 px_2(px_1.x - this->pillar_width, px_1.y, bottom_p.z);
			Pointf3 px_3(top_p.x - this->pillar_width / 2, top_p.y, top_p.z);

			y_plane_points.push_back(Point(scale_(px_0.x), scale_(px_0.z)));
			y_plane_points.push_back(Point(scale_(px_1.x), scale_(px_1.z)));
			y_plane_points.push_back(Point(scale_(px_2.x), scale_(px_2.z)));
			y_plane_points.push_back(Point(scale_(px_3.x), scale_(px_3.z)));

			Pointf3 px_4(top_p.x, top_p.y + this->pillar_width / 2, top_p.z);
			Pointf3 px_5(px_4.x, px_4.y, bottom_p.z);
			Pointf3 px_6(px_5.x, px_5.y - this->pillar_width, bottom_p.z);
			Pointf3 px_7(top_p.x, top_p.y - this->pillar_width / 2, top_p.z);

			x_plane_points.push_back(Point(scale_(px_4.y), scale_(px_4.z)));
			x_plane_points.push_back(Point(scale_(px_5.y), scale_(px_5.z)));
			x_plane_points.push_back(Point(scale_(px_6.y), scale_(px_6.z)));
			x_plane_points.push_back(Point(scale_(px_7.y), scale_(px_7.z)));

			vertices.push_back(px_0);
			vertices.push_back(px_1);
			vertices.push_back(px_2);
			vertices.push_back(px_3);
			vertices.push_back(px_4);
			vertices.push_back(px_5);
			vertices.push_back(px_6);
			vertices.push_back(px_7);

			if (x_plane_points.size() != 0)
			{
				this->map_x_polygons[this->x_Num].push_back(Polygon(x_plane_points));
			}
			if (y_plane_points.size() != 0)
			{
				this->map_y_polygons[this->y_Num].push_back(Polygon(y_plane_points));
			}

			facets.push_back(Point3(0, 1, 3));
			facets.push_back(Point3(3, 1, 2));
			facets.push_back(Point3(0 + 4, 1 + 4, 3 + 4));
			facets.push_back(Point3(3 + 4, 1 + 4, 2 + 4));

			TriangleMesh mesh_temp(vertices, facets);

			std::vector<Point3>().swap(facets);
			Pointf3s().swap(vertices);

			mesh_lspt.merge(mesh_temp);
		}

		mesh_lspt.repair();
		if (mesh_lspt.UnableRepair())
		{
			LOGINFO("Pillar::GeneralMesh Unable Repair");
			return TriangleMesh();
		}
		return mesh_lspt;
	}

	void LatticeSPT_Pointf3::set_parameter(const LatticeSupportMesh &latticeSupportMesh)
	{
		this->down_contact_width = latticeSupportMesh.down_contact_width;
		this->up_insert_z = latticeSupportMesh.up_insert_z;
		this->up_connection_height = latticeSupportMesh.up_connection_height;
		this->up_contact_height = latticeSupportMesh.up_contact_height;
		this->up_contact_width = latticeSupportMesh.up_contact_width;
		this->maximum_length = latticeSupportMesh.lattice_height * 2;
		this->angleD = latticeSupportMesh.lattice_grid_angle;
	}

	// 检查顶部支撑长度
	void LatticeSPT_Pointf3::check_support_length(LatticeSupportMesh &latticeSupportMesh)
	{
		if (this->lattice_spt_length() > this->maximum_length)
		{
			bool b_find = false;
			Pointf3 target_point;

			// coord_t x_num = this->x_num;
			// coord_t y_num = this->y_num;
			// 原有的晶格点的支撑数量减1
			Pillar &pillar_old = latticeSupportMesh.pillars[x_num][x_num];
			pillar_old.pillar_nodes[z_num].support_num--;

			int expand_num = 1;
			int start_x_num = x_num - expand_num;
			int start_y_num = y_num - expand_num;

			for (size_t x = 0; x < expand_num * 3; x++)
			{
				if (start_x_num + x < 0 || start_x_num + x > latticeSupportMesh.x_num_max - 1)
					continue;

				for (size_t y = 0; y < expand_num * 3; y++)
				{
					if (start_y_num + y < 0 || start_y_num + y > latticeSupportMesh.y_num_max - 1)
						continue;

					Pillar &pillar_0 = latticeSupportMesh.pillars[start_x_num + x][start_y_num + y];

					if (this->is_node_selected(this->UpPoint, pillar_0, latticeSupportMesh) == true)
					{
						target_point = this->DwPoint;
						b_find = true;
						// 新的晶格点的支撑数量加1
						pillar_0.pillar_nodes[z_num].support_num++;
					}
				}
			}

			if (b_find == false)
			{
				target_point = Pointf3(this->UpPoint.x, this->UpPoint.y, 0);
			}

			// 优先使用台柱撑
			Pointf3 p(this->UpPoint.x, this->UpPoint.y, this->UpPoint.z - this->up_contact_height - this->up_connection_height);
			Pointf3 pout;
			if (latticeSupportMesh.is_lattice_spt_intersect(p, target_point, pout)) // 顶支撑出现和物体干涉的情况
			{
				// LOGINFO("添加顶部支撑发生干涉，原始点 = [%s], 交点 = [%s]", target_point.dump_perl().c_str(), pout.dump_perl().c_str());
				// 将z值换成线段和物体最上的交点
				// 计算竖直的交点
				Pointf3 p1 = p - 0.1 * Pointf3(0, 0, 1);
				Pointf3 p2(p1.x, p1.y, 0);
				Pointf3 p3;
				if (latticeSupportMesh.is_intersect_new(latticeSupportMesh.pObj, p1, p2, p3)) // 竖直有交点
				{
					target_point = p3;
				}
				else // 竖直没有交点，直接连
				{
					target_point = pout;
				}
				latticeSupportMesh.pillars[x_num][y_num].pillar_nodes[z_num].support_num--;
				this->z_num = -1;
			}

			this->DwPoint = target_point;
		}
	}

	bool LatticeSPT_Pointf3::is_node_selected(Pointf3 select_point, Pillar &pillar, LatticeSupportMesh &latticeSupportMesh)
	{
		if (pillar.is_activated == false)
		{
			return false;
		}
		Pointf3 up = select_point;
		this->UpPoint = select_point;

		coord_t m = pillar.pillar_nodes.size() - 1;
		while (m >= 0)
		{
			if (pillar.pillar_nodes[m].z_Num == -1)
			{
				m--;
				continue;
			}

			Pointf3 dp = latticeSupportMesh.get_pillar_node_position(pillar.x_Num, pillar.y_Num, m);
			if (dp.z >= up.z)
			{
				m--;
				continue;
			}
			// 计算夹角
			double t1 = up.z - dp.z - this->up_connection_height - this->up_contact_height;
			double t2 = Pointf(up.x, up.y).distance_to(Pointf(dp.x, dp.y));
			double angle = atan2(t1, t2) * 180 / PI;
			if (angle < latticeSupportMesh.down_contact_angle)
			{
				m--;
				continue;
			}

			if (this->current_length == DBL_MAX)
			{
				this->current_length = this->UpPoint.z;
			}

			if (this->get_lattice_spt_length(up, dp) < this->current_length)
			{
				this->current_length = this->get_lattice_spt_length(up, dp);
				this->minimum_current_length_point = dp;

				this->x_num = pillar.x_Num;
				this->y_num = pillar.y_Num;
				this->z_num = m;
			}
			m--;
		}

		if (this->minimum_current_length_point == Pointf3(0, 0, 0))
		{
			this->minimum_current_length_point = Pointf3(up.x, up.y, 0);
		}

		this->DwPoint = this->minimum_current_length_point;
		return true;
	}

	TriangleMesh LatticeSPT_Pointf3::GenerateMesh()
	{
		// 三角面片顶点索引
		std::vector<Point3> facets;
		// 顶点数组
		Pointf3s vertices;

		if (this->z_num == -1) // 存在干涉，没有连接到节点上
		{
			Pointf3 px_0_c = this->UpPoint + this->up_insert_z * Pointf3(0, 0, 1);
			Pointf3 px_1_c = this->UpPoint + (this->up_contact_height) * Pointf3(0, 0, -1);
			Pointf3 px_2_c = this->UpPoint + (this->up_contact_height + this->up_connection_height) * Pointf3(0, 0, -1);
			Pointf3 px_3_c = this->DwPoint + (this->up_contact_height + this->up_connection_height) * Pointf3(0, 0, 1);
			Pointf3 px_4_c = this->DwPoint + (this->up_contact_height) * Pointf3(0, 0, 1);
			Pointf3 px_5_c = this->DwPoint + this->up_insert_z * Pointf3(0, 0, -1);

			double dis = Pointf(this->UpPoint.x, this->UpPoint.y).distance_to(Pointf(this->DwPoint.x, this->DwPoint.y));
			if (this->UpPoint.z - this->DwPoint.z <= (up_contact_height + up_connection_height) * 2 + dis) // 距离过短
			{
				Pointf3 px_0(px_0_c.x + this->up_contact_width / 2, px_0_c.y, px_0_c.z);
				Pointf3 px_1(px_5_c.x + this->up_contact_width / 2, px_5_c.y, px_5_c.z);
				Pointf3 px_2(px_5_c.x - this->up_contact_width / 2, px_5_c.y, px_5_c.z);
				Pointf3 px_3(px_0_c.x - this->up_contact_width / 2, px_0_c.y, px_0_c.z);

				Pointf3 px_4(px_0_c.x, px_0_c.y + this->up_contact_width / 2, px_0_c.z);
				Pointf3 px_5(px_5_c.x, px_5_c.y + this->up_contact_width / 2, px_5_c.z);
				Pointf3 px_6(px_5_c.x, px_5_c.y - this->up_contact_width / 2, px_5_c.z);
				Pointf3 px_7(px_0_c.x, px_0_c.y - this->up_contact_width / 2, px_0_c.z);

				vertices.push_back(px_0);
				vertices.push_back(px_1);
				vertices.push_back(px_2);
				vertices.push_back(px_3);
				vertices.push_back(px_4);
				vertices.push_back(px_5);
				vertices.push_back(px_6);
				vertices.push_back(px_7);

				facets.push_back(Point3(0, 1, 3));
				facets.push_back(Point3(3, 1, 2));
				facets.push_back(Point3(4, 5, 7));
				facets.push_back(Point3(7, 5, 6));
			}
			else
			{
				Pointf3 px_0(px_0_c.x + this->up_contact_width / 2, px_0_c.y, px_0_c.z);
				Pointf3 px_1(px_1_c.x + this->up_contact_width / 2, px_1_c.y, px_1_c.z);
				Pointf3 px_2(px_2_c.x + this->down_contact_width / 2, px_2_c.y, px_2_c.z);
				Pointf3 px_3(px_3_c.x + this->down_contact_width / 2, px_3_c.y, px_3_c.z);
				Pointf3 px_4(px_4_c.x + this->up_contact_width / 2, px_4_c.y, px_4_c.z);
				Pointf3 px_5(px_5_c.x + this->up_contact_width / 2, px_5_c.y, px_5_c.z);
				Pointf3 px_6(px_5_c.x - this->up_contact_width / 2, px_5_c.y, px_5_c.z);
				Pointf3 px_7(px_4_c.x - this->up_contact_width / 2, px_4_c.y, px_4_c.z);
				Pointf3 px_8(px_3_c.x - this->down_contact_width / 2, px_3_c.y, px_3_c.z);
				Pointf3 px_9(px_2_c.x - this->down_contact_width / 2, px_2_c.y, px_2_c.z);
				Pointf3 px_10(px_1_c.x - this->up_contact_width / 2, px_1_c.y, px_1_c.z);
				Pointf3 px_11(px_0_c.x - this->up_contact_width / 2, px_0_c.y, px_0_c.z);

				Pointf3 px_12(px_0_c.x, px_0_c.y + this->up_contact_width / 2, px_0_c.z);
				Pointf3 px_13(px_1_c.x, px_1_c.y + this->up_contact_width / 2, px_1_c.z);
				Pointf3 px_14(px_2_c.x, px_2_c.y + this->down_contact_width / 2, px_2_c.z);
				Pointf3 px_15(px_3_c.x, px_3_c.y + this->down_contact_width / 2, px_3_c.z);
				Pointf3 px_16(px_4_c.x, px_4_c.y + this->up_contact_width / 2, px_4_c.z);
				Pointf3 px_17(px_5_c.x, px_5_c.y + this->up_contact_width / 2, px_5_c.z);
				Pointf3 px_18(px_5_c.x, px_5_c.y - this->up_contact_width / 2, px_5_c.z);
				Pointf3 px_19(px_4_c.x, px_4_c.y - this->up_contact_width / 2, px_4_c.z);
				Pointf3 px_20(px_3_c.x, px_3_c.y - this->down_contact_width / 2, px_3_c.z);
				Pointf3 px_21(px_2_c.x, px_2_c.y - this->down_contact_width / 2, px_2_c.z);
				Pointf3 px_22(px_1_c.x, px_1_c.y - this->up_contact_width / 2, px_1_c.z);
				Pointf3 px_23(px_0_c.x, px_0_c.y - this->up_contact_width / 2, px_0_c.z);

				vertices.push_back(px_0);
				vertices.push_back(px_1);
				vertices.push_back(px_2);
				vertices.push_back(px_3);
				vertices.push_back(px_4);
				vertices.push_back(px_5);
				vertices.push_back(px_6);
				vertices.push_back(px_7);
				vertices.push_back(px_8);
				vertices.push_back(px_9);
				vertices.push_back(px_10);
				vertices.push_back(px_11);

				vertices.push_back(px_12);
				vertices.push_back(px_13);
				vertices.push_back(px_14);
				vertices.push_back(px_15);
				vertices.push_back(px_16);
				vertices.push_back(px_17);
				vertices.push_back(px_18);
				vertices.push_back(px_19);
				vertices.push_back(px_20);
				vertices.push_back(px_21);
				vertices.push_back(px_22);
				vertices.push_back(px_23);

				facets.push_back(Point3(11, 0, 1));
				facets.push_back(Point3(11, 1, 10));
				facets.push_back(Point3(10, 1, 2));
				facets.push_back(Point3(10, 2, 9));
				facets.push_back(Point3(9, 2, 3));
				facets.push_back(Point3(9, 3, 8));
				facets.push_back(Point3(8, 3, 4));
				facets.push_back(Point3(8, 4, 7));
				facets.push_back(Point3(7, 4, 5));
				facets.push_back(Point3(7, 5, 6));

				facets.push_back(Point3(11 + 12, 0 + 12, 1 + 12));
				facets.push_back(Point3(11 + 12, 1 + 12, 10 + 12));
				facets.push_back(Point3(10 + 12, 1 + 12, 2 + 12));
				facets.push_back(Point3(10 + 12, 2 + 12, 9 + 12));
				facets.push_back(Point3(9 + 12, 2 + 12, 3 + 12));
				facets.push_back(Point3(9 + 12, 3 + 12, 8 + 12));
				facets.push_back(Point3(8 + 12, 3 + 12, 4 + 12));
				facets.push_back(Point3(8 + 12, 4 + 12, 7 + 12));
				facets.push_back(Point3(7 + 12, 4 + 12, 5 + 12));
				facets.push_back(Point3(7 + 12, 5 + 12, 6 + 12));
			}
		}
		else
		{
			Pointf3 px_0_c = this->insert_up_point();
			Pointf3 px_1_c = this->UpPoint + (this->up_contact_height) * Pointf3(0, 0, -1);
			Pointf3 px_2_c = this->UpPoint + (this->up_contact_height + this->up_connection_height) * Pointf3(0, 0, -1);
			Pointf3 px_3_c = this->DwPoint;

			double dis = Pointf(this->UpPoint.x, this->UpPoint.y).distance_to(Pointf(this->DwPoint.x, this->DwPoint.y));
			if (this->UpPoint.z - this->DwPoint.z < up_contact_height + up_connection_height + dis)
			{
				Pointf3 px_0(px_0_c.x + this->up_contact_width / 2, px_0_c.y, px_0_c.z);
				Pointf3 px_1(px_3_c.x + this->down_contact_width / 2, px_3_c.y, px_3_c.z);
				Pointf3 px_2(px_3_c.x - this->down_contact_width / 2, px_3_c.y, px_3_c.z);
				Pointf3 px_3(px_0_c.x - this->up_contact_width / 2, px_0_c.y, px_0_c.z);

				Pointf3 px_4(px_0_c.x, px_0_c.y + this->up_contact_width / 2, px_0_c.z);
				Pointf3 px_5(px_3_c.x, px_3_c.y + this->down_contact_width / 2, px_3_c.z);
				Pointf3 px_6(px_3_c.x, px_3_c.y - this->down_contact_width / 2, px_3_c.z);
				Pointf3 px_7(px_0_c.x, px_0_c.y - this->up_contact_width / 2, px_0_c.z);

				vertices.push_back(px_0);
				vertices.push_back(px_1);
				vertices.push_back(px_2);
				vertices.push_back(px_3);
				vertices.push_back(px_4);
				vertices.push_back(px_5);
				vertices.push_back(px_6);
				vertices.push_back(px_7);

				facets.push_back(Point3(0, 1, 3));
				facets.push_back(Point3(3, 1, 2));
				facets.push_back(Point3(4, 5, 7));
				facets.push_back(Point3(7, 5, 6));
			}
			else
			{
				Pointf3 px_0(px_0_c.x + this->up_contact_width / 2, px_0_c.y, px_0_c.z);
				Pointf3 px_1(px_1_c.x + this->up_contact_width / 2, px_1_c.y, px_1_c.z);
				Pointf3 px_2(px_2_c.x + this->down_contact_width / 2, px_2_c.y, px_2_c.z);
				Pointf3 px_3(px_3_c.x + this->down_contact_width / 2, px_3_c.y, px_3_c.z);
				Pointf3 px_4(px_3_c.x - this->down_contact_width / 2, px_3_c.y, px_3_c.z);
				Pointf3 px_5(px_2_c.x - this->down_contact_width / 2, px_2_c.y, px_2_c.z);
				Pointf3 px_6(px_1_c.x - this->up_contact_width / 2, px_1_c.y, px_1_c.z);
				Pointf3 px_7(px_0_c.x - this->up_contact_width / 2, px_0_c.y, px_0_c.z);

				Pointf3 px_8(px_0_c.x, px_0_c.y + this->up_contact_width / 2, px_0_c.z);
				Pointf3 px_9(px_1_c.x, px_1_c.y + this->up_contact_width / 2, px_1_c.z);
				Pointf3 px_10(px_2_c.x, px_2_c.y + this->down_contact_width / 2, px_2_c.z);
				Pointf3 px_11(px_3_c.x, px_3_c.y + this->down_contact_width / 2, px_3_c.z);
				Pointf3 px_12(px_3_c.x, px_3_c.y - this->down_contact_width / 2, px_3_c.z);
				Pointf3 px_13(px_2_c.x, px_2_c.y - this->down_contact_width / 2, px_2_c.z);
				Pointf3 px_14(px_1_c.x, px_1_c.y - this->up_contact_width / 2, px_1_c.z);
				Pointf3 px_15(px_0_c.x, px_0_c.y - this->up_contact_width / 2, px_0_c.z);

				vertices.push_back(px_0);
				vertices.push_back(px_1);
				vertices.push_back(px_2);
				vertices.push_back(px_3);
				vertices.push_back(px_4);
				vertices.push_back(px_5);
				vertices.push_back(px_6);
				vertices.push_back(px_7);

				vertices.push_back(px_8);
				vertices.push_back(px_9);
				vertices.push_back(px_10);
				vertices.push_back(px_11);
				vertices.push_back(px_12);
				vertices.push_back(px_13);
				vertices.push_back(px_14);
				vertices.push_back(px_15);

				facets.push_back(Point3(0, 1, 7));
				facets.push_back(Point3(7, 1, 6));
				facets.push_back(Point3(6, 1, 5));
				facets.push_back(Point3(2, 5, 1));
				facets.push_back(Point3(5, 2, 3));
				facets.push_back(Point3(5, 3, 4));

				facets.push_back(Point3(0 + 8, 1 + 8, 7 + 8));
				facets.push_back(Point3(7 + 8, 1 + 8, 6 + 8));
				facets.push_back(Point3(6 + 8, 1 + 8, 5 + 8));
				facets.push_back(Point3(2 + 8, 5 + 8, 1 + 8));
				facets.push_back(Point3(5 + 8, 2 + 8, 3 + 8));
				facets.push_back(Point3(5 + 8, 3 + 8, 4 + 8));
			}
		}

		// 构建mesh
		TriangleMesh mesh(vertices, facets);
		mesh.checkonly = true;

		if (angleD != 0)
			mesh.rotate(angleD);
		return mesh;
	}

	// 生成需要加支撑的点
	bool LatticeSupportMesh::init_contact_support_points()
	{
		LOGINFO("start init_contact_support_points");
		DWORD timecount = GetTickCount();
		DWORD tt = timecount;

		contact_support_points.clear();
		LOGINFO("init_contact_support_points angel_step = %f", angel_step);
		LOGINFO("init_contact_support_points bound_num_min = %d", bound_num_min);
		LOGINFO("init_contact_support_points exclude_tuqi_area = %f", exclude_tuqi_area);
		LOGINFO("init_contact_support_points suspend_angel = %f", suspend_angel);
		LOGINFO("init_contact_support_points x_cell_length = %f", x_cell_length);
		LOGINFO("init_contact_support_points y_cell_length = %f", y_cell_length);
		LOGINFO("init_contact_support_points z_cell_length = %f", z_cell_length);
		LOGINFO("init_contact_support_points lattice_slice_thickness = %f", lattice_slice_thickness);
		LOGINFO("init_contact_support_points suspend_angel = %f", suspend_angel);
		LOGINFO("init_contact_support_points lattice_spt_distance = %f", lattice_spt_distance);
		// 根据切片获得待加的支撑点  没有faceID信息
		LatticeSPT_PointsGenerate Test_slice(Obj, lattice_slice_thickness, suspend_angel, lattice_spt_distance);
		Test_slice.Slice_Obj();
		LOGINFO("Slice_Obj cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		pObj->Lspt_Points = Test_slice.GetSPT_PointsVec();
		LOGINFO("1114 1 lattice pObj->Lspt_Points size  =%d", pObj->Lspt_Points.size());
		LOGINFO("GetSPT_PointsVec cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 对接触的支撑点进行修正  并赋值faceID信息
		pObj->Modify_SuspendPoints(lattice_spt_distance / 5, lattice_spt_distance / 5, lattice_slice_thickness);
		LOGINFO("Modify_SuspendPoints cost_times =%d", GetTickCount() - timecount);
		LOGINFO("1114 2 lattice pObj->Lspt_Points size  =%d", pObj->Lspt_Points.size());
		timecount = GetTickCount();

		// 对支撑点进行简化
		pObj->Simply_SuspendPoints(x_cell_length, y_cell_length, z_cell_length, reten_factor, tuqi_factor);
		LOGINFO("1114 3 lattice pObj->Lspt_Points size  =%d", pObj->Lspt_Points.size());
		contact_support_points = pObj->Lspt_Points;
		LOGINFO("Simply_SuspendPoints cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 处理横杆中轴线
		for (int i = 0; i < crossbar_midLines.size(); i++)
		{
			LOGINFO("crossbar_midLines.size() = [%d]", crossbar_midLines.size());
			Pointf3s seg_vec = crossbar_midLines[i].GetSegLines(crossbar_spt_distance, false);
			for (int j = 0; j < seg_vec.size(); j++)
			{
				LatticeSPT_Pointf3 _Lspt(0.0, 0.5, 0.2, 1.0);
				Pointf3 up = Pointf3(seg_vec[j].x, seg_vec[j].y, seg_vec[j].z);
				up.rotate(-lattice_grid_angle / 180 * PI);
				_Lspt.UpPoint = up;
				_Lspt.DwPoint = Pointf3(up.x, up.y, 0.0);
				_Lspt.x_num = 0;
				_Lspt.y_num = 0;
				_Lspt.Form_FaceID = -1;
				_Lspt.spt_type = LatticeSPT_Type::LSPT_Crossbar;
				_Lspt.Weight = 1;

				contact_support_points.push_back(_Lspt);
			}
		}
		LOGINFO("处理横杆中轴线 cost_times =%d", GetTickCount() - timecount);
		LOGINFO("init_contact_support_points cost_times =%d", GetTickCount() - tt);
		LOGINFO("contact_support_points.size() = [%d]", contact_support_points.size());

		if (contact_support_points.empty())
			return false;
		else
			return true;
	}

	// 生成加支撑物体的二维投影
	bool LatticeSupportMesh::generate_support_thumbnails()
	{
		DWORD timecount = GetTickCount();
		LOGINFO("start generate_support_thumbnails");
		// 计算投影
		if (support_thumbnails.empty())
		{
			support_thumbnails = pObj->horizontal_projection_slm(90); // 计算的expolygons，坐标放大了1e6倍
			if (support_thumbnails.empty())
			{
				LOGINFO("ERROR generate_support_thumbnails");
				return false;
			}
		}
		if (false)
		{
			SVG svg1("thumbnail1.svg");
			svg1.draw(support_thumbnails, "green");
			svg1.Close();
		}
		LOGINFO("horizontal_projection_slm cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 投影外扩排版间距
		ExPolygons temp_thumbnails = offset_ex(support_thumbnails, scale_(arrange_distance));
		ExPolygons new_thumbnails = temp_thumbnails;

		LOGINFO("offset_ex cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 基台范围检查
		if (bed_type.points.size() > 3) // 基台存在
		{
			ExPolygons beds;
			ExPolygon bed_ex;
			bed_ex.contour = bed_type;
			beds.push_back(bed_ex);

			new_thumbnails.clear();
			new_thumbnails = intersection_ex(beds, temp_thumbnails);
			if (false)
			{
				SVG svg2("new_thumbnails.svg");
				svg2.draw(beds, "red");
				svg2.draw(new_thumbnails, "green");
				svg2.Close();
			}
		}
		else
		{
			LOGINFO("LatticeSupportMesh::generate_support_thumbnails bed_type is empty!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			return false;
		}

		if (new_thumbnails.empty())
		{
			LOGINFO("new_thumbnails.empty()!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			return false;
		}

		LOGINFO("intersection_ex cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 投影简化
		support_thumbnails.clear();
		for (ExPolygons::const_iterator it = new_thumbnails.begin(); it != new_thumbnails.end(); ++it)
			support_thumbnails += it->simplify(scale_(arrange_distance / 2));
		support_thumbnails = union_ex(support_thumbnails);

		LOGINFO("simplify union_ex cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		if (false)
		{
			SVG svg1("thumbnail2.svg");
			svg1.draw(bed_type, "red");
			svg1.draw(support_thumbnails, "green");
			svg1.Close();
		}
		LOGINFO("end generate_support_thumbnails");

		return true;
	}

	bool LatticeSupportMesh::_is_in_thumbnails(Point p)
	{
		if (support_thumbnails.empty())
		{
			if (generate_support_thumbnails() == false)
				return false;
		}

		if (support_thumbnails.empty())
			return false;

		for (int i = 0; i < support_thumbnails.size(); i++)
		{
			ExPolygon ex = support_thumbnails.at(i);
			if (ex.contour.contains_b(p))
				return true;
		}

		return false;
	}

	// 晶格初始化
	bool LatticeSupportMesh::init_lattice_grid()
	{
		LOGINFO("start init_lattice_grid");
		coord_t lattice_width_scale = scale_(lattice_width);

		BoundingBoxf3 bb = pObj->bounding_box();

		// 包围盒外扩
		bb.min.x -= lattice_width * extand_factor;
		bb.max.x += lattice_width * extand_factor;
		bb.min.y -= lattice_width * extand_factor;
		bb.max.y += lattice_width * extand_factor;

		// 根据包围盒建立网格
		// 确定起始点位置
		start_location = bb.min;
		center_location = bb.center();

		x_num_max = ceil((bb.max.x - bb.min.x) / lattice_width);
		y_num_max = ceil((bb.max.y - bb.min.y) / lattice_width);

		LOGINFO("x_num_max = [%d], y_num_max = [%d]", x_num_max, y_num_max);

		// 建立初始网格点
		for (coord_t i = 0; i < x_num_max; i++)
		{
			for (coord_t j = 0; j < y_num_max; j++)
			{
				pillars[i][j].set_grid_position(i, j);
				Pointf point = this->get_pillar_position(i, j);

				// 设置该晶格柱所在的区域的类型
				//  计算碰撞的三角面片
				std::vector<int> facet_ids = pObj->GetFacetIdxByZaxisFast(point.x, point.y);
				if (facet_ids.empty() == false)
				{
					stl_face_side _side = pObj->stl.facet_start[facet_ids.at(0)].face_side;
					stl_face_type _type = pObj->stl.facet_start[facet_ids.at(0)].face_type;

					// 如果有reten标记则标为reten， 如果有sw标记则标为sw，其他按照第一个标记
					if (facet_ids.size() > 1)
					{
						for (int k = 1; k < facet_ids.size(); k++)
						{
							if (pObj->stl.facet_start[facet_ids.at(k)].face_type == RetentionMesh_Part ||
								pObj->stl.facet_start[facet_ids.at(k)].face_type == Fulcrum_Part ||
								pObj->stl.facet_start[facet_ids.at(k)].face_type == Nail_Part ||
								pObj->stl.facet_start[facet_ids.at(k)].face_type == StippledWax_Part)
							{
								_side = pObj->stl.facet_start[facet_ids.at(k)].face_side;
								_type = pObj->stl.facet_start[facet_ids.at(k)].face_type;

								LOGINFO("find a pillar in RetentionMesh_Part Or StippledWax_Part, face_type = [%d]", _type);
							}
						}
					}

					if (_type == RetentionMesh_Part ||
						_type == Fulcrum_Part ||
						_type == Nail_Part ||
						_type == StippledWax_Part)
					{
						LOGINFO("find a pillar in RetentionMesh_Part Or StippledWax_Part, face_type = [%d]", _type);
					}

					pillars[i][j].set_pillar_type(_type, _side);
				}
			}
		}
		LOGINFO("end init_lattice_grid");

		return true;
	}

	// 建立台柱的拓扑关系
	bool LatticeSupportMesh::generate_pillars()
	{
		LOGINFO("start generate_pillars");
		DWORD timecount = GetTickCount();
		DWORD tt = timecount;

		// 支撑范围检查
		if (generate_support_thumbnails() == false)
			return false;
		LOGINFO("generate_support_thumbnails cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 晶格初始化，建立初始网格
		if (init_lattice_grid() == false)
			return false;
		LOGINFO("init_lattice_grid cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 生成需要加支撑的点
		if (init_contact_support_points() == false)
			return false;
		LOGINFO("init_contact_support_points cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 顶部支撑点聚类, 激活台柱
		cluster_contact_support_points(this->avoid_reten_area, this->avoid_tuqi_area);
		LOGINFO("cluster_contact_support_points cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 计算各个有支撑点激活台柱的节点,并得到作为主干的节点类型，同时设置该台柱上所有顶支撑的下支撑点坐标
		PillarNode_Type type = generate_all_support_pillar_nodes();
		LOGINFO("generate_all_support_pillar_nodes cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 各个激活台柱的延伸extand_factor
		extand_pillars();
		LOGINFO("extand_pillars cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 检查过长的顶部支撑
		check_all_support_length();
		LOGINFO("check_all_support_length cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		// 建立各个网格点之间的拓扑关系
		generate_pillars_topology(type);
		LOGINFO("generate_pillars_topology cost_times =%d", GetTickCount() - timecount);
		timecount = GetTickCount();

		check_pillars();
		LOGINFO("check_pillars cost_times =%d", GetTickCount() - timecount);
		LOGINFO("generate_pillars cost_times =%d", GetTickCount() - tt);

		return true;
	}

	// 添加顶部支撑，寻找最近点
	bool LatticeSupportMesh::get_nearest_pillar_node_position(Pointf3 select_point, Pointf3 &target_point)
	{
		// 判断支撑点落入的网格点
		coord_t xi = ceil((select_point.x - start_location.x) / this->lattice_width);
		coord_t yi = ceil((select_point.y - start_location.y) / this->lattice_width);
		xi = xi > x_num_max - 1 ? x_num_max - 1 : xi;
		yi = yi > y_num_max - 1 ? y_num_max - 1 : yi;

		// 确定支撑点连到哪个晶格点上
		coord_t sx, sy;
		Pointf sp(select_point.x, select_point.y);
		coordf_t dis = DBL_MAX;

		coord_t x_num = xi;
		coord_t y_num = yi;

		int expand_num = 1;
		int start_x_num = x_num - expand_num;
		int start_y_num = y_num - expand_num;

		LatticeSPT_Pointf3 lattice_spt;
		lattice_spt.set_parameter(*this);

		bool r = false;
		for (size_t x = 0; x < expand_num * 3; x++)
		{
			for (size_t y = 0; y < expand_num * 3; y++)
			{
				Pillar &pillar_0 = this->pillars[start_x_num + x][start_y_num + y];
				r = lattice_spt.is_node_selected(select_point, pillar_0, *this);
				if (r)
				{
					target_point = lattice_spt.DwPoint;
				}
			}
		}

		if (r == false)
		{
			target_point = Pointf3(select_point.x, select_point.y, 0.0);
			// return false;
		}

		// 优先使用台柱撑
		Pointf3 p(select_point.x, select_point.y, select_point.z - this->up_contact_height - this->up_connection_height);
		Pointf3 pout;
		bool result = false;
		if (this->is_lattice_spt_intersect(p, target_point, pout)) // 顶支撑出现和物体干涉的情况
		{
			LOGINFO("添加顶部支撑发生干涉，原始点 = [%s], 交点 = [%s]", target_point.dump_perl().c_str(), pout.dump_perl().c_str());
			// 将z值换成线段和物体最上的交点
			// 计算竖直的交点
			Pointf3 p1 = select_point - 0.1 * Pointf3(0, 0, 1);
			Pointf3 p2(p1.x, p1.y, 0);
			Pointf3 p3;
			if (is_intersect_new(pObj, p1, p2, p3)) // 竖直有交点
			{
				target_point = p3;
			}
			else // 竖直没有交点，直接连
			{
				target_point = pout;
			}
			this->pillars[lattice_spt.x_num][lattice_spt.y_num].pillar_nodes[lattice_spt.z_num].support_num--;
			result = true;
		}

		return result;
	}

	Pointf LatticeSupportMesh::get_pillar_position(coord_t x, coord_t y)
	{
		coordf_t xi = start_location.x + lattice_width * x;
		coordf_t yi = start_location.y + lattice_width * y;

		Pointf p = Pointf(xi, yi);
		return p;
	}

	Pointf3 LatticeSupportMesh::get_pillarnode_position(coord_t x, coord_t y, coord_t z)
	{
		coordf_t xi = start_location.x + lattice_width * x;
		coordf_t yi = start_location.y + lattice_width * y;
		Pointf p = Pointf(xi, yi);

		coordf_t zi = lattice_height * z;

		return Pointf3(p.x, p.y, zi);
	}

	// 顶部支撑点连接到相应的台柱上，激活相应的台柱
	bool LatticeSupportMesh::cluster_contact_support_points(bool reten, bool sw)
	{
		LOGINFO("start cluster_contact_support_points");
		// coordf_t lattice_width_scale = scale_(lattice_width);
		for (LatticeSPT_Pointf3s::iterator csp = contact_support_points.begin(); csp != contact_support_points.end(); csp++)
		{
			// 判断支撑点落入的网格点
			coord_t xi = ceil((csp->UpPoint.x - start_location.x) / lattice_width); // x索引向后取整
			coord_t yi = ceil((csp->UpPoint.y - start_location.y) / lattice_width); // y索引向后取整
			xi = xi > x_num_max - 1 ? x_num_max - 1 : xi;
			yi = yi > y_num_max - 1 ? y_num_max - 1 : yi;

			xi = xi < 0 ? 0 : xi;
			yi = yi < 0 ? 0 : yi;

			// 确定支撑点连到哪个晶格点上
			coord_t sx = xi, sy = yi;
			Pointf sp(csp->UpPoint.x, csp->UpPoint.y);
			coordf_t dis = DBL_MAX;

			// 遍历周围四个点，计算距离最近的格点
			for (coord_t i = 0; i < 2; i++)
			{
				if (xi - i < 0)
					continue;
				for (coord_t j = 0; j < 2; j++)
				{
					if (yi - j < 0)
						continue;
					Pointf pp = get_pillar_position(xi - i, yi - j);
					if (_is_in_thumbnails(Point(scale_(pp.x), scale_(pp.y))) == false ||
						_is_in_thumbnails(Point(scale_(pp.x - pillar_width / 2), scale_(pp.y))) == false ||
						_is_in_thumbnails(Point(scale_(pp.x + pillar_width / 2), scale_(pp.y))) == false ||
						_is_in_thumbnails(Point(scale_(pp.x), scale_(pp.y - pillar_width / 2))) == false ||
						_is_in_thumbnails(Point(scale_(pp.x), scale_(pp.y + pillar_width / 2))) == false)
					{
						// LOGINFO("find a Point is not in_thumbnails!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
						continue;
					}

					coordf_t tmp = sp.distance_to(pp);
					if (tmp < dis)
					{
						dis = tmp;
						sx = xi - i;
						sy = yi - j;
					}
				}
			}

			// 遍历四个点没有找到落在投影内的点，降低要求只找落在盘上的晶格点
			if (dis == DBL_MAX)
			{
				for (coord_t i = 0; i < 2; i++)
				{
					if (xi - i < 0)
						continue;
					for (coord_t j = 0; j < 2; j++)
					{
						if (yi - j < 0)
							continue;
						Pointf pp = get_pillar_position(xi - i, yi - j);
						if (bed_type.contains_b(Point(scale_(pp.x), scale_(pp.y))) == false ||
							bed_type.contains_b(Point(scale_(pp.x - pillar_width / 2), scale_(pp.y))) == false ||
							bed_type.contains_b(Point(scale_(pp.x + pillar_width / 2), scale_(pp.y))) == false ||
							bed_type.contains_b(Point(scale_(pp.x), scale_(pp.y - pillar_width / 2))) == false ||
							bed_type.contains_b(Point(scale_(pp.x), scale_(pp.y + pillar_width / 2))) == false)
						{
							// LOGINFO("find a Point is not bed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
							continue;
						}
						coordf_t tmp = sp.distance_to(pp);
						if (tmp < dis)
						{
							dis = tmp;
							sx = xi - i;
							sy = yi - j;
						}
					}
				}
			}

			if (dis == DBL_MAX)
			{
				LOGINFO("Find a support point which cannot connnect to a lattice!");
				continue;
			}

			// 设置支撑点连接的网格点
			csp->x_num = sx;
			csp->y_num = sy;

			LOGINFO("cluster a contact_support_points!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

			Pillar &pillar = pillars[sx][sy];
			pillar.is_activated = true;

			// 固位网区域检查
			stl_face_type _type = pillars[sx][sy].face_type;
			if (reten && (_type == RetentionMesh_Part || _type == Fulcrum_Part || _type == Nail_Part))
			{
				LOGINFO("Find a support point which cannot connnect to a lattice, RetentionMesh_Part! _type = [%d]", _type);
				pillar.is_activated = false;
			}

			// 花纹区域检查
			if (sw && _type == StippledWax_Part)
			{
				LOGINFO("Find a support point which cannot connnect to a lattice, StippledWax_Part! _type = [%d]", _type);
				pillar.is_activated = false;
			}

			pillar.support_points.push_back(*csp);
		}

		LOGINFO("end cluster_contact_support_points");

		return true;
	}

	// P1,P2要有相同的x,y值，否则失效
	bool LatticeSupportMesh::is_intersect_new(TriangleMesh *mesh, Pointf3 p1, Pointf3 p2, Pointf3 &p3)
	{
		return mesh->is_intersect_new(p1, p2, p3);
	}

	// 检查两个点之间的线段和物体是否相交，确保p1的z值大于p2, p3记录最高的交点
	bool LatticeSupportMesh::is_intersect(Pointf3 p1, Pointf3 p2, Pointf3 &p3)
	{
		return pObj->is_intersect(p1, p2, p3);
	}

	// 检查晶格支撑是否干涉, node1要在node2上面(z1>z2)
	bool LatticeSupportMesh::is_intersect(PillarNode node1, PillarNode node2, int type, Pointf3 &pout)
	{
		// LOGINFO("start lattice is_intersect!");

		// DWORD timecount = GetTickCount();

		Pointf3 p1 = get_pillarnode_position(node1.x_Num, node1.y_Num, node1.z_Num);
		Pointf3 p2 = get_pillarnode_position(node2.x_Num, node2.y_Num, node2.z_Num);
		Pointf3 p3(0, 0, 0);
		// LOGINFO("start is_intersect node, type = [%d], p1 = [%s], p2 = [%s]", type, p1.dump_perl().c_str(), p2.dump_perl().c_str());
		bool result = false;
		double width = this->lattice_min_distance; // pillar_width / 2 * 6;//0.75

		if (type == -1)
		{
			result = is_intersect_new(pObj, p1, p2, p3);
			if (result)
				pout = p3;

			if (/*result == */ false) // 中线不相交，检查十字棱柱的四个边 //add by gao 20200828,暂不检查台柱四个边
			{
				// x-pillar_width/2,y
				Pointf3 p10(p1.x - width, p1.y, p1.z);
				Pointf3 p20(p2.x - width, p2.y, p2.z);
				Pointf3 p30(0, 0, 0);
				bool result0 = is_intersect_new(pObj, p10, p20, p30);
				if (result0)
				{
					pout.x = p1.x;
					pout.y = p1.y;
					pout.z = p30.z;
				}

				// x+pillar_width/2,y
				Pointf3 p11(p1.x + width, p1.y, p1.z);
				Pointf3 p21(p2.x + width, p2.y, p2.z);
				Pointf3 p31(0, 0, 0);
				bool result1 = is_intersect_new(pObj, p11, p21, p31);
				if (result1)
				{
					pout.z = p31.z;
					pout.x = p1.x;
					pout.y = p1.y;
				}

				// x,y-pillar_width/2
				Pointf3 p12(p1.x, p1.y - width, p1.z);
				Pointf3 p22(p2.x, p2.y - width, p2.z);
				Pointf3 p32(0, 0, 0);
				bool result2 = is_intersect_new(pObj, p12, p22, p32);
				if (result2)
				{
					pout.z = p32.z;
					pout.x = p1.x;
					pout.y = p1.y;
				}

				// x,y+pillar_width/2
				Pointf3 p13(p1.x, p1.y + width, p1.z);
				Pointf3 p23(p2.x, p2.y + width, p2.z);
				Pointf3 p33(0, 0, 0);
				bool result3 = is_intersect_new(pObj, p13, p23, p33);
				if (result3)
				{
					pout.z = p33.z;
					pout.x = p1.x;
					pout.y = p1.y;
				}

				result = result0 || result1 || result2 || result3;
			}
		}
		else if (type == 0)
		{
			Pointf3 p11 = rotate_pointf3(rotation_matrix0, p1);
			Pointf3 p21 = rotate_pointf3(rotation_matrix0, p2);
			// LOGINFO("start is_intersect node, type = [%d], p1 = [%s], p2 = [%s]", type, p11.dump_perl().c_str(), p21.dump_perl().c_str());

			result = is_intersect_new(&mesh0, p11, p21, p3);

			if (result == false) // 中线不相交，检查一字面片的两个边
			{
				// x-pillar_width/2,y
				Pointf3 p1x0(p1.x - width, p1.y, p1.z);
				Pointf3 p2x0(p2.x - width, p2.y, p2.z);
				Pointf3 p30(0, 0, 0);
				Pointf3 p1x01 = rotate_pointf3(rotation_matrix0, p1x0);
				Pointf3 p2x01 = rotate_pointf3(rotation_matrix0, p2x0);
				bool result0 = is_intersect_new(&mesh0, p1x01, p2x01, p30);

				// x+pillar_width/2,y
				Pointf3 p1x1(p1.x + width, p1.y, p1.z);
				Pointf3 p2x1(p2.x + width, p2.y, p2.z);
				Pointf3 p31(0, 0, 0);
				Pointf3 p1x11 = rotate_pointf3(rotation_matrix0, p1x1);
				Pointf3 p2x11 = rotate_pointf3(rotation_matrix0, p2x1);
				bool result1 = is_intersect_new(&mesh0, p1x11, p2x11, p31);

				result = result0 || result1;
			}

			if (result)
				pout = rotate_pointf3(rotation_matrix0.inverse(), p3);
		}
		else if (type == 1)
		{
			Pointf3 p11 = rotate_pointf3(rotation_matrix1, p1);
			Pointf3 p21 = rotate_pointf3(rotation_matrix1, p2);
			// LOGINFO("start is_intersect node, type = [%d], p1 = [%s], p2 = [%s]", type, p11.dump_perl().c_str(), p21.dump_perl().c_str());

			result = is_intersect_new(&mesh1, p11, p21, p3);

			if (result == false) // 中线不相交，检查一字面片的两个边
			{
				// x-pillar_width/2,y
				Pointf3 p1x0(p1.x - width, p1.y, p1.z);
				Pointf3 p2x0(p2.x - width, p2.y, p2.z);
				Pointf3 p30(0, 0, 0);
				Pointf3 p1x01 = rotate_pointf3(rotation_matrix1, p1x0);
				Pointf3 p2x01 = rotate_pointf3(rotation_matrix1, p2x0);
				bool result0 = is_intersect_new(&mesh1, p1x01, p2x01, p30);

				// x+pillar_width/2,y
				Pointf3 p1x1(p1.x + width, p1.y, p1.z);
				Pointf3 p2x1(p2.x + width, p2.y, p2.z);
				Pointf3 p31(0, 0, 0);
				Pointf3 p1x11 = rotate_pointf3(rotation_matrix1, p1x1);
				Pointf3 p2x11 = rotate_pointf3(rotation_matrix1, p2x1);
				bool result1 = is_intersect_new(&mesh1, p1x11, p2x11, p31);

				result = result0 || result1;
			}

			if (result)
				pout = rotate_pointf3(rotation_matrix1.inverse(), p3);
		}
		else if (type == 2)
		{
			Pointf3 p11 = rotate_pointf3(rotation_matrix2, p1);
			Pointf3 p21 = rotate_pointf3(rotation_matrix2, p2);
			// LOGINFO("start is_intersect node, type = [%d], p1 = [%s], p2 = [%s]", type, p11.dump_perl().c_str(), p21.dump_perl().c_str());

			result = is_intersect_new(&mesh2, p11, p21, p3);

			if (result == false) // 中线不相交，检查一字面片的两个边
			{
				// x,y-pillar_width/2
				Pointf3 p1x0(p1.x, p1.y - width, p1.z);
				Pointf3 p2x0(p2.x, p2.y - width, p2.z);
				Pointf3 p30(0, 0, 0);
				Pointf3 p1x01 = rotate_pointf3(rotation_matrix2, p1x0);
				Pointf3 p2x01 = rotate_pointf3(rotation_matrix2, p2x0);
				bool result0 = is_intersect_new(&mesh2, p1x01, p2x01, p30);

				// x,y+pillar_width/2
				Pointf3 p1x1(p1.x, p1.y + width, p1.z);
				Pointf3 p2x1(p2.x, p2.y + width, p2.z);
				Pointf3 p31(0, 0, 0);
				Pointf3 p1x11 = rotate_pointf3(rotation_matrix2, p1x1);
				Pointf3 p2x11 = rotate_pointf3(rotation_matrix2, p2x1);
				bool result1 = is_intersect_new(&mesh2, p1x11, p2x11, p31);

				result = result0 || result1;
			}

			if (result)
				pout = rotate_pointf3(rotation_matrix2.inverse(), p3);
		}
		else if (type == 3)
		{
			Pointf3 p11 = rotate_pointf3(rotation_matrix3, p1);
			Pointf3 p21 = rotate_pointf3(rotation_matrix3, p2);
			// LOGINFO("start is_intersect node, type = [%d], p1 = [%s], p2 = [%s]", type, p11.dump_perl().c_str(), p21.dump_perl().c_str());

			result = is_intersect_new(&mesh3, p11, p21, p3);

			if (result == false) // 中线不相交，检查一字面片的两个边
			{
				// x,y-pillar_width/2
				Pointf3 p1x0(p1.x, p1.y - width, p1.z);
				Pointf3 p2x0(p2.x, p2.y - width, p2.z);
				Pointf3 p30(0, 0, 0);
				Pointf3 p1x01 = rotate_pointf3(rotation_matrix3, p1x0);
				Pointf3 p2x01 = rotate_pointf3(rotation_matrix3, p2x0);
				bool result0 = is_intersect_new(&mesh3, p1x01, p2x01, p30);

				// x,y+pillar_width/2
				Pointf3 p1x1(p1.x, p1.y + width, p1.z);
				Pointf3 p2x1(p2.x, p2.y + width, p2.z);
				Pointf3 p31(0, 0, 0);
				Pointf3 p1x11 = rotate_pointf3(rotation_matrix3, p1x1);
				Pointf3 p2x11 = rotate_pointf3(rotation_matrix3, p2x1);
				bool result1 = is_intersect_new(&mesh3, p1x11, p2x11, p31);

				result = result0 || result1;
			}

			if (result)
				pout = rotate_pointf3(rotation_matrix3.inverse(), p3);
		}

		// LOGINFO("is_intersect(PillarNode node1, PillarNode node2, int type) times =%d", GetTickCount() - timecount);
		// LOGINFO("end is_intersect(PillarNode node1, PillarNode node2, int type)");

		return result;
	}

	PillarNode_Type LatticeSupportMesh::generate_one_support_pillar_nodes(Pillar &pillar)
	{
		// LOGINFO("start generate_one_support_pillar_nodes");

		// DWORD timecount = GetTickCount();

		coord_t k = 0;
		if (pillar.is_activated)
		{
			coordf_t x = start_location.x + pillar.x_Num * lattice_width;
			coordf_t y = start_location.y + pillar.y_Num * lattice_width;

			coord_t s_num = pillar.support_points.size();
			double up_height = lattice_min_distance; // 顶端支撑的预留高度
			double pillar_height = 0.0;				 // 台柱的高度

			// 计算支撑点的最大Z值
			double max_z = 0.0;
			for (auto var : pillar.support_points)
			{
				if (var.UpPoint.z > max_z)
					max_z = var.UpPoint.z;
			}

			// 计算碰撞的三角面片
			std::vector<int> facet_ids = pObj->GetFacetIdxByZaxisFast(x, y);
			if (facet_ids.size() == 0)
			{
				// LOGINFO("this pillar does not has CollsionZ!!");
				pillar_height = max_z - up_height;
			}
			else
			{
				// 记录碰撞信息
				pillar.CollsionZ_map.clear();
				for (int i = 0; i < facet_ids.size(); i++)
				{
					double collsion_z = pObj->GetPointZByZaxis(facet_ids[i], x, y);
					pillar.CollsionZ_map[collsion_z] = facet_ids[i];
				}

				// 射线求交的第二高点
				double maxZ = pillar.get_second_maxcollsionZ();
				pillar_height = maxZ - up_height;
			}

			// 生成台柱节点
			if (pillar_height <= 0)
			{
				// LOGINFO("射线的交点高度过小，无法生成台柱，只有一个0节点!!");
				// pillar.pillar_nodes[0].set_grid_position(pillar.x_Num, pillar.y_Num, 0);

				// 只有一个0节点，下支撑点直接到基台
				for (LatticeSPT_Pointf3s::iterator csp = pillar.support_points.begin(); csp != pillar.support_points.end(); csp++)
				{
					csp->DwPoint = Pointf3(csp->UpPoint.x, csp->UpPoint.y, 0.0);
					csp->set_pillar_position(0);
				}

				return None;
			}

			k = floor(pillar_height / lattice_height);

			if (k > 0)
			{
				for (int kk = 0; kk <= k; kk++)
				{
					pillar.pillar_nodes[kk].set_grid_position(pillar.x_Num, pillar.y_Num, kk);
				}
			}
		}

		// 设置该台柱上所有顶支撑的下支撑点
		// LOGINFO("x_Num = %d, y_Num = %d, 该台柱共有【%d】个节点， 共有【%d】个顶部支撑点", pillar.x_Num, pillar.y_Num, k, pillar.support_points.size());
		for (LatticeSPT_Pointf3s::iterator csp = pillar.support_points.begin(); csp != pillar.support_points.end(); csp++)
		{
			csp->set_parameter(*this);
			// LOGINFO("start to computer a DwPoint");

			Pointf3 up(csp->UpPoint.x, csp->UpPoint.y, csp->UpPoint.z - up_contact_height);
			if (pillar.is_activated)
			{
				// 计算支撑点应该连接到哪个晶格点

				coord_t m = k;
				while (m >= 0)
				{
					if (pillar.pillar_nodes[m].z_Num == -1)
					{
						m--;
						continue;
					}

					Pointf3 dp = this->get_pillarnode_position(pillar.x_Num, pillar.y_Num, m);
					// LOGINFO("m = %d, up = %s, dp = %s", m, dp.dump_perl().c_str(), dp.dump_perl().c_str());
					if (dp.z < up.z)
					{
						// 计算夹角
						double t1 = up.z - dp.z;
						double t2 = Pointf(up.x, up.y).distance_to(Pointf(dp.x, dp.y));
						double angle = atan2(t1, t2) * 180 / PI;
						// LOGINFO("t1 = [%f], t2 = [%f], down_contact_angle = %d, angle = %f", t1, t2, down_contact_angle, angle);

						if (angle >= down_contact_angle)
							break;
					}
					m--;
				}

				if (m <= 0)
				{
					csp->DwPoint = Pointf3(up.x, up.y, 0);
					csp->set_pillar_position(0);
				}
				else
				{
					csp->DwPoint = this->get_pillarnode_position(pillar.x_Num, pillar.y_Num, m);
					csp->set_pillar_position(m);
					pillar.pillar_nodes[csp->z_num].support_num++;
				}
			}
			else // 台柱未激活，直接连接到基台上
			{
				csp->DwPoint = Pointf3(up.x, up.y, 0);
				csp->set_pillar_position(0);
			}

			Pointf3 pout;
			if (is_intersect(up, csp->DwPoint, pout)) // 顶支撑出现和物体干涉的情况
			{
				// LOGINFO("find a intersect support, pillar.x_Num = [%d], pillar.y_Num = [%d], pillar.z_Num = [%d]", pillar.x_Num, pillar.y_Num, m);
				// 计算竖直的交点
				Pointf3 p1 = csp->UpPoint - 0.1 * Pointf3(0, 0, 1);
				Pointf3 p2;
				if (is_intersect_new(pObj, p1, Pointf3(p1.x, p1.y, 0), p2)) // 竖直有交点
				{
					if (csp->spt_type == LSPT_Crossbar) // 横杆单独处理
					{
						csp->DwPoint = p2;
					}
					else //
					{
						// 太短的支撑没价值，直接抛弃
						if (p1.z - p2.z < 2 * this->up_contact_height)
						{
							csp->GenerateValid = false;
							// LOGINFO("竖直有交点,find too short top_lattice_support, height = [%f]", p1.z - p2.z);
						}
						else
						{
							// 检查竖直支撑的十字的四条边和物体的相交情况
							bool result = false;

							double width = this->down_contact_width;							  // 初始距离为下十字宽度的2倍
							double height = this->up_contact_height + this->up_connection_height; // 高度缩减

							// 高度小于2倍的上支撑宽度，直接连接，宽度和高度要更新为上支撑的宽度和高度
							if (p1.z - p2.z < 2 * height)
							{
								width = this->up_contact_width;
								height = this->up_contact_height;
							}

							Pointf3 p10(p1.x - width, p1.y, p1.z - height);
							Pointf3 p20(p2.x - width, p2.y, p2.z + height);
							Pointf3 p30(0, 0, 0);
							bool result0 = is_intersect_new(pObj, p10, p20, p30);

							// x+pillar_width,y
							Pointf3 p11(p1.x + width, p1.y, p1.z - height);
							Pointf3 p21(p2.x + width, p2.y, p2.z + height);
							Pointf3 p31(0, 0, 0);
							bool result1 = is_intersect_new(pObj, p11, p21, p31);

							// x,y-pillar_width
							Pointf3 p12(p1.x, p1.y - width, p1.z - height);
							Pointf3 p22(p2.x, p2.y - width, p2.z + height);
							Pointf3 p32(0, 0, 0);
							bool result2 = is_intersect_new(pObj, p12, p22, p32);

							// x,y+pillar_width
							Pointf3 p13(p1.x, p1.y + width, p1.z - height);
							Pointf3 p23(p2.x, p2.y + width, p2.z + height);
							Pointf3 p33(0, 0, 0);
							bool result3 = is_intersect_new(pObj, p13, p23, p33);

							result = result0 || result1 || result2 || result3;

							// 四条边有相交，不造型
							if (result)
							{
								csp->GenerateValid = false;
								// LOGINFO("竖直有交点,四条边有相交，不造型 top_lattice_support, height = [%f]", p1.z - p2.z);
							}
							else
								csp->DwPoint = p2;
						}
					}
				}
				else // 竖直没有交点，直接连
				{
					// 太短的支撑没价值，直接抛弃
					if (p1.z - pout.z < 2 * this->up_contact_height && csp->spt_type != LSPT_Crossbar)
					{
						csp->GenerateValid = false;
						LOGINFO("竖直没有交点,find too short top_lattice_support, height = [%f]", p1.z - pout.z);
					}
					else
						csp->DwPoint = pout;
				}

				// 将z值换成线段和物体最上的交点
				pillar.pillar_nodes[csp->z_num].support_num--;
				csp->set_pillar_position(-1);
			}
			else
			{
			}

			// 检测顶部支撑是否距离物体过近，过近则不造型
			if (csp->spt_type == LSPT_Crossbar)
				continue;

			if (csp->z_num == -1)
			{
				// 检查三个点，上端点、中点、下端点
				Pointf3 p_up(csp->UpPoint.x, csp->UpPoint.y, csp->UpPoint.z - up_contact_height - up_connection_height);
				if (csp->GenerateValid)
				{
					std::vector<int> facet_indexs_up = pObj->get_Pointf3_gridfactes(p_up, 5);
					if (facet_indexs_up.empty() == false)
					{
						double dis_up = pObj->get_min_distance_point_to_facets(facet_indexs_up, p_up);

						LOGINFO("facet_indexs_up.size = [%d],get_min_distance_point_to_facets dis_up = [%f], down_contact_width = [%f]", facet_indexs_up.size(), dis_up, down_contact_width * down_contact_width);
						if (dis_up < down_contact_width * down_contact_width)
						{
							LOGINFO("delete a top support!");

							csp->GenerateValid = false;
							// pillar.pillar_nodes[csp->z_num].support_num--;
						}
					}
				}

				Pointf3 p_down(csp->DwPoint.x, csp->DwPoint.y, csp->DwPoint.z + up_contact_height + up_connection_height);
				if (csp->GenerateValid)
				{
					std::vector<int> facet_indexs_down = pObj->get_Pointf3_gridfactes(p_down, 5);
					if (facet_indexs_down.empty() == false)
					{
						double dis_down = pObj->get_min_distance_point_to_facets(facet_indexs_down, p_down);
						LOGINFO("facet_indexs_down.size = [%d],get_min_distance_point_to_facets dis_down = [%f], down_contact_width = [%f]", facet_indexs_down.size(), dis_down, down_contact_width * down_contact_width);
						if (dis_down < down_contact_width * down_contact_width)
						{
							LOGINFO("delete a top support!");
							csp->GenerateValid = false;
							// pillar.pillar_nodes[csp->z_num].support_num--;
						}
					}
				}

				Pointf3 p_center((p_up.x + p_down.x) / 2.0,
								 (p_up.y + p_down.y) / 2.0,
								 (p_up.z + p_down.z) / 2.0);
				if (csp->GenerateValid && p_up.z > p_down.z)
				{
					std::vector<int> facet_indexs_center = pObj->get_Pointf3_gridfactes(p_center, 5);
					if (facet_indexs_center.empty() == false)
					{
						double dis_center = pObj->get_min_distance_point_to_facets(facet_indexs_center, p_center);
						LOGINFO("facet_indexs_center.size = [%d],get_min_distance_point_to_facets dis_center = [%f], down_contact_width = [%f]", facet_indexs_center.size(), dis_center, down_contact_width * down_contact_width);
						if (dis_center < down_contact_width * down_contact_width)
						{
							LOGINFO("delete a top support!");
							csp->GenerateValid = false;
							// pillar.pillar_nodes[csp->z_num].support_num--;
						}
					}
				}
			}
			else
			{
				// 检查下端点
				Pointf3 p_down(csp->DwPoint.x, csp->DwPoint.y, csp->DwPoint.z);
				if (csp->GenerateValid)
				{
					std::vector<int> facet_indexs_down = pObj->get_Pointf3_gridfactes(p_down, 5);
					if (facet_indexs_down.empty() == false)
					{
						double dis_down = pObj->get_min_distance_point_to_facets(facet_indexs_down, p_down);

						LOGINFO("z != -1, facet_indexs_down.size = [%d],get_min_distance_point_to_facets dis_down = [%f], down_contact_width = [%f]", facet_indexs_down.size(), dis_down, down_contact_width * down_contact_width);
						if (dis_down < down_contact_width * down_contact_width)
						{
							LOGINFO("delete a top support!");
							csp->GenerateValid = false;
							pillar.pillar_nodes[csp->z_num].support_num--;
							csp->set_pillar_position(-1);
						}
					}
				}
			}
			// LOGINFO("UpPoint = [%s], DwPoint = [%s]", csp->UpPoint.dump_perl().c_str(), csp->DwPoint.dump_perl().c_str());
		}

		// LOGINFO("generate_one_support_pillar_nodes times =%d", GetTickCount() - timecount);

		return pillar.pillar_nodes[k - 1].node_type;
	}

	void LatticeSupportMesh::check_all_support_length()
	{
		for (coord_t i = 0; i < x_num_max; i++)
		{
			for (coord_t j = 0; j < y_num_max; j++)
			{
				Pillar &pillar = pillars[i][j];
				if (pillar.is_activated == false || pillar.support_points.empty() || pillar.is_valid() == false)
					continue;

				for (LatticeSPT_Pointf3s::iterator csp = pillar.support_points.begin(); csp != pillar.support_points.end(); csp++)
				{
					coord_t x = csp->x_num;
					coord_t y = csp->y_num;

					csp->set_parameter(*this);
					csp->check_support_length(*this);

					if (csp->x_num != x || csp->y_num != y)
					{
						this->pillars[csp->x_num][csp->y_num].support_points.push_back(*csp);
						pillar.support_points.erase(csp);
						csp--;
					}
				}
			}
		}
	}

	// 对所有的有支撑点的台柱计算节点，返回节点数多的那一种节点类型
	PillarNode_Type LatticeSupportMesh::generate_all_support_pillar_nodes()
	{
		LOGINFO("start generate_all_support_pillar_nodes");
		DWORD timecount = GetTickCount();

		coord_t odd = 0;
		coord_t even = 0;

		for (coord_t i = 0; i < x_num_max; i++)
		{
			for (coord_t j = 0; j < y_num_max; j++)
			{
				Pillar &pillar = pillars[i][j];
				if (/*pillar.is_activated == false || */ pillar.support_points.empty())
					continue;

				PillarNode_Type type = this->generate_one_support_pillar_nodes(pillar);
				if (type == Odd)
					odd++;
				else if (type == Even)
					even++;
			}
		}

		LOGINFO("generate_all_support_pillar_nodes times =%d", GetTickCount() - timecount);

		if (odd >= even)
			return Odd;
		else
			return Even;
	}

	bool LatticeSupportMesh::generate_all_support_pillar_nodes_bythreads(unsigned int _threads)
	{
		return true;
	}

	bool LatticeSupportMesh::extand_pillars()
	{
		LOGINFO("start extand_pillars");
		for (coord_t i = 0; i < x_num_max; i++)
		{
			for (coord_t j = 0; j < y_num_max; j++)
			{
				Pillar &center_pillar = pillars[i][j];

				if (center_pillar.support_points.size() == 0)
					continue;

				extand_a_pillar(center_pillar);
			}
		}
		LOGINFO("end extand_pillars");

		return true;
	}

	bool LatticeSupportMesh::extand_a_pillar(Pillar &center_pillar)
	{
		coord_t sx = center_pillar.x_Num;
		coord_t sy = center_pillar.y_Num;
		coord_t sz = center_pillar.get_top_node_id();

		if (extand_factor == 0)
			return true;

		// 支撑点台柱周围延伸激活
		for (coord_t i = -extand_factor; i <= extand_factor; i++)
		{
			if (sx + i < 0 || sx + i > x_num_max - 1)
				continue;

			for (coord_t j = -extand_factor; j <= extand_factor; j++)
			{
				if (i == 0 && j == 0)
					continue;
				if (i * j != 0)
					continue;
				if (sy + j < 0 || sy + j > y_num_max - 1)
					continue;

				// 检查点是否落在投影范围内
				Pointf pp = get_pillar_position(sx + i, sy + j);
				if (_is_in_thumbnails(Point(scale_(pp.x), scale_(pp.y))) == false ||
					_is_in_thumbnails(Point(scale_(pp.x - pillar_width / 2), scale_(pp.y))) == false ||
					_is_in_thumbnails(Point(scale_(pp.x + pillar_width / 2), scale_(pp.y))) == false ||
					_is_in_thumbnails(Point(scale_(pp.x), scale_(pp.y - pillar_width / 2))) == false ||
					_is_in_thumbnails(Point(scale_(pp.x), scale_(pp.y + pillar_width / 2))) == false)
				{
					// LOGINFO("extand_a_pillar, find a Point is not in_thumbnails!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
					continue;
				}

				Pillar &new_active_pillar = pillars[sx + i][sy + j];
				// if (new_active_pillar.is_activated)//如果已经被激活，则跳过
				//	continue;

				// 激活新的台柱并设置节点
				coord_t z = sz - abs(i) - abs(j);

				// 台柱向上求交，确定最高点
				double up_height = lattice_min_distance; // 顶端支撑的预留高度
				double pillar_height = 0.0;
				// 计算碰撞的三角面片
				Pointf p = this->get_pillar_position(sx + i, sy + j);
				std::vector<int> facet_ids = pObj->GetFacetIdxByZaxisFast(p.x, p.y);
				if (facet_ids.size() != 0)
				{
					// 记录碰撞信息
					new_active_pillar.CollsionZ_map.clear();
					for (int i = 0; i < facet_ids.size(); i++)
					{
						double collsion_z = pObj->GetPointZByZaxis(facet_ids[i], p.x, p.y);
						new_active_pillar.CollsionZ_map[collsion_z] = facet_ids[i];
					}

					// 射线求交的第二高点
					double maxZ = new_active_pillar.get_second_maxcollsionZ();
					pillar_height = maxZ - up_height;
					int z1 = floor(pillar_height / lattice_height);
					if (z1 < z)
						z = z1;
				}

				// 检查该台柱的区域
				// 固位网区域检查
				stl_face_type _type = new_active_pillar.face_type;
				if (this->avoid_reten_area && (_type == RetentionMesh_Part || _type == Fulcrum_Part || _type == Nail_Part))
				{
					LOGINFO("Find a support point which cannot connnect to a lattice, RetentionMesh_Part! _type = [%d]", _type);
					continue;
				}

				// 花纹区域检查
				if (this->avoid_tuqi_area && _type == StippledWax_Part)
				{
					LOGINFO("Find a support point which cannot connnect to a lattice, StippledWax_Part! _type = [%d]", _type);
					continue;
				}

				if (z > 0)
				{
					new_active_pillar.is_activated = true;

					for (coord_t k = 0; k <= z; k++)
						new_active_pillar.pillar_nodes[k].set_grid_position(sx + i, sy + j, k);
				}
			}
		}

		return true;
	}

	bool LatticeSupportMesh::check_pillars()
	{
		LOGINFO("start check_pillars!");
		DWORD timecount = GetTickCount();
		int itr_num = 0;
		bool changed = true;
		while (changed)
		{
			itr_num++;
			changed = false;
			for (int i = 0; i < x_num_max; i++)
			{
				for (int j = 0; j < y_num_max; j++)
				{
					Pillar &pillar = this->pillars[i][j];
					if (pillar.is_valid() == false || pillar.is_activated == false)
						continue;

					coord_t k = pillar.get_top_node_id();
					// LOGINFO("k = [%d], pillar.size() = [%d]", k, pillar.pillar_nodes.size());
					if (k < 0)
					{
						continue;
					}
					PillarNode &node = pillar.pillar_nodes[k];
					// LOGINFO("node.x_num = [%d], node.y_num = [%d],node.z_num = [%d],node.support_num = [%d], node.up_connect_num = [%d], node.down_connect_num = [%d]", node.x_Num, node.y_Num, node.z_Num, node.support_num, node.up_connect_num, node.down_connect_num);
					if (node.support_num <= 0 && node.up_connect_num <= 0) // 最上面的台柱向上没有任何连接
					{
						node.connect_with_next = false;
						node.z_Num = -1;

						if (node.pillar_node_0_ptr != nullptr)
						{
							node.pillar_node_0_ptr->up_connect_num--;
							node.pillar_node_0_ptr = nullptr;
						}
						if (node.pillar_node_1_ptr != nullptr)
						{
							node.pillar_node_1_ptr->up_connect_num--;
							node.pillar_node_1_ptr = nullptr;
						}
						if (node.pillar_node_2_ptr != nullptr)
						{
							node.pillar_node_2_ptr->up_connect_num--;
							node.pillar_node_2_ptr = nullptr;
						}
						if (node.pillar_node_3_ptr != nullptr)
						{
							node.pillar_node_3_ptr->up_connect_num--;
							node.pillar_node_3_ptr = nullptr;
						}

						node.down_connect_num = 0;
						if (k > 0)
							pillar.pillar_nodes[k - 1].up_connect_num--;

						LOGINFO("111111111111,find a node, node.support_num == 0 && node.up_connect_num == 0!!!");
						changed = true;
					}
				}
			}
		}

		LOGINFO("itr_num = %d, check_pillars times =%d", itr_num, GetTickCount() - timecount);

		int kkk = 0;
		if (true)
		{
			LOGINFO("start pillar_simplify");
			int ttt = 0;
			for (int i = 0; i < x_num_max; i++)
			{
				for (int j = 0; j < y_num_max; j++)
				{
					Pillar &pillar = this->pillars[i][j];
					if (pillar.is_valid() == false || pillar.is_activated == false)
						continue;

					coord_t node_num = pillar.get_top_node_id();
					ttt += node_num;

					bool IsSimplify = false;
					if (pillar.face_type == StippledWax_Part)
					{
						IsSimplify = pillar_simplify_tuqi;
					}
					else if (pillar.face_type == RetentionMesh_Part || pillar.face_type == Nail_Part || pillar.face_type == Fulcrum_Part)
					{
						IsSimplify = pillar_simplify_reten;
					}
					else
					{
						IsSimplify = pillar_simplify;
					}

					if (IsSimplify)
					{
						for (int k = node_num; k >= 0; k--)
						{
							if (node_num - k < 2) // 最上面两层不减
								continue;

							PillarNode &node = pillar.pillar_nodes[k];
							if (node.connect_with_next == false)
								continue;

							if (node.down_connect_num >= 4)
							{
								// LOGINFO("find a lattice pillar node can be simplify!!!!!!!!");
								node.connect_with_next = false;
								node.down_connect_num--;
								pillar.pillar_nodes[k - 1].up_connect_num--;

								kkk++;

								if (k > 0)
								{
									PillarNode &node_down = pillar.pillar_nodes[k - 1];
									if (node_down.support_num <= 0 && node_down.up_connect_num <= 0)
									{
										node_down.connect_with_next = false;
										node_down.down_connect_num--;
										if (k > 1)
											pillar.pillar_nodes[k - 2].up_connect_num--;
									}
								}
								if (k > 1)
								{
									PillarNode &node_down_down = pillar.pillar_nodes[k - 2];
									if (node_down_down.support_num <= 0 && node_down_down.up_connect_num <= 0)
									{
										node_down_down.connect_with_next = false;
										node_down_down.down_connect_num--;
										if (k > 2)
											pillar.pillar_nodes[k - 3].up_connect_num--;
									}
								}
							}
						}
					}
				}
			}

			LOGINFO("end pillar_simplify, node num = [%d] simplify num = [%d], ", ttt, kkk);
		}

		if (kkk > 0)
		{
			timecount = GetTickCount();
			itr_num = 0;
			changed = true;
			while (changed)
			{
				itr_num++;
				changed = false;
				for (int i = 0; i < x_num_max; i++)
				{
					for (int j = 0; j < y_num_max; j++)
					{
						Pillar &pillar = this->pillars[i][j];
						if (pillar.is_valid() == false || pillar.is_activated == false)
							continue;

						coord_t k = pillar.get_top_node_id();
						// LOGINFO("k = [%d], pillar.size() = [%d]", k, pillar.pillar_nodes.size());
						if (k < 0)
						{
							continue;
						}
						PillarNode &node = pillar.pillar_nodes[k];
						// LOGINFO("node.x_num = [%d], node.y_num = [%d],node.z_num = [%d],node.support_num = [%d], node.up_connect_num = [%d], node.down_connect_num = [%d]", node.x_Num, node.y_Num, node.z_Num, node.support_num, node.up_connect_num, node.down_connect_num);
						if (node.support_num <= 0 && node.up_connect_num <= 0) // 最上面的台柱向上没有任何连接
						{
							node.connect_with_next = false;
							node.z_Num = -1;

							if (node.pillar_node_0_ptr != nullptr)
							{
								node.pillar_node_0_ptr->up_connect_num--;
								node.pillar_node_0_ptr = nullptr;
							}
							if (node.pillar_node_1_ptr != nullptr)
							{
								node.pillar_node_1_ptr->up_connect_num--;
								node.pillar_node_1_ptr = nullptr;
							}
							if (node.pillar_node_2_ptr != nullptr)
							{
								node.pillar_node_2_ptr->up_connect_num--;
								node.pillar_node_2_ptr = nullptr;
							}
							if (node.pillar_node_3_ptr != nullptr)
							{
								node.pillar_node_3_ptr->up_connect_num--;
								node.pillar_node_3_ptr = nullptr;
							}

							node.down_connect_num = 0;
							if (k > 0)
								pillar.pillar_nodes[k - 1].up_connect_num--;

							LOGINFO("111111111111,find a node, node.support_num == 0 && node.up_connect_num == 0!!!");
							changed = true;
						}
					}
				}
			}
			LOGINFO("itr_num = %d, check_pillars times =%d", itr_num, GetTickCount() - timecount);
		}

		return true;
	}

	bool LatticeSupportMesh::init_mesh()
	{
		// LOGINFO("start init_mesh!");

		Vector3 z(0, 0, 1);
		Eigen::Vector3d v_to(0, 0, 1);
		LOGINFO("lattice_height = [%f], lattice_width = [%f]", lattice_height, lattice_width);
		Pointf3 p(0.0, 0.0, lattice_height);
		Pointf3 p0(-lattice_width, 0.0, 0.0);
		Pointf3 p1(lattice_width, 0.0, 0.0);
		Pointf3 p2(0.0, -lattice_width, 0.0);
		Pointf3 p3(0.0, lattice_width, 0.0);

		Pointf3 pp0 = p - p0;
		Pointf3 pp1 = p - p1;
		Pointf3 pp2 = p - p2;
		Pointf3 pp3 = p - p3;

		Vector3 vn0(pp0.x, pp0.y, pp0.z);
		Vector3 vn1(pp1.x, pp1.y, pp1.z);
		Vector3 vn2(pp2.x, pp2.y, pp2.z);
		Vector3 vn3(pp3.x, pp3.y, pp3.z);

		vn0.normalize();
		vn1.normalize();
		vn2.normalize();
		vn3.normalize();

		Eigen::Vector3d v_from0(vn0.x, vn0.y, vn0.z);
		Eigen::Vector3d v_from1(vn1.x, vn1.y, vn1.z);
		Eigen::Vector3d v_from2(vn2.x, vn2.y, vn2.z);
		Eigen::Vector3d v_from3(vn3.x, vn3.y, vn3.z);

		rotation_matrix0 = Eigen::Quaterniond::FromTwoVectors(v_from0, v_to).toRotationMatrix();
		rotation_matrix1 = Eigen::Quaterniond::FromTwoVectors(v_from1, v_to).toRotationMatrix();
		rotation_matrix2 = Eigen::Quaterniond::FromTwoVectors(v_from2, v_to).toRotationMatrix();
		rotation_matrix3 = Eigen::Quaterniond::FromTwoVectors(v_from3, v_to).toRotationMatrix();

		std::ostringstream debug_str;
		debug_str << "rotation_matrix0:" << rotation_matrix0 << std::endl;
		debug_str << "rotation_matrix1:" << rotation_matrix1 << std::endl;
		debug_str << "rotation_matrix2:" << rotation_matrix2 << std::endl;
		debug_str << "rotation_matrix3:" << rotation_matrix3 << std::endl;
		LOGINFO(debug_str.str().c_str());

		// LOGINFO("start init_mesh0!");
		mesh0 = TriangleMesh(pObj->rotate(rotation_matrix0));
		mesh0.set_step_length(1, 1, 1);
		mesh0.init_hash_facets();

		// mesh0.write_binary("mesh0.stl");

		// LOGINFO("start init_mesh1!");
		mesh1 = TriangleMesh(pObj->rotate(rotation_matrix1));
		mesh1.set_step_length(1, 1, 1);
		mesh1.init_hash_facets();

		// mesh1.write_binary("mesh1.stl");

		// LOGINFO("start init_mesh2!");
		mesh2 = TriangleMesh(pObj->rotate(rotation_matrix2));
		mesh2.set_step_length(1, 1, 1);
		mesh2.init_hash_facets();

		// mesh2.write_binary("mesh2.stl");

		// LOGINFO("start init_mesh3!");
		mesh3 = TriangleMesh(pObj->rotate(rotation_matrix3));
		mesh3.set_step_length(1, 1, 1);
		mesh3.init_hash_facets();

		// mesh3.write_binary("mesh3.stl");

		// LOGINFO("end init_mesh!");
		return true;
	}

	Pointf3 LatticeSupportMesh::rotate_pointf3(Eigen::Matrix3d rotation_matrix, Pointf3 p)
	{
		Eigen::Vector3d p1(p.x, p.y, p.z);
		Eigen::Vector3d p2 = rotation_matrix * p1;

		return Pointf3(p2(0), p2(1), p2(2));
	}

	bool LatticeSupportMesh::generate_pillars_topology(PillarNode_Type type)
	{
		LOGINFO("start generate_pillars_topology!");

		DWORD timecount = GetTickCount();
		// TriangleMesh mesh;

		// 遍历台柱的所有节点
		for (coord_t i = 0; i < x_num_max; i++)
		{
			for (coord_t j = 0; j < y_num_max; j++)
			{
				Pillar &pillar = pillars[i][j];
				if (pillar.is_activated == false)
					continue;

				coord_t node_num = pillar.get_top_node_id();
				coord_t tt = node_num % 2;
				for (coord_t k = node_num; k >= 0; k--)
				{
					PillarNode &node = pillar.pillar_nodes[k];

					// 向下连接
					if (k == 0)
						node.connect_with_next = false;

					if (k > 0 && node.connect_with_next == true)
					{
						node.down_connect_num++;
						Pointf3 pout(0, 0, 0);
						PillarNode &node_down = pillar.pillar_nodes[k - 1];
						// 干涉检查
						if (is_intersect(node, node_down, -1, node.intersect_point) == true)
						{
							LOGINFO("find a intersect vertical lattice");
							LOGINFO("node, x_num=[%d], y_num=[%d], z_num=[%d]", node.x_Num, node.y_Num, node.z_Num);
							LOGINFO("node_next, x_num=[%d], y_num=[%d], z_num=[%d]", node_down.x_Num, node_down.y_Num, node_down.z_Num);
							// node.connect_with_next = false;
							node.is_intersect = true;

							if (k < node_num - 2)
							{
								if (node.node_type == type)
								{
									if (node_down.support_num <= 0 && node_down.up_connect_num <= 0)
										node_down.connect_with_next == false;
								}
								// else
								//{
								//	PillarNode& node_up = pillar.pillar_nodes[k + 1];
								//	node_up.connect_with_next = false;
								// }
							}
						}
						else
						{
							node_down.up_connect_num++;
						}
					}

					if (k != node_num && k != 0 && node.connect_with_next == false)
					{
						LOGINFO("k != node_num && k != 0 && node.connect_with_next == false");
						PillarNode &node_up = pillar.pillar_nodes[k + 1];
						if (node_up.connect_with_next == false)
						{
							continue;
						}
					}

					coord_t k1 = -1;
					if (node.node_type == type) // 多点，向四个方向连接
					{
						k1 = k - 1;
						if (bottom_strengthen && k1 < 0)
							k1 = 0;
					}
					else // 少点
					{
						if (/*pillar.is_support_point == true && */ node_num - k < 2)
							k1 = k - 1;
					}

					stl_face_type _type = pillar.face_type;

					if (_type == StippledWax_Part)
					{
						if (tuqi_area_pillar_connect == false)
							continue;

						if (pillar_connect_jump_tuqi && k % 2 != tt)
							continue;

						if (k * lattice_height < min_pillar_connect_height_tuqi)
							continue;
					}
					else if (_type == RetentionMesh_Part || _type == Fulcrum_Part || _type == Nail_Part)
					{
						if (reten_area_pillar_connect == false)
							continue;

						if (pillar_connect_jump_reten && k % 2 != tt)
							continue;

						if (k * lattice_height < min_pillar_connect_height_reten)
							continue;
					}
					else
					{
						if (k * lattice_height < min_pillar_connect_height)
							continue;

						if (pillar_connect_jump && k % 2 != tt)
							continue;
					}

					if (k1 >= 0)
					{
						if (i > 0) // 与（x_Num - 1，y_Num）台柱的链接节点
						{
							Pillar &p0 = pillars[i - 1][j];

							stl_face_type _type0 = p0.face_type;
							bool check0 = true;
							if (tuqi_area_pillar_connect == false && _type0 == StippledWax_Part)
								check0 = false;

							if (reten_area_pillar_connect == false && (_type0 == RetentionMesh_Part || _type0 == Fulcrum_Part || _type0 == Nail_Part))
								check0 = false;

							if (p0.x_Num != i - 1)
								LOGINFO("p0.x_Num != i - 1!!! p0.x_Num = [%d], i - 1 = [%d]", p0.x_Num, i - 1);
							if (p0.y_Num != j)
								LOGINFO("p0.y_Num != j!!! p0.y_Num = [%d], j = [%d]", p0.y_Num, j);

							if (check0 && p0.is_activated && p0.is_valid() && p0.pillar_nodes.size() > k1)
							{
								// 干涉检查
								Pointf3 pout(0, 0, 0);
								if (p0.pillar_nodes[k1].is_valid())
								{
									if ((k == 0 && k1 == 0) || is_intersect(node, p0.pillar_nodes[k1], 0, pout) == false)
									{
										node.pillar_node_0_ptr = &p0.pillar_nodes[k1];
										node.down_connect_num++;
										p0.pillar_nodes[k1].up_connect_num++;
									}
									else
									{
									}
								}
							}
						}

						if (i < x_num_max - 1) // 与（x_Num + 1，y_Num）台柱的链接节点
						{
							Pillar &p1 = pillars[i + 1][j];

							stl_face_type _type1 = p1.face_type;
							bool check1 = true;
							if (tuqi_area_pillar_connect == false && _type1 == StippledWax_Part)
								check1 = false;

							if (reten_area_pillar_connect == false && (_type1 == RetentionMesh_Part || _type1 == Fulcrum_Part || _type1 == Nail_Part))
								check1 = false;

							if (p1.x_Num != i + 1)
								LOGINFO("p0.x_Num != i + 1!!! p0.x_Num = [%d], i + 1 = [%d]", p1.x_Num, i + 1);
							if (p1.y_Num != j)
								LOGINFO("p0.y_Num != j!!! p0.y_Num = [%d], j = [%d]", p1.y_Num, j);

							if (check1 && p1.is_activated && p1.is_valid() && p1.pillar_nodes.size() > k1)
							{
								Pointf3 pout(0, 0, 0);
								if (p1.pillar_nodes[k1].is_valid())
								{
									if ((k == 0 && k1 == 0) || is_intersect(node, p1.pillar_nodes[k1], 1, pout) == false)
									{
										node.pillar_node_1_ptr = &p1.pillar_nodes[k1];
										node.down_connect_num++;
										p1.pillar_nodes[k1].up_connect_num++;
									}
									else
									{
									}
								}
							}
						}

						if (j > 0) // 与（x_Num，y_Num - 1）台柱的链接节点
						{
							Pillar &p2 = pillars[i][j - 1];

							stl_face_type _type2 = p2.face_type;
							bool check2 = true;
							if (tuqi_area_pillar_connect == false && _type2 == StippledWax_Part)
								check2 = false;

							if (reten_area_pillar_connect == false && (_type2 == RetentionMesh_Part || _type2 == Fulcrum_Part || _type2 == Nail_Part))
								check2 = false;

							if (p2.x_Num != i)
								LOGINFO("p0.x_Num != i!!! p0.x_Num = [%d], i = [%d]", p2.x_Num, i);
							if (p2.y_Num != j - 1)
								LOGINFO("p0.y_Num != j - 1!!! p0.y_Num = [%d], j - 1 = [%d]", p2.y_Num, j - 1);

							if (check2 && p2.is_activated && p2.is_valid() && p2.pillar_nodes.size() > k1)
							{
								Pointf3 pout(0, 0, 0);
								if (p2.pillar_nodes[k1].is_valid())
								{
									if ((k == 0 && k1 == 0) || is_intersect(node, p2.pillar_nodes[k1], 2, pout) == false)
									{
										node.pillar_node_2_ptr = &p2.pillar_nodes[k1];
										node.down_connect_num++;
										p2.pillar_nodes[k1].up_connect_num++;
									}
									else
									{
									}
								}
							}
						}

						if (j < y_num_max - 1) // 与（x_Num，y_Num + 1）台柱的链接节点
						{
							Pillar &p3 = pillars[i][j + 1];

							stl_face_type _type3 = p3.face_type;
							bool check3 = true;
							if (tuqi_area_pillar_connect == false && _type3 == StippledWax_Part)
								check3 = false;

							if (reten_area_pillar_connect == false && (_type3 == RetentionMesh_Part || _type3 == Fulcrum_Part || _type3 == Nail_Part))
								check3 = false;

							if (p3.x_Num != i)
								LOGINFO("p0.x_Num != i!!! p0.x_Num = [%d], i = [%d]", p3.x_Num, i);
							if (p3.y_Num != j + 1)
								LOGINFO("p0.y_Num != j + 1!!! p0.y_Num = [%d], j + 1 = [%d]", p3.y_Num, j + 1);

							if (check3 && p3.is_activated && p3.is_valid() && p3.pillar_nodes.size() > k1)
							{
								Pointf3 pout(0, 0, 0);
								if (p3.pillar_nodes[k1].is_valid())
								{
									if ((k == 0 && k1 == 0) || is_intersect(node, p3.pillar_nodes[k1], 3, pout) == false)
									{
										node.pillar_node_3_ptr = &p3.pillar_nodes[k1];
										node.down_connect_num++;
										p3.pillar_nodes[k1].up_connect_num++;
									}
									else
									{
									}
								}
							}
						}
					}
				}
			}
		}

		LOGINFO("generate_pillars_topology times =%d", GetTickCount() - timecount);
		// mesh.write_binary("is_intersect.stl");
		return true;
	}

	void PillarNode::set_parameter(const LatticeSupportMesh &latticeSupportMesh)
	{
		this->node_width = std::sqrt(
							   (std::pow(latticeSupportMesh.lattice_width, 2) + std::pow(latticeSupportMesh.lattice_height, 2)) / std::pow(latticeSupportMesh.lattice_height, 2)) *
						   latticeSupportMesh.pillar_width;
		if (latticeSupportMesh.bottom_height <= DBL_MIN)
		{
			this->bottom_height = latticeSupportMesh.pillar_width;
		}
		else
		{
			this->bottom_height = latticeSupportMesh.bottom_height;
		}
		this->is_connect_bottom = latticeSupportMesh.bottom_strengthen;
	}

	void PillarNode::set_grid_position(coord_t x, coord_t y, coord_t z)
	{
		// LOGINFO("start set_grid_position, x = [%d], y = [%d], z = [%d]", x, y, z);
		x_Num = x;
		y_Num = y;
		z_Num = z;

		if ((x + y + z) % 2 == 0)
			node_type = Even;
		else
			node_type = Odd;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////
	// LatticeSPT_PointsGenerate
	/////////////////////////////////////////////////////////////////////////////////////////////////

	std::vector<LatticeSPT_Pointf3> LatticeSPT_PointsGenerate::GetSPT_PointsVec()
	{
		LOGINFO("---------LatticeSPT_PointsGenerate::GetSPT_PointsVec() begin----------");
		std::vector<LatticeSPT_Pointf3> _ret;
		for (size_t i = 0; i < slice_Layers.size(); i++)
		{
			double Z_val = z_Vec[slice_Layers[i].Layer_ID];
			for (size_t j = 0; j < slice_Layers[i].spt_points.size(); j++)
			{
				Pointf spt_point = slice_Layers[i].spt_points[j];
				LatticeSPT_Pointf3 _Lspt(0.0, 0.5, 0.2, 1.0);
				_Lspt.UpPoint = Pointf3(spt_point.x, spt_point.y, Z_val);
				_Lspt.DwPoint = Pointf3(spt_point.x, spt_point.y, 0.0);
				_Lspt.x_num = 0;
				_Lspt.y_num = 0;
				_Lspt.Form_FaceID = -1; // 待定
				_Lspt.spt_type = LatticeSPT_Type::LSPT_Other;
				_Lspt.Weight = 1;
				_ret.push_back(_Lspt);
			}
		}

		LOGINFO("_ret.size() =[%d]", _ret.size());

		LOGINFO("---------LatticeSPT_PointsGenerate::GetSPT_PointsVec() end----------");

		return _ret;
	}

	bool LatticeSPT_PointsGenerate::GetSPT_zVec()
	{
		z_Vec.clear();
		if (OBJ_copy.facets_count() == 0)
		{
			LOGINFO("OBJ_copy.facets_count() == 0");
			return false;
		}
		if (OBJ_copy.UnableRepair())
		{
			LOGINFO("OBJ_copy.UnableRepair() == false");
			return false;
		}
		if (Slice_Z < 0.0001)
		{
			LOGINFO("Slice_Z < 0.0001");
			return false;
		}

		for (double layer_z = obj_min_z + Slice_Z / 2; layer_z <= obj_max_z; layer_z += Slice_Z)
		{
			z_Vec.push_back(layer_z);
		}
		// LOGINFO("obj_min_z[%f] obj_max_z[%f] Slice_Z[%f] z_Vec.size() = %d",
		//	obj_min_z, obj_max_z, Slice_Z, z_Vec.size());
		return true;
	}

	float LatticeSPT_PointsGenerate::GetSPT_zVal(size_t Layer_ID)
	{
		return z_Vec[Layer_ID];
	}

	void LatticeSPT_PointsGenerate::_Make_SliceLayer(size_t i, std::vector<ExPolygons> *p_exs)
	{
		// LOGINFO("this->slice_Layers[%d / %d] _Make_SliceLayer", i, this->slice_Layers.size());

		this->slice_Layers[i].Layer_ID = i;
		Polygons sp_pp;
		sp_pp.clear();
		ExPolygons this_exs = p_exs->at(i);

		for (int j = 0; j < this_exs.size(); j++)
		{
			this_exs[j].simplify_p(SCALED_RESOLUTION, &sp_pp);
		}
		this->slice_Layers[i].Obj_Slices = union_ex(sp_pp);
	}

	void LatticeSPT_PointsGenerate::_preMake_SuspendAreas(size_t i, coord_t offset_slice)
	{
		// LOGINFO("this->slice_Layers[%d / %d] _preMake_SuspendAreas", i, this->slice_Layers.size());
		ExPolygons obj_areas = this->slice_Layers[i].Obj_Slices;
		ExPolygons sub_obj_areas = this->GetDownObjArea(i, 1);
		// 悬垂偏移
		sub_obj_areas = offset_ex(sub_obj_areas, offset_slice);
		this->slice_Layers[i].Suspend_Areas = diff_ex(obj_areas, sub_obj_areas, true);
		// LOGINFO("this->slice_Layers[%d] Suspend_Areas.size() = %d", i, this->slice_Layers[i].Suspend_Areas.size());
		//  测试代码
		if (false)
		{
			char _name[255];
			sprintf(_name, "Suspend_Areas[%d].svg", i);
			SVG _svg(_name);
			_svg.draw(this->slice_Layers[i].Obj_Slices, "blue");
			_svg.draw(this->slice_Layers[i].Suspend_Areas, "red");
			_svg.Close();
		}
	}

	// 对物体进行切片
	size_t LatticeSPT_PointsGenerate::Slice_Obj()
	{
		if (this->GetSPT_zVec() == false)
		{
			LOGINFO("GetSPT_zVec == false");
			return 0;
		}
		//
		boost::thread_group *p_work_group = NULL;
		TriangleMesh mesh_copy(OBJ_copy);
		std::vector<ExPolygons> layers_ex;
		layers_ex.resize(z_Vec.size());
		TriangleMeshSlicer<Z> sliceMesh_p(&mesh_copy);
		sliceMesh_p.worker_pp = &p_work_group;
		sliceMesh_p.slice(z_Vec, &layers_ex, boost::thread::hardware_concurrency());
		LOGINFO("slice Finish layers_ex.size() == %d,  z_Vec.size() = %d", layers_ex.size(), z_Vec.size());
		if (layers_ex.size() == 0 || z_Vec.size() != layers_ex.size())
		{
			LOGINFO("slice Error", layers_ex.size(), z_Vec.size());
			return 0;
		}
		// 测试代码
		if (false)
		{
			for (int i = 0; i < layers_ex.size(); i++)
			{
				char _name[255];
				sprintf(_name, "LatticeSPT_PointsGenerate_Slice_Obj [%d].svg", i);
				SVG _svg(_name);
				ExPolygons exs = layers_ex[i];
				_svg.draw(exs, "blue");
				_svg.Close();
			}
		}

		//
		slice_Layers.clear();
		slice_Layers.resize(z_Vec.size());
		if (this->slice_Layers.size() == 0)
		{
			return 0;
		}

		//  多线程处理  切片
		LOGINFO("------------_Make_SliceLayer begin--------------");
		parallelize<size_t>(
			0,
			this->slice_Layers.size() - 1,
			boost::bind(&LatticeSPT_PointsGenerate::_Make_SliceLayer, this, _1, &layers_ex),
			p_work_group,
			boost::thread::hardware_concurrency());
		LOGINFO("-------------_Make_SliceLayer end-------------");
		LOGINFO("offset_slice = %f", Slice_Z / tan(spt_angle * PI / 180));
		coord_t offset_slice = scale_(Slice_Z / tan(spt_angle * PI / 180));
		//  多线程处理  计算悬垂面
		LOGINFO("------------_preMake_SuspendPoints end-------------");
		parallelize<size_t>(
			0,
			this->slice_Layers.size() - 1,
			boost::bind(&LatticeSPT_PointsGenerate::_preMake_SuspendAreas, this, _1, offset_slice),
			p_work_group,
			boost::thread::hardware_concurrency());
		LOGINFO("------------_preMake_SuspendPoints end-------------");
		// 多线程处理  计算支撑点
		LOGINFO("------------generate_oneLayer_spt_points end-------------");
		parallelize<size_t>(
			0,
			this->slice_Layers.size() - 1,
			boost::bind(&LatticeSPT_PointsGenerate::generate_oneLayer_spt_points, this, _1),
			p_work_group,
			boost::thread::hardware_concurrency());
		LOGINFO("------------generate_oneLayer_spt_points end-------------");

		// for (int i = 0; i < this->slice_Layers.size(); i++) {
		//	this->simply_oneLayer_spt_points(i);
		// }

		return slice_Layers.size();
	}

	ExPolygons LatticeSPT_PointsGenerate::GetDownObjArea(size_t this_layer_id, size_t down_layer_Num)
	{
		ExPolygons _ret;
		_ret.clear();
		if (down_layer_Num == 0 || this->slice_Layers.size() == 0)
		{
			LOGINFO("down_layer_Num == 0 || this->slice_Layers.size() == 0");
			return _ret;
		}

		for (size_t i = 1; i <= down_layer_Num; i++)
		{
			size_t down_id = this_layer_id - i;
			if (down_id < 0 || down_id >= this->slice_Layers.size())
				continue;
			ExPolygons down_layer_obj = this->slice_Layers[down_id].Obj_Slices;
			_ret.insert(_ret.end(), down_layer_obj.begin(), down_layer_obj.end());
		}
		if (down_layer_Num > 1 && _ret.size() > 0)
			_ret = union_ex(_ret);
		return _ret;
	}

	ExPolygons LatticeSPT_PointsGenerate::GetDownSptArea(size_t this_layer_id, size_t down_layer_Num)
	{
		ExPolygons _ret;
		_ret.clear();
		std::vector<Pointf> spt_points;
		spt_points.clear();
		if (down_layer_Num == 0 || this->slice_Layers.size() == 0)
		{
			LOGINFO("down_layer_Num == 0 || this->slice_Layers.size() == 0");
			return _ret;
		}
		for (size_t i = 1; i <= down_layer_Num; i++)
		{
			size_t down_id = this_layer_id - i;
			if (down_id < 0 || down_id >= this->slice_Layers.size())
				continue;
			std::vector<Pointf> down_spt_points = this->slice_Layers[down_id].spt_points;
			spt_points.insert(spt_points.end(), down_spt_points.begin(), down_spt_points.end());
		}
		LOGINFO("GetDownSptArea spt_points.size() = %d", spt_points.size());
		coord_t spt_scale_raduis = scale_(this->spt_radius);
		for (size_t i = 0; i < spt_points.size(); i++)
		{
			ExPolygon one_ex;
			one_ex.contour.points.push_back(Point(spt_scale_raduis / 2, spt_scale_raduis));
			one_ex.contour.points.push_back(Point(spt_scale_raduis, spt_scale_raduis / 2));
			one_ex.contour.points.push_back(Point(spt_scale_raduis, -spt_scale_raduis / 2));
			one_ex.contour.points.push_back(Point(spt_scale_raduis / 2, -spt_scale_raduis));
			one_ex.contour.points.push_back(Point(-spt_scale_raduis / 2, -spt_scale_raduis));
			one_ex.contour.points.push_back(Point(-spt_scale_raduis, -spt_scale_raduis / 2));
			one_ex.contour.points.push_back(Point(-spt_scale_raduis, spt_scale_raduis / 2));
			one_ex.contour.points.push_back(Point(-spt_scale_raduis / 2, spt_scale_raduis));
			one_ex.contour.translate(Point(scale_(spt_points[i].x), scale_(spt_points[i].y)));
			_ret.push_back(one_ex);
		}
		_ret = union_ex(_ret);
		return _ret;
	}

	void LatticeSPT_PointsGenerate::generate_oneLayer_spt_points(size_t Ly_ID)
	{
		if (Ly_ID >= this->slice_Layers.size())
		{
			return;
		}
		ExPolygons Sus_exs = this->slice_Layers[Ly_ID].Suspend_Areas;
		this->slice_Layers[Ly_ID].spt_points.clear();
		for (int i = 0; i < Sus_exs.size(); i++)
		{
			std::vector<Pointf> ps_ex = this->generate_spt_points(Sus_exs[i]);
			this->slice_Layers[Ly_ID].spt_points.insert(this->slice_Layers[Ly_ID].spt_points.end(), ps_ex.begin(), ps_ex.end());
		}
		LOGINFO("slice_Layers[%d] spt_points size = [%d]", Ly_ID, this->slice_Layers[Ly_ID].spt_points.size());
		return;
	}

	void LatticeSPT_PointsGenerate::simply_oneLayer_spt_points(size_t Ly_ID)
	{
		if (Ly_ID >= this->slice_Layers.size())
		{
			return;
		}
		std::vector<Pointf> layer_spts = this->slice_Layers[Ly_ID].spt_points;
		ExPolygons downlayers_sptexs = this->GetDownSptArea(Ly_ID, std::ceil(spt_radius / Slice_Z));
		for (std::vector<Pointf>::iterator pointf_It = layer_spts.begin(); pointf_It != layer_spts.end();)
		{
			bool IsDelete = false;
			Point spt_point = Point(scale_(pointf_It->x), scale_(pointf_It->y));
			for (int i = 0; i < downlayers_sptexs.size(); i++)
			{
				if (downlayers_sptexs[i].contains_b(spt_point))
				{
					IsDelete = true;
					break;
				}
			}
			if (IsDelete)
				pointf_It = layer_spts.erase(pointf_It);
			else
				pointf_It++;
		}
		// LOGINFO("slice_Layers[%d]  downlayers_sptexs size = [%d]  spt_points size = [%d], simply size = [%d]",
		// Ly_ID, downlayers_sptexs.size(), this->slice_Layers[Ly_ID].spt_points.size(), layer_spts.size());
		this->slice_Layers[Ly_ID].spt_points = layer_spts;

		// 测试代码
		if (false)
		{
			char _name[255];
			sprintf(_name, "downlayers_sptexs[%d].svg", Ly_ID);
			SVG _svg(_name);
			_svg.draw(downlayers_sptexs, "red");
			_svg.Close();
		}

		return;
	}

	std::vector<Pointf> LatticeSPT_PointsGenerate::generate_spt_points(ExPolygon suspend_area)
	{
		coord_t Radius = scale_(spt_radius / 10); // 以十分之一的间距生成支撑，再化简

		std::vector<Pointf> support_points;
		BoundingBox bb = suspend_area.contour.bounding_box();

		coord_t sx = floor((bb.max.x - bb.min.x) / Radius);
		coord_t sy = floor((bb.max.y - bb.min.y) / Radius);
		// LOGINFO("sx = %d, sy = %d", sx, sy);
		coord_t Radius_x = adjust_solid_spacing(bb.size().x, Radius);
		coord_t Radius_y = adjust_solid_spacing(bb.size().y, Radius);

		Point start_point = bb.min;

		for (int i = 0; i <= sx; i++)
		{
			for (int j = 0; j <= sy; j++)
			{
				Point p00(start_point.x + i * Radius_x, start_point.y + j * Radius_y);
				Point p01(start_point.x + i * Radius_x, start_point.y + (j + 1) * Radius_y);
				Point p10(start_point.x + (i + 1) * Radius_x, start_point.y + j * Radius_y);
				Point p11(start_point.x + (i + 1) * Radius_x, start_point.y + (j + 1) * Radius_y);

				// if (!suspend_area.contains(p00) && !suspend_area.contains(p01) &&
				//	!suspend_area.contains(p10) && !suspend_area.contains(p11))
				//	continue;

				Points pts;
				pts.push_back(p00);
				pts.push_back(p10);
				pts.push_back(p11);
				pts.push_back(p01);

				ExPolygon grid;
				grid.contour = Polygon(pts);

				ExPolygons intersects = intersection_ex(grid, suspend_area);
				if (!intersects.empty())
				{
					Point cp(start_point.x + (i + 0.5) * Radius_x, start_point.y + (j + 0.5) * Radius_y); // 网格中心点
					Point nearest = cp;
					double dis = INT_MAX;
					for (int k = 0; k < intersects.size(); k++)
					{
						Point ex = intersects.at(k).find_centroid_inside();
						double temp = ex.distance_to(cp);
						if (temp < dis)
						{
							dis = temp;
							nearest = ex;
						}
					}

					support_points.push_back(Pointf(unscale(nearest.x), unscale(nearest.y)));
				}
			}
		}

		return support_points;
	}

	coord_t LatticeSPT_PointsGenerate::adjust_solid_spacing(const coord_t width, const coord_t distance, const coordf_t factor_max)
	{
		assert(width >= 0);
		assert(distance > 0);
		const int number_of_intervals = floor(width / distance);
		if (number_of_intervals == 0)
			return distance;

		coord_t distance_new = (width / number_of_intervals);

		const coordf_t factor = coordf_t(distance_new) / coordf_t(distance);
		assert(factor > 1. - 1e-5);

		// How much could the extrusion width be increased? By 20%.
		// Because of this limit, this method is not idempotent: each run
		// will increment distance by 20%.
		if (factor > factor_max)
			distance_new = floor((double)distance * factor_max + 0.5);

		assert((distance_new * number_of_intervals) <= width);

		return distance_new;
	}

}