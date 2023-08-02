#include <Sheet.hpp>
#include <FaceInfo.hpp>
// #include <Settings.hpp>

using namespace std;

typedef std::pair<v2d, v2d> Segment;
typedef std::tuple<v2d, v2d, v2d> Module;
struct Chain {
	int start=-1, end=-1; // starting io, ending io and the leds that it contain
	vector<int> leds;
	void clear() {
		start = -1; end = -1; leds.clear();
	}
};


v3f v3d2f(v3d v) {
	return v3f(v[0],v[1], v[2]);
}
v3d v3f2d(v3f v) {
	return v3d(v[0],v[1], v[2]);
}

double lengthV(v3f v0, v3f v1) {
	return sqrt(dot(v0-v1, v0-v1));
}
double lengthV(v2f v0, v2f v1) {
	return sqrt(dot(v0-v1, v0-v1));
}
double lengthV(v2d v0, v2d v1) {
	return sqrt(dot(v0-v1, v0-v1));
}

v2d ortho(v2d d) {
	d = normalize(d); 
	return v2d(-d[1], d[0]); 
}

/* -------------------------------------------------------- */
pair<double,v2d> externalDistanceToTriangle(v2d q, v2d p0, v2d p1, v2d p2) { // todo merge this function and the one in vorlayout together in some helper
	double dist = 0.0;
	v2d    grad(0.0); // only stores the x and y component
	double d01 = dot(q - p0, ortho(p1 - p0));
	double d12 = dot(q - p1, ortho(p2 - p1));
	double d20 = dot(q - p2, ortho(p0 - p2));
	if (d01 < 0.0 || d12 < 0.0 || d20 < 0.0) { // the point is outside the triangle and thus demands to be accounted for
		float u01 = dot(q - p0, normalize(p1 - p0)) / length(p1 - p0);
		float u12 = dot(q - p1, normalize(p2 - p1)) / length(p2 - p1);
		float u20 = dot(q - p2, normalize(p0 - p2)) / length(p0 - p2);
		if (d01 < 0.0) {
			if (u01 > 0.0 && u01 < 1.0) {
				dist = -d01;
				grad = ortho(p1 - p0);
			} else if (u01 <= 0.0) {
				dist = length(q - p0);
				grad = normalize(p0 - q);
			} else {
				dist = length(q - p1);
				grad = normalize(p1 - q);
			}
		} else if (d12 < 0.0) {
			if (u12 > 0.0 && u12 < 1.0) {
				dist = -d12;
				grad = ortho(p2 - p1);
			} else if (u12 <= 0.0) {
				dist = length(q - p1);
				grad = normalize(p1 - q);
			} else {
				dist = length(q - p2);
				grad = normalize(p2 - q);
			}
		} else /* (d20 < 0.0) */ {
			if (u20 > 0.0 && u20 < 1.0) {
				dist = -d20;
				grad = ortho(p0 - p2);
			} else if (u20 <= 0.0) {
				dist = length(q - p2);
				grad = normalize(p2 - q);
			} else {
				dist = length(q - p0);
				grad = normalize(p0 - q);
			}
		}
		// printf("Gradient val: %f %f\n", grad[0], grad[1]);
	}
	return make_pair(dist,grad);
}

bool traceInModule(v2d pt, Module mod) {
	v2d m0 = get<0>(mod);
	v2d m1 = get<1>(mod);
	v2d m2 = get<2>(mod);
	v2d m3 = m0 + (m1-m0) + (m2-m0);
	double dist0 = externalDistanceToTriangle(pt, m0, m1, m2).first;
	double dist1 = externalDistanceToTriangle(pt, m2, m1, m3).first;

	if(dist0 <= 0 or dist1 <= 0) {
		return true;
	}
	else {
		return false;
	}
}

vector<int> removeAdjValues(vector<int> vec) {
    int len = vec.size();
	vector<int> rvec = vec;
	int pos = 0; 
	while(pos < len) {
		if (pos == len - 1)
			return rvec;
		if (rvec[pos] == rvec[pos + 1]) {
			rvec.erase(rvec.begin() + pos);
			len = len - 1; 
		}
		else
			pos++;
	}
	return rvec;
}

vector<int> rebaseGraph(vector<int> vec, int start_face) { // start_face is the face containing the connector
	size_t id = 0; 
	vector<int> ids; 
	for(id=0; id < vec.size(); id++) {
		if(vec[id] == start_face) {
			ids.push_back(id);
		}
	}
	id = ids[ids.size()-1]; // choose the last id as the one that makes the base of the graph
	vector<int> rvec; 
	for(size_t i=id; i < vec.size(); i++) {
		rvec.push_back(vec[i]);
	}
	for(size_t i = 1; i < id; i++) {
		rvec.push_back(vec[i]);
	}
	rvec.push_back(start_face);
	return rvec;
}

int get_index(vector<size_t> edges, size_t edge) {
	int index = 0;
	for(auto e: edges) {
		if( e == edge) {
			return index;
		}
		index++;
	}
	return -1; 
}

int main(int argc, char **argv) {
	map<int, int> leds_global_to_trace_order;
	std::string file_stg{};
	std::string file_sht{};
	std::string file_fis{};
	std::string file_out{};

	std::vector<std::vector<int>> led_chain_per_triangle;
	Sheet sh;


	if (argc == 1) {
		// file_sht = "../data/sqtorus-048_05_half.sheet";
		// file_fis = "../data/sqtorus-048_05_half_1515_2i.trc";
		// file_sht = "../data/cat-102_05_half.sheet";
		// file_fis = "../data/cat-102_05_half_1515_2i.trc";
		file_sht = "/home/mbhargav/Desktop/phd/electronics/pcbend/data/results/architecture-080_big-perimeter/architecture-080.sheet";
		file_fis = "/home/mbhargav/Desktop/phd/electronics/pcbend/data/results/architecture-080_big-perimeter/faces/architecture-080.fis";
		// file_sht = "../data/icosa-020-try.sheet";
		// file_fis = "../data/icosa-020_1515_10i.trc";
	}

	if (argc == 5) {
		file_stg = argv[1];
		file_sht = argv[2];
		file_fis = argv[3];
		file_out = argv[4];
		sh.use_settings(file_stg);
	}

	std::vector<FaceInfo> fis = parse_face_info(file_fis);

	// Settings stgs = read_settings(file_stg);
	sh.read_sheet(file_sht);
	sh.init_sheet();

	// --------------------- FIGURE OUT THE ORDER OF TRAVERSAL ------------------------------------ 
	int numPatches = sh.patch_count();
	
	// TODO separate fis per patches: 
	std::vector<std::vector<FaceInfo>> pfis(numPatches);
	for(auto fi: fis) {
		std::pair<size_t, size_t> pair = sh.sfid_to_pfid(fi.fid);
		size_t pid = pair.first; 
		pfis[pid].push_back(fi);
	}
	
	// --------------------- ORDER THE LEDS IN THE ORDER OF .LED file ------------------------------------
	// sort fis based on their fids because that is how I process them in .led files
	std::vector<std::vector<FaceInfo>> pfis_ordered(numPatches);
	for(int pi = 0; pi < numPatches; pi++) {
		vector<pair<FaceInfo, int>> t_fis_fids;
		for(auto fi: pfis[pi]) {
			t_fis_fids.push_back(make_pair(fi, fi.fid));
		}
		std::sort(t_fis_fids.begin(), t_fis_fids.end(), [](auto &left, auto &right)
				{ return left.second < right.second; });
		
		for(auto fi: t_fis_fids) {
			pfis_ordered[pi].push_back(fi.first);
		}
	}

	map<int,int> ledMap;
	vector<vector<vector<Chain>>> chainsPerFacePerPatch(numPatches);
	// face chains // chains equal number of things covered between input and output. Can be empty can contain some information. 
	vector<vector<Chain>> chainsPerFace; // if there is a -1 in a chain that means that it is a connector remember that. 
	chainsPerFace.resize(fis.size()); // there are as many chains in a face as there are number of hinge edges.  
	// we have to circle around the hinge edges in order to uncover all of them


	int ledCount = 0; 
	for(int pi = 0; pi < numPatches; pi++) {
		printf("Ordering patch : %d\n", pi);
		std::vector<ExtendedVertex> vertex_order_contour = sh.get_vertex_order(pi); // NOTE Generalize this for multi-patch function later
		std::vector<int> face_traversal_order;
		for(auto voc: vertex_order_contour) { 
			if(get<0>(voc) == VertexType::face) {
				face_traversal_order.push_back(get<3>(voc));
			}
		}
		std::vector<int> face_traversal_order_clean = removeAdjValues(face_traversal_order);
		int startFace = face_traversal_order_clean[0];
		int lastFace = face_traversal_order_clean[face_traversal_order_clean.size()-1];
		if(startFace != lastFace) {
			face_traversal_order_clean.push_back(startFace);
		}

		// each face is visited as many times as it as hinge edges!!
		// find the first instance of the face that contains the connector
		// remove the graph that before it and append at the very end
		int face_id_with_connector = -1; 
		for(auto fi: pfis[pi]) {
			if(fi.has_connector)
				face_id_with_connector = fi.fid;
		}
		assert(face_id_with_connector != -1);
		if(face_id_with_connector == -1) {
			// assign first face of the patch as the face which hosts the connector
			face_id_with_connector = pfis[pi][0].fid;
		}

		std::vector<int> face_traversal_order_rearranged = rebaseGraph(face_traversal_order_clean, face_id_with_connector);

		float abs = 1e-8;
		int prev_module_count = 0; 
		for(auto fi: pfis_ordered[pi]) {
			printf("face id: %d\n", int(fi.fid));
			auto isHinge = fi.is_hinge_edge;
			auto modules = fi.modules;
			auto traces = fi.traces;
			auto cis = fi.hinge_ios;
			auto wis = fi.hinge_ios_wide;
			auto connector = fi.connector;
			// printf("hinge situation: %d %d %d\n", int(isHinge[0]), int(isHinge[1]), int(isHinge[2]));

			vector<bool> everyModuleFoundATrace(modules.size(), false);
			vector<bool> everyTraceFoundAType(traces.size(), false);

			vector<tuple<int,int,int>> edge_to_connector_trace; // it stores the id of the trace that connects an LED to the inputs/output and a flag if it is wide hinge or normal hinge
			int trace_id = 0; 
			Chain chain; 
			bool trace_start = false;

			for(auto trace: traces) {
				// CHECK in CIDs
				int cid = 0;
				bool found_trace_type = false; 
				for(auto ci: cis) { // it is a vector of 3 segments each containing input pin and output pin for a hinge in a triangle
					if((lengthV(ci.first, trace.first) < abs) or lengthV(ci.second, trace.first) < abs 
							or lengthV(ci.first, trace.second) < abs or lengthV(ci.second, trace.second) < abs) {

						// check if the other end is in module or is empty
						int module_id = prev_module_count;
						for(auto mod : modules) {
							if(traceInModule(trace.first, mod) or traceInModule(trace.second, mod)) {
								printf("trace_id: %d is connected to c io: %d and module: %d\n", trace_id, cid, module_id);
								found_trace_type = true; // go to the next trace
								if(not trace_start) {
									trace_start = true;
									chain.start = cid;
									chain.leds.push_back(module_id);
								}
								else {
									trace_start = false;
									chain.end = cid;
									chainsPerFace[fi.fid].push_back(chain);
									chain.clear();								
								}
								break;
							}
							module_id++;
						} 
						if(fi.has_connector) {
							// check if the io is connected directly with connector?
							if (traceInModule(trace.first, connector) or traceInModule(trace.second, connector)) {
								printf("trace_id: %d is connected to c io: %d and connector\n", trace_id, cid);
								found_trace_type = true; // go to the next trace
								if(not trace_start) {
									trace_start = true;
									chain.start = cid;
									chain.leds.push_back(-1); // add the connector
								}
								else {
									trace_start = false;
									chain.end = cid;
									chainsPerFace[fi.fid].push_back(chain);
									chain.clear();								
								}							
							}
						}

						if(found_trace_type)
							break; // was part of type1
						else {
							printf("trace_id: %d is connected to c io: %d and second part is part of detour polyline\n", trace_id, cid);
							found_trace_type = true; // go to the next trace
							if(not trace_start) {
								trace_start = true;
								chain.start = cid;
							}
							else {
								trace_start = false;
								chain.end = cid;
								chainsPerFace[fi.fid].push_back(chain); // insert an empty chain when it is part of the detour chain
								chain.clear();								
							}
							break;
						}
					}
					cid++;
				}
				if(found_trace_type) {everyTraceFoundAType[trace_id] = true; trace_id++; continue;} // no need to check other trace types
				
				// CHECK in WIDs
				int wid = 0;
				for(auto wi: wis) {
					if((lengthV(wi.first, trace.first) < abs) or lengthV(wi.second, trace.first) < abs 
							or lengthV(wi.first, trace.second) < abs or lengthV(wi.second, trace.second) < abs) {

						// check if the other end is in module or is empty
						int module_id = prev_module_count;
						for(auto mod : modules) {
							if(traceInModule(trace.first, mod) or traceInModule(trace.second, mod)) {
								printf("trace_id: %d is connected to w io: %d and module: %d\n", trace_id, wid, module_id);
								found_trace_type = true; // go to the next trace
								if(not trace_start) {
									trace_start = true;
									chain.start = wid;
									chain.leds.push_back(module_id);
								}
								else {
									trace_start = false;
									chain.end = wid;
									chainsPerFace[fi.fid].push_back(chain);
									chain.clear();

								}
								break;
							}
							module_id++;
						}
						if(fi.has_connector) {
							// check if the io is connected directly with connector?
							if (traceInModule(trace.first, connector) or traceInModule(trace.second, connector)) {
								printf("trace_id: %d is connected to c io: %d and connector\n", trace_id, wid);
								found_trace_type = true; // go to the next trace
								if(not trace_start) {
									trace_start = true;
									chain.start = wid;
									chain.leds.push_back(-1); // add the connector
								}
								else {
									trace_start = false;
									chain.end = wid;
									chainsPerFace[fi.fid].push_back(chain);
									chain.clear();								
								}							
							}
						}
						if(found_trace_type)
							break; // was part of type1
						else {
							printf("trace_id: %d is connected to w io: %d and second part is part of detour polyline\n", trace_id, wid);
							found_trace_type = true; // go to the next trace
							if(not trace_start) { 
								trace_start = true;
								chain.start = wid;
							}
							else {
								trace_start = false;
								chain.end = wid;
								chainsPerFace[fi.fid].push_back(chain); // insert an empty chain when it is part of the detour chain
								chain.clear();								
							}						
							break;
						}
					}
					// one end is connected to wid
					wid++;
				}
				if(found_trace_type) {everyTraceFoundAType[trace_id] = true; trace_id++; continue;} // no need to check other trace types


				// CHECK FOR MODULE-MODULE
				int module_id0 = prev_module_count;
				for(auto module0: modules) {
					int module_id1 = prev_module_count;
					for(auto module1: modules) {
						if(traceInModule(trace.first, module0) and traceInModule(trace.second, module1)) {
							printf("trace_id: %d joins module: %d and module: %d\n", trace_id, module_id0, module_id1);
							found_trace_type = true;
							everyModuleFoundATrace[module_id0-prev_module_count] = true;
							everyModuleFoundATrace[module_id1-prev_module_count] = true;
							
							if(trace_start) { // this can only happen when trace is started at somepoint. 
								if(chain.leds[chain.leds.size()-1] == module_id0) {
									chain.leds.push_back(module_id1);
								}
								else if (chain.leds[chain.leds.size()-1] == module_id1) {
									chain.leds.push_back(module_id0);
								}
								else {
									printf("Something went wrong debug!!!\n");
									exit(1);
								}
							}

							break;
						}
						module_id1++;
					}
					if(found_trace_type) {
						break;
					}
					module_id0++;
				}

				if(fi.has_connector) {
					int module_id = prev_module_count;
					for(auto mod: modules) {
						if((traceInModule(trace.first, connector) and traceInModule(trace.second, mod)) 
							or traceInModule(trace.first, mod) and traceInModule(trace.second, connector)) {
							printf("trace_id: %d joins connector and module :%d\n", trace_id, module_id);
							found_trace_type = true;
							everyModuleFoundATrace[module_id-prev_module_count] = true;
							if(trace_start) { // this can only happen when trace is started at somepoint. 
								if(chain.leds[chain.leds.size()-1] == module_id) {
									chain.leds.push_back(-1); // add a connector represented by -1
								}
								else if (chain.leds[chain.leds.size()-1] == -1) { // if it is ia connector then add the next module
									chain.leds.push_back(module_id);
								}
								else {
									printf("Something went wrong debug!!!\n");
									exit(1);
								}
							}						
							break;
						}
						module_id++;
					}
				}
				if(found_trace_type) {everyTraceFoundAType[trace_id] = true; trace_id++; continue;} // no need to check other trace types
				else { // it is neither trace type 0 or 1 so should be type 2
					printf("trace_id: %d is not connected by either any module/connector/io so it is part of the detour line\n", trace_id);
					everyTraceFoundAType[trace_id] = true;
					trace_id++;
					continue;
				}
				trace_id++;

				bool moduleCheck = true;
				for(auto mc: everyModuleFoundATrace) {
					moduleCheck = mc and moduleCheck;
				}

				bool traceCheck = true;
				for(auto tc: everyTraceFoundAType) {
					traceCheck = tc and traceCheck;
				}
				if(not moduleCheck or not traceCheck) {
					printf("THERE HAS BEEN SOME ISSUES WITH FACE : %d\nDEBUG!!!\n", int(fi.fid));
					exit(1);
				}
				printf("\n");
			}
			prev_module_count += modules.size();
		}
		
		// NOW I HAVE ALL THE CHAINS OF EVERY FACE IN A ORDER	
		int fi_end_index = -1;
		for(int fi =0; fi < face_traversal_order_rearranged.size(); fi++) {
			int fi_curr = face_traversal_order_rearranged[fi];
			if(fi < face_traversal_order_rearranged.size()-1) {
				int fi_next = face_traversal_order_rearranged[fi+1];
				std::vector<size_t> edges_curr = sh.get_edges(fi_curr);
				int edge_common = sh.get_edge_id_between_faces(fi_curr, fi_next);
				int edge_index = get_index(edges_curr, edge_common);
				// printf("fi_curr: %d fi_next : %d edge_index: %d\n", fi_curr, fi_next, edge_index);
				assert(edge_index!=-1);

				if(fi == 0) { // we start off with a connector
					fi_end_index = edge_index; 
					vector<Chain> chains = chainsPerFace[fi_curr];
					for(auto chain: chains) {
						if(chain.end == edge_index) {
							bool foundConnector = false;
							for(auto led: chain.leds) {
								if(led == -1)
									foundConnector = true; 
								if(foundConnector and led!=-1) {
									ledMap[ledCount] = led; ledCount++;
									// start mapping LEDs once the connector is found
								}
							}
							break;
						}
					}
				}
				else {
					vector<Chain> chains = chainsPerFace[fi_curr];
					for(auto chain: chains) {
						if(chain.end == edge_index) {
							for(auto led: chain.leds) {
								ledMap[ledCount] = led; ledCount++;
								// start mapping LEDs once the connector is found
							}
							break;
						}
					}				
				}
			}
			if(fi == face_traversal_order_rearranged.size()-1) { // we start off with a connector
				fi_end_index = fi_end_index; 
				vector<Chain> chains = chainsPerFace[fi_curr];
				Chain chain; 
				for(auto tchain: chains) {
					if(tchain.end == fi_end_index) {
						for(auto led: tchain.leds) {
							if(led == -1) {
								break;
							}
							ledMap[ledCount] = led; ledCount++;
							// map LEDs while the connector is not found
						}
						break;
					}
				}
			}
		}
		printf("Finished one patch: %d\n", pi);
	}
	
	
	// final MAP
	// string mapfile = file_fis.substr(0, file_fis.size()-3) + "map";
	std::ofstream outfile(file_out, std::ios::out | std::ios::trunc);
	outfile << "# leds in the physical connection maps to leds in the.led file\n";
	for(auto res: ledMap) {
		printf("leds in the physical connection maps to :%d in the .led file: %d \n", res.first, res.second);
		outfile << res.first << " " << res.second << endl;
	}
}