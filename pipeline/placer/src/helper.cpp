#include "helper.hpp"

std::vector<double> parse_doubles(const std::string &line) {
	std::vector<double> vals;
	char sep = ' ';

	size_t curr_id = 0;
	size_t sep_id = 0;
	// cout << line << endl; 
	while (sep_id != std::string::npos) {
		sep_id = line.find(sep, curr_id);
		if (sep_id == std::string::npos) {
			// cout << line.substr(curr_id) << endl;
			if(line.substr(curr_id).empty() or line.substr(curr_id) == " ")  // check for last empty space
				continue;  
			string str = line.substr(curr_id);
			double d;
			try {
				d = std::stod(str);
				vals.push_back(d);
			} catch (const std::invalid_argument&) {
				// std::cerr << "Argument is invalid\n";
				continue; // throw;
			} catch (const std::out_of_range&) {
				// std::cerr << "Argument is out of range for a double\n";
				continue; // throw;
			}


		}
		else {
			// cout << line.substr(curr_id, sep_id - curr_id) << endl;
			vals.push_back(std::stod(line.substr(curr_id, sep_id - curr_id)));
			curr_id = sep_id + 1;
		}
	}

	return vals;
}

std::vector<int> parseLine2int(std::string data, std::string delim) {
	std::vector<int> vals;
	size_t pos = 0;
	std::string token;
	while ((pos = data.find(delim)) != std::string::npos)
	{
		token = data.substr(0, pos);
		// std::cout << token << std::endl;
		vals.push_back(stoi(token));
		data.erase(0, pos + delim.length());

		if ((pos = data.find(delim)) == std::string::npos) {
			vals.push_back(stoi(data));
			// cout << data << endl; 
		}
	}
	// std::cout << pos << std::endl;
	return vals; 
}

std::vector<double> parseLine2double(std::string data, std::string delim) {
	std::vector<double> vals;
	size_t pos = 0;
	std::string token;
	while ((pos = data.find(delim)) != std::string::npos) {
		token = data.substr(0, pos);
		// std::cout << token << std::endl;
		vals.push_back(stod(token));
		data.erase(0, pos + delim.length());
	}
	// std::cout << pos << std::endl;
	return vals; 
}

void parseSheet(std::string file_sht, vector<v3f> &sheet_vertices, vector<vector<int>> &sheet_faces, vector<vector<double>> &sheet_offsets, std::map<int, vector<int>> &face2Edge) {
    std::ifstream file_in_sht(file_sht);
	if (!file_in_sht.is_open()) {
		std::cerr << "[ERROR] Could not open file " << file_sht << '\n';
		return;
	}
	// create the flatface2worldface mapping
	std::string line;
    std::map<int, double> edge2offset;
	while (std::getline(file_in_sht, line)) {
        // cout << line << endl;
		if(line[0] == 'v') {
			// cout << line << endl;
			vector<double> vertex = parse_doubles(line.substr(2));
			sheet_vertices.push_back(v3f(vertex[0], vertex[1], vertex[2]));
            // cout << vertex[0] << " " << vertex[1] << " " << vertex[2] << endl;
		}
		else if(line[0] == 'f' and not (line.substr(0,2) == "fe")) {
			vector<int> face = parseLine2int(line.substr(2), " ");
			sheet_faces.push_back(vector<int>{face});
            // cout << "f" << face[0] << " " << face[1] << " " << face[2] << endl;
		}
        else if(line.substr(0,2) == "fe") { // the line is off the format 
            vector<int> info = parseLine2int(line.substr(3), " ");
            face2Edge[info[0]] = vector<int> {info[1], info[2], info[3]};
            // cout << info[0] << " " << info[1] << " " << info[2] << " " << info[3] << endl;
        }
		else if(line[0] == 'e') {
        // else if(line.substr(0,11) == "# Map Edges") { // the line is off the format 
            // cout << line << endl;
            vector<double> info = parse_doubles(line.substr(2));
            // cout << info[0] << " " << info[1] << " " << info[2] << " " << info[3] << " " << info[4] << " " << info[5] << endl;
            int edge_index = int(info[0]);
            int f0 = int(info[1]), f1 = int(info[2]);
            double offset = info[3];
            bool isHinge = int(info[4]), isBoundary = int(info[5]);
            edge2offset[edge_index] = offset;
        }
        else {
            // nothing here for now..
        }
	}

    // generate face2offset map that i use. [0,1,2] correspond to their opposite vertex.
    sheet_offsets.resize(sheet_faces.size(), vector<double>{0, 0, 0});
    ForIndex(fi, sheet_offsets.size()) {
        vector<int> edges = face2Edge[fi];
        
        double offsete0 = edge2offset[edges[0]];
        double offsete1 = edge2offset[edges[1]];
        double offsete2 = edge2offset[edges[2]];
        sheet_offsets[fi] = {offsete1, offsete2 , offsete0}; // because edge0 is between 0 and 1. Offsets are mapped from opposite vertex
    }
}

void printValidity(Validity v) {
	switch (v) {
	case Validity::success:
		std::cout << "success\n";
		break;
	case Validity::overlap:
		std::cout << "overlap\n";
		break;
	case Validity::outOfBounds:
		std::cout << "OOBounds\n";
		break;
	case Validity::tooSmall:
		std::cout << "tooSmall\n";
		break;
	case Validity::notInsideWall:
		std::cout << "notInsideWall\n";
		break;
    case Validity::edgeLengthConstrainViolated:
        std::cout << "edgeLengthConstrainViolated\n";
        break;
	}
}

void performTriangleOffset(v2d vpos0, v2d &vpos1, v2d &vpos2, double offset) {
	v2d v12 = vpos2 - vpos1, v12n = normalize(v12);
	v2d v01 = vpos0 - vpos1, v01n = normalize(v01);
	v2d v02 = vpos0 - vpos2, v02n = normalize(v02);
	v2d perp = v01 - dot(v01, v12n) * v12n;

	v2d perp_u = normalize(perp);

	v2d v01_d = v01n * (offset / (dot(v01n, perp_u)));
	v2d v02_d = v02n * (offset / (dot(v02n, perp_u)));

	// update v1...
	vpos1 = vpos1 + v01_d;
	vpos2 = vpos2 + v02_d;
}

void performTriangleOffset(v2f vpos0, v2f &vpos1, v2f &vpos2, float offset) {

	v2f v12 = vpos2 - vpos1, v12n = normalize(v12);
	v2f v01 = vpos0 - vpos1, v01n = normalize(v01);
	v2f v02 = vpos0 - vpos2, v02n = normalize(v02);
	v2f perp = v01 - dot(v01, v12n) * v12n;

	v2f perp_u = normalize(perp);

	v2f v01_d = v01n * (float(offset) / (dot(v01n, perp_u)));
	v2f v02_d = v02n * (float(offset) / (dot(v02n, perp_u)));

	// update v1...
	vpos1 = vpos1 + v01_d;
	vpos2 = vpos2 + v02_d;
}
double euclideanDistance(v3f v0, v3f v1) {
	return sqrt(dot(v0 - v1, v0 - v1));
}

double euclideanDistance(v2d v0, v2d v1) {
	return sqrt(dot(v0 - v1, v0 - v1));
}

void generateSphere(double radius, vector<double> &s_points, vector<int> &s_faces) {
    int rings = 10, sectors = 10; 
    float const R = 1. / (float)(rings - 1);
    float const S = 1. / (float)(sectors - 1);
    int r, s;

    s_points.resize(rings * sectors * 3);
    std::vector<double>::iterator v = s_points.begin();
    for (r = 0; r < rings; r++)
        for (s = 0; s < sectors; s++)
        {
            float const y = sin(-M_PI_2 + M_PI * r * R);
            float const x = cos(2 * M_PI * s * S) * sin(M_PI * r * R);
            float const z = sin(2 * M_PI * s * S) * sin(M_PI * r * R);
            *v++ = x * radius;
            *v++ = y * radius;
            *v++ = z * radius;
        }

    s_faces.resize(rings * sectors * 4);
    std::vector<int>::iterator i = s_faces.begin();
    for (r = 0; r < rings; r++) {
        for (s = 0; s < sectors; s++) {
            *i++ = r * sectors + s;
            *i++ = r * sectors + (s + 1);
            *i++ = (r + 1) * sectors + (s + 1);
            *i++ = (r + 1) * sectors + s;
        }
    }
}

void findCommonVertex(int &shared_v0, int &shared_v1, vector<int> f0, vector<int> f1) {
    vector<int> C;
    sort(f0.begin(), f0.end());
    sort(f1.begin(), f1.end());
    set_intersection(f0.begin(), f0.end(),
                     f1.begin(), f1.end(),
                     back_inserter(C));

    // printvec(C, "C contains");
    shared_v0 = C[0];
    shared_v1 = C[1];

    if(shared_v0 == -1 or shared_v1 == -1) {
        printf("Complain common pair not found!!! EXIT!!\n");
        exit(1);
    }

}

float getNorm(v3f v) {
    return sqrt(dot(v,v));
}

float getDistancePointToLine(v3f p, v3f v, v3f ed) {
    v3f edn = normalize(ed);
    return getNorm((p-v) - dot( (p-v),edn)*edn );
}

v3f projectPointOnPlane(v3f p, v3f normal, v3f vp) {
    v3f projected_vec = dot((vp-p),normal)*normal;
    return p + projected_vec; 
}

m4x4f valMul(m4x4f mat, float val)
{
    m4x4f r;
    r.fill(float(0));
    ForIndex(i, 4) {
        ForIndex(j, 4) {
            r.at(i, j) = mat.at(i, j) * val;
        }
    }
    return (r);
}

m4x4f getRotationMatrix(float angle, v3f axis) {
	if(! (getNorm(axis) < 1e-10) ) {
		axis = axis / getNorm(axis);
	}
    float ca = cos(angle);
    float sa = sin(angle);
    m4x4f I = m4x4f::identity();
    float aavals[16] = {
                            axis[0]*axis[0], axis[0]*axis[1], axis[0]*axis[2], 0, 
                            axis[1]*axis[0], axis[1]*axis[1], axis[1]*axis[2], 0, 
                            axis[2]*axis[0], axis[2]*axis[1], axis[2]*axis[2], 0, 
                            0,               0,               0,              0}; 
    m4x4f aa = m4x4f(aavals); // axis'*axis
    float Avals[16] = {
        0, axis[2], -axis[1], 0, 
        -axis[2], 0, axis[0], 0,
        axis[1], -axis[0], 0, 0, 
        0,     0,       0,    0
    };

    m4x4f A = m4x4f(Avals);
    m4x4f R = valMul((I - aa), ca) + aa + valMul(A, sa); 
    return R; 
}

v3f applyRotationMatrix(m4x4f rotationMatrix, v3f v) {
    v4f vec = {v[0], v[1], v[2], 0.0f};
    v4f rvec = rotationMatrix * vec;
    return v3f(rvec[0], rvec[1], rvec[2]);
}

double polygon_area(const std::vector<v2d>& pts) {
	auto cross = [](v2d a, v2d b) { return a[0] * b[1] - a[1] * b[0]; };
	double area = 0.0;
	for (size_t i = 0; i < pts.size(); ++i) {
		area += cross(pts[i], pts[(i + 1) % pts.size()]);
	}
	return 0.5 * std::abs(area);
}

double norm(const vector<double> &vec) {
	double ans = 0;
	for (auto v : vec)
	{
		ans += v * v;
	}
	ans = sqrt(ans);
	return ans;
}

double triangle_area(v2d p0, v2d p1, v2d p2) {
	v3d v1 = v3d(p1 - p0, 0.0);
	v3d v2 = v3d(p2 - p0, 0.0);
	return 0.5 * length(cross(v1, v2));
}

double average(std::vector<double> const& v){

	if(v.size() == 0) {
		return 0; 
	}
	double sum = 0; 
	for(auto vi : v) {
		sum += vi;
	}	

    return sum / v.size(); 
}