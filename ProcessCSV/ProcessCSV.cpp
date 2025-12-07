#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
using namespace std;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector4i;
using Eigen::Matrix4d;

using namespace std;
using namespace Eigen;

static string trim(const string &s)
{
  size_t a = s.find_first_not_of(" \t\r\n");
  if(a == string::npos)
    return "";
  size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

static vector<string> split_csv(const string &line)
{
  vector<string> out;
  string cur;
  bool inq = false;

  for(char c : line)
  {
    if(c == '"')
      inq = !inq;
    else if(c == ',' && !inq)
    {
      out.push_back(trim(cur));
      cur.clear();
    }
    else
    {
      cur.push_back(c);
    }
  }
  out.push_back(trim(cur));
  return out;
}

Vector3d recover_xyz_vec(const Vector4d &clip, const Matrix4d &inv_mvp)
{
  Vector4d v = inv_mvp * clip;
  double w = v[3];
  if(fabs(w) < 1e-12)
    w = 1e-12;
  return v.head<3>() / w;
}

int main(int argc, char **argv)
{
  if(argc < 2)
  {
    cerr << "usage: prog <input_path>\n";
    return 1;
  }

  string root = argv[1];
  string out_path = root + ".obj";

  // ---------------------------------------------------------
  // reference object-space and clip-space
  // ---------------------------------------------------------
  double vertdist = 15.0;
  Matrix<double, 4, 4> ref_obj;
  ref_obj << vertdist, vertdist, vertdist, 1.0, vertdist, -vertdist, vertdist, 1.0, vertdist, vertdist, -vertdist, 1.0, -vertdist, vertdist, vertdist, 1.0;

  // load reference clip-space from referenceverts.bin
  string root_dir = root;
  {
    size_t p1 = root_dir.find_last_of("/\\");
    if(p1 != string::npos)
      root_dir = root_dir.substr(0, p1 + 1);
    else
      root_dir = "";
  }
  string ref_path = root_dir + "referenceverts.bin";
  ifstream rf(ref_path, ios::binary);
  if(!rf)
  {
    cerr << "cannot open " << ref_path << "\n";
    return 1;
  }
  Matrix<double, 4, 4> ref_clip;
  for(int r = 0; r < 4; r++)
  {
    float x, y, z, w;
    rf.read(reinterpret_cast<char *>(&x), 4);
    rf.read(reinterpret_cast<char *>(&y), 4);
    rf.read(reinterpret_cast<char *>(&z), 4);
    rf.read(reinterpret_cast<char *>(&w), 4);
    ref_clip(r, 0) = x;
    ref_clip(r, 1) = y;
    ref_clip(r, 2) = z;
    ref_clip(r, 3) = w;
    rf.seekg(8, ios::cur); // skip texcoord0
  }
  rf.close();
  Matrix<double, 16, 16> A;
  Vector<double, 16> b;
  int row = 0;
  for(int i = 0; i < 4; i++)
  {
    double ox = ref_obj(i, 0);
    double oy = ref_obj(i, 1);
    double oz = ref_obj(i, 2);
    double ow = ref_obj(i, 3);

    double cx = ref_clip(i, 0);
    double cy = ref_clip(i, 1);
    double cz = ref_clip(i, 2);
    double cw = ref_clip(i, 3);

    A.row(row).setZero();
    A(row, 0) = ox;
    A(row, 1) = oy;
    A(row, 2) = oz;
    A(row, 3) = ow;
    b(row++) = cx;

    A.row(row).setZero();
    A(row, 4) = ox;
    A(row, 5) = oy;
    A(row, 6) = oz;
    A(row, 7) = ow;
    b(row++) = cy;

    A.row(row).setZero();
    A(row, 8) = ox;
    A(row, 9) = oy;
    A(row, 10) = oz;
    A(row, 11) = ow;
    b(row++) = cz;

    A.row(row).setZero();
    A(row, 12) = ox;
    A(row, 13) = oy;
    A(row, 14) = oz;
    A(row, 15) = ow;
    b(row++) = cw;
  }

  // ---------------------------------------------------------
  // Solve MVP
  // ---------------------------------------------------------
  Vector<double, 16> sol = A.colPivHouseholderQr().solve(b);
  Matrix4d mvp;
  for(int i = 0; i < 4; i++)
    for(int j = 0; j < 4; j++)
      mvp(i, j) = sol(i * 4 + j);

  Matrix4d inv_mvp = mvp.inverse();








struct VData
  {
    int idx;
    Vector3d pos;
  };
  struct FullVert
  {
    int orig;
    Vector3d ogxyz;
    Vector2d uv;
    Vector4i normal;
    Vector3d pos;
  };

  vector<VData> all_verts;
  vector<int> face_indices;
  vector<int> all_orig_indices;
  vector<int> all_orig_local_indices_global;
  vector<FullVert> full;
  int vertex_offset = 0;

  int file_index = 0;
  while(true)
  {
    string path_out_csv = root + "_out" + to_string(file_index) + ".csv";
    string path_out_bin = root + "_out" + to_string(file_index) + ".bin";
    string path_in_csv = root + "_in" + to_string(file_index) + ".csv";
    string path_in_bin = root + "_in" + to_string(file_index) + ".bin";

    // --- PHASE 1a: read out CSV and out-bin for recovered positions ---
    ifstream fx(path_out_csv);
    if(!fx)
    {
      if(file_index == 0)
      {
        cerr << "cannot open " << path_out_csv << "\n";
        return 1;
      }
      break;
    }

    string line;
    getline(fx, line);
    vector<string> header_out = split_csv(line);
    header_out.erase(
        remove_if(header_out.begin(), header_out.end(), [](const string &s) { return s.empty(); }),
        header_out.end());

    vector<int> file_idx;
    while(getline(fx, line))
    {
      if(!line.empty())
        file_idx.push_back(stoi(line));
    }
    fx.close();

    int num_vertices = (int)file_idx.size();

    ifstream fb_out(path_out_bin, ios::binary);
    if(!fb_out)
    {
      cerr << "cannot open " << path_out_bin << "\n";
      return 1;
    }

    int num_floats_per_vertex = (int)header_out.size();    // now defined
    vector<float> out_buf(num_vertices * num_floats_per_vertex);
    fb_out.read((char *)out_buf.data(), out_buf.size() * sizeof(float));
    fb_out.close();

    vector<VData> verts;
    verts.reserve(num_vertices);
    for(int vi = 0; vi < num_vertices; ++vi)
    {
      int base = vi * num_floats_per_vertex;
      Vector4d v(num_floats_per_vertex > 0 ? out_buf[base + 0] : 0.0,
                 num_floats_per_vertex > 1 ? out_buf[base + 1] : 0.0,
                 num_floats_per_vertex > 2 ? out_buf[base + 2] : 0.0,
                 num_floats_per_vertex > 3 ? out_buf[base + 3] : 0.0);
      VData vd;
      vd.idx = file_idx[vi];
      vd.pos = recover_xyz_vec(v, inv_mvp);
      vd.pos.array() -= vertdist;
      verts.push_back(vd);
    }

    // --- PHASE 1b: read in CSV header and in-bin attributes ---
    ifstream fin_csv(path_in_csv);
    if(!fin_csv)
    {
      cerr << "cannot open " << path_in_csv << "\n";
      return 1;
    }
    getline(fin_csv, line);
    vector<string> header_in = split_csv(line);
    header_in.erase(
        remove_if(header_in.begin(), header_in.end(), [](const string &s) { return s.empty(); }),
        header_in.end());
    fin_csv.close();

    auto trim = [](const string &s) {
      string o = s;
      o.erase(remove_if(o.begin(), o.end(), ::isspace), o.end());
      return o;
    };
    auto find_col_exact = [&](const string &name) {
      for(size_t i = 0; i < header_in.size(); ++i)
        if(trim(header_in[i]) == name)
          return (int)i;
      return -1;
    };

    int pos_x_col = find_col_exact("POSITION.x");
    int pos_y_col = find_col_exact("POSITION.y");
    int pos_z_col = find_col_exact("POSITION.z");
    int tex_x_col = find_col_exact("TEXCOORD0.x");
    int tex_y_col = find_col_exact("TEXCOORD0.y");
    int n0_col = find_col_exact("NORMAL.x");
    int n1_col = find_col_exact("NORMAL.y");
    int n2_col = find_col_exact("NORMAL.z");
    int n3_col = find_col_exact("NORMAL.w");

    vector<int> col_size;
    for(const string &tok : header_in)
      col_size.push_back(tok.rfind("NORMAL.", 0) == 0 ? 1 : 4);
    int stride_in = 0;
    vector<int> col_offset;
    for(int s : col_size)
    {
      col_offset.push_back(stride_in);
      stride_in += s;
    }

    ifstream fb_in(path_in_bin, ios::binary);
    if(!fb_in)
    {
      cerr << "cannot open " << path_in_bin << "\n";
      return 1;
    }
    vector<char> in_buf(num_vertices * stride_in);
    fb_in.read(in_buf.data(), in_buf.size());
    fb_in.close();

    vector<Vector3d> og_pos_file(num_vertices);
    vector<Vector2d> tex_file(num_vertices);
    vector<Vector4i> normals_file(num_vertices);

    for(int vi = 0; vi < num_vertices; ++vi)
    {
      char *base = in_buf.data() + vi * stride_in;

      float px = 0, py = 0, pz = 0;
      if(pos_x_col >= 0)
        memcpy(&px, base + col_offset[pos_x_col], 4);
      if(pos_y_col >= 0)
        memcpy(&py, base + col_offset[pos_y_col], 4);
      if(pos_z_col >= 0)
        memcpy(&pz, base + col_offset[pos_z_col], 4);
      og_pos_file[vi] = Vector3d(px, py, pz);

      float ux = 0, uy = 0;
      if(tex_x_col >= 0)
        memcpy(&ux, base + col_offset[tex_x_col], 4);
      if(tex_y_col >= 0)
        memcpy(&uy, base + col_offset[tex_y_col], 4);
      tex_file[vi] = Vector2d(ux, uy);

      unsigned char n0 = 0, n1 = 0, n2 = 0, n3 = 0;
      if(n0_col >= 0)
        n0 = *(unsigned char *)(base + col_offset[n0_col]);
      if(n1_col >= 0)
        n1 = *(unsigned char *)(base + col_offset[n1_col]);
      if(n2_col >= 0)
        n2 = *(unsigned char *)(base + col_offset[n2_col]);
      if(n3_col >= 0)
        n3 = *(unsigned char *)(base + col_offset[n3_col]);
      normals_file[vi] = Vector4i(n0, n1, n2, n3);
    }

    // --- PHASE 2: dedupe & build FullVert (all_verts, face_indices, all_orig_indices, full) ---
    int max_idx = 0, min_idx = INT_MAX;
    for(auto &v : verts)
    {
      max_idx = max(max_idx, v.idx);
      min_idx = min(min_idx, v.idx);
    }

    vector<int> idx_to_unique(max_idx + 1, -1);
    vector<VData> unique;
    vector<int> file_orig_indices;
    vector<int> file_orig_local_indices;

    for(int vi = 0; vi < num_vertices; ++vi)
    {
      auto &v = verts[vi];
      if(idx_to_unique[v.idx] < 0)
      {
        idx_to_unique[v.idx] = (int)unique.size();
        unique.push_back(v);
        file_orig_indices.push_back(v.idx - min_idx);
        file_orig_local_indices.push_back(vi);
      }
    }

    vector<int> indices;
    for(auto &v : verts)
      indices.push_back(idx_to_unique[v.idx] + vertex_offset);

    for(auto &u : unique)
      all_verts.push_back(u);
    for(auto &id : indices)
      face_indices.push_back(id);
    for(auto &oi : file_orig_indices)
      all_orig_indices.push_back(oi);
    for(auto &li : file_orig_local_indices)
      all_orig_local_indices_global.push_back(li);
    vertex_offset = (int)all_verts.size();

    for(size_t i = 0; i < unique.size(); ++i)
    {
      FullVert f;
      int local_attr_idx = file_orig_local_indices[i];
      f.orig = file_orig_indices[i];
      f.pos = unique[i].pos;
      if(local_attr_idx >= 0)
      {
        f.ogxyz = og_pos_file[local_attr_idx];
        f.uv = tex_file[local_attr_idx];
        f.normal = normals_file[local_attr_idx];
      }
      else
      {
        f.ogxyz = Vector3d::Zero();
        f.uv = Vector2d::Zero();
        f.normal = Vector4i(0, 0, 0, 0);
      }
      full.push_back(f);
    }

    file_index++;
  }

  // --- PHASE 3: group by Z-range buckets ---
  unordered_map<int, vector<int>> groups;
  for(size_t i = 0; i < full.size(); ++i)
  {
    int bucket = max((int)floor((5.0 - full[i].pos.z()) / 10.0), 0);
    groups[bucket].push_back((int)i);
  }

  // --- PHASE 4: calculate X offset weights between bucket 0 and bucket 1 ---
  vector<double> weights;    // will store 3 d.p number for each vertex in bucket 0
  auto round_dp = [](double val, int dp) {
    double factor = pow(10.0, dp);
    return round(val * factor) / factor;
  };

  remove("error.txt");

  // get bucket 0 and bucket 1 vertices
  auto &bucket0 = groups[0];
  auto &bucketNext = groups[1];

  for(size_t i = 0; i < bucket0.size(); ++i)
  {
    int id0 = bucket0[i];
    auto &v0 = full[id0];

    // initial candidate set = same-orig
    vector<int> base_candidates;
    for(int id1 : bucketNext)
      if(v0.orig == full[id1].orig)
        base_candidates.push_back(id1);

    // helpers
    auto match_norm = [&](auto &a, auto &b, int tol) {
      return abs(a[0] - b[0]) <= tol && abs(a[1] - b[1]) <= tol && abs(a[2] - b[2]) <= tol &&
             abs(a[3] - b[3]) <= tol;
    };

    auto match_uv = [&](double a, double b, double tol) { return fabs(a - b) <= tol; };

    auto match_xyz = [&](double a, double b, double tol) { return fabs(a - b) <= tol; };

    auto narrow_normals = [&](vector<int> in, int tol) {
      vector<int> out;
      for(int id1 : in)
      {
        auto &v1 = full[id1];
        if(match_norm(v0.normal, v1.normal, tol))
          out.push_back(id1);
      }
      return out;
    };

    auto narrow_uv = [&](vector<int> in, double tol) {
      vector<int> out;
      for(int id1 : in)
      {
        auto &v1 = full[id1];
        if(match_uv(v0.uv.x(), v1.uv.x(), tol) && match_uv(v0.uv.y(), v1.uv.y(), tol))
          out.push_back(id1);
      }
      return out;
    };

    auto narrow_xyz = [&](vector<int> in, double tol) {
      vector<int> out;
      for(int id1 : in)
      {
        auto &v1 = full[id1];
        if(match_xyz(v0.ogxyz.x(), v1.ogxyz.x(), tol) &&
           match_xyz(v0.ogxyz.y(), v1.ogxyz.y(), tol) && match_xyz(v0.ogxyz.z(), v1.ogxyz.z(), tol))
          out.push_back(id1);
      }
      return out;
    };

    auto attempt = [&](vector<int> cur, auto func) {
      vector<int> narrowed = func(cur);
      if(narrowed.size() == 1)
        return narrowed;
      if(narrowed.empty())
        return cur;       // keep previous set
      return narrowed;    // multiple -> keep narrowed
    };

    vector<int> cur = base_candidates;

    // --- initial pass ---
    cur = attempt(cur, [&](auto &c) { return narrow_normals(c, 2); });
    if(cur.size() != 1)
      cur = attempt(cur, [&](auto &c) { return narrow_uv(c, 1e-4); });
    if(cur.size() != 1)
      cur = attempt(cur, [&](auto &c) { return narrow_xyz(c, 1e-4); });

    // --- loop block #1 ---
    if(cur.size() != 1)
    {
      cur = attempt(cur, [&](auto &c) { return narrow_normals(c, 5); });
      if(cur.size() != 1)
        cur = attempt(cur, [&](auto &c) { return narrow_uv(c, 1e-3); });
      if(cur.size() != 1)
        cur = attempt(cur, [&](auto &c) { return narrow_xyz(c, 1e-3); });
    }

    // --- loop block #2 ---
    if(cur.size() != 1)
    {
      cur = attempt(cur, [&](auto &c) { return narrow_normals(c, 10); });
      if(cur.size() != 1)
        cur = attempt(cur, [&](auto &c) { return narrow_uv(c, 1e-2); });
      if(cur.size() != 1)
        cur = attempt(cur, [&](auto &c) { return narrow_xyz(c, 1e-2); });
    }

    if(cur.size() != 1) // hopefully dont get this happening
    {
      std::ofstream ferr("error.txt", std::ios::app);
      ferr << "ERROR: v0 id=" << i << " narrowing failed. " << cur.size() << " candidates remain\n";
      ferr.close();
      weights.push_back(0.0);
      continue;
    }

    int best = cur[0];

    // apply x offset calc, bone will have moved by 7.331 (LEET gamer moment) but 0.008 margin
    double dx = full[best].pos.x() - v0.pos.x();
    dx = (dx < 0.008 ? 0.008 : (dx > 7.323 ? 7.323 : dx));
    double pct = (dx - 0.008) / (7.323 - 0.008);
    weights.push_back(round_dp(pct, 3));
  }


  // write weights to file next to exe
  ofstream fout("weights.txt");
  if(fout)
  {
    for(double w : weights)
      fout << w << "\n";
    fout.close();
  }






  // -----------------------------
  // write the full OBJ
  // -----------------------------
  ofstream out(out_path);
  if(!out)
  {
    cerr << "cannot write " << out_path << "\n";
    return 1;
  }

  for(const auto &v : all_verts)
    out << "v " << v.pos[0] << " " << v.pos[1] << " " << v.pos[2] << "\n";

  for(size_t i = 0; i + 2 < face_indices.size(); i += 3)
    out << "f " << face_indices[i] + 1 << " " << face_indices[i + 1] + 1 << " "
        << face_indices[i + 2] + 1 << "\n";

  std::cout << "Wrote OBJ: " << out_path << "\n";
  return 0;
}