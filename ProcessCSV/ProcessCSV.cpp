#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <assimp/scene.h>
#include <assimp/Exporter.hpp>
#include <thread>
#include <chrono>
#include <sys/stat.h>
#include <windows.h>

std::string getExeDir()
{
  char path[MAX_PATH];
  GetModuleFileNameA(nullptr, path, MAX_PATH);

  std::string exePath(path);
  size_t pos = exePath.find_last_of("\\/");
  return (pos == std::string::npos) ? std::string() : exePath.substr(0, pos + 1);
}

using namespace std;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector4i;
using Eigen::Matrix4d;

using namespace std;
using namespace Eigen;
#undef min
#undef max
#include <limits>

std::string exeDir;

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

struct Bone
{
  string name;
  int parent;
  vector<float> weights;
  Vector3d pos;
};

bool dirExists(const std::string &path)
{
  struct stat info;
  return stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

std::string MakeOutFilePath(const std::string &path, const std::string &outDir)
{
  size_t slashPos = path.find_last_of("/\\");
  std::string filename = (slashPos == std::string::npos) ? path : path.substr(slashPos + 1);

  std::string dir = outDir;
  if(!dir.empty() && dir.back() != '\\')
    dir += "\\";

  std::string fullPath = dir + filename;

  // convert all forward slashes to backslashes
  for(size_t i = 0; i < fullPath.size(); ++i)
  {
    if(fullPath[i] == '/')
      fullPath[i] = '\\';
  }

  return fullPath;
}

extern "C" int safe_export_call(int (*fn)(void *, const char *), void *scene, const char *outpath);
int my_exporter(void *s, const char *out)
{
  const aiScene *scene = static_cast<const aiScene *>(s);
  Assimp::Exporter e;
  return e.Export(scene, "collada", out) == aiReturn_SUCCESS ? 0 : 1;
}

void SerializeAllMatToIndices(
    const std::string &path,
    const std::vector<std::vector<std::pair<unsigned int, std::vector<unsigned int>>>> &allMaterialToIndices)
{
  std::ofstream file(path);
  if(!file.is_open())
    return;

  for(const auto &mesh : allMaterialToIndices)
  {
    file << "mesh\n";
    for(const auto &mat : mesh)
    {
      file << mat.first << ":";
      for(auto idx : mat.second)
        file << idx << ",";
      file << "\n";
    }
    file << "endmesh\n";
  }
  file.close();
}

typedef void(__cdecl *PatchDaeFileFunc)(const char *, const char *);
void CallPatchDaeFileDLL(
    const std::string &outFile,
    const std::vector<std::vector<std::pair<unsigned int, std::vector<unsigned int>>>> &allMaterialToIndices)
{
  std::string tempPath = getExeDir() + "allmatinfo.txt";
  SerializeAllMatToIndices(tempPath, allMaterialToIndices);

  HMODULE dll = LoadLibraryA("tinyxml2patcher.dll");
  if(!dll)
  {
    std::cerr << "Failed to load tinyxml2patcher.dll" << std::endl;
    return;
  }
  PatchDaeFileFunc patchFunc = (PatchDaeFileFunc)GetProcAddress(dll, "PatchDaeFile_C");
  if(!patchFunc)
  {
    std::cerr << "Failed to find PatchDaeFile_C in DLL" << std::endl;
    FreeLibrary(dll);
    return;
  }
  patchFunc(outFile.c_str(), tempPath.c_str());
  std::remove(tempPath.c_str());
  FreeLibrary(dll);
}

void SaveDaeFile(const std::string &path, const std::string &outName, const std::string &modelName,
                 const std::vector<Vector3d> &vertsPos, const std::vector<Vector2d> &vertsUV,
                 const std::vector<Vector4i> &vertsNormal, const std::vector<int> &face_indices,
                 const std::vector<Bone> &bones, unsigned int matCount,
                 const std::vector<std::vector<std::pair<unsigned int, std::vector<unsigned int>>>> &allMaterialToIndices)

{
  aiScene *scene = new aiScene();
  scene->mRootNode = new aiNode();
  scene->mRootNode->mName = "Scene";

  // create armature node
  aiNode *armatureNode = new aiNode();
  armatureNode->mName = "Armature";

  // attach armature node as child of Scene
  scene->mRootNode->mNumChildren = 1;
  scene->mRootNode->mChildren = new aiNode *[1]{armatureNode};
  armatureNode->mParent = scene->mRootNode;

  // build all bones
  const size_t boneCount = bones.size();
  // one aiNode per bone, indexed the same
  std::vector<aiNode *> boneNodes(boneCount);
  // first pass: create all nodes
  for(size_t i = 0; i < boneCount; ++i)
  {
    aiNode *n = new aiNode();
    n->mName = bones[i].name;
    boneNodes[i] = n;
  }
  // second pass: count children
  std::vector<unsigned int> childCounts(boneCount, 0);
  unsigned int armatureChildren = 0;
  for(size_t i = 0; i < boneCount; ++i)
  {
    if(bones[i].parent >= 0)
      childCounts[bones[i].parent]++;
    else
      armatureChildren++;
  }
  // allocate children arrays
  for(size_t i = 0; i < boneCount; ++i)
  {
    if(childCounts[i] > 0)
    {
      boneNodes[i]->mNumChildren = childCounts[i];
      boneNodes[i]->mChildren = new aiNode *[childCounts[i]];
    }
  }
  // allocate armature children
  armatureNode->mNumChildren = armatureChildren;
  armatureNode->mChildren = new aiNode *[armatureChildren];
  // reset counters for filling
  std::fill(childCounts.begin(), childCounts.end(), 0);
  unsigned int armIdx = 0;
  // third pass: wire hierarchy
  for(size_t i = 0; i < boneCount; ++i)
  {
    aiNode *node = boneNodes[i];

    if(bones[i].parent >= 0)
    {
      aiNode *parent = boneNodes[bones[i].parent];
      unsigned int &slot = childCounts[bones[i].parent];
      parent->mChildren[slot++] = node;
      node->mParent = parent;
    }
    else
    {
      armatureNode->mChildren[armIdx++] = node;
      node->mParent = armatureNode;
    }
  }

  // set bone positions
  for(size_t i = 0; i < bones.size(); ++i)
  {
    aiVector3D p((float)bones[i].pos.x(), (float)bones[i].pos.y(), (float)bones[i].pos.z());
    if(bones[i].parent >= 0)
    {
      auto &pp = bones[bones[i].parent].pos;
      p -= aiVector3D((float)pp.x(), (float)pp.y(), (float)pp.z());
    }
    aiMatrix4x4::Translation(p, boneNodes[i]->mTransformation);
  }

  // create materials per object
  scene->mNumMaterials = matCount;
  scene->mMaterials = new aiMaterial *[matCount];
  for(int fi = 0; fi < matCount; fi++)
  {
    scene->mMaterials[fi] = new aiMaterial();
    float ambient[3] = {0.5f, 0.5f, 0.5f};
    scene->mMaterials[fi]->AddProperty(ambient, 3, AI_MATKEY_COLOR_AMBIENT);
    std::string texName = outName + "#" + std::to_string(fi) + ".png";
    aiString texPath(texName.c_str());
    scene->mMaterials[fi]->AddProperty(&texPath, AI_MATKEY_TEXTURE_DIFFUSE(0));
  }

  // create mesh
  aiMesh *mesh = new aiMesh();
  mesh->mName = aiString(modelName.c_str());

  mesh->mNumVertices = (unsigned int)vertsPos.size();
  mesh->mVertices = new aiVector3D[mesh->mNumVertices];
  for(unsigned int i = 0; i < mesh->mNumVertices; ++i)
    mesh->mVertices[i] =
        aiVector3D((float)vertsPos[i].x(), (float)vertsPos[i].y(), (float)vertsPos[i].z());

  mesh->mNumFaces = (unsigned int)(face_indices.size() / 3);
  mesh->mFaces = new aiFace[mesh->mNumFaces];
  for(unsigned int i = 0; i < mesh->mNumFaces; ++i)
  {
    aiFace &f = mesh->mFaces[i];
    f.mNumIndices = 3;
    f.mIndices = new unsigned int[3]{(unsigned int)face_indices[i * 3 + 0],
                                     (unsigned int)face_indices[i * 3 + 1],
                                     (unsigned int)face_indices[i * 3 + 2]};
  }
  mesh->mMaterialIndex = 0;
  // normals
  mesh->mNormals = new aiVector3D[mesh->mNumVertices];
  for(unsigned int i = 0; i < mesh->mNumVertices; ++i)
  {
    auto &n = vertsNormal[i];
    mesh->mNormals[i] = aiVector3D((float)n.x(), (float)n.y(), (float)n.z());
  }
  // uvs (channel 0)
  mesh->mTextureCoords[0] = new aiVector3D[mesh->mNumVertices];
  mesh->mNumUVComponents[0] = 2;
  for(unsigned int i = 0; i < mesh->mNumVertices; ++i)
  {
    mesh->mTextureCoords[0][i] = aiVector3D((float)vertsUV[i].x(), 1.f - (float)vertsUV[i].y(), 0.0f);
  }
  // default vertex colors (keep)
  mesh->mColors[0] = new aiColor4D[mesh->mNumVertices];
  for(unsigned int i = 0; i < mesh->mNumVertices; ++i)
    mesh->mColors[0][i] = aiColor4D(1.f, 1.f, 1.f, 1.f);

  // assign bones to mesh
  mesh->mNumBones = (unsigned int)bones.size();
  mesh->mBones = new aiBone *[mesh->mNumBones];
  for(unsigned int i = 0; i < mesh->mNumBones; ++i)
  {
    aiBone *b = new aiBone();
    b->mName = boneNodes[i]->mName;
    b->mNumWeights = 0;
    b->mWeights = nullptr;
    b->mOffsetMatrix = aiMatrix4x4();
    mesh->mBones[i] = b;
  }

// populate bone weights and fix bind pose
  for(unsigned int i = 0; i < mesh->mNumBones; ++i)
  {
    aiBone *b = mesh->mBones[i];
    b->mNumWeights = mesh->mNumVertices;
    b->mWeights = new aiVertexWeight[b->mNumWeights];

    // assign weights
    for(unsigned int vi = 0; vi < mesh->mNumVertices; ++vi)
    {
      b->mWeights[vi].mVertexId = vi;
      b->mWeights[vi].mWeight = bones[i].weights[vi];
    }

    // compute bind pose: inverse of global transform of bone node
    aiMatrix4x4 global = boneNodes[i]->mTransformation;
    aiNode *p = boneNodes[i]->mParent;
    while(p)
    {
      global = p->mTransformation * global;
      p = p->mParent;
    }
    b->mOffsetMatrix = global.Inverse();
  }

  // add mesh to scene
  scene->mMeshes = new aiMesh *[1]{mesh};
  scene->mNumMeshes = 1;

  // create and place mesh node under armature
  aiNode *meshNode = new aiNode();
  meshNode->mName = aiString(modelName.c_str());
  meshNode->mNumMeshes = 1;
  meshNode->mMeshes = new unsigned int[1]{0};
  meshNode->mParent = armatureNode;
  // append to armature children
  aiNode **c = new aiNode *[armatureNode->mNumChildren + 1];
  for(unsigned int i = 0; i < armatureNode->mNumChildren; ++i)
    c[i] = armatureNode->mChildren[i];
  c[armatureNode->mNumChildren] = meshNode;
  delete[] armatureNode->mChildren;
  armatureNode->mChildren = c;
  armatureNode->mNumChildren++;

  // export scene
  Assimp::Exporter exporter;
  std::string outFile = path + ".dae";
  std::string pathDir = path;
  size_t pos = pathDir.find_last_of("/\\");
  if(pos != std::string::npos)
    pathDir = pathDir.substr(0, pos + 1); // include the slash
  // run exporter using the c file to catch crashes
  for(;;)
  {
    int rc = safe_export_call(my_exporter, const_cast<aiScene *>(scene), outFile.c_str());
    if(rc == 0)
      break;
    std::cout << "[Failed] leaving in 1s\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::exit(0);
  }
  CallPatchDaeFileDLL(outFile, allMaterialToIndices);
  // convert to fbx
  struct stat buf;
  std::string exePath = exeDir + "\\FbxConverter.exe";
  std::string fbxPath = pathDir + outName + ".fbx";
  if(stat(exePath.c_str(), &buf) == 0)
  {
    std::string cmd = "\"" + exePath + "\" \"" + outFile + "\" \"" + fbxPath + "\"";
    cmd = "\"" + cmd + "\"";
    system(cmd.c_str());
  }
  std::remove(outFile.c_str());
  std::cout << std::endl << "Saved file as " << fbxPath << std::endl;
}

int main(int argc, char **argv)
{
  char exePath[MAX_PATH];
  GetModuleFileNameA(NULL, exePath, MAX_PATH);
  std::string pathStr(exePath);
  size_t pos = pathStr.find_last_of("\\/");
  exeDir = (pos == std::string::npos) ? "." : pathStr.substr(0, pos);

  std::string root = "C:\\Users\\Blurro\\Downloads\\robloxexport\\robloxmesh";
  std::string outName = "wawawa";
  bool debug = false;
  if (!debug)
  {
    if(argc < 3)    // expecting both input and output
    {
      std::cerr << "usage: prog <input_path> <filename>\n";
      system("pause");
      return 1;
    }

    root = argv[1];
    outName = argv[2];
  }

  // ---------------------------------------------------------
  // reference object-space and clip-space
  // ---------------------------------------------------------
  double vertdist = 15.0;
  Matrix<double, 4, 4> ref_obj;
  ref_obj << vertdist, vertdist, vertdist, 1.0, vertdist, -vertdist, vertdist, 1.0, vertdist, vertdist, -vertdist, 1.0, -vertdist, vertdist, vertdist, 1.0;
  
  std::string fbxroot = root;
  std::string root_dir = getExeDir() + "temp\\";
  root = root_dir + ((root.find_last_of("/\\") == std::string::npos)
                            ? root
                            : root.substr(root.find_last_of("/\\") + 1));

  // load reference clip-space from referenceverts.bin
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

  // ---------------------------------------------------------
  // datablock scan + bonePos / nameData extraction
  // ---------------------------------------------------------
  vector<Vector3d> allVerts;

 // --- read all datablocks first
  for(int i = 0;; ++i)
  {
    string dbpath = root_dir + "datablock" + to_string(i) + ".bin";
    ifstream f(dbpath, ios::binary);
    if(!f)
      break;

    float x = 0, y = 0, z = 0, w = 0;

    // read vertex 0
    f.read(reinterpret_cast<char *>(&x), 4);
    f.read(reinterpret_cast<char *>(&y), 4);
    f.read(reinterpret_cast<char *>(&z), 4);
    f.read(reinterpret_cast<char *>(&w), 4);

    Vector4d clip0;
    clip0 << x, y, z, w;

    Vector3d xyz0 = recover_xyz_vec(clip0, inv_mvp);
    xyz0.array() -= vertdist;
    allVerts.push_back(xyz0);

    // condition
    if(static_cast<int>(round(xyz0.x())) == 10 && xyz0.y() < -5.0)
    {
      // vertex index 8 = float index 48
      f.seekg(48 * sizeof(float), ios::beg);

      f.read(reinterpret_cast<char *>(&x), 4);
      f.read(reinterpret_cast<char *>(&y), 4);
      f.read(reinterpret_cast<char *>(&z), 4);
      f.read(reinterpret_cast<char *>(&w), 4);

      Vector4d clip8;
      clip8 << x, y, z, w;

      Vector3d xyz8 = recover_xyz_vec(clip8, inv_mvp);
      xyz8.array() -= vertdist;
      allVerts.push_back(xyz8);
    }

    f.close();
  }

  // calc spacing between blocks on z axis where x=10 and y<-5
  int zAxisDist = 0;
  int maxZ = std::numeric_limits<int>::min();
  int secondMaxZ = std::numeric_limits<int>::min();

  for(auto &v : allVerts)
  {
    if(static_cast<int>(round(v.x())) == 10 && static_cast<int>(round(v.y())) < -5)
    {
      int z = static_cast<int>(round(v.z()));

      if(z > maxZ)
      {
        // previous max becomes candidate for secondMax if distinct
        if(maxZ != std::numeric_limits<int>::min() && maxZ != z)
          secondMaxZ = maxZ;
        maxZ = z;
      }
      else if(z < maxZ && z > secondMaxZ)
      {
        secondMaxZ = z;
      }
    }
  }
  if(maxZ != std::numeric_limits<int>::min() && secondMaxZ != std::numeric_limits<int>::min())
  {
    zAxisDist = maxZ - secondMaxZ;
  }

  // TEST WITH ZAXISDIST 0
  //zAxisDist = 0;

// --- process bonePos: only y > -5 and z < 0
  vector<Vector3d> bonePos;
  for(auto &v : allVerts)
    if(v.z() < 0.0 && v.y() > -5.0)
      bonePos.push_back(v);
  // sort descending by z
  sort(bonePos.begin(), bonePos.end(),
       [](const Vector3d &a, const Vector3d &b) { return a.z() > b.z(); });
  // normalize z by index
  for(size_t i = 0; i < bonePos.size(); ++i)
    bonePos[i].z() += static_cast<double>(i + 1) * zAxisDist;

  //std::cout << "[debug] bonePos count after filter = " << bonePos.size() << "\n";
  //for(size_t i = 0; i < bonePos.size(); ++i) std::cout << "[debug] pre-sort " << i << " x=" << bonePos[i].x() << " y=" << bonePos[i].y() << " z=" << bonePos[i].z() << "\n";

// --- process nameData: only y < -5, bucketed by exact rounded z descending, char from X value
  vector<Vector3d> verts;
  for(auto &v : allVerts)
  {
    if(v.y() < -5.0)
      verts.push_back(v);
  }
  auto round1 = [](double v) { return round(v * 10.0) / 10.0; };
  const string charset = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_<";
  vector<string> strings;

  int bucketIndex = 0;

  while(true)
  {
    int bucketZ = -zAxisDist * bucketIndex;    // go down along z
    vector<Vector3d> bucket;
    for(auto &v : verts)
    {
      int rz = static_cast<int>(round(v.z()));
      if(rz == bucketZ)
        bucket.push_back(v);
    }

   if(bucket.empty())
    {
      if(bucketIndex == 0)
      {
        bucketIndex++;
        continue;
      }
      break;
    }

    sort(bucket.begin(), bucket.end(),
         [&](const Vector3d &a, const Vector3d &b) { return round1(a.y()) > round1(b.y()); });

    string s;
    for(size_t i = 0; i + 1 < bucket.size(); ++i)
    {
      if(round1(bucket[i].y()) != round1(bucket[i + 1].y()))
        continue;

      const Vector3d &a = bucket[i];
      const Vector3d &b = bucket[i + 1];

      const Vector3d &data = (round1(a.x()) == 10.0) ? b : a;
      double x = round1(data.x());
      int idx = static_cast<int>(round(x * 10.0));

      if(idx >= 0 && idx < (int)charset.size())
        s.push_back(charset[idx]);

      ++i;    // skip paired vertex
    }

    strings.push_back(s);
    bucketIndex++;
  }

  // --- parse strings into bone hierarchy
  string modelName = "RobloxMesh";
  vector<Bone> bones;

  if(!strings.empty())
    modelName = strings[0];

  bool replaceFile = true;
  if(outName.empty())
  {
    outName = modelName;    // used in batch export path
    replaceFile = false;
  }

  for(size_t i = 1; i < strings.size(); ++i)
  {
    const string &s = strings[i];

    size_t sep = s.find('<');

    if(sep == string::npos)
    {
      bones.push_back({s, -1});
      continue;
    }

    string name = s.substr(0, sep);
    string parentName = s.substr(sep + 1);

    int parentIndex = -1;
    for(size_t j = 0; j < bones.size(); ++j)
    {
      if(bones[j].name == parentName)
      {
        parentIndex = static_cast<int>(j);
        break;
      }
    }

    bones.push_back({name, parentIndex});
  }

  // update with xyz from bonePos
  for(size_t i = 0; i < bones.size(); ++i)
    bones[i].pos = bonePos[i];

  // ---------------------------------------------------------
  struct VData
  {
    int idx;
    Vector3d pos;
  };
  struct FullVert
  {
    int orig;
    int source_file;
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

    // append uniques + provenance
    for(size_t i = 0; i < unique.size(); ++i)
    {
      all_verts.push_back(unique[i]);
    }

    for(auto &id : indices)
      face_indices.push_back(id);

    for(auto &oi : file_orig_indices)
      all_orig_indices.push_back(oi);

    for(auto &li : file_orig_local_indices)
      all_orig_local_indices_global.push_back(li);

    for(size_t i = 0; i < unique.size(); ++i)
    {
      FullVert f;
      int local_attr_idx = file_orig_local_indices[i];

      f.orig = file_orig_indices[i];
      f.source_file = file_index;
      f.pos = unique[i].pos;
      //f.byte_offset = file_orig_local_indices[i] * stride_in;

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

    vertex_offset = (int)all_verts.size();
    file_index++;
  }

  // --- PHASE 3: group by Z-range buckets ---
  double topZ = zAxisDist / 2.0;
  double bucketWidth = zAxisDist;
  unordered_map<int, vector<int>> groups;
  for(size_t i = 0; i < full.size(); ++i)
  {
    int bucket = max((int)floor((topZ - full[i].pos.z()) / bucketWidth), 0);
    groups[bucket].push_back((int)i);
  }

  // --- PHASE 4: calculate X offset weights between bucket 0 and bucket 1 ---
  auto round_dp = [zAxisDist](double val, int dp) {
    double factor = pow(zAxisDist, dp);
    return round(val * factor) / factor;
  };

  std::string errorPath = getExeDir() + "error.txt";
  std::remove(errorPath.c_str());

  // ensure bucket0 exists
  if(groups.find(0) == groups.end())
    groups[0] = {};

  auto &bucket0 = groups[0];
  for(size_t bi = 0; bi < bones.size(); ++bi)
  {
    auto itBucketNext = groups.find((int)bi + 1);
    auto &bone = bones[bi];
    bone.weights.clear();

    // if the next bucket doesn't exist, fill weights with 0
    if(itBucketNext == groups.end() || itBucketNext->second.empty())
    {
      bone.weights.resize(bucket0.size(), 0.0f);
      continue;
    }

    auto &bucketNext = itBucketNext->second;

    // preindex bucketNext by original-file-orig
    unordered_map<int, vector<int>> orig_to_next;
    orig_to_next.reserve(bucketNext.size() * 2);
    for(int id1 : bucketNext)
      orig_to_next[full[id1].orig].push_back(id1);

    // tolerance tiers
    const vector<int> normal_tols = {2, 5, 10};
    const vector<double> uv_tols = {1e-4, 1e-3, 1e-2};
    const vector<double> xyz_tols = {1e-4, 1e-3, 1e-2};

    for(size_t i = 0; i < bucket0.size(); ++i)
    {
      int id0 = bucket0[i];
      auto &v0 = full[id0];

      auto it = orig_to_next.find(v0.orig);
      if(it == orig_to_next.end())
      {
        bone.weights.push_back(0.0f);
        continue;
      }

      vector<int> orig_candidates = it->second;
      vector<int> candidates;

      // precompute norm, uv, pos candidates using smallest tolerance tier first
      int n_tol = normal_tols[0];
      double uv_tol = uv_tols[0];
      double xyz_tol = xyz_tols[0];

      vector<int> norm_candidates, uv_candidates, pos_candidates;
      for(int c : it->second)    // orig_candidates
      {
        auto &f = full[c];
        bool norm_ok =
            abs(v0.normal[0] - f.normal[0]) <= n_tol && abs(v0.normal[1] - f.normal[1]) <= n_tol &&
            abs(v0.normal[2] - f.normal[2]) <= n_tol && abs(v0.normal[3] - f.normal[3]) <= n_tol;
        bool uv_ok = fabs(v0.uv.x() - f.uv.x()) <= uv_tol && fabs(v0.uv.y() - f.uv.y()) <= uv_tol;
        bool pos_ok = fabs(v0.ogxyz.x() - f.ogxyz.x()) <= xyz_tol &&
                      fabs(v0.ogxyz.y() - f.ogxyz.y()) <= xyz_tol &&
                      fabs(v0.ogxyz.z() - f.ogxyz.z()) <= xyz_tol;

        if(norm_ok)
          norm_candidates.push_back(c);
        if(uv_ok)
          uv_candidates.push_back(c);
        if(pos_ok)
          pos_candidates.push_back(c);
      }

      vector<int> last_nonempty;
      bool matched = false;

      // 1. norm only
      if(!norm_candidates.empty())
      {
        last_nonempty = norm_candidates;
        if(norm_candidates.size() == 1)
        {
          candidates = norm_candidates;
          matched = true;
        }
      }

      // 2. uv only
      if(!matched && !uv_candidates.empty())
      {
        last_nonempty = uv_candidates;
        if(uv_candidates.size() == 1)
        {
          candidates = uv_candidates;
          matched = true;
        }
      }

      // 3. norm AND uv
      if(!matched)
      {
        vector<int> tmp;
        for(int c : norm_candidates)
          if(find(uv_candidates.begin(), uv_candidates.end(), c) != uv_candidates.end())
            tmp.push_back(c);
        if(!tmp.empty())
          last_nonempty = tmp;
        if(tmp.size() == 1)
        {
          candidates = tmp;
          matched = true;
        }
      }

      // 4. norm AND pos
      if(!matched)
      {
        vector<int> tmp;
        for(int c : norm_candidates)
          if(find(pos_candidates.begin(), pos_candidates.end(), c) != pos_candidates.end())
            tmp.push_back(c);
        if(!tmp.empty())
          last_nonempty = tmp;
        if(tmp.size() == 1)
        {
          candidates = tmp;
          matched = true;
        }
      }

      // 5. uv AND pos
      if(!matched)
      {
        vector<int> tmp;
        for(int c : uv_candidates)
          if(find(pos_candidates.begin(), pos_candidates.end(), c) != pos_candidates.end())
            tmp.push_back(c);
        if(!tmp.empty())
          last_nonempty = tmp;
        if(tmp.size() == 1)
        {
          candidates = tmp;
          matched = true;
        }
      }

      // 6. norm AND uv AND pos
      if(!matched)
      {
        vector<int> tmp;
        for(int c : norm_candidates)
          if(find(uv_candidates.begin(), uv_candidates.end(), c) != uv_candidates.end() &&
             find(pos_candidates.begin(), pos_candidates.end(), c) != pos_candidates.end())
            tmp.push_back(c);
        if(!tmp.empty())
          last_nonempty = tmp;
        if(tmp.size() == 1)
        {
          candidates = tmp;
          matched = true;
        }
      }

      // fallback hopefully never happens
      if(!matched)
        candidates = last_nonempty;

   if(candidates.size() != 1)
      {
        std::ofstream ferr(errorPath, std::ios::app);
        ferr << "ERROR: v0 id=" << i << " narrowing failed. " << candidates.size()
             << " candidates remain\n";
        ferr << "  v0: source_file=" << v0.source_file << " pos=(" << v0.pos.x() << "," << v0.pos.y() << "," << v0.pos.z() << ")\n";
        for(size_t ci = 0; ci < candidates.size(); ++ci)
        {
          auto &c = full[candidates[ci]];
          ferr << "    candidate " << ci << ": source_file=" << c.source_file << " pos=(" << c.pos.x() << "," << c.pos.y()
               << "," << c.pos.z() << ")\n";
        }
        ferr.close();
        bone.weights.push_back(0.0f);
        continue;
      }

      int best = candidates[0];
      double dx = full[best].pos.x() - v0.pos.x();
      dx = (dx < 0.008 ? 0.008 : (dx > 7.323 ? 7.323 : dx)); // 7.331 LEET but 0.008 margin lol (so floating point noise resulting in 0.001 or 0.999 is clamped to 0.0 or 1.0)
      double pct = (dx - 0.008) / (7.323 - 0.008);
      bone.weights.push_back((float)round_dp(pct, 3));
    }
  }

 // --- filter to bucket0 only, preserving source file info ---
  vector<FullVert> bucket0_full;
  bucket0_full.reserve(groups[0].size());
  vector<int> bucket0_source_file;
  bucket0_source_file.reserve(groups[0].size());

  // full_to_new maps indices in `full` -> new compact index (or -1)
  vector<int> full_to_new((int)full.size(), -1);

  int newi = 0;
  for(int old_idx : groups[0])
  {
    if(old_idx < 0 || old_idx >= (int)full.size())
      continue;
    full_to_new[old_idx] = newi;
    bucket0_full.push_back(full[old_idx]);
    bucket0_source_file.push_back(full[old_idx].source_file);
    ++newi;
  }

  // remap face indices
  vector<int> new_face_indices;
  new_face_indices.reserve(face_indices.size());
  for(int idx : face_indices)
  {
    if(idx < 0 || idx >= (int)full_to_new.size())
      continue;
    int mapped = full_to_new[idx];
    if(mapped >= 0)
      new_face_indices.push_back(mapped);
  }

  // overwrite old arrays
  full = std::move(bucket0_full);
  face_indices = std::move(new_face_indices);

  // --- build allMaterialToIndices using bucket0_source_file ---
  vector<vector<pair<unsigned int, vector<unsigned int>>>> allMaterialToIndices(1);
  auto &meshMatVec = allMaterialToIndices[0];

  // map srcFile -> material index
  unordered_map<int, int> srcToMatIdx;
  int nextMatIdx = 0;
  for(size_t fi = 0; fi + 2 < face_indices.size(); fi += 3)
  {
    int i0 = face_indices[fi];
    int srcFile = bucket0_source_file[i0];
    if(srcToMatIdx.find(srcFile) == srcToMatIdx.end())
      srcToMatIdx[srcFile] = nextMatIdx++;
  }

  // allocate material slots
  meshMatVec.clear();
  meshMatVec.resize(nextMatIdx);
  for(const auto &kv : srcToMatIdx)
  {
    int srcFile = kv.first;
    int matIdx = kv.second;
    meshMatVec[matIdx].first = srcFile;
    meshMatVec[matIdx].second.clear();
  }

  // --- determine output name avoiding overwrites ---
  std::string pathDir = fbxroot;
  size_t posss = pathDir.find_last_of("/\\");
  if(posss != std::string::npos)
    pathDir = pathDir.substr(0, posss + 1);    // include the slash
  if(!replaceFile)
  {
    std::string baseName = outName;
    int suffix = 1;
    while(true)
    {
      std::ifstream f(pathDir + outName + ".fbx");
      if(!f.good())
        break;    // file doesn't exist, use this name

      outName = baseName + "_" + std::to_string(suffix);
      suffix++;
    }
  }
  // write texeids.txt
  {
    std::ofstream fout(getExeDir() + "texeids.txt");
    fout << outName << "\n";    // first line = outName
    for(size_t m = 0; m < meshMatVec.size(); ++m)
      fout << meshMatVec[m].first << "\n";
  }

   // remap material IDs to be dense 0..N-1
  unordered_map<unsigned int, unsigned int> remap;
  unsigned int next = 0;
  for(auto &m : meshMatVec)
  {
    unsigned int &id = m.first;
    auto it = remap.find(id);
    if(it == remap.end())
      it = remap.emplace(id, next++).first;
    id = it->second;
  }

  // append triangles per material
  for(size_t fi = 0; fi + 2 < face_indices.size(); fi += 3)
  {
    int i0 = face_indices[fi + 0];
    int i1 = face_indices[fi + 1];
    int i2 = face_indices[fi + 2];
    int srcFile = bucket0_source_file[i0];
    int matIdx = srcToMatIdx[srcFile];
    auto &vec = meshMatVec[matIdx].second;
    vec.push_back(i0);
    vec.push_back(i1);
    vec.push_back(i2);
  }

  // --- prep vertex buffers for dae from FullVert ---
  vector<Vector3d> vertsPos;
  vector<Vector2d> vertsUV;
  vector<Vector4i> vertsNormal;

  vertsPos.reserve(full.size());
  vertsUV.reserve(full.size());
  vertsNormal.reserve(full.size());

  // compute corrected normals
  vector<Vector3d> accumN(full.size(), Vector3d(0, 0, 0));
  for(const auto &m : meshMatVec)
  {
    const auto &idx = m.second;

    for(size_t i = 0; i + 2 < idx.size(); i += 3)
    {
      unsigned int i0 = idx[i + 0];
      unsigned int i1 = idx[i + 1];
      unsigned int i2 = idx[i + 2];

      const Vector3d &p0 = full[i0].pos;
      const Vector3d &p1 = full[i1].pos;
      const Vector3d &p2 = full[i2].pos;

      Vector3d fn = (p1 - p0).cross(p2 - p0);

      if(fn.squaredNorm() > 1e-20)
        fn.normalize();

      accumN[i0] += fn;
      accumN[i1] += fn;
      accumN[i2] += fn;
    }
  }

  for(size_t i = 0; i < full.size(); ++i)
  {
    const auto &v = full[i];

    vertsPos.push_back(v.pos);
    vertsUV.push_back(v.uv);

    Vector3d n = accumN[i];
    if(n.squaredNorm() < 1e-20)
      n = Vector3d(0, 0, 1);
    else
      n.normalize();

    vertsNormal.emplace_back((int)std::round(n.x() * 127.0), (int)std::round(n.y() * 127.0),
                             (int)std::round(n.z() * 127.0), 0);
  }

  // normalize bone weights
  if(bones.size() < 1)
  {
    cerr << "no bones found\n";
  }
  else
  {
    size_t nVerts = full.size();
    size_t nBones = bones.size();

    for(size_t bi = 0; bi < nBones; ++bi)
      if(bones[bi].weights.size() != nVerts)
        bones[bi].weights.assign(nVerts, 0.0f);

    // 1) parent->child subtraction
    for(size_t bi = 0; bi < nBones; ++bi)
    {
      int p = bones[bi].parent;
      if(p < 0 || (size_t)p >= nBones)
        continue;
      for(size_t vi = 0; vi < nVerts; ++vi)
        bones[p].weights[vi] -= bones[bi].weights[vi];
    }

    // clamp negatives
    for(size_t bi = 0; bi < nBones; ++bi)
      for(size_t vi = 0; vi < nVerts; ++vi)
        if(bones[bi].weights[vi] < 0.0f)
          bones[bi].weights[vi] = 0.0f;

    // 2) redistribute tiny deficits/excess per vertex
    const double EPS = 1e-6;
    for(size_t vi = 0; vi < nVerts; ++vi)
    {
      double sum = 0.0;
      for(size_t bi = 0; bi < nBones; ++bi)
        sum += bones[bi].weights[vi];

      if(fabs(sum - 1.0) < EPS)
        continue;

      if(sum < 1.0 - EPS)
      {
        // add missing weight to largest bone, prioritize bone 0 if all zeros
        double deficit = 1.0 - sum;
        size_t best = 0;

        double bestVal = bones[0].weights[vi];
        for(size_t bi = 1; bi < nBones; ++bi)
          if(bones[bi].weights[vi] > bestVal)
          {
            bestVal = bones[bi].weights[vi];
            best = bi;
          }
        bones[best].weights[vi] += (float)deficit;
      }
      else
      {
        // subtract excess from smallest bone iteratively, prioritize bone 0 if tied
        double excess = sum - 1.0;
        while(excess > EPS)
        {
          size_t smallest = 0;
          double smallestVal = bones[0].weights[vi];
          for(size_t bi = 1; bi < nBones; ++bi)
            if(bones[bi].weights[vi] < smallestVal)
            {
              smallestVal = bones[bi].weights[vi];
              smallest = bi;
            }

          if(smallestVal <= 0.0)
            break;

          double take = min(smallestVal, excess);
          bones[smallest].weights[vi] -= (float)take;
          excess -= take;
        }

        // fallback: scale down all to force sum == 1
        double newSum = 0.0;
        for(size_t bi = 0; bi < nBones; ++bi)
          newSum += bones[bi].weights[vi];
        if(newSum > 0.0 && fabs(newSum - 1.0) > EPS)
        {
          double scale = 1.0 / newSum;
          for(size_t bi = 0; bi < nBones; ++bi)
            bones[bi].weights[vi] = (float)(bones[bi].weights[vi] * scale);
        }
      }
    }
  }

  // save dae
  SaveDaeFile(fbxroot, outName, modelName, vertsPos, vertsUV, vertsNormal, face_indices, bones, (int)meshMatVec.size(), allMaterialToIndices);
  return 0;
}