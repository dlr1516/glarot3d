/**
 * GLAROT-3D - Geometric LAndmark relations ROTation-invariant 3D 
 * Copyright (C) 2017 Dario Lodi Rizzini.
 *
 * glarot3d is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * glarot3d is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with glarot3d.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <iostream>
#include <vector>
#include <queue>
#include <cstdlib>
#include <cmath>
#include <Eigen/Dense>
#include <boost/filesystem.hpp> 
#include <glarot3d/CubeSymmetry.h>
#include <glarot3d/Glarot3d.h>
#include <glarot3d/CorrespondenceGraph.h>
#include <glarot3d/ParamMap.h>
#include <glarot3d/Profiler.h>

#include <glarot3d/3rdparty/gnuplot-iostream.h>
#include <glarot3d/utils.h>

#ifdef USE_PCL
  #include <pcl/point_types.h>
  #include <pcl/registration/icp.h>
#endif

const int ANGLE_NUM = 2;
const int RANGE_NUM = 70;
const double RANGE_RES = 0.25;

const double DPOS_THRES = 0.5;   // m
const double DANG_THRES = 10.0;  // deg

struct KeyframeData
{
  int id;
  Eigen::Affine3d pose;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  std::vector<Eigen::Vector3d> points;  // keypoints observed in frame
  std::vector<int> connectedIds;        // ids of connected frames
  glarot3d::Glarot3d signature;

  KeyframeData() : signature(ANGLE_NUM,RANGE_NUM,RANGE_RES) { }

  KeyframeData(int angleNum,int rangeNum,double rangeRes) : signature(angleNum,rangeNum,rangeRes) { }
};

bool compareKeyframeData(const KeyframeData& kf1,const KeyframeData& kf2)
{
  return (kf1.id < kf2.id);
}

struct PrecisionRecallPoint
{
  int threshold;
  int truePos;
  int trueNeg;
  int falsePos;
  int falseNeg;

  PrecisionRecallPoint(int thr) : threshold(thr), truePos(0), trueNeg(0), falsePos(0), falseNeg(0) { }
};

void loadMap(std::string filename,std::vector<KeyframeData>& frames);

bool parseFrameLine(const char* str,KeyframeData& frame);

//void loadMapDesc(std::string filename,std::vector<KeyframeData>& frames);

void computeSignature(KeyframeData& kfdata,int angleNum,int rangeNum,double rangeRes);

void localizeGlarot3d(int id,const std::vector<KeyframeData>& kfdata,int k,int diff,std::vector<std::pair<glarot3d::Glarot3d::Accumulator,int> >& nearest,bool norm1min);

void localizeGlarot3dConnected(int isrc,const std::vector<KeyframeData>& kfdata,int k,int diff,std::vector<std::pair<glarot3d::Glarot3d::Accumulator,int> >& nearest,bool norm1min);

void estimateTransformation(int id1,int id2,const std::vector<KeyframeData>& kfdata);

int countMatching(const KeyframeData& kf1,const KeyframeData& kf2,double assocToll,double& errPos,double& errAng);

double countPointOverlap2(const KeyframeData& kf1,const KeyframeData& kf2,double assocToll);

void associateNN(const std::vector<Eigen::Vector3d>& points1,const std::vector<Eigen::Vector3d>& points2,double assocToll,std::vector<int>& assoc12);

double distPos(int id1,int id2,const std::vector<KeyframeData>& kfdata);

double distAng(int id1,int id2,const std::vector<KeyframeData>& kfdata);

void computeRotationAxis(const Eigen::Quaterniond& q,Eigen::Vector3d& axis,double& angle);

void computeTransfDiff(const Eigen::Affine3d& transf1,const Eigen::Affine3d& transf2,double& errPos,double& errAng);

bool computeTransform(const std::vector<Eigen::Vector3d>& points1,const std::vector<Eigen::Vector3d>& points2,
                      const std::vector<std::pair<int,int> >& indices,Eigen::Affine3d& transform);

std::string getFilenamePrefixPostfix(std::string pre,std::string filenameIn,std::string post);


int main(int argc,char** argv)
{
  glarot3d::ParamMap params;
  std::string filenameIn;
  std::string filenameCfg;
  std::string filenameOut;
  int neighNum;
  int diff;
  int incr;
  int angleNum;
  int rangeNum;
  double rangeRes;
  double dposThres, dangThres;
  double assocToll;
  bool norm1min;
  std::vector<KeyframeData> kfdata;

  params.read(argc,argv);  
  params.getParam("in",filenameIn,std::string(""));
  params.getParam("cfg",filenameCfg,std::string(""));

  params.read(filenameCfg);
  params.read(argc,argv);
  params.getParam("neigh",neighNum,10);
  params.getParam("diff",diff,10);
  params.getParam("incr",incr,1);
  params.getParam("angleNum",angleNum,ANGLE_NUM);
  params.getParam("rangeNum",rangeNum,RANGE_NUM);
  params.getParam("rangeRes",rangeRes,RANGE_RES);
  params.getParam("dposThres",dposThres,DPOS_THRES);
  params.getParam("dangThresDeg",dangThres,DANG_THRES);
  dangThres = M_PI/180.0*dangThres; 
  params.getParam("norm1min",norm1min,true);
  params.getParam("assocToll",assocToll,0.5);

  std::stringstream ss;
  ss << std::setfill('0') << std::setw(3) << "_angleNum" << angleNum << "_rangeNum" << rangeNum << "_rangeRes" << (int)(100.0*rangeRes) << "_diff" << diff << "_";
  std::string filenameOutDefault = glarot3d::generateStampedString(ss.str(),"");
  filenameOutDefault = getFilenamePrefixPostfix("result_glarot3d_",filenameIn,filenameOutDefault);
  params.getParam("out",filenameOut,filenameOutDefault);  //std::string("result_glarot3d.txt"));

  std::cout << "Params:\n";
  params.write(std::cout);

  return -1;

  std::cout << "\n\nReading..." << std::endl;
  loadMap(filenameIn,kfdata);
  std::cout << "Loaded " << kfdata.size() << " keyframes" << std::endl;

  std::cout << "Sorting keyframes by ID" << std::endl;
  std::sort(kfdata.begin(),kfdata.end(),compareKeyframeData);

  int countKf = 0;
  int pointNum = 0; 
  double pathLen = 0.0;
  for (int i = 0; i < kfdata.size(); ++i) {
    pointNum += kfdata[i].points.size();
    if (i > 0) pathLen += (kfdata[i].position - kfdata[i-1].position).norm();
    std::cerr << kfdata[i].position.transpose() << "\n";
    countKf++;
  }
  if (countKf > 0) {
    std::cout << "Average points / keyframe: " << (1.0 * pointNum / countKf) << ", path len " << pathLen << std::endl;
  }
  std::cout << "Found " << countKf << " keyframes" << std::endl;
  return 0;

  countKf = 0;
  for (auto& kf : kfdata) {
    std::cout << "computing signature " << kf.id << ", " << kf.points.size() << " points" << std::endl;
    computeSignature(kf,angleNum,rangeNum,rangeRes);
//    }
//    else {
//      kf.signature.reset(angleNum,rangeNum,rangeRes);
//    }

    if (kf.signature.angleNum() != angleNum || kf.signature.rangeNum() != rangeNum) {
      std::cerr << "[WARNING] kf.signature.angleNum() " << kf.signature.angleNum() << ",  kf.signature.rangeNum() " <<  kf.signature.rangeNum() << std::endl;
    }
    countKf++;
  }

  g_prof.printInfo();

  // Number of matching points is the threshold
  std::vector<PrecisionRecallPoint> curvePR;
  curvePR.push_back(PrecisionRecallPoint(1));
  curvePR.push_back(PrecisionRecallPoint(2));
  curvePR.push_back(PrecisionRecallPoint(5));
  curvePR.push_back(PrecisionRecallPoint(8));
  curvePR.push_back(PrecisionRecallPoint(10));
  curvePR.push_back(PrecisionRecallPoint(12));
  curvePR.push_back(PrecisionRecallPoint(15));
  curvePR.push_back(PrecisionRecallPoint(18));
  curvePR.push_back(PrecisionRecallPoint(20));
  curvePR.push_back(PrecisionRecallPoint(22));
  curvePR.push_back(PrecisionRecallPoint(25));
  curvePR.push_back(PrecisionRecallPoint(28));
  curvePR.push_back(PrecisionRecallPoint(30));
  curvePR.push_back(PrecisionRecallPoint(35));
  curvePR.push_back(PrecisionRecallPoint(40));
  curvePR.push_back(PrecisionRecallPoint(45));
  curvePR.push_back(PrecisionRecallPoint(50));
  curvePR.push_back(PrecisionRecallPoint(55));
  curvePR.push_back(PrecisionRecallPoint(60));
  curvePR.push_back(PrecisionRecallPoint(65));
  curvePR.push_back(PrecisionRecallPoint(70));
  curvePR.push_back(PrecisionRecallPoint(75));
  curvePR.push_back(PrecisionRecallPoint(80));
  curvePR.push_back(PrecisionRecallPoint(85));
  curvePR.push_back(PrecisionRecallPoint(90));
  curvePR.push_back(PrecisionRecallPoint(95));
  curvePR.push_back(PrecisionRecallPoint(100));
  curvePR.push_back(PrecisionRecallPoint(105));
  curvePR.push_back(PrecisionRecallPoint(110));
  curvePR.push_back(PrecisionRecallPoint(115));
  curvePR.push_back(PrecisionRecallPoint(120));
  curvePR.push_back(PrecisionRecallPoint(125));
  curvePR.push_back(PrecisionRecallPoint(130));
  curvePR.push_back(PrecisionRecallPoint(140));
  curvePR.push_back(PrecisionRecallPoint(150));
  curvePR.push_back(PrecisionRecallPoint(160));
  curvePR.push_back(PrecisionRecallPoint(170));
  curvePR.push_back(PrecisionRecallPoint(180));
  curvePR.push_back(PrecisionRecallPoint(190));
  curvePR.push_back(PrecisionRecallPoint(200));
//  curvePR.push_back(PrecisionRecallPoint(1000));
//  curvePR.push_back(PrecisionRecallPoint(3000));
//  curvePR.push_back(PrecisionRecallPoint(5000));
//  curvePR.push_back(PrecisionRecallPoint(7000));
//  curvePR.push_back(PrecisionRecallPoint(9000));
//  curvePR.push_back(PrecisionRecallPoint(10000));
//  curvePR.push_back(PrecisionRecallPoint(15000));
//  curvePR.push_back(PrecisionRecallPoint(20000));
//  curvePR.push_back(PrecisionRecallPoint(25000));
//  curvePR.push_back(PrecisionRecallPoint(30000));
//  curvePR.push_back(PrecisionRecallPoint(35000));
//  curvePR.push_back(PrecisionRecallPoint(40000));
//  curvePR.push_back(PrecisionRecallPoint(45000));
//  curvePR.push_back(PrecisionRecallPoint(50000));
//  curvePR.push_back(PrecisionRecallPoint(60000));
//  curvePR.push_back(PrecisionRecallPoint(70000));
//  curvePR.push_back(PrecisionRecallPoint(80000));

//  curvePR.push_back(PrecisionRecallPoint(100000));
//  curvePR.push_back(PrecisionRecallPoint(150000));
//  curvePR.push_back(PrecisionRecallPoint(180000));
//  curvePR.push_back(PrecisionRecallPoint(200000));
//  curvePR.push_back(PrecisionRecallPoint(500000));
//  curvePR.push_back(PrecisionRecallPoint(1000000));

  std::ofstream fileOut(filenameOut);
  if (!fileOut) {
    std::cerr << "Cannot open output file \"" << filenameOut << "\"" << std::endl;
    return -1;
  }
  fileOut << "#kf_query_index kf_query_id kf_match_index kf_match_id matching_num dpos dang[deg]" << std::endl;

  for (int i = 0; i < kfdata.size() && i < 0; i+=incr) {
    std::vector<std::pair<glarot3d::Glarot3d::Accumulator,int> > nearest;
    PRUN("localizeGlarot3d()");
    localizeGlarot3dConnected(i,kfdata,neighNum,diff,nearest,norm1min);
    PSTOP("localizeGlarot3d()");
    std::cout << "neighbors of kf " << i << " (id " << kfdata[i].id << ")" << std::endl;

    bool foundPositive = false;
    bool firstCandidate = true;
    int positiveThresMin;
    for (auto& pd : nearest) {
      int id2 = pd.second;
      double dpos = distPos(i,id2,kfdata);
      double dang = distAng(i,id2,kfdata);
      double dposMatching, dangMatching;

      PRUN("countMatching()");
      int matchingNum = countMatching(kfdata[i],kfdata[id2],assocToll,dposMatching, dangMatching);
      PSTOP("countMatching()");

      // First positive point
      foundPositive = (dposMatching < dposThres && dangMatching < dangThres);

      // Updates precision-recall curves
      for (auto& pointPR : curvePR) {
        if (foundPositive && matchingNum > pointPR.threshold) {
          pointPR.truePos++;
        }
        else if (foundPositive && matchingNum <= pointPR.threshold) {
          pointPR.falseNeg++;
        }
        else if (!foundPositive && matchingNum <= pointPR.threshold) {
          pointPR.trueNeg++;
        }
        else if (!foundPositive && matchingNum > pointPR.threshold) {
          pointPR.falsePos++;
        }
      }

      std::cout << "  " << id2 << ": glarot dist " << pd.first << "\t dpos " << dpos << ", dang[deg] " << (180.0/M_PI*dang)
        << "\t matchingNum " << matchingNum << ", dposMatching " << dposMatching << " dangMatching[deg] " << (180.0/M_PI*dangMatching) 
        << "\t positive? " << foundPositive << std::endl;

      fileOut << i << "\t " << kfdata[i].id << "\t " << id2 << "\t " << kfdata[id2].id << "\t " << matchingNum << "\t " << dposMatching << "\t " << (180.0/M_PI*dangMatching) << std::endl;
    }
  }
  fileOut.close();

  std::cout << "\n\nPrecision-Recall Data:\n";
  std::cout << "# \tTP \tTN \tFP \tFN \tprecision \trecall\n";
  for (auto& pointPR : curvePR) {
    double precision, recall;

    if (pointPR.truePos + pointPR.falsePos > 0) {
      precision = 1.0 * pointPR.truePos / (pointPR.truePos + pointPR.falsePos);
    }
    else {
      precision = 0.0;
    }

    if (pointPR.truePos + pointPR.falseNeg > 0) {
      recall = 1.0 * pointPR.truePos / (pointPR.truePos + pointPR.falseNeg);
    }
    else {
      recall = 0.0;
    }
     
    std::cout << pointPR.threshold << " \t" << pointPR.truePos << " \t" << pointPR.trueNeg << " \t"
      << pointPR.falsePos << " \t" << pointPR.falseNeg << " \t" << precision << " \t" << recall << "\n";
  }

  g_prof.printInfo();
  return 0;
}



//void loadMap(std::string filename,std::vector<KeyframeData>& frames)
//{g
//  const size_t MAX_LEN = 16000;
//  char buffer[MAX_LEN];
//  Eigen::Vector3d p;
//  double tx, ty, tz, qx, qy, qz, qw;
//  int pointNum;
//  double pointNumAvg;
//  std::string tag;

//  std::ifstream file(filename);
//  if (!file) {
//    std::cerr << "Cannot open file \"" << filename << "\"" << std::endl;
//    return;
//  }

//  std::cout << "Reading \"" << filename << "\"" << std::endl;
//  pointNumAvg = 0.0;
//  while (file.getline(buffer, MAX_LEN)) {
//    // keyframe head
//    KeyframeData kfdata;
//    std::stringstream ss(buffer);
//    ss >> tag >> kfdata.id >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

//    // Converts translation quaternion to Eigen::Affine3d
////    kfdata.pose = Eigen::Affine3d::Identity();
////    Eigen::Quaterniond q(qw,qx,qy,qz);
////    kfdata.pose.pretranslate(Eigen::Translation3d(tx, ty, tz));
//    kfdata.position << tx, ty, tz;
//    kfdata.orientation = Eigen::Quaterniond(qw,qx,qy,qz);
//    kfdata.pose = Eigen::Translation3d(tx, ty, tz) * Eigen::Quaterniond(qw,qx,qy,qz);

//    ss >> pointNum;
//    std::cout << "keyframe " << kfdata.id << " reading " << pointNum << " points";
//    for (int i = 0; i < pointNum && ss >> p.x() >> p.y() >> p.z(); ++i) {
//      kfdata.points.push_back(p);
//      std::cout << ".";
//    }
//    std::cout << std::endl;
//    frames.push_back(kfdata);
//    pointNumAvg += pointNum;
//  }
//  file.close();
//  if (frames.size() > 0) {
//    std::cout << "Average number of points " << (pointNumAvg / frames.size()) << std::endl;
//  }
//}

void loadMap(std::string filename,std::vector<KeyframeData>& frames)
{
  KeyframeData frame;
  std::string line;
  int count;

  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Cannot open file \"" << filename << "\"" << std::endl;
    return;
  }

  count = 0;
  while (std::getline(file,line)) {
    std::cout << "read line " << count << " length " << line.size() << std::endl;
    if (parseFrameLine(line.c_str(),frame)) {
      frames.push_back(frame);
    }
    count++;
  }
  file.close();

  std::cout << "Reading \"" << filename << "\"" << std::endl;
}

bool parseFrameLine(const char* str,KeyframeData& frame)
{
  std::stringstream ss(str);
  std::string tag;
  double tx, ty, tz, qx, qy, qz, qw;
  double fx, fy, fz;
  int pointNum, connectedNum;
  int connId;
  Eigen::Vector3d p;

  ss >> tag >> frame.id >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

  if (tag.at(0) == '#') {
    std::cout << "[COMMENT]" << std::endl;
    return false;
  }

  frame.position << tx, ty, tz;
  frame.orientation = Eigen::Quaterniond(qw,qx,qy,qz);
  frame.pose = Eigen::Translation3d(tx, ty, tz) * Eigen::Quaterniond(qw,qx,qy,qz);

  // Clear frame: otherwise, if the same KeyframeData is passed every time, the feature set cumulate!
  ss >> pointNum;
  frame.points.clear(); 
  frame.points.reserve(pointNum);
  for (int i = 0; i < pointNum && ss >> fx >> fy >> fz; ++i) {
    p << fx, fy, fz;
    frame.points.push_back(p);
  }

  frame.connectedIds.clear();
  if (ss >> connectedNum) {
    frame.connectedIds.reserve(connectedNum);
    for (int i = 0; i < connectedNum && ss >> connId; ++i) {
      frame.connectedIds.push_back(connId);
    }
  }

  std::cout << " -> " << frame.points.size() << " points, " << frame.connectedIds.size() << " ids of connected frames" << std::endl;

  return true;
}


void computeSignature(KeyframeData& kfdata,int angleNum,int rangeNum,double rangeRes)
{
  Eigen::Vector3d vij, vji;
  double dmax = 0.0;
  double davg = 0.0;

  kfdata.signature.reset(angleNum,rangeNum,rangeRes);

  int pairNum = 0;
  PRUN("Glarot3d Computation");
  for (int i = 0; i < kfdata.points.size(); ++i) {
    for (int j = i+1; j < kfdata.points.size(); ++j) {
      vij = kfdata.points[i] - kfdata.points[j];
      vji = -vij;
      if (kfdata.signature.inside(vij)) {
         kfdata.signature.value(vij)++;
         kfdata.signature.value(vji)++;
         if (vij.norm() > dmax) dmax = vij.norm();
         davg += vij.norm();
         pairNum++;
      }
    }
  }
  PSTOP("Glarot3d Computation");
  if (pairNum > 0) davg = davg / pairNum;
  std::cout << "  signature from points " << kfdata.points.size() << ", pairs " << pairNum << ", range avg " << davg << " max " << dmax << std::endl;
}

//void findClosest(int id,const std::vector<KeyframeData>& kfdata)
//{
//  Glarot3d::Accumulator dist, distMin;
//  int imin;
//  CubeSymmetry sym;

//  for (int i = 0; i < kfdata.size(); ++i) {
//    if (i != id) {
//      dist = kfdata[id].signature.normL1Min(kfdata[i].signature,sym);
//    }
//  }  
//}

void localizeGlarot3d(int id,const std::vector<KeyframeData>& kfdata,int k,int diff,std::vector<std::pair<glarot3d::Glarot3d::Accumulator,int> >& nearest,bool norm1min)
{
  // Pushes the distances between scans[id].glare and other glare
  typedef std::pair<glarot3d::Glarot3d::Accumulator, int> Type;
  typedef std::vector<Type> Container;
  typedef std::greater<Type> Comparator;
  std::priority_queue<Type,Container,Comparator>  q;

  glarot3d::Glarot3d::Accumulator dist, distMin;
  int imin;
  glarot3d::CubeSymmetry sym;

  for (int i = 0; i < kfdata.size(); ++i) {
    if (abs(i - id) > diff) {

      if (kfdata[id].signature.angleNum() != kfdata[i].signature.angleNum() || kfdata[id].signature.rangeNum() != kfdata[i].signature.rangeNum()) {
        std::cerr << "comparing signatures of frames " << id << " (id " << kfdata[id].id << ") and " << i << " (id " << kfdata[i].id << ") " 
          << ": angleNum " << kfdata[id].signature.angleNum() << " vs " << kfdata[i].signature.angleNum()
          << ", rangeNum " << kfdata[id].signature.rangeNum() << " vs " << kfdata[i].signature.rangeNum()  
          << std::endl;
        exit(-1);
      }

      if (norm1min) {
        PRUN("normL1Min");
        dist = kfdata[id].signature.normL1Min(kfdata[i].signature,sym); 
        PSTOP("normL1Min");
      }
      else {
        dist = kfdata[id].signature.normL1(kfdata[i].signature); 
      }
      //assert(!std::isnan(dist));
      q.push(std::make_pair(dist, i));
    }
  }

  // Copies the k-best minimum values into nearest
  PRUN("candidate queue");
  nearest.clear();
  for (int i = 0; i < k && !q.empty(); ++i) {
    nearest.push_back(q.top());
    q.pop();
  }
  PSTOP("candidate queue");
}

void localizeGlarot3dConnected(int isrc,const std::vector<KeyframeData>& kfdata,int k,int diff,std::vector<std::pair<glarot3d::Glarot3d::Accumulator,int> >& nearest,bool norm1min)
{
  // Pushes the distances between scans[id].glare and other glare
  typedef std::pair<glarot3d::Glarot3d::Accumulator, int> Type;
  typedef std::vector<Type> Container;
  typedef std::greater<Type> Comparator;
  std::priority_queue<Type,Container,Comparator>  q;

  glarot3d::Glarot3d::Accumulator dist, distMin;
  int imin;
  glarot3d::CubeSymmetry sym;

  // Searching ONLY in the set of frames connected to keyframe isrc
  for (auto& connId : kfdata[isrc].connectedIds) {
    int idst = -1;
    for (int i = 0; i < kfdata.size(); ++i) {
      if (kfdata[i].id == connId) {
        idst = i; 
        break;
      }
    }

    if (idst>= 0 && abs(idst - isrc) > diff) {

      if (kfdata[isrc].signature.angleNum() != kfdata[idst].signature.angleNum() || kfdata[isrc].signature.rangeNum() != kfdata[idst].signature.rangeNum()) {
        std::cerr << __FUNCTION__ << ": comparing signatures of frames " << isrc << " (id " << kfdata[isrc].id << ") and " << idst << " (id " << kfdata[idst].id << ") " 
          << ": angleNum " << kfdata[isrc].signature.angleNum() << " vs " << kfdata[idst].signature.angleNum()
          << ", rangeNum " << kfdata[isrc].signature.rangeNum() << " vs " << kfdata[idst].signature.rangeNum()  
          << std::endl;
        exit(-1);
      }

      if (norm1min) {
        PRUN("normL1Min");
        dist = kfdata[isrc].signature.normL1Min(kfdata[idst].signature,sym); 
        PSTOP("normL1Min");
      }
      else {
        dist = kfdata[isrc].signature.normL1(kfdata[idst].signature); 
      }
      //assert(!std::isnan(dist));
      q.push(std::make_pair(dist, idst));
    }
  }

  // Copies the k-best minimum values into nearest
  PRUN("candidate queue");
  nearest.clear();
  for (int i = 0; i < k && !q.empty(); ++i) {
    nearest.push_back(q.top());
    q.pop();
  }
  PSTOP("candidate queue");
}

int countMatching(const KeyframeData& kf1,const KeyframeData& kf2,double assocToll,double& errPos,double& errAng)
{
  Eigen::Affine3d transf1T2;   // keyframe 2 w.r.t. keyframe 1
  //Eigen::Vector3d p2transf;
  std::vector<Eigen::Vector3d> pointsTransf2;
  std::vector<std::pair<int,int> > assocMutual;
  std::vector<int> assoc1;
  std::vector<int> assoc2;
  int i1, i2;
  int n1 = kf1.points.size();
  int n2 = kf2.points.size();
  int n12, nmin;

  // kf1.pose -> wT1   represent keyframe 1 w.r.t. world frame w
  // kf2.pose -> wT2   represent keyframe 2 w.r.t. world frame w
  // Thus, 1T2 = inv(wT1) * wT2 = 1Tw * wT2 and 2T1 
  transf1T2 = kf1.pose.inverse() * kf2.pose;

  pointsTransf2.reserve(n2);
  for (auto& p2 : kf2.points) {
    pointsTransf2.push_back(transf1T2 * p2);
  }

  PRUN("associateNN()");
  associateNN(kf1.points,pointsTransf2,10.0*assocToll,assoc1);
  associateNN(pointsTransf2,kf1.points,10.0*assocToll,assoc2);
  PSTOP("associateNN()");

  double distAvg = 0.0;
  n12 = 0;
  for (int i1 = 0; i1 < assoc1.size(); ++i1) {
    i2 = assoc1[i1];
    if (0 <= i2 && i2 < assoc2.size()) {
      if (assoc2[i2] == i1) {
        distAvg += (kf1.points[i1] - kf2.points[i2]).norm();
        n12++;
        assocMutual.push_back( std::make_pair(i1,i2) );
      }
    }
  }
  if (n12 > 0) {
    distAvg = distAvg / (1.0*n12);
  }
//  std::cout << "  n12 " << n12 << ", distAvg " << distAvg << std::endl;
  Eigen::Affine3d transfMutual;
  //double errPos, errAng;
  PRUN("computeTransform()");
  computeTransform(kf1.points,kf2.points,assocMutual,transfMutual);
  computeTransfDiff(transfMutual,transf1T2,errPos,errAng);
  PSTOP("computeTransform()");
//  std::cout << "Registration mutual assoc:\n" << transfMutual.matrix() << "\nGroudtruth:\n" << transf1T2.matrix() 
//    << "\nerrPos " << errPos << ", errAng[deg] " << (180.0/M_PI*errAng) << std::endl;

  if (n12 > n1 || n12 > n2) {
    std::cout << "kf1 id " << kf1.id << " vs kf2 id " << kf2.id << ": n1 " << n1 << ", n2 " << n2 << ", n12 " << n12 << std::endl;
  }
  assert(n12 <= n1 && n12 <= n2);
  //return (2.0 * n12 / (double)(n1 + n2));
  nmin = std::min(n1,n2);

  if (nmin == 0) return 0.0;

  return n12;
}

double countPointOverlap2(const KeyframeData& kf1,const KeyframeData& kf2,double assocToll)
{
  Eigen::Affine3d transf1T2;   // keyframe 2 w.r.t. keyframe 1
  //Eigen::Vector3d p2transf;
  std::vector<Eigen::Vector3d> pointsTransf2;
  std::vector<int> assoc1;
  std::vector<int> assoc2;
  int i1, i2;
  int n1 = kf1.points.size();
  int n2 = kf2.points.size();
  int n12, nmin;

  // kf1.pose -> wT1   represent keyframe 1 w.r.t. world frame w
  // kf2.pose -> wT2   represent keyframe 2 w.r.t. world frame w
  // Thus, 1T2 = inv(wT1) * wT2 = 1Tw * wT2 and 2T1 
  transf1T2 = kf1.pose.inverse() * kf2.pose;

  pointsTransf2.reserve(n2);
  for (auto& p2 : kf2.points) {
    pointsTransf2.push_back(transf1T2 * p2);
  }

  associateNN(kf1.points,pointsTransf2,assocToll,assoc1);
  n12 = 0;
  for (int i1 = 0; i1 < assoc1.size(); ++i1) {
    i2 = assoc1[i1];
    if (0 <= i2 && i2 < assoc2.size()) {
      n12++;
    }
  }
  //std::cout << "  n12 " << n12 << std::endl;

  return (2.0 * n12 / (double)(n1 + n2));
}

void associateNN(const std::vector<Eigen::Vector3d>& points1,const std::vector<Eigen::Vector3d>& points2,double assocToll,std::vector<int>& assoc12)
{
  double dist, distMin;
  int imin2;

  assoc12.clear();
  assoc12.resize(points1.size());
  for (int i1 = 0; i1 < points1.size(); ++i1) {
    imin2 = -1;
    distMin = 2.0 * assocToll;
    for (int i2 = 0; i2 < points2.size(); ++i2) {
      dist = (points2[i2] - points1[i1]).norm();
      if (dist < distMin) {
        imin2 = i2;
        distMin = dist;
      }
    }
    assoc12[i1] = imin2;
  }
}


double distPos(int id1,int id2,const std::vector<KeyframeData>& kfdata)
{
  return (kfdata[id1].position - kfdata[id2].position).norm();
}

double distAng(int id1,int id2,const std::vector<KeyframeData>& kfdata)
{
  Eigen::Vector3d axis;
  double angle;
  Eigen::Quaterniond dq = kfdata[id1].orientation * kfdata[id2].orientation.inverse();
  computeRotationAxis(dq,axis,angle);
  return angle;
}

void computeRotationAxis(const Eigen::Quaterniond& q,Eigen::Vector3d& axis,double& angle)
{
  double d = sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  angle = 2.0 * atan2(d,q.w());
  if (std::abs(sin(angle)) > 1e-3) {
    axis.x() = q.x() / sin(angle);
    axis.y() = q.y() / sin(angle);
    axis.z() = q.z() / sin(angle);
    axis.normalize();
  }
  else {
    std::cerr << __FILE__ << "," << __LINE__ << ": matrix closer to identity" << std::endl;
  }
}

void computeTransfDiff(const Eigen::Affine3d& transf1,const Eigen::Affine3d& transf2,double& errPos,double& errAng)
{
  Eigen::Affine3d transfDiff;
  Eigen::Quaterniond dq;
  Eigen::Vector3d axisRot;

  transfDiff = transf1 * transf2.inverse();
  errPos = transfDiff.translation().norm();
  dq = transfDiff.rotation();
  computeRotationAxis(dq,axisRot,errAng);
}

bool computeTransform(const std::vector<Eigen::Vector3d>& points1,const std::vector<Eigen::Vector3d>& points2,
                      const std::vector<std::pair<int,int> >& indices,Eigen::Affine3d& transform)
{
  typedef Eigen::Vector3d Point;
  typedef Eigen::Matrix3d Matrix;
  typedef Eigen::Affine3d Transformation;
  Point t1 = Point::Zero();
  Point t2 = Point::Zero();
  Matrix S = Matrix::Zero();

  // Compute centroids t1 and t2 of the two sets
  int n = 0;
  for (int i = 0; i < (int)indices.size(); ++i) {
    if (0 <= indices[i].first && indices[i].first < (int)points1.size() && 
        0 <= indices[i].second && indices[i].second < (int)points2.size()) 
    {
      t1 += points1[indices[i].first]; 
      t2 += points2[indices[i].second];
      n++;
    }
  }
  if (n == 0) {
    return false;
  }	
  t1 = (1.0/n) * t1;
  t2 = (1.0/n) * t2;

  // Computes the "covariance" matrix
  for (int i = 0; i < (int)indices.size(); ++i) {
    if (0 <= indices[i].first && indices[i].first < (int)points1.size() && 
        0 <= indices[i].second && indices[i].second < (int)points2.size()) 
    {
      S += (points2[indices[i].second] - t2) * (points1[indices[i].first] - t1).transpose();
    }
  }

  // Computes rotation matrix from SVD according to procustes solution
  Eigen::MatrixXd Sdyn(3,3);
  Sdyn = S;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Sdyn, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Matrix I = Matrix::Identity(3, 3);
  I(2, 2) = d;
  Matrix R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  transform.linear() = R;
//  transform.translation() = (t2 - R * t1);
  transform.translation() = (t1 - R * t2);

  return true;
}


std::string getFilenamePrefixPostfix(std::string pre,std::string filenameIn,std::string post)
{
  boost::filesystem::path filepathIn(filenameIn);
  boost::filesystem::path filepathOut = filepathIn.parent_path() / boost::filesystem::path(pre + filepathIn.stem().string() + post + filepathIn.extension().string());
  return filepathOut.string();
}
