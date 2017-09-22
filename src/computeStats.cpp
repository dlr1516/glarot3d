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
#include <iomanip>
#include <vector>
#include <glarot3d/ParamMap.h>
#include <boost/filesystem.hpp> 


struct ItemData
{
  int i1;
  int id1;
  int i2;
  int id2;
  int matchingNum;
  double dpos;
  double dang;
};

struct PrecisionRecallPoint
{
  int threshold;
  int truePos;
  int trueNeg;
  int falsePos;
  int falseNeg;

  PrecisionRecallPoint(int thr) : threshold(thr), truePos(0), trueNeg(0), falsePos(0), falseNeg(0) { }
};

void loadResults(std::string filename,std::vector<ItemData>& data);

std::string getFilenamePrefixPostfix(std::string pre,std::string filenameIn,std::string post);


int main(int argc,char** argv)
{
  glarot3d::ParamMap params;
  std::string filenameRes;
  std::string filenameCfg;
  std::string filenameOut;
  std::stringstream ss;
  double dposThres, dangThres;
  int thresMin, thresMax, thresInc;
  std::vector<ItemData> items;
  std::vector<PrecisionRecallPoint> curvePR;


  params.read(argc,argv);  
  params.getParam("res",filenameRes,std::string(""));
  params.getParam("cfg",filenameCfg,std::string(""));

  params.read(filenameCfg);
  params.read(argc,argv);
  params.getParam("dposThres",dposThres,0.5);
  params.getParam("dangThres",dangThres,10.0);
  params.getParam("thresMin",thresMin,0);
  params.getParam("thresMax",thresMax,140);
  params.getParam("thresInc",thresInc,5);
  ss << std::setfill('0') << std::setw(3) << "_dpos" << (int)(10.0*dposThres) << "_dang" << (int)(10.0*dangThres);
  params.getParam("out",filenameOut,getFilenamePrefixPostfix("pr_",filenameRes,ss.str()));

  std::cout << "Params:\n";
  params.write(std::cout);

  // Loads result file
  loadResults(filenameRes,items);
  std::cout << "Loaded " << items.size() << " comparison items" << std::endl;

  // Creates precision recall values
  for (int thres = thresMin; thres <= thresMax; thres += thresInc) {
    curvePR.push_back(PrecisionRecallPoint((double)thres));
  }

  // Computes stats...
  std::cout << "\nEvaluation" << std::endl;
  for (int i = 0; i < items.size(); ++i) {
      bool foundPositive = (items[i].dpos < dposThres && items[i].dang < dangThres);

      std::cout << "  " << items[i].i1 << "\t " << items[i].i2 << "\t matchN " << items[i].matchingNum 
        << "\t dpos " << items[i].dpos << "\t dang " << items[i].dang << "\t pos " << foundPositive << std::endl;

      // Updates precision-recall curves
      for (auto& pointPR : curvePR) {
        if (foundPositive && items[i].matchingNum >= pointPR.threshold) {
          pointPR.truePos++;
        }
        else if (foundPositive && items[i].matchingNum < pointPR.threshold) {
          pointPR.falseNeg++;
        }
        else if (!foundPositive && items[i].matchingNum < pointPR.threshold) {
          pointPR.trueNeg++;
        }
        else if (!foundPositive && items[i].matchingNum >= pointPR.threshold) {
          pointPR.falsePos++;
        }
      }
  }

  // Displays final precision-recall
  std::cout << "\n\n# Precision-Recall of \"" << filenameRes << "\"\n"
    << "# dposThres " << dposThres << "  dangThres[deg] " << dangThres << std::endl;
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

    if (pointPR.truePos + pointPR.falseNeg == 0 || pointPR.truePos + pointPR.falsePos == 0) {
      std::cout << "#";
    }
     
    std::cout << pointPR.threshold << " \t" << pointPR.truePos << " \t" << pointPR.trueNeg << " \t"
      << pointPR.falsePos << " \t" << pointPR.falseNeg << " \t" << precision << " \t" << recall << "\n";
  }

  
  std::ofstream fileOut(filenameOut);
  if (fileOut) {
      fileOut << "# Precision-Recall of \"" << filenameRes << "\"\n"
        << "# dposThres " << dposThres << "  dangThres[deg] " << dangThres << "\n"
        << "# \tTP \tTN \tFP \tFN \tprecision \trecall\n";
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

      if (pointPR.truePos + pointPR.falseNeg == 0 || pointPR.truePos + pointPR.falsePos == 0) {
        fileOut << "#";
      }
     
      fileOut << pointPR.threshold << " \t" << pointPR.truePos << " \t" << pointPR.trueNeg << " \t"
        << pointPR.falsePos << " \t" << pointPR.falseNeg << " \t" << precision << " \t" << recall << "\n";
    }
  }

  return 0;
}


void loadResults(std::string filename,std::vector<ItemData>& data)
{
  ItemData item;
  std::string line;
  int count;

  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Cannot open file \"" << filename << "\"" << std::endl;
    return;
  }

  count = 0;
  while (std::getline(file,line)) {
    if (line.at(0) != '#') {
      std::stringstream ss(line);
      if (ss >> item.i1 >> item.id1 >> item.i2 >> item.id2 >> item.matchingNum >> item.dpos >> item.dang) {
        std::cout << "[line " << count << "]: comparison (" << item.i1 << "," << item.i2 << ")  dpos " << item.dpos << "  dang[deg] " << item.dang << std::endl;
        data.push_back(item);
      }
      else {
        std::cout << "[line " << count << "]: invalid line\n  \"" << line << "\"" << std::endl;
      }
    }
    else {
      std::cout << "[line " << count << "]: COMMENT" << std::endl;
    }
    count++;
  }

  file.close();
}

std::string getFilenamePrefixPostfix(std::string pre,std::string filenameIn,std::string post)
{
  boost::filesystem::path filepathIn(filenameIn);
  boost::filesystem::path filepathOut = filepathIn.parent_path() / boost::filesystem::path(pre + filepathIn.stem().string() + post + filepathIn.extension().string());
  return filepathOut.string();
}

