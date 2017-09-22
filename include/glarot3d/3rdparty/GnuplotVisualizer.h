#pragma once

#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <eigen3/Eigen/Geometry>

#include <boost/lexical_cast.hpp>

#include "gnuplot-iostream.h"

enum class PlotType {
	Point,
	Line,
	Vector
};

class GnuplotVisualizer {
public:

	GnuplotVisualizer() :
	gp("gnuplot -rv -persist"), first(true), disabled(false) {
		//		gp.set_auto_close(true);
	};
	GnuplotVisualizer(const GnuplotVisualizer& orig);

	virtual ~GnuplotVisualizer() {
		gp.clearTmpfiles();
		gp.close();
	};

	void createFilePng(std::string filename,std::string title) {
		gp << "set terminal pngcairo size 1024,768 enhanced font 'Times-Roman,14'\n"
               << "set output \"" << filename <<"\"\n";
		gp << "set title \"" << title << "\"" << std::endl; 
	}
	
	void checkInit(){
		if (disabled) return;
		if (first) {
			first = false;
//			gp << "unset label" << std::endl;
			gp << "plot ";
		} else {
			gp << ", ";
		}
	}

	template <typename T>
	void addPointVector(const std::string& name, const std::vector<T>& v, int pt = 1, int ps = 1, int lt = 1) {
		checkInit();
		std::vector<boost::tuple<double, double> > points;
		for (auto& el : v) {
			points.push_back(boost::make_tuple(el(0), el(1)));
		}
		gp << gp.file1d(points, "/tmp/" + name) << " w p ";
		gp << "pt " << pt << " ";
		gp << "ps " << ps << " ";
		gp << "lt " << lt << " ";
		gp << "title '" << name << "' ";
	}

	template <typename T, typename R>
	void addPointVector(const std::string& name, const std::vector<T>& v, const R& trans, int pt = 1, int ps = 1, int lt = 1) {
		checkInit();
		std::vector<boost::tuple<double, double> > points;
		for (auto& el : v) {
			const T& p = trans * el;
			points.push_back(boost::make_tuple(p(0), p(1)));
		}
		gp << gp.file1d(points, "/tmp/" + name) << " w p ";
		gp << "pt " << pt << " ";
		gp << "ps " << ps << " ";
		gp << "lt " << lt << " ";
		gp << "title '" << name << "' ";
	}

	template <typename T>
	void addPoint(const std::string& name, const T& p, int pt = 1, int ps = 1, int lt = 1) {
		checkInit();
		std::vector<boost::tuple<double, double> > zero;
		zero.push_back(boost::make_tuple(p(0), p(1)));
		//		for(auto& el : v){
		//				points.push_back(boost::make_tuple(-el(1), el(0)));
		//		}
		gp << gp.file1d(zero, "/tmp/" + name) << " w p ";
		gp << "pt " << pt << " ";
		gp << "ps " << ps << " ";
		gp << "lt " << lt << " ";
		gp << "title '" << name << "' ";
	}

	template <typename T>
	void addOrientedPointVector(const std::string& name, const std::vector<T>& v, const std::vector<double>& o, double radius = 1, int lt = 1) {
		checkInit();

		std::vector<boost::tuple<double, double, double> > circles;
		for (auto& el : v) {
			circles.push_back(boost::make_tuple(el(0), el(1), radius));
		}

		gp << gp.file1d(circles, "/tmp/" + name) << " w circle ";
		gp << "lt " << lt << " ";
		gp << "title '" << name << "', ";

		T versor;
		versor << radius, 0;
		std::vector<boost::tuple<double, double, double, double> > vecs;
		//		for (auto& el : v) {
		//			Eigen::Affine2d tr;
		//			tr.prerotate(Eigen::Rotation2Df())
		//			Eigen::Rotation2Df rot(theta);
		//		Eigen::Vector2f transl
		//			vecs.push_back(boost::make_tuple(el(0), el(1), radius));
		//		}

		for (int i = 0; i < v.size(); ++i) {
			Eigen::Affine2d tr = Eigen::Affine2d::Identity();
			tr.prerotate(Eigen::Rotation2Dd(o[i]));
			//			tr.pretranslate(v[i]);
			T vec = tr*versor;
			vecs.push_back(boost::make_tuple(v[i](0), v[i](1), vec(0), vec(1)));
		}

		gp << gp.file1d(vecs, "/tmp/vec_" + name) << " w vector ";
		gp << "lt " << lt << " ";
		gp << "title 'vec_" << name << "' ";
	}

	template <typename T>
	void addOrientedPoint(const std::string& name, const T& v, const double o, double radius = 1, int lt = 1) {
		checkInit();
		//
		std::vector<boost::tuple<double, double, double> > circles;
		//				for (auto& el : v) {
		circles.push_back(boost::make_tuple(v(0), v(1), radius));
		//				}

		gp << gp.file1d(circles, "/tmp/circ_" + name) << " w circle ";
		gp << "lt " << lt << " ";
		gp << "title '" << name << "', ";

		T versor;
		versor << radius, 0;
		std::vector<boost::tuple<double, double, double, double> > vecs;

		Eigen::Affine2d tr = Eigen::Affine2d::Identity();
		tr.prerotate(Eigen::Rotation2Dd(o));
		//			tr.pretranslate(v[i]);
		T vec = tr*versor;
		vecs.push_back(boost::make_tuple(v(0), v(1), vec(0), vec(1)));

		gp << gp.file1d(vecs, "/tmp/vec_" + name) << " w vector ";
		gp << "lt " << lt << " ";
		gp << "title 'vec_" << name << "' ";
	}
	
	template <typename T>
	void addTextPoint(const T& v, const std::string& text, int lt = 1,int fs = 10){
		if (disabled) return;
		labels.push_back(boost::make_tuple(text, v(0), v(1), lt, fs));
	}

 	template <typename T>
	void addCircle(const std::string& name, const T& p, double radius = 1, int lt = 1) {
		checkInit();

		std::vector<boost::tuple<double, double, double> > circles;
		circles.push_back(boost::make_tuple(p(0), p(1), radius));

		gp << gp.file1d(circles, "/tmp/" + name) << " w circle ";
		gp << "lt " << lt << " ";
		gp << "title '" << name << "' ";
	}

	template <typename T>
	void addHistogramCorner(const std::string& name, const T& p, const std::vector<T>& L, const std::vector<T>& R, double radius = 1, int slices = 16, int lt = 1) {
		checkInit();

		std::vector<boost::tuple<double, double, double> > circles;
		circles.push_back(boost::make_tuple(p(0), p(1), radius));

		gp << gp.file1d(circles, "/tmp/" + name) << " w circle ";
		gp << "lt " << 9 << " ";
		gp << "title '" << name << "', ";


		T versor;
		versor << radius, 0;
		std::vector<boost::tuple<double, double, double, double> > slice;

		for (int i = 0; i < slices; ++i) {
			Eigen::Affine2d tr = Eigen::Affine2d::Identity();
			tr.prerotate(Eigen::Rotation2Dd(i * 2 * M_PI / slices));
			//			tr.pretranslate(v[i]);
			T vec = tr*versor;
			slice.push_back(boost::make_tuple(p(0), p(1), vec(0), vec(1)));
		}

		gp << gp.file1d(slice, "/tmp/vec_" + name) << " w vector ";
		gp << "nohead lt " << 9 << " ";
		gp << "title 'slice_" << name << "', ";

		std::vector<boost::tuple<double, double, double, double> > vecs;
		vecs.push_back(boost::make_tuple(p(0), p(1), L[L.size() - 1](0) - p(0), L[L.size() - 1](1) - p(1)));
		for (int i = L.size() - 2; i >= 0; --i) {
			//if ((p - L[i]).norm() < radius)
			vecs.push_back(boost::make_tuple(p(0), p(1), L[i](0) - p(0), L[i](1) - p(1)));
		}
		gp << gp.file1d(vecs, "/tmp/vecL_" + name) << " w vector ";
		gp << "nohead lt " << lt << " ";
		gp << "title 'vec_" << name << "', ";

		vecs.clear();
		vecs.push_back(boost::make_tuple(p(0), p(1), R[0](0) - p(0), R[0](1) - p(1)));
		for (int i = 1; i < R.size(); ++i) {
			//if ((p - R[i]).norm() < radius)
			vecs.push_back(boost::make_tuple(p(0), p(1), R[i](0) - p(0), R[i](1) - p(1)));
		}
		gp << gp.file1d(vecs, "/tmp/vecR_" + name) << " w vector ";
		gp << "nohead lt " << lt + 3 << " ";
		gp << "title 'vec_" << name << "' ";
	}
	
	template <typename T>
	void addHistogramDescriptor(const std::string& name, const T& p, const std::vector<double>& h, double radius = 1, int slices = 16, int lt = 1) {
		checkInit();

		std::vector<boost::tuple<double, double, double> > circles;
		circles.push_back(boost::make_tuple(p(0), p(1), radius));

		gp << gp.file1d(circles, "/tmp/" + name) << " w circle ";
		gp << "lt " << 9 << " ";
		gp << "title '" << name << "', ";


		T versor;
		versor << radius, 0;
		std::vector<boost::tuple<double, double, double, double> > slice;
		assert(h.size() == slices && "error with histogram lenght");
		for (int i = 0; i < slices; ++i) {
			Eigen::Affine2d tr = Eigen::Affine2d::Identity();
			tr.prerotate(Eigen::Rotation2Dd(i * 2 * M_PI / slices));
			//			tr.pretranslate(v[i]);
			T vec = tr*versor;
			slice.push_back(boost::make_tuple(p(0), p(1), vec(0), vec(1)));
			
			labels.push_back(boost::make_tuple(boost::lexical_cast<std::string>(h[i]), p(0) + vec(0), p(1) + vec(1), lt));
		}

		gp << gp.file1d(slice, "/tmp/vec_" + name) << " w vector ";
		gp << "nohead lt " << 9 << " ";
		gp << "title 'slice_" << name << "' ";

		
	}

	template <typename T>
	void addTriangularCorner(const std::string& name, const T& p, const T& L, const T& R, double radius = 1, int slices = 16, int lt = 1) {
		checkInit();

		std::vector<boost::tuple<double, double, double, double> > vecs;
		vecs.push_back(boost::make_tuple(p(0), p(1), L(0) - p(0), L(1) - p(1)));
		//		for (int i = L.size()-2; i >= 0; --i) {
		//			//if ((p - L[i]).norm() < radius)
		//				vecs.push_back(boost::make_tuple(L[i+1](0), L[i+1](1), L[i](0) - L[i+1](0), L[i](1) - L[i+1](1)));
		//		}
		gp << gp.file1d(vecs, "/tmp/vecL_" + name) << " w vector ";
		gp << "nohead lt " << lt << " ";
		gp << "title 'vecL_" << name << "', ";

		vecs.clear();
		vecs.push_back(boost::make_tuple(p(0), p(1), R(0) - p(0), R(1) - p(1)));
		//		for (int i = 1; i < R.size(); ++i) {
		//			//if ((p - R[i]).norm() < radius)
		//				vecs.push_back(boost::make_tuple(R[i-1](0), R[i-1](1), R[i](0) - R[i-1](0), R[i](1) - R[i-1](1)));
		//		}
		gp << gp.file1d(vecs, "/tmp/vecR_" + name) << " w vector ";
		gp << "nohead lt " << lt + 3 << " ";
		gp << "title 'vecR_" << name << "', ";

		vecs.clear();
		vecs.push_back(boost::make_tuple(L(0), L(1), R(0) - L(0), R(1) - L(1)));
		//		for (int i = 1; i < R.size(); ++i) {
		//			//if ((p - R[i]).norm() < radius)
		//				vecs.push_back(boost::make_tuple(R[i-1](0), R[i-1](1), R[i](0) - R[i-1](0), R[i](1) - R[i-1](1)));
		//		}
		gp << gp.file1d(vecs, "/tmp/vecD_" + name) << " w vector ";
		gp << "nohead lt " << 3 << " ";
		gp << "title 'vecD_" << name << "' ";
	}

	template <typename T>
	void addConstellationVector(const std::string& name, T p, const std::vector<T>& f2, int lt = 1, int lc = 1) {
		checkInit();

		std::vector<boost::tuple<double, double, double, double> > vecs;
		for (auto& i : f2) {
			vecs.push_back(boost::make_tuple(p(0), p(1), i(0) - p(0), i(1) - p(1)));
		}

		gp << gp.file1d(vecs, "/tmp/" + name) << " w vector ";
		gp << "nohead lt " << lt << " ";
		gp << "lc " << lc << " ";
		gp << "title 'vec_" << name << "' ";
	}

	template <typename T>
	void addAssociationVector(const std::string& name, const std::vector<T>& f1, const std::vector<T>& f2, std::vector<std::pair<int, int> >& match, int lt = 1, int lc = 1) {
		checkInit();

		std::vector<boost::tuple<double, double, double, double> > vecs;
		for (auto& i : match) {
			if (i.second >= 0) {
				vecs.push_back(boost::make_tuple(f1[i.first](0), f1[i.first](1), f2[i.second](0) - f1[i.first](0), f2[i.second](1) - f1[i.first](1)));
			}
		}

		gp << gp.file1d(vecs, "/tmp/" + name) << " w vector ";
		gp << "lt " << lt << " ";
		gp << "lc " << lc << " ";
		gp << "title 'vec_" << name << "' ";
	}

	inline void plot() {
		if (disabled) return;
		gp << std::endl;
		for (int i = 0; i < labels.size(); ++i){
			gp << "set label \"" << labels[i].get<0>() << "\" at " << labels[i].get<1>() << "," << labels[i].get<2>() << " tc lt " << labels[i].get<3>() << " font \"Arial," << labels[i].get<4>() << "\"" << std::endl;
		}
		labels.clear();
		first = true;
	};

	void setWinParam(std::string t, const std::pair<float, float>& xrange = std::make_pair(0.f, 0.f),
			const std::pair<float, float>& yrange = std::make_pair(0.f, 0.f)) {
		if (disabled) return;
		gp << "set term wxt title '" << t << "' noraise\n";
		gp << "set size ratio -1\n";
		gp << "unset key\n";
		if (std::abs(xrange.first - xrange.second) > 1e-5) {
			gp << "set xrange [" << xrange.first << ":" << xrange.second << "]\n";
		}
		if (std::abs(yrange.first - yrange.second) > 1e-5) {
			gp << "set yrange [" << yrange.first << ":" << yrange.second << "]\n";
		}
	};
	
	inline void setEnable(bool enable){disabled = !enable;};


private:
	Gnuplot gp;
	bool first;
	std::vector<boost::tuple<std::string, double, double, double, int> > labels;
	//	std::string winTitle;
	bool disabled;
};

extern GnuplotVisualizer g_gnuplot;
