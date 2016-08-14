


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <iostream>
#include <vector>
#include <queue>

#include <cmath>

namespace ending{

	typedef double Orient;
	typedef std::vector<Orient> Orient_s;


	//************** Mat22 **************//   //OK
	class Mat22{
	private:
		double a[4];
	public:
		Mat22(){ a[0] = a[1] = a[2] = a[3] = 0.0; }
		Mat22(double *n){
			for (int i = 0; i < 4; i++)a[i] = n[i];
		}
		Mat22(double z, double x, double c, double v){
			a[0] = z;
			a[1] = x;
			a[2] = c;
			a[3] = v;
		}
		Mat22(const Mat22 &m){
			for (int i = 0; i < 4; i++)a[i] = m.a[i];
		}

		cv::Point operator*(cv::Point &p){
			cv::Point n((int)(a[0] * p.x + a[1] * p.y + 0.5), (int)(a[2] * p.x + a[3] * p.y + 0.5));
			return n;
		}

		double &operator[](int idx){
			if (idx < 4)return a[idx];
			return a[3];
		}
	};


	//************** RotationMatrix **************//   //OK
	class RotationMatrix{
	private:
		Mat22 rm;
		double angle;

	public:
		RotationMatrix(){}

		RotationMatrix(double ang){
			angle = ang*CV_PI / 180.0;
			rm[0] = rm[3] = cos(angle);
			rm[2] = sin(angle);
			rm[1] = -rm[2];
		}

		RotationMatrix(const RotationMatrix &r){
			rm = r.rm;
			angle = r.angle;
		}

		void rotate(cv::Point &p, Orient &o){
			p = (*this) * p;
			o -= angle;
			if (o < 0.0)o += 2 * (float)CV_PI;
		}

		cv::Point operator*(cv::Point &p){
			return rm*p;
		}

		RotationMatrix &operator=(const RotationMatrix &r){
			rm = r.rm;
			angle = r.angle;
			return *this;
		}

		static std::vector<RotationMatrix> create(double angv){
			std::vector<RotationMatrix> rt;
			RotationMatrix r;

			for (double i = 0; i < 360.0; i += angv){
				r.angle = i*CV_PI / 180.0;
				r.rm[0] = r.rm[3] = cos(r.angle);
				r.rm[2] = sin(r.angle);
				r.rm[1] = -r.rm[2];
				rt.push_back(r);
			}
			return rt;
		}

	};

	//************** RotationMatrices **************//    //OK (?)
	class RotationMatrices{
	private:
		std::vector<RotationMatrix> rotation_matrices_;
	public:
		RotationMatrices(){ rotation_matrices_.clear(); }

		RotationMatrices(double angular_velocity_){
			rotation_matrices_.clear();
			rotation_matrices_ = RotationMatrix::create(angular_velocity_);
		}

		RotationMatrix &get(int idx){
			if (idx < rotation_matrices_.size())return rotation_matrices_[idx];
			else return rotation_matrices_[rotation_matrices_.size() - 1];
		}

		void create(double angv){
			rotation_matrices_.clear();
			rotation_matrices_ = RotationMatrix::create(angv);
		}

		void clear(){
			rotation_matrices_.clear();
		}

		void release(){
			std::vector<RotationMatrix>().swap(rotation_matrices_);
		}
	};


	/********* Template *********/   //OK
	
	class Template{
	private:
		std::vector<cv::Point> coords;
		std::vector<Orient> orientations;
		cv::Size imgSize;
		cv::Size size_;
		cv::Point center;
		double scaled;
	public:
		Template(){}

		Template(cv::Mat &templ){
			scaled = 1;
			create(templ);
		}

		Template(const Template &t){
			coords = t.coords;
			orientations = t.orientations;
			size_ = t.size_;
			imgSize = t.imgSize;
			center = t.center;
			scaled = t.scaled;
		}

		void resize(double scale){
			for (int i = 0; i < coords.size(); i++){
				coords[i].x = (int)(coords[i].x * (scale / scaled) + 0.5);
				coords[i].y = (int)(coords[i].y * (scale / scaled) + 0.5);
			}
			scaled = scale;
		}

		//define center = (0,0)
		std::vector<cv::Point> &getCoords(){
			return coords;
		}

		//define [0, 2PI]
		std::vector<Orient> &getOrientations(){
			return orientations;
		}

		cv::Size &size(){
			return imgSize;
		}

		cv::Size &getSize(){
			return size_;
		}

		cv::Point &getCenter(){
			return center;
		}

		void clear(){
			coords.clear();
			orientations.clear();
			imgSize = cv::Size(0, 0);
			size_ = cv::Size(0, 0);
			center = cv::Point(0, 0);
		}

		void release(){
			std::vector<cv::Point>().swap(coords);
			std::vector<Orient>().swap(orientations);
		}

		void create(cv::Mat &templ){   //OK
			std::vector<cv::Point> local_coords;
			std::vector<Orient> local_orientations;
			imgSize = size_ = templ.size();

			while (findContour(templ, local_coords)) {
				findContourOrientations(local_coords, local_orientations);

				coords.insert(coords.end(), local_coords.begin(), local_coords.end());
				orientations.insert(orientations.end(), local_orientations.begin(), local_orientations.end());
				local_coords.clear();
				local_orientations.clear();
			}


			
			cv::Point min, max;
			min.x = size_.width;
			min.y = size_.height;
			max.x = 0;
			max.y = 0;

			center = cv::Point(0, 0);
			for (size_t i = 0; i<coords.size(); ++i) {
				center.x += coords[i].x;
				center.y += coords[i].y;

				if (min.x>coords[i].x) min.x = coords[i].x;
				if (min.y>coords[i].y) min.y = coords[i].y;
				if (max.x<coords[i].x) max.x = coords[i].x;
				if (max.y<coords[i].y) max.y = coords[i].y;
			}

			size_.width = max.x - min.x;
			size_.height = max.y - min.y;
			int coords_size = (int)coords.size();

			center.x /= MAX(coords_size, 1);
			center.y /= MAX(coords_size, 1);

			for (int i = 0; i<coords_size; ++i) {
				coords[i].x -= center.x;
				coords[i].y -= center.y;
			}
		}

		static bool findContour(cv::Mat &img, std::vector<cv::Point>&);
		static bool findFirstContourPoint(cv::Mat&, cv::Point &);
		static float getAngle(cv::Point a, cv::Point b, int& dx, int& dy);
		static void findContourOrientations(std::vector<cv::Point>&, std::vector<Orient>&);
		static void followContour(cv::Mat &, std::vector<cv::Point> &, int direction = -1);
	};


	bool Template::findContour(cv::Mat &templ_img, std::vector<cv::Point>& coords){
		cv::Point start_point;

		bool found = findFirstContourPoint(templ_img, start_point);
		if (found) {
			coords.push_back(start_point);
			followContour(templ_img, coords);
			return true;
		}

		return false;
	}

	bool Template::findFirstContourPoint(cv::Mat& templ_img, cv::Point &p){
		for (int y = 0; y<templ_img.rows; ++y) {
			for (int x = 0; x<templ_img.cols; ++x) {
				if (templ_img.at<uchar>(y, x) != 0) {
					p.x = x;
					p.y = y;
					return true;
				}
			}
		}
		return false;
	}

	float Template::getAngle(cv::Point a, cv::Point b, int& dx, int& dy){
		dx = b.x - a.x;
		dy = -(b.y - a.y);  // in image coordinated Y axis points downward
		float angle = atan2((float)dy, (float)dx);

		if (angle<0) {
			angle += (float)CV_PI;
		}

		return angle;
	}

	void Template::findContourOrientations(std::vector<cv::Point>& coords, std::vector<Orient>& orientations){
		const int M = 5;
		int coords_size = (int)coords.size();

		std::vector<float> angles(2 * M);
		orientations.insert(orientations.begin(), coords_size, float(-3 * CV_PI)); // mark as invalid in the beginning

		if (coords_size<2 * M + 1) {  // if contour not long enough to estimate orientations, abort
			return;
		}

		for (int i = M; i<coords_size - M; ++i) {
			cv::Point crt = coords[i];
			cv::Point other;
			int k = 0;
			int dx, dy;
			// compute previous M angles
			for (int j = M; j>0; --j) {
				other = coords[i - j];
				angles[k++] = getAngle(other, crt, dx, dy);
			}
			// compute next M angles
			for (int j = 1; j <= M; ++j) {
				other = coords[i + j];
				angles[k++] = getAngle(crt, other, dx, dy);
			}

			// get the middle two angles
			std::nth_element(angles.begin(), angles.begin() + M - 1, angles.end());
			std::nth_element(angles.begin() + M - 1, angles.begin() + M, angles.end());
			//        sort(angles.begin(), angles.end());

			// average them to compute tangent
			orientations[i] = (angles[M - 1] + angles[M]) / 2;
		}
	}

	void Template::followContour(cv::Mat &templ_img, std::vector<cv::Point> &coords, int direction){
		const int dir[][2] = { { -1, -1 }, { -1, 0 }, { -1, 1 }, { 0, 1 }, { 1, 1 }, { 1, 0 }, { 1, -1 }, { 0, -1 } };
		cv::Point next;
		unsigned char ptr;

		assert(direction == -1 || !coords.empty());

		cv::Point crt = coords.back();

		// mark the current pixel as visited
		templ_img.at<uchar>(crt.y, crt.x) = 0;
		if (direction == -1) {
			for (int j = 0; j<7; ++j) {
				next.x = crt.x + dir[j][1];
				next.y = crt.y + dir[j][0];
				if (next.x >= 0 && next.x < templ_img.cols &&
					next.y >= 0 && next.y < templ_img.rows){
					ptr = templ_img.at<uchar>(next.y, next.x);
					if (ptr != 0) {
						coords.push_back(next);
						followContour(templ_img, coords, j);
						// try to continue contour in the other direction
						reverse(coords.begin(), coords.end());
						followContour(templ_img, coords, (j + 4) % 8);
						break;
					}
				}
			}
		}
		else {
			int k = direction;
			int k_cost = 3;
			next.x = crt.x + dir[k][1];
			next.y = crt.y + dir[k][0];
			if (next.x >= 0 && next.x < templ_img.cols &&
				next.y >= 0 && next.y < templ_img.rows){
				ptr = templ_img.at<uchar>(next.y, next.x);
				if (ptr != 0) {
					k_cost = std::abs(dir[k][1]) + std::abs(dir[k][0]);
				}
				int p = k;
				int n = k;

				for (int j = 0; j<3; ++j) {
					p = (p + 7) % 8;
					n = (n + 1) % 8;
					next.x = crt.x + dir[p][1];
					next.y = crt.y + dir[p][0];
					if (next.x >= 0 && next.x < templ_img.cols &&
						next.y >= 0 && next.y < templ_img.rows){
						ptr = templ_img.at<uchar>(next.y, next.x);
						if (ptr != 0) {
							int p_cost = std::abs(dir[p][1]) + std::abs(dir[p][0]);
							if (p_cost<k_cost) {
								k_cost = p_cost;
								k = p;
							}
						}
						next.x = crt.x + dir[n][1];
						next.y = crt.y + dir[n][0];
						if (next.x >= 0 && next.x < templ_img.cols &&
							next.y >= 0 && next.y < templ_img.rows){
							ptr = templ_img.at<uchar>(next.y, next.x);
							if (ptr != 0) {
								int n_cost = std::abs(dir[n][1]) + std::abs(dir[n][0]);
								if (n_cost<k_cost) {
									k_cost = n_cost;
									k = n;
								}
							}
						}
					}
				}

				if (k_cost != 3) {
					next.x = crt.x + dir[k][1];
					next.y = crt.y + dir[k][0];
					if (next.x >= 0 && next.x < templ_img.cols &&
						next.y >= 0 && next.y < templ_img.rows) {
						coords.push_back(next);
						followContour(templ_img, coords, k);
					}
				}
			}
		}
	}



	/********* Matcher *********/

	class Matcher{
	private:

		std::vector<Template> templates;

		double templScale_ = 1;
		int maxMatches_ = 20;
		double minMatchDistance_ = 1.0;
		int padX_ = 3;
		int padY_ = 3;
		int scales_ = 5;
		double minScale_ = 0.6;
		double maxScale_ = 1.6;
		double orientationWeight_ = 0.5;


	public:
		Matcher(double templScale = 1, int maxMatches = 20, double minMatchDistance = 1.0, int padX = 3,
			int padY = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6,
			double orientationWeight = 0.5){
			
			templScale_ = templScale;
			maxMatches_ = maxMatches;
			minMatchDistance_ = minMatchDistance;
			padX_ = padX;
			padY_ = padY;
			scales_ = scales;
			minScale_ = minScale;
			maxScale_ = maxScale;
			orientationWeight_ = orientationWeight;
		}

		void init(double templScale = 1, int maxMatches = 20, double minMatchDistance = 1.0, int padX = 3,
			int padY = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6,
			double orientationWeight = 0.5){

			templScale_ = templScale;
			maxMatches_ = maxMatches;
			minMatchDistance_ = minMatchDistance;
			padX_ = padX;
			padY_ = padY;
			scales_ = scales;
			minScale_ = minScale;
			maxScale_ = maxScale;
			orientationWeight_ = orientationWeight;
		}

		void clear(){
			templates.clear();
		}

		void release(){
			std::vector<Template>().swap(templates);
		}

		void setTemplate(cv::Mat &templ){
			templates.clear();
			Template t(templ);
			templates.push_back(t);
		}

		void setTemplate(Template &t){
			templates.clear();
			templates.push_back(t);
		}

		void setTemplate(std::vector<Template> &t){
			templates = t;
		}

		size_t addTemplate(cv::Mat &templ){
			Template t(templ);
			templates.push_back(t);
			return templates.size() - 1;
		}

		size_t addTemplate(Template &t){
			templates.push_back(t);
			return templates.size() - 1;
		}

		size_t addTemplate(std::vector<Template> &t){
			size_t n = templates.size();
			for (int i = 0; i < t.size(); i++)templates.push_back(t[i]);
			return n;
		}
	};


	






	/********* ChamferMatcher *********/


	class ChamferMatcher{
	private:
		std::vector<Matcher> matchers;

		bool use_orientation_ = true;
		double templScale_ = 1;
		int maxMatches_ = 20;
		double minMatchDistance_ = 1.0;
		int padX_ = 3;
		int padY_ = 3;
		int scales_ = 5;
		double minScale_ = 0.6;
		double maxScale_ = 1.6;
		double orientationWeight_ = 0.5;
		double truncate_ = 20;

	public:
		ChamferMatcher(double templScale = 1, int maxMatches = 20, double minMatchDistance = 1.0, int padX = 3,
			int padY = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6,
			double orientationWeight = 0.5, double truncate = 20){
			templScale_ = templScale;
			maxMatches_ = maxMatches;
			minMatchDistance_ = minMatchDistance;
			padX_ = padX;
			padY_ = padY;
			scales_ = scales;
			minScale_ = minScale;
			maxScale_ = maxScale;
			orientationWeight_ = orientationWeight;
			truncate_ = truncate;
		}

		void setMatcher(Matcher &m);   //OK
		void setMatcher(cv::Mat &templ);  //OK
		size_t addMatcher(Matcher &m);  //OK
		size_t addMatcher(cv::Mat &templ);  //OK

		void setMatchers(std::vector<Matcher> &m);  //OK
		void setMatchers(std::vector<cv::Mat> &templ);  //OK
		size_t addMatchers(std::vector<Matcher> &m);  //OK
		size_t addMatchers(std::vector<cv::Mat> &templ);  //OK

		void createDistanceTransform(cv::Mat& edges_img, cv::Mat& dist_img, cv::Mat& annotate_img, float truncate_dt, float a = 1.0, float b = 1.5);  //OK
		void createEdgeOrientationMap(cv::Mat& edge_img, cv::Mat& orientation_img);  //OK

		void createMaps(cv::Mat& edge_img, cv::Mat &dis_img, cv::Mat &ont_img);  //OK
		void fillNonContourOrientations(cv::Mat& annotated_img, cv::Mat& orientation_img);  //OK


		int matching(cv::Mat& img, cv::Mat& templ,std::vector<std::vector<cv::Point> >& results, std::vector<float>& costs);
		
		void clear(){
			matchers.clear();
		}

		void release(){
			std::vector<Matcher>().swap(matchers);
		}
	};

	void ChamferMatcher::setMatcher(Matcher &m){
		matchers.clear();
		matchers.push_back(m);
	}

	void ChamferMatcher::setMatcher(cv::Mat &templ){
		Matcher m(templScale_, maxMatches_, minMatchDistance_, padX_,
			padY_, scales_, minScale_, maxScale_,
			orientationWeight_);
		m.setTemplate(templ);
		matchers.clear();
		matchers.push_back(m);
	}

	size_t ChamferMatcher::addMatcher(Matcher &m){
		matchers.push_back(m);
		return matchers.size() - 1;
	}

	size_t ChamferMatcher::addMatcher(cv::Mat &templ){
		Matcher m(templScale_, maxMatches_, minMatchDistance_, padX_,
			padY_, scales_, minScale_, maxScale_,
			orientationWeight_);
		m.setTemplate(templ);
		matchers.push_back(m);
		return matchers.size() - 1;
	}

	void ChamferMatcher::setMatchers(std::vector<Matcher> &m){
		matchers.clear();
		matchers = m;
	}

	void ChamferMatcher::setMatchers(std::vector<cv::Mat> &templ){
		setMatcher(templ[0]);
		for (int i = 1; i < templ.size(); i++){
			addMatcher(templ[i]);
		}
	}

	size_t ChamferMatcher::addMatchers(std::vector<Matcher> &m){
		size_t t = matchers.size();
		for (int i = 0; i < m.size(); i++){
			matchers.push_back(m[0]);
		}
		return t;
	}

	size_t ChamferMatcher::addMatchers(std::vector<cv::Mat> &templ){
		size_t t = matchers.size();
		for (int i = 0; i < templ.size(); i++){
			addMatcher(templ[i]);
		}
		return t;
	}

	void ChamferMatcher::createDistanceTransform(cv::Mat& edges_img, cv::Mat& dist_img, cv::Mat& annotate_img, float truncate_dt, float a, float b){
		int d[][2] = { { -1, -1 }, { 0, -1 }, { 1, -1 },
		{ -1, 0 }, { 1, 0 },
		{ -1, 1 }, { 0, 1 }, { 1, 1 } };


		cv::Size s = edges_img.size();
		int w = s.width;
		int h = s.height;
		// set distance to the edge pixels to 0 and put them in the queue
		std::queue<std::pair<int, int> > q;

		for (int y = 0; y<h; ++y) {
			for (int x = 0; x<w; ++x) {
				// initialize
				annotate_img.at<cv::Vec2i>(y, x)[0] = x;
				annotate_img.at<cv::Vec2i>(y, x)[1] = y;

				uchar edge_val = edges_img.at<uchar>(y, x);
				if ((edge_val != 0)) {
					q.push(std::make_pair(x, y));
					dist_img.at<float>(y, x) = 0;
				}
				else {
					dist_img.at<float>(y, x) = -1;
				}
			}
		}

		// breadth first computation of distance transform
		std::pair<int, int> crt;
		while (!q.empty()) {
			crt = q.front();
			q.pop();

			int x = crt.first;
			int y = crt.second;

			float dist_orig = dist_img.at<float>(y, x);
			float dist;

			for (size_t i = 0; i<sizeof(d) / sizeof(d[0]); ++i) {
				int nx = x + d[i][0];
				int ny = y + d[i][1];

				if (nx<0 || ny<0 || nx >= w || ny >= h) continue;

				if (std::abs(d[i][0] + d[i][1]) == 1) {
					dist = (dist_orig)+a;
				}
				else {
					dist = (dist_orig)+b;
				}

				float dt = dist_img.at<float>(ny, nx);

				if (dt == -1 || dt>dist) {
					dist_img.at<float>(ny, nx) = dist;
					q.push(std::make_pair(nx, ny));

					annotate_img.at<cv::Vec2i>(ny, nx)[0] = annotate_img.at<cv::Vec2i>(y, x)[0];
					annotate_img.at<cv::Vec2i>(ny, nx)[1] = annotate_img.at<cv::Vec2i>(y, x)[1];
				}
			}
		}
		// truncate dt

		if (truncate_dt>0) {
			cv::Mat dist_img_thr = dist_img.clone();
			cv::threshold(dist_img, dist_img_thr, truncate_dt, 0.0, cv::THRESH_TRUNC);
			dist_img_thr.copyTo(dist_img);
		}
	}

	void ChamferMatcher::createEdgeOrientationMap(cv::Mat& edge_img, cv::Mat& orientation_img){
		cv::Mat contour_img(edge_img.size(), CV_8UC1);

		orientation_img.setTo(3 * (-CV_PI));
		std::vector<cv::Point> coords;
		std::vector<Orient> orientations;

		while (Template::findContour(edge_img, coords)) {

			Template::findContourOrientations(coords, orientations);

			// set orientation pixel in orientation image
			for (size_t i = 0; i<coords.size(); ++i) {
				int x = coords[i].x;
				int y = coords[i].y;
				//            if (orientations[i]>-CV_PI)
				//    {
				//CV_PIXEL(unsigned char, contour_img, x, y)[0] = 255;
				contour_img.at<uchar>(y, x) = 255;
				//    }
				//CV_PIXEL(float, orientation_img, x, y)[0] = orientations[i];
				orientation_img.at<float>(y, x) = (float)orientations[i];
			}

			coords.clear();
			orientations.clear();
		}
	}

	void ChamferMatcher::createMaps(cv::Mat& edge_img, cv::Mat &dist_img, cv::Mat &orientation_img){
		CV_Assert(edge_img.channels() == 1);

		cv::Mat annotated_img;

		annotated_img.create(edge_img.size(), CV_32SC2);
		dist_img.create(edge_img.size(), CV_32FC1);
		dist_img.setTo(0);
		// Computing distance transform
		createDistanceTransform(edge_img, dist_img, annotated_img, (float)truncate_);


		//orientation_img = NULL;
		if (use_orientation_) {
			orientation_img.create(edge_img.size(), CV_32FC1);
			orientation_img.setTo(0);
			cv::Mat edge_clone = edge_img.clone();
			createEdgeOrientationMap(edge_clone, orientation_img);
			edge_clone.release();
			fillNonContourOrientations(annotated_img, orientation_img);
		}

		// TODO: Template matching
	}

	void ChamferMatcher::fillNonContourOrientations(cv::Mat& annotated_img, cv::Mat& orientation_img){
		int cols = annotated_img.cols;
		int rows = annotated_img.rows;

		assert(orientation_img.cols == cols && orientation_img.rows == rows);

		for (int y = 0; y<rows; ++y) {
			for (int x = 0; x<cols; ++x) {
				int xorig = annotated_img.at<cv::Vec2i>(y, x)[0];
				int yorig = annotated_img.at<cv::Vec2i>(y, x)[1];

				if (x != xorig || y != yorig) {
					//orientation_img.at<float>(yorig,xorig)=orientation_img.at<float>(y,x);
					orientation_img.at<float>(y, x) = orientation_img.at<float>(yorig, xorig);
				}
			}
		}
	}





	/********* RotationInvariantChamferMatcher *********/

	class RotationInvariantChamferMatcher : public ChamferMatcher{
	private:
		RotationMatrices rotation_matrices_;
	public:
		RotationInvariantChamferMatcher(){

		}

		void clear(){
			
		}
	};



	int chamferMatching(cv::Mat& img, cv::Mat& templ,
		std::vector<std::vector<cv::Point> >& results, std::vector<float>& costs,
		double templScale = 1, int maxMatches = 20, double minMatchDistance = 1.0, int padX = 3,
		int padY = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6,
		double orientationWeight = 0.5, double truncate = 20){
		/************/
		CV_Assert(img.type() == CV_8UC1 && templ.type() == CV_8UC1);

		ChamferMatcher cm(templScale, maxMatches, minMatchDistance, padX, padY, scales, minScale, maxScale, orientationWeight, truncate);
		cm.setMatcher(templ);


		return 0;
	}

}