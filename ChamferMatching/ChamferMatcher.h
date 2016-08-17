


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <iostream>
#include <algorithm>
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

		void create(double ang){
			angle = ang*CV_PI / 180.0;
			rm[0] = rm[3] = cos(angle);
			rm[2] = sin(angle);
			rm[1] = -rm[2];
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
			create(angular_velocity_);
		}

		RotationMatrices(const RotationMatrices &rm){
			rotation_matrices_ = rm.rotation_matrices_;
		}

		RotationMatrix &get(int idx){
			if (idx < rotation_matrices_.size())return rotation_matrices_[idx];
			else return rotation_matrices_[rotation_matrices_.size() - 1];
		}

		size_t create(double angv){
			rotation_matrices_.clear();
			for (double i = 0; i < 360.0; i += angv){
				RotationMatrix r(i);
				rotation_matrices_.push_back(r);
			}
			return rotation_matrices_.size();
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
		std::vector<Template> scaled_templates;
		std::vector<cv::Point> coords;
		std::vector<Orient> orientations;
		cv::Size imgSize; //original image size (scaled)
		cv::Size size_;   //bounding box size (scaled)
		cv::Point center;  //bounding box center
		cv::Point moment;  //original image moment
		double scale;
	public:
		Template(){}

		Template(cv::Mat templ){
			scale = 1.0;
			create(templ);
		}   //OK

		
		Template(const Template &t){
			scaled_templates = t.scaled_templates;
			coords = t.coords;
			orientations = t.orientations;
			size_ = t.size_;
			imgSize = t.imgSize;
			center = t.center;
			scale = t.scale;
			moment = t.moment;
		}   //OK

		//if scaled size doesn't exist then add a new one and return template&
		Template &resize(double new_scale){
			if (fabs(scale - new_scale)<1e-6) return *this;

			for (size_t i = 0; i<scaled_templates.size(); ++i) {
				if (fabs(scaled_templates[i].scale - new_scale)<1e-6) {
					return scaled_templates[i];
				}
			}

			double scale_factor = new_scale / scale;

			Template tpl;
			tpl.scale = new_scale;

			tpl.center.x = (int)(center.x*scale_factor + 0.5);
			tpl.center.y = (int)(center.y*scale_factor + 0.5);

			tpl.moment.x = (int)(moment.x*scale_factor + 0.5);
			tpl.moment.y = (int)(moment.y*scale_factor + 0.5);

			tpl.size_.width = (int)(size_.width*scale_factor + 0.5);
			tpl.size_.height = (int)(size_.height*scale_factor + 0.5);

			tpl.imgSize.width = (int)(imgSize.width*scale_factor + 0.5);
			tpl.imgSize.height = (int)(imgSize.height*scale_factor + 0.5);

			tpl.coords.resize(coords.size());
			tpl.orientations.resize(orientations.size());
			for (size_t i = 0; i<coords.size(); ++i) {
				tpl.coords[i].x = (int)(coords[i].x*scale_factor + 0.5);
				tpl.coords[i].y = (int)(coords[i].y*scale_factor + 0.5);
				tpl.orientations[i] = orientations[i];
			}
			scaled_templates.push_back(tpl);

			return scaled_templates[scaled_templates.size()-1];
		}

		//copy this template (without scaled templates)
		Template copy(){
			Template t;
			t.coords = coords;
			t.orientations = orientations;
			t.size_ = size_;
			t.imgSize = imgSize;
			t.center = center;
			t.scale = scale;
			t.moment = moment;
			return t;
		}

		//clone this template (with scaled templates)
		Template clone(){
			return *this;
		}

		//output = CV_8UC3
		void show(cv::Mat &output, cv::Vec3b &color = cv::Vec3b(0, 255, 0)){
			output = cv::Mat::zeros(imgSize, CV_8UC3);
			for (int i = 0; i < coords.size(); i++){
				output.at<cv::Vec3b>(coords[i].y + center.y, coords[i].x + center.x) = color;
			}
		}

		//output = CV_8UC1
		void show(cv::Mat &output, uchar color = 255){
			output = cv::Mat::zeros(imgSize, CV_8UC1);
			for (int i = 0; i<coords.size(); i++){
				output.at<uchar>(coords[i].y + center.y, coords[i].x + center.x) = color;
			}
		}

		void showboundingBox(cv::Mat &output, uchar color = 255){
			output = cv::Mat::zeros(size_, CV_8UC1);
			for (int i = 0; i<coords.size(); i++){
				output.at<uchar>(coords[i].y + size_.height/2, coords[i].x + size_.width/2) = color;
			}
		}

		//get specific template by index
		Template &get(int idx){
			return scaled_templates[idx];
		}

		//define center = (0,0)
		std::vector<cv::Point> &getCoords(){
			return coords;
		}

		//image coordinates
		std::vector<cv::Point> getPoints(){
			std::vector<cv::Point> p;
			for (int i = 0; i < coords.size(); i++){
				p.push_back(cv::Point(coords[i].y + center.y, coords[i].x + center.x));
			}
			return p;
		}

		//get orientations of all points
		std::vector<Orient> &getOrientations(){
			return orientations;
		}

		//original image size
		cv::Size &size(){   
			return imgSize;
		}

		//bounding box size
		cv::Size &getSize(){   
			return size_;
		}

		//get center coordinate
		cv::Point &getCenter(){
			return center;
		}

		double getScale(){
			return scale;
		}

		//initialize but no release memory
		void clear(){
			coords.clear();
			orientations.clear();
			scaled_templates.clear();
			imgSize = cv::Size(0, 0);
			size_ = cv::Size(0, 0);
			center = cv::Point(0, 0);
			moment = cv::Point(0, 0);
		}

		//release memory
		void release(){
			std::vector<cv::Point>().swap(coords);
			std::vector<Orient>().swap(orientations);
			std::vector<Template>().swap(scaled_templates);
		}

		//create template
		void create(cv::Mat &t){   //OK
			cv::Mat templ = t.clone();
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

			moment = cv::Point(0, 0);
			for (size_t i = 0; i<coords.size(); ++i) {
				moment.x += coords[i].x;
				moment.y += coords[i].y;

				if (min.x>coords[i].x) min.x = coords[i].x;
				if (min.y>coords[i].y) min.y = coords[i].y;
				if (max.x<coords[i].x) max.x = coords[i].x;
				if (max.y<coords[i].y) max.y = coords[i].y;
			}

			size_.width = max.x - min.x;
			size_.height = max.y - min.y;
			int coords_size = (int)coords.size();

			moment.x /= MAX(coords_size, 1);
			moment.y /= MAX(coords_size, 1);

			center.x = (max.x + min.x) / 2;
			center.y = (max.y + min.y) / 2;

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

	public:
		class MatcherConfig{
			friend class Matcher;
		private:
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
			MatcherConfig(double templScale = 1, int maxMatches = 20, double minMatchDistance = 20,
				int padX = 3, int padY = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6,
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

			void set(double templScale = 1, int maxMatches = 20, double minMatchDistance = 20,
				int padX = 3, int padY = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6,
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

			void setTemplScale(double ts){
				templScale_ = ts;
			}

			void setMaxMatches(int m){
				maxMatches_ = m;
			}

			void setMinMatchDistance(double m){
				minMatchDistance_ = m;
			}

			void setPadX(int p){
				padX_ = p;
			}

			void setPadY(int p){
				padY_ = p;
			}

			void setScales(int s){
				scales_ = s;
			}

			void setMinScale(double s){
				minScale_ = s;
			}

			void setMaxScale(double s){
				maxScale_ = s;
			}

			void setOrientationWeight(double o){
				orientationWeight_ = o;
			}
			void setTruncate(double t){
				truncate_ = t;
			}
		};
	private:

		std::vector<Template> templates;   //rotated templ
		MatcherConfig matcherconfig;

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
		class SlidingWindow{  //OK
			friend class Matcher;
		private:
			bool has_next_ = true;
			cv::Point cur_point_ = cv::Point(-1,-1);
			double cur_scale_ = 0;

			double scale_step_ = 0;
			int scaled_time_ = 0;

			cv::Point upper_bound_ = cv::Point(-1,-1);
			cv::Point lower_bound_ = cv::Point(-1,-1);

			int x_step_ = 3;
			int y_step_ = 3;
			int scales_ = 5;
			double minScale_ = 0.6;
			double maxScale_ = 1.6;

		public:

			SlidingWindow(){ has_next_ = false; }

			SlidingWindow(const SlidingWindow &sw){
				upper_bound_ = sw.upper_bound_;
				lower_bound_ = sw.lower_bound_;
				x_step_ = sw.x_step_;
				y_step_ = sw.y_step_;
				scales_ = sw.scales_;
				minScale_ = sw.minScale_;
				maxScale_ = sw.maxScale_;
				cur_point_ = sw.cur_point_;

				scaled_time_ = sw.scaled_time_;
				cur_scale_ = sw.cur_scale_;
				scale_step_ = sw.scale_step_;
			}

			//define  [lower_bound, upper_bound)
			SlidingWindow(cv::Point lower_bound, cv::Point upper_bound, int x_step=3, int y_step=3, int scales=5, double minScale=0.6, double maxScale=1.6){
				upper_bound_ = upper_bound;
				lower_bound_ = lower_bound;
				x_step_ = x_step;
				y_step_ = y_step;
				scales_ = scales;
				minScale_ = minScale;
				maxScale_ = maxScale;
				cur_point_ = lower_bound_;

				scaled_time_ = 0;
				cur_scale_ = minScale;
				scale_step_ = (maxScale - minScale) / scales;
			}

			//define   [0, imageSize)
			SlidingWindow(cv::Size imageSize, int x_step = 3, int y_step = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6){
				lower_bound_ = cv::Point(0, 0);
				upper_bound_ = cv::Point(imageSize.width, imageSize.height);
				x_step_ = x_step;
				y_step_ = y_step;
				scales_ = scales;
				minScale_ = minScale;
				maxScale_ = maxScale;
				cur_point_ = lower_bound_;

				scaled_time_ = 0;
				cur_scale_ = minScale;
				scale_step_ = (maxScale - minScale) / scales;
			}

			void set(cv::Point lower_bound, cv::Point upper_bound, int x_step = 3, int y_step = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6){
				has_next_ = true;
				upper_bound_ = upper_bound;
				lower_bound_ = lower_bound;
				x_step_ = x_step;
				y_step_ = y_step;
				scales_ = scales;
				minScale_ = minScale;
				maxScale_ = maxScale;
				cur_point_ = lower_bound_;

				scaled_time_ = 0;
				cur_scale_ = minScale;
				scale_step_ = (maxScale - minScale) / scales;
			}

			void set(cv::Size imageSize, int x_step = 3, int y_step = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6){
				has_next_ = true;
				lower_bound_ = cv::Point(0, 0);
				upper_bound_ = cv::Point(imageSize.width, imageSize.height);
				x_step_ = x_step;
				y_step_ = y_step;
				scales_ = scales;
				minScale_ = minScale;
				maxScale_ = maxScale;
				cur_point_ = lower_bound_;

				scaled_time_ = 0;
				cur_scale_ = minScale;
				scale_step_ = (maxScale - minScale) / scales;
			}

			bool hasNext(){
				return has_next_;
			}  //OK

			//restart (init)
			void re(){
				cur_point_ = lower_bound_;
				cur_scale_ = minScale_;
				scaled_time_ = 0;
			}

			//sliding window return current point and window size
			//if it does not have next then will return point(-1,-1) and hasNext will return false;
			std::pair<cv::Point, double> next(){
				std::pair<cv::Point, double> next_p = std::make_pair(cur_point_, cur_scale_);
				cur_point_.x += x_step_;

				if (cur_point_.x >= upper_bound_.x){
					cur_point_.x = 0;
					cur_point_.y += y_step_;

					if (cur_point_.y >= upper_bound_.y){
						cur_point_.y = 0;
						cur_scale_ += scale_step_;
						scaled_time_++;

						if (scaled_time_ == scales_){
							has_next_ = false;
							scaled_time_ = 0;
							cur_scale_ = minScale_;
							cur_point_ = cv::Point(-1, -1);

						}
					}
				}

				return next_p;
			}  //OK
		};

	public:
		class MatchPoint{
			friend class Matcher;
		private:
			cv::Point point_;  //template center
			double costs_;
			Template tp;

		public:
			MatchPoint(){}

			MatchPoint(Template &match_template, cv::Point point, double costs){
				tp = match_template.copy();
				point_ = point;
				costs_ = costs;
			}

			MatchPoint(const MatchPoint &m){
				point_ = m.point_;
				costs_ = m.costs_;
				tp = m.tp;
			}

			void set(Template &match_template, cv::Point point, double costs){
				tp = match_template.copy();
				point_ = point;
				costs_ = costs;
			}

			cv::Point point(){
				return point_;
			}

			double costs(){
				return costs_;
			}

			void setCosts(double costs){
				costs_ = costs;
			}

			Template &getTemplate(){
				return tp;
			}

			//define center = (0,0)
			std::vector<cv::Point> getMatchCoords(){
				return tp.getCoords();
			}

			//image coodinate
			std::vector<cv::Point> getMatchPoints(){
				return tp.getPoints();
			}

			//output = CV_8UC3
			void showMatch(cv::Mat &output, cv::Vec3b &color = cv::Vec3b(0,255,0)){
				std::vector<cv::Point> p = tp.getPoints();
				for (int i = 0; i < p.size(); i++){
					output.at<cv::Vec3b>(p[i].x, p[i].y) = color;
				}
			}

			//output = CV_8UC1
			void showMatch(cv::Mat &output, uchar color = 255){
				std::vector<cv::Point> p = tp.getPoints();
				for (int i = 0; i < p.size(); i++){
					output.at<uchar>(p[i].x, p[i].y) = 255;
				}
			}

			MatchPoint copy(){
				MatchPoint m;
				m.point_ = point_;
				m.costs_ = costs_;
				m.tp = tp.copy();
				return m;
			}

		};

	private:
		std::vector<MatchPoint> matchpoints;

		SlidingWindow slidingwindow;
		bool slidingwindowExists = false;

		bool createSlidingWindow(cv::Point lower_bound, cv::Point upper_bound){

			slidingwindow = SlidingWindow(lower_bound, upper_bound, padX_, padY_, scales_, minScale_, maxScale_);

			return true;
		}

	public:
		Matcher(double templScale = 1, int maxMatches = 20, double minMatchDistance = 1.0, int padX = 3,
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

		void init(double templScale = 1, int maxMatches = 20, double minMatchDistance = 1.0, int padX = 3,
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

		void clear(){
			templates.clear();
		}

		void release(){
			std::vector<Template>().swap(templates);
			std::vector<MatchPoint>().swap(matchpoints);
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

		std::vector<MatchPoint> &getMatchPoints(){
			return matchpoints;
		}

		void releaseMatchPoint(){
			std::vector<MatchPoint>().swap(matchpoints);
		}

		private:
			static bool sortingFunction(MatchPoint &a, MatchPoint &b){
				return a.costs() < b.costs();
			}
		public:

		void filter(){
			std::vector<MatchPoint> matches;
			matches.resize(maxMatches_);

			int match_count = 0;

			std::sort(matchpoints.begin(), matchpoints.end(), ending::Matcher::sortingFunction);

			for (int i = 0; i < matchpoints.size(); i++){
				int j = 0;
				for (j = 0; j < match_count; j++){
					if (std::abs(matchpoints[i].point().x - matches[j].point().x) + std::abs(matchpoints[i].point().y - matches[j].point().y) < minMatchDistance_)break;
				}
				if (j >= match_count){
					matches[match_count] = matchpoints[i];
					match_count++;
				}
				if (match_count >= maxMatches_)break;
			}

			/*

			for (int i = 0; i < matchpoints.size(); i++){

				bool real_match = true;
				for (int j = 0; j < match_count; j++){
					if (std::abs(matchpoints[i].point().x - matches[j].point().x) + std::abs(matchpoints[i].point().y - matches[j].point().y) < minMatchDistance_){
						real_match = false;
						if (matchpoints[i].costs() < matches[j].costs()){
							matches[j].setCosts(matchpoints[i].costs());
						}

						int k = j;
						while (k>0) {
							if (matches[k - 1].costs() > matches[k].costs()) {
								std::swap(matches[k - 1], matches[k]);
							}
							k--;
						}
						break;
					}
				}

				if (real_match){
					if (match_count < maxMatches_){
						matches[match_count] = matchpoints[i];
						match_count++;
					}
					else{
						if (matches[match_count - 1].costs() < matchpoints[i].costs()){
							break;
						}

						int n = 0;
						while (matches[n].costs() < matchpoints[i].costs())n++;

						int k = match_count - 2;
						while (k >= n){
							matches[k + 1] = matches[k];
							k--;
						}
						matches[n] = matchpoints[i];
					}

				}  //if



			}  //for*/

			matchpoints.resize(matches.size());
			for (int i = 0; i < matches.size(); i++){
				matchpoints[i] = matches[i].copy();
			}
		}

		MatchPoint *localmatching(cv::Point &loc, Template &tp, cv::Mat &dist_img, cv::Mat &orient_img){

			double alpha = orientationWeight_;
			double beta = 1 - alpha;

			double dist_cost = 0;
			double orient_cost = 0;
			std::vector<cv::Point> &p = tp.getCoords();
			std::vector<Orient> &o = tp.getOrientations();

			int valid_orient = 0;

			for (int i = 0; i < p.size(); i++){
				dist_cost += (double)dist_img.at<float>(loc.y + p[i].y, loc.x + p[i].x);
				double ori = (double)orient_img.at<float>(loc.y + p[i].y, loc.x + p[i].x);
				if (o[i] >= -CV_PI && ori >= -CV_PI){
					orient_cost += fabs(ori-o[i]);
					valid_orient++;
				}
			}

			double cost = (dist_cost / truncate_) / p.size();

			if (valid_orient > 0){
				cost = (beta*cost + alpha*(orient_cost / (2 * CV_PI)) / valid_orient);
			}
			if (cost == 0.0){
				std::cout << "error" << std::endl;
			}
			if (cost > 0){
				MatchPoint *mp = new MatchPoint(tp, loc, cost);
				return mp;
			}

			return NULL;
		}

		//if your dist_img size or your matching range is same as last time you matching then you can try redefine_range=false to promote your matching speed
		size_t matching(cv::Mat &dist_img, cv::Mat &orientation_img, bool redefine_range = true){
			cv::Size sz = dist_img.size();
			if (slidingwindowExists == false || redefine_range == true)slidingwindowExists = createSlidingWindow(cv::Point(0,0), cv::Point(sz.width, sz.height));

			for (int t_num = 0; t_num < templates.size(); t_num++){
				while (slidingwindow.hasNext()){
					std::pair<cv::Point, double> cur = slidingwindow.next();

					cv::Point p = cur.first;
					double s = cur.second;

					Template &tp = templates[t_num].resize(s);

					cv::Size tc = tp.getSize();
					if (p.x - tc.width / 2 < 0 || p.x + tc.width / 2 >= sz.width)continue;
					if (p.y - tc.height / 2 < 0 || p.y + tc.height / 2 >= sz.height)continue;

					std::vector<cv::Point> tt = tp.getCoords();

					MatchPoint *mp = localmatching(p, tp, dist_img, orientation_img);
					if (mp != NULL){
						matchpoints.push_back((*mp));
						delete mp;
					}
				}
			}

			filter();

			return matchpoints.size();
		}

		//if your dist_img size or your matching range is same as last time you matching then you can try redefine_range=false to promote your matching speed
		size_t matching(cv::Mat &dist_img, cv::Mat &orientation_img, cv::Point lower_bound, cv::Point upper_bound, bool redefine_range = true){
			if (slidingwindowExists == false || redefine_range == true)slidingwindowExists = createSlidingWindow(lower_bound, upper_bound);

			for (int t_num = 0; t_num < templates.size(); t_num++){
				while (slidingwindow.hasNext()){
					std::pair<cv::Point, double> cur = slidingwindow.next();

					cv::Point p = cur.first;
					double s = cur.second;

					Template &tp = templates[t_num].resize(s);

					cv::Size tc = tp.getSize();
					if (p.x - tc.width / 2 < lower_bound.x || p.x + tc.width / 2 > upper_bound.x)continue;
					if (p.y - tc.height / 2 < lower_bound.y || p.y + tc.height / 2 > upper_bound.y)continue;

					MatchPoint *mp = localmatching(p, tp, dist_img,orientation_img);
					if (mp != NULL){
						matchpoints.push_back(*mp);
						delete mp;
					}
				}
			}

			filter();

			return matchpoints.size();
		}
	};


	






	/********* ChamferMatcher *********/


	class ChamferMatcher{
	private:
		std::vector<Matcher> matchers;
		cv::Mat distimg;
		cv::Mat orientimg;

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

		bool matcherExist = false;

		

	public:
		ChamferMatcher(bool use_orientation = true, double templScale = 1, int maxMatches = 20, double minMatchDistance = 1.0, int padX = 3,
			int padY = 3, int scales = 5, double minScale = 0.6, double maxScale = 1.6,
			double orientationWeight = 0.5, double truncate = 20){
			use_orientation_ = use_orientation;
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

	public:

		void createDistanceTransform(cv::Mat& edges_img, cv::Mat& dist_img, cv::Mat& annotate_img, float truncate_dt, float a = 1.0, float b = 1.5);  //OK
		void createEdgeOrientationMap(cv::Mat& edge_img, cv::Mat& orientation_img);  //OK

		void createMaps(cv::Mat& edge_img, cv::Mat &dis_img, cv::Mat &ont_img);  //OK
		void fillNonContourOrientations(cv::Mat& annotated_img, cv::Mat& orientation_img);  //OK

		int matching(cv::Mat& img, cv::Mat& templ, std::vector<std::vector<cv::Point>>& results, std::vector<float>& costs, bool rebuild_matcher = true);
		int matching(cv::Mat& dist_img, cv::Mat& orient_img, cv::Mat& templ, std::vector<std::vector<cv::Point>>& results, std::vector<float>& costs, bool rebuild_matcher = true);
		int matching(std::vector<std::vector<cv::Point>>& results, std::vector<float>& costs);

		std::vector<int> multimatching(cv::Mat& img, std::vector<std::vector<std::vector<cv::Point>>>& results, std::vector<std::vector<float>>& costs);
		std::vector<int> multimatching(cv::Mat& dist_img, cv::Mat& orient_img, std::vector<std::vector<std::vector<cv::Point>>>& results, std::vector<std::vector<float>>& costs);
		std::vector<int> multimatching(std::vector<std::vector<std::vector<cv::Point>>>& results, std::vector<std::vector<float>>& costs);
		
		void clear(){
			matchers.clear();
			matcherExist = false;
		}  //OK

		void release(){
			std::vector<Matcher>().swap(matchers);
			distimg.release();
			orientimg.release();
			matcherExist = false;
		}  //OK
	};

	void ChamferMatcher::setMatcher(Matcher &m){
		matchers.clear();
		matchers.push_back(m);
		matcherExist = true;
	}  //OK

	void ChamferMatcher::setMatcher(cv::Mat &templ){
		Matcher m(templScale_, maxMatches_, minMatchDistance_, padX_,
			padY_, scales_, minScale_, maxScale_,
			orientationWeight_);
		m.setTemplate(templ);
		matchers.clear();
		matchers.push_back(m);
		matcherExist = true;
	}  //OK

	size_t ChamferMatcher::addMatcher(Matcher &m){
		matchers.push_back(m);
		matcherExist = true;
		return matchers.size() - 1;
	}  //OK

	size_t ChamferMatcher::addMatcher(cv::Mat &templ){
		Matcher m(templScale_, maxMatches_, minMatchDistance_, padX_,
			padY_, scales_, minScale_, maxScale_,
			orientationWeight_);
		m.setTemplate(templ);
		matchers.push_back(m);
		matcherExist = true;
		return matchers.size() - 1;
	}  //OK

	void ChamferMatcher::setMatchers(std::vector<Matcher> &m){
		matchers.clear();
		matchers = m;
		matcherExist = true;
	}  //OK

	void ChamferMatcher::setMatchers(std::vector<cv::Mat> &templ){
		setMatcher(templ[0]);
		for (int i = 1; i < templ.size(); i++){
			addMatcher(templ[i]);
		}
		matcherExist = true;
	}  //OK

	size_t ChamferMatcher::addMatchers(std::vector<Matcher> &m){
		size_t t = matchers.size();
		for (int i = 0; i < m.size(); i++){
			matchers.push_back(m[0]);
		}
		matcherExist = true;
		return t;
	}  //OK

	size_t ChamferMatcher::addMatchers(std::vector<cv::Mat> &templ){
		size_t t = matchers.size();
		for (int i = 0; i < templ.size(); i++){
			addMatcher(templ[i]);
		}
		matcherExist = true;
		return t;
	}  //OK

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
	}  //OK

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
	}  //OK

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
	}  //OK

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
	}  //OK

	//add template
	int ChamferMatcher::matching(cv::Mat& img, cv::Mat& templ, std::vector<std::vector<cv::Point>>& results, std::vector<float>& costs, bool rebuild_matcher){
		createMaps(img.clone(), distimg, orientimg);

		Matcher *m;
		bool releaseMatcher = false;
		if (matcherExist == false || rebuild_matcher == true || matchers.size() <= 0){
			m = new Matcher(templScale_, maxMatches_, minMatchDistance_, padX_, padY_, scales_, minScale_, maxScale_, orientationWeight_, truncate_);
			releaseMatcher = true;
		}
		else{
			m = &matchers[0];
		}
		m->setTemplate(templ);
		int num = (int)m->matching(distimg, orientimg);


		results.resize(num);
		costs.resize(num);

		int bestIdx = -1;
		double minCost = DBL_MAX;

		std::vector<Matcher::MatchPoint> &matches = m->getMatchPoints();

		for (int i = 0; i < num; i++)
		{
			Matcher::MatchPoint& match = matches[i];
			double cval = match.costs();
			if (cval < minCost)
			{
				minCost = cval;
				bestIdx = (int)i;
			}
			costs[i] = (float)cval;

			std::vector<cv::Point> coords = match.getTemplate().getCoords();
			cv::Point off = match.point();
			

			for (int j = 0; j < coords.size(); j++){
				coords[j].x += off.x;
				coords[j].y += off.y;
			}

			coords.insert(coords.begin(), off);

			results[i] = coords;

		}

		distimg.release();
		orientimg.release();

		if (releaseMatcher){
			delete m;
		}

		return bestIdx;
	}

	int ChamferMatcher::matching(cv::Mat& dist_img, cv::Mat& orient_img, cv::Mat& templ, std::vector<std::vector<cv::Point>>& results, std::vector<float>& costs, bool rebuild_matcher){
		distimg = dist_img.clone();
		orientimg = orient_img.clone();

		Matcher *m;
		bool releaseMatcher = false;

		if (matcherExist == false || rebuild_matcher == true || matchers.size() <= 0){
			m = new Matcher(templScale_, maxMatches_, minMatchDistance_, padX_, padY_, scales_, minScale_, maxScale_, orientationWeight_, truncate_);
			releaseMatcher = true;
		}
		else{
			m = &matchers[0];
		}

		m->setTemplate(templ);
		int num = (int)m->matching(distimg, orientimg);

		results.resize(num);
		costs.resize(num);

		int bestIdx = -1;
		double minCost = DBL_MAX;

		std::vector<Matcher::MatchPoint> &matches = m->getMatchPoints();

		for (int i = 0; i < num; i++)
		{
			Matcher::MatchPoint& match = matches[i];
			double cval = match.costs();
			if (cval < minCost)
			{
				minCost = cval;
				bestIdx = (int)i;
			}
			costs[i] = (float)cval;

			std::vector<cv::Point> coords = match.getTemplate().getCoords();
			cv::Point off = match.point();

			for (int j = 0; j < coords.size(); j++){
				coords[j].x += off.x;
				coords[j].y += off.y;
			}

			coords.insert(coords.begin(), off);
			
			results[i] = coords;

		}

		distimg.release();
		orientimg.release();

		if (releaseMatcher){
			delete m;
		}

		return bestIdx;
	}

	int ChamferMatcher::matching(std::vector<std::vector<cv::Point>>& results, std::vector<float>& costs){
		
		if (matchers.size() <= 0){
			std::cout << "No matcher found..." << std::endl;
			return -1;
		}

		Matcher *m = &matchers[0];


		int num = (int)m->matching(distimg, orientimg);

		results.resize(num);
		costs.resize(num);

		int bestIdx = -1;
		double minCost = DBL_MAX;

		std::vector<Matcher::MatchPoint> &matches = m->getMatchPoints();

		for (int i = 0; i < num; i++)
		{
			Matcher::MatchPoint& match = matches[i];
			double cval = match.costs();
			if (cval < minCost)
			{
				minCost = cval;
				bestIdx = (int)i;
			}
			costs[i] = (float)cval;

			std::vector<cv::Point> coords = match.getTemplate().getCoords();
			cv::Point off = match.point();

			for (int j = 0; j < coords.size(); j++){
				coords[j].x += off.x;
				coords[j].y += off.y;
			}

			coords.insert(coords.begin(), off);

			results[i] = coords;

		}

		distimg.release();
		orientimg.release();

		return bestIdx;
	}


	std::vector<int> ChamferMatcher::multimatching(cv::Mat& img, std::vector<std::vector<std::vector<cv::Point>>>& results, std::vector<std::vector<float>>& costs){
		createMaps(img.clone(), distimg, orientimg);

		std::vector<int> bestIdx(matchers.size(), -1);
		results.resize(matchers.size());
		costs.resize(matchers.size());

		for (int i = 0; i < matchers.size(); i++){
			int best = -1;
			double minCost = DBL_MAX;

			int num = (int)matchers[i].matching(distimg, orientimg);

			results[i].resize(num);
			costs[i].resize(num);

			std::vector<Matcher::MatchPoint> &matches = matchers[i].getMatchPoints();

			for (int j = 0; j < num; j++){
				Matcher::MatchPoint& match = matches[j];
				double cval = match.costs();
				if (cval < minCost)
				{
					minCost = cval;
					best = (int)j;
				}
				costs[i][j] = (float)cval;

				std::vector<cv::Point> coords = match.getTemplate().getCoords();
				cv::Point off = match.point();


				for (int n = 0; n < coords.size(); n++){
					coords[n].x += off.x;
					coords[n].y += off.y;
				}

				coords.insert(coords.begin(), off);

				results[i][j] = coords;
			}

			bestIdx[i] = best;
		}

		distimg.release();
		orientimg.release();

		return bestIdx;
	}

	std::vector<int> ChamferMatcher::multimatching(cv::Mat& dist_img, cv::Mat& orient_img, std::vector<std::vector<std::vector<cv::Point>>>& results, std::vector<std::vector<float>>& costs){
		distimg = dist_img;
		orientimg = orient_img;

		std::vector<int> bestIdx(matchers.size(), -1);
		results.resize(matchers.size());
		costs.resize(matchers.size());

		for (int i = 0; i < matchers.size(); i++){
			int best = -1;
			double minCost = DBL_MAX;

			int num = (int)matchers[i].matching(distimg, orientimg);

			results[i].resize(num);
			costs[i].resize(num);

			std::vector<Matcher::MatchPoint> &matches = matchers[i].getMatchPoints();

			for (int j = 0; j < num; j++){
				Matcher::MatchPoint& match = matches[j];
				double cval = match.costs();
				if (cval < minCost)
				{
					minCost = cval;
					best = (int)j;
				}
				costs[i][j] = (float)cval;

				std::vector<cv::Point> coords = match.getTemplate().getCoords();
				cv::Point off = match.point();


				for (int n = 0; n < coords.size(); n++){
					coords[n].x += off.x;
					coords[n].y += off.y;
				}

				coords.insert(coords.begin(), off);

				results[i][j] = coords;
			}

			bestIdx[i] = best;
		}

		distimg.release();
		orientimg.release();

		return bestIdx;
	}

	std::vector<int> ChamferMatcher::multimatching(std::vector<std::vector<std::vector<cv::Point>>>& results, std::vector<std::vector<float>>& costs){
		std::vector<int> bestIdx(matchers.size(), -1);
		results.resize(matchers.size());
		costs.resize(matchers.size());

		for (int i = 0; i < matchers.size(); i++){
			int best = -1;
			double minCost = DBL_MAX;

			int num = (int)matchers[i].matching(distimg, orientimg);

			results[i].resize(num);
			costs[i].resize(num);

			std::vector<Matcher::MatchPoint> &matches = matchers[i].getMatchPoints();

			for (int j = 0; j < num; j++){
				Matcher::MatchPoint& match = matches[j];
				double cval = match.costs();
				if (cval < minCost)
				{
					minCost = cval;
					best = (int)j;
				}
				costs[i][j] = (float)cval;

				std::vector<cv::Point> coords = match.getTemplate().getCoords();
				cv::Point off = match.point();


				for (int n = 0; n < coords.size(); n++){
					coords[n].x += off.x;
					coords[n].y += off.y;
				}

				coords.insert(coords.begin(), off);

				results[i][j] = coords;
			}

			bestIdx[i] = best;
		}

		distimg.release();
		orientimg.release();

		return bestIdx;
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

		ChamferMatcher cm(true, templScale, maxMatches, minMatchDistance, padX, padY, scales, minScale, maxScale, orientationWeight, truncate);
		cm.setMatcher(templ);


		return 0;
	}

}