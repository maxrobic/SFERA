#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <numeric>

#include "angle.hpp"
//#include "Odometry.hpp"

using namespace cv;

class BayesianFilter
{
private:
  Angle ang_;
  int sample_num_;

  double pdfVMF(Point3d Zk, double k, Point3d x)
    {
    if (k == 0.0)
        return 1.0 / (4.0 * M_PI);
    else
        return k * exp(k * (Zk.dot(x) - 1.0)) / (2.0*M_PI * (1.0 - exp(-2.0 * k)));
    }


  Mat sampleVMF(int n, Point3d u, double k)
    {
    // method for sampling n points from the von Mises-Fisher distribution
    Mat ksi = Mat(n, 1, CV_64F), w = Mat(n, 1, CV_64F);
    time_t t = time(NULL);
    RNG rng(t);
    rng.fill(ksi, RNG::UNIFORM, (double) 0, (double) 1);
    // calculate inversion for distribution of w
    // 1 + (log(ksi) + log(1 - exp(-2*k)*(ksi-1)/ksi)) / k
    subtract(ksi, 1.0, w); divide(w, ksi, w);
    w = w * exp(-2.0 * k);
    subtract(1.0, w, w);
    log(w,w); log(ksi, ksi);
    add(ksi, w, w);
    divide(w, k, w);
    add(1.0, w, w);

    Mat v = Mat(n, 1, CV_64F), v_cos = Mat(n, 1, CV_64F), v_sin = Mat(n, 1, CV_64F);
    t = time(NULL);
    rng(t);
    rng.fill(v, RNG::UNIFORM, (double) 0, (double) 1);
    v = 2.0 * M_PI * v;
    for (int i = 0; i < (int) v.rows; i++)
    {
        v_cos.at<double>(i,0) = cos(v.at<double>(i,0));
        v_sin.at<double>(i,1) = sin(v.at<double>(i,0));
    }

    // calculate ksi = sqrt(1 - w.^2)
    pow(w, 2.0, ksi);
    subtract(1.0, ksi, ksi);
    sqrt(ksi, ksi);
    multiply(ksi, v_cos, v_cos);
    multiply(ksi, v_sin, v_sin);

    Mat samples = Mat(n, 4, CV_64F);
    std::vector<Mat> aux;
    aux.push_back(v_cos);
    aux.push_back(v_sin);
    aux.push_back(w);
    aux.push_back(Mat::ones(n, 1, CV_64F));
    hconcat(aux, samples);
    transpose(samples, samples);

    // the samples are around (0,0,1) and we need to rotate them to be around u
    double phi = atan2(u.y, u.x), theta = acos(u.z);
    Mat R;
    // rotation around y axis
    R = (Mat_<double>(4,4) <<
        cos(theta), 0.0, sin(theta), 0.0,
        0.0, 1.0, 0.0, 0.0,
        -sin(theta), 0.0, cos(theta), 0.0,
        0.0, 0.0, 0.0, 1.0);
    samples = R * samples;
    // rotation around z axis
    R = (Mat_<double>(4,4) <<
        cos(phi), -sin(phi), 0.0, 0.0,
        sin(phi), cos(phi), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0);
    samples = R * samples;

    return samples;
    }

  double A(double k)
    {
    return 1 / tanh(k) - 1 / k;
    }
  double dA(double k)
    {
    double csch = 2.0 / (exp(k) - exp(-k));
    return 1 / (k*k) - csch*csch;
    }
  double Ainv(double y, double guess)
    {
    // Initial guess
    double x = guess, residual = 0.0, deriv;
    // Invert using Newton’s method
    do {
        residual = A(x)-y, deriv = dA(x);
        x -= residual/deriv;
    } while (fabs((float) residual) > 1e-5f);

    return x;
    }
  double convolveVMF(double k1, double k2)
    {
    return Ainv(A(k1) * A(k2), min(k1, k2));
    }

public:
  Mat samples_;
  Point3d est_u_;
  double est_k_;
  bool initialized_=false;
  std::vector<cv::Vec3d> rot_velocities_;
  std::vector<double> dt_vec_;
  int nInit = 0;

  BayesianFilter()
    {
    initialized_ = false;
    }

  ~BayesianFilter()
    {
    }

  void initialize(Point3d u, double k)
    {
    sample_num_ = 300;
    initialized_ = true;
    samples_ = sampleVMF(sample_num_, u, k);
    est_u_ = u;
    est_k_ = k;
   
    std::cout << "Bayesian tracker has been initialized at position XYZ = ( " << est_u_.x<<" , " << est_u_.y << " , "<< est_u_.z << " ) on the sphere"<<std::endl;
    nInit++;
    }

  void predict(/*Odometry& odom,*/ double p_noise_k, std::vector<Point3d> Zk, double dt)
    {
    // calculate the rotational speed of the object
    cv::Vec3d object_th(0.0, 0.0, 0.0);
    double dt_sum = 0.0;

    if (!rot_velocities_.empty())
    {
        cv::Vec3d mean_rot_vel(0.0, 0.0, 0.0);

        //cv::Vec3d total_dt;
        //std::vector<cv::Vec3d> omegadt_;
        //cv::multiply(rot_velocities_, dt_vec_, omegadt_);
        //mean_rot_vel = accumulate(omegadt_.begin(), omegadt_.end(), mean_rot_vel, std::plus<>{});
        //dt_sum = accumulate(dt_vec_.begin(), dt_vec_.end(), dt_sum, std::plus<>{});
         for(int i = 0; i < rot_velocities_.size(); i++){
          mean_rot_vel += rot_velocities_[i]*dt_vec_[i];
          dt_sum += dt_vec_[i];
         }

        mean_rot_vel = mean_rot_vel / dt_sum;

        // calculate the rotation needed due to objects movement
        object_th[0] = mean_rot_vel[0] * dt; //ang_.assure180(mean_rot_vel[0] * dt);
        object_th[1] = mean_rot_vel[1] * dt; //ang_.assure180(mean_rot_vel[1] * dt);
        object_th[2] = mean_rot_vel[2] * dt; //ang_.assure180(mean_rot_vel[2] * dt);

        Mat R = Mat_<double>(3,3);
        cv::Rodrigues(object_th, R);
   

        Mat u_hom = (Mat_<double>(3,1) << est_u_.x, est_u_.y, est_u_.z, 1.0);

        u_hom = R * u_hom;
        est_u_.x = u_hom.at<double>(0,0);
        est_u_.y = u_hom.at<double>(1,0);
        est_u_.z = u_hom.at<double>(2,0);
    }

    // transform the point as if it was on a fixed distance
    //double scale = 0.8;
    //est_u_ = odom.tfToCurrentFrame(scale * est_u_);
   
    // keep the size of the velocity vector bounded
    if (rot_velocities_.size() > 5){
        rot_velocities_.erase(rot_velocities_.begin());
        dt_vec_.erase(dt_vec_.begin());
        }
    if(Zk.empty() && !rot_velocities_.empty())
    {
       // when no measurements remove the oldest velocity
        rot_velocities_.erase(rot_velocities_.begin());
        dt_vec_.erase(dt_vec_.begin());
    }
    // convolution with process noise vMF
    est_k_ = convolveVMF(est_k_, p_noise_k);
    //std::cout<<" K ="<< est_k_ << std::endl; 
    if(est_k_<50){
      initialized_=false;
      std::cout<< " Need to reinitialise " << std::endl;
    }

    samples_ = sampleVMF(sample_num_, est_u_, est_k_);
    }

void update(std::vector<Point3d> Zk_vec, double k, double dt)
    {
    Point3d Zk, u_prev = est_u_;
    double min_dist = 2.0 * M_PI, dot_dist;
    int min_idx = 0;
    if (!Zk_vec.empty())
    {
        // Find the closest measurement to the current object using angle between vectors
        for (int i = 0; i < (int) Zk_vec.size(); i++)
        {
        //        log_map_dist = sqrt(pow(log(est_k_ / k),2) + pow(acos(est_u_.dot(Zk_vec.at(i))), 2));
        double dist_ang = est_u_.dot(Zk_vec.at(i));
        dot_dist = acos(min(max(dist_ang,-1.0),1.0)); // to prevent NaN !
        if (dot_dist < min_dist)
        {
            min_dist = dot_dist;
            min_idx = i;
        }
        }
        // If there are no measurements close to the state
        if (min_dist > ang_.degToRad(25.0)){
          return;
        }
        

        Zk = Zk_vec[min_idx];
        double k_prod = sqrt(pow(est_k_,2) + pow(k,2) + 2.0 * est_k_ * k * est_u_.dot(Zk));
        est_u_ = (est_k_ * est_u_ + k * Zk);
        est_u_ = est_u_ * (1.0 / k_prod);
        est_k_ = k_prod;
        //calculate rotational velocities via theta_u representation
        cv::Vec3d omega;
        double theta;
        cv::Point3d u_axe;
        
        double dist_theta = est_u_.dot(u_prev);
        theta = acos(min(max(dist_theta,-1.0),1.0));
        u_axe = est_u_.cross(u_prev);
        cv::Vec3d theta_u(u_axe.x,u_axe.y,u_axe.z);
        cv::normalize(theta_u);
        theta_u = theta * theta_u;
        omega = theta_u / dt;
        rot_velocities_.push_back(omega);
        dt_vec_.push_back(dt);
    }
    }
};

/*   
            std::cout<<"rot_velocities.size() = " << rot_velocities_.size() << std::endl;
    std::cout <<"dt_vec.size() = "<< dt_vec_.size() <<std::endl;
               std::cout<<"rot_velocities "<< i <<" = "<< rot_velocities_[i] <<std::endl;
          std::cout<<"rot_velocities "<< i <<" = "<< dt_vec_[i] <<std::endl;
     std::cout <<" R = " << R << std::endl;
        std::cout <<" u_hom = " << u_hom << std::endl;
        std::cout <<" mean_rot_vel_before = " << mean_rot_vel << std::endl; 
        std::cout <<" dt_sum = " << mean_rot_vel << std::endl; 
        std::cout <<" mean_rot_vel = " << mean_rot_vel << std::endl; 
        std::cout <<" object_th = " << object_th << std::endl; 
std::cout <<" 
    std::cout<<" K ="<< est_k_ << std::endl; 
est_u bef theta_u ="<< est_u_ <<std::endl;
        std::cout <<" u_prev bef. theta_u ="<< u_prev <<std::endl;
        std::cout <<" est_u . u_prev = " << est_u_.dot(u_prev)<<std::endl;
         std::cout<<"theta = "<< theta << std::endl;
         std::cout<<"theta_u ="<< theta_u << std::endl;
                 std::cout<<"omega ="<< omega << std::endl;
*/