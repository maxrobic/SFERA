#ifndef EVENT3D_H
#define EVENT3D_H
// MIS Laboratory, University of Picardy Jules Verne
// Author: Maxime Robic maxime.robic@u-picardie.fr
#include "metavision/sdk/base/events/event2d.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>

namespace Metavision {

/// @brief Basic extension class to represent 3D events:
class Event3d : public Event2d {
public:
    ///@brief X position in the Cartesian frame
    double X;
    ///@brief Y position in the Cartesian frame
    double Y;
    ///@brief Z position in the Cartesian frame
    double Z;

    /// @brief Default constructor
    Event3d() = default;

    /// @brief Constructor from Event2d
    inline Event3d(const Event2d &ev) : Event2d(ev) {}
    using Event2d::Event2d;

    /// @brief Constructor from existing events adding 3D coordinates
    /// @param ev Existing 2D event
    /// @param X X position in the Cartesian frame
    /// @param Y Y position in the Cartesian frame
    /// @param Z Z position in the Cartesian frame
    inline Event3d(const Event2d &ev, double X, double Y, double Z) {
        this->x=ev.x;
        this->y=ev.y;
        this->p=ev.p;
        this->t=ev.t;
        this->X=X;
        this->Y=Y;
        this->Z=Z;
    }

    /// @brief Constructor from scratch
    /// @param x Column position of the event in the sensor
    /// @param y Row position of the event in the sensor
    /// @param p Polarity specialising the event
    /// @param t Timestamp of the event (in us)
    /// @param X X position in the Cartesian frame
    /// @param Y Y position in the Cartesian frame
    /// @param Z Z position in the Cartesian frame
    inline Event3d(unsigned short x, unsigned short y, short p, timestamp t, double X, double Y, double Z) {
        this->x=x;
        this->y=y;
        this->p=p;
        this->t=t;
        this->X=X;
        this->Y=Y;
        this->Z=Z;
    }
    
    /// @brief return the bearing vector of an event in the unit sphere (rho=1) Warning: spherical coordinates projected from image frame, i.e X-rotated from 90°
    inline cv::Vec2f getBearingUnit() {
        cv::Vec2f bearing;
        double alpha;
        double beta;
        if(this->Y == 1 || this-> Y == -1){
            std::cout<<"Bearing undefined for this coordinate (Z=1 or Z=-1)"<<std::endl;
            return 0;
        }
        else{
        beta=asin(-Y);
        alpha=atan2(Z,X);
        bearing[0]=alpha;
        bearing[1]=beta;
        }
        return bearing;

    }
    
    /// @brief return the bearing vector of a 3D event 
    inline cv::Vec3f getBearing() {
        cv::Vec3f bearing;
        double rho = sqrt(powf(this->X,2)+powf(this->Y,2)+powf(this->Z,2));
        double alpha;
        double beta;
        if(this->Z == rho || this-> Z == -rho){
            std::cout<<"Bearing undefined for this coordinate (Z=1 or Z=-1)"<<std::endl;
            return 0;
        }
        else{
        beta=asin(Z/rho);
        alpha=atan2(Y,X);
        bearing[0]=alpha;
        bearing[1]=beta;
        bearing[2]=rho;
        }
        return bearing;

    }

  

};

} // namespace Metavision

//METAVISION_DEFINE_EVENT_TRAIT(Metavision::EventCD, 12, "CD")

#endif // METAVISION_SDK_BASE_EVENT_CD_H

/* @brief Constructor
    /// @param x Column position of the event in the sensor
    /// @param y Row position of the event in the sensor
    /// @param p Polarity specialising the event
    /// @param t Timestamp of the event (in us)
    inline Event2d(unsigned short x, unsigned short y, short p, timestamp t) : x(x), y(y), p(p), t(t) {}*/