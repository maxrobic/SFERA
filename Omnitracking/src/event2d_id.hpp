#ifndef EVENT2D_ID_H
#define EVENT2D_ID_H

#include "metavision/sdk/base/events/event2d.h"

namespace Metavision {

/// @brief Basic extension class to represent 3D events:
class Event2d_id : public Event2d {
public:
    ///@brief 
    short idCam; // 0: master / 1: slave


    /// @brief Default constructor
    Event2d_id() = default;

    /// @brief Constructor from Event2d
    inline Event2d_id(const Event2d &ev) : Event2d(ev) {}
    using Event2d::Event2d;

    /// @brief Constructor from existing events adding 3D coordinates
    /// @param ev Existing 2D event
    /// @param id id of the cam
    inline Event2d_id(const Event2d &ev, short id) {
        this->x=ev.x;
        this->y=ev.y;
        this->p=ev.p;
        this->t=ev.t;
        this->idCam=id;
    }

    /// @brief Constructor from scratch
    /// @param x Column position of the event in the sensor
    /// @param y Row position of the event in the sensor
    /// @param p Polarity specialising the event
    /// @param t Timestamp of the event (in us)
    /// @param id  id of the cam
    inline Event2d_id(unsigned short x, unsigned short y, short p, timestamp t, short id) {
        this->x=x;
        this->y=y;
        this->p=p;
        this->t=t;
        this->idCam=id;
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