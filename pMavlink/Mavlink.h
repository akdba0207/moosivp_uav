/*****************************************************************/
/*    NAME: Dongbin Kim                                          */
/*    ORGN: University of Hartford                             */
/*    (based on pMavlink by  David Battle @ Mission Systems Pty Ltd  */
/*    FILE: translation.cpp                                      */
/*    DATE: 21 July 2024                                          */
/*****************************************************************/
#ifndef Mavlink_HEADER
#define Mavlink_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "mavlink.h" // mavlink library

class Mavlink : public CMOOSApp
{
 public:
  Mavlink();
  ~Mavlink();

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

  CMOOSGeodesy m_geodesy;
  bool         m_geo_ok;

  bool m_verbose_output{false};

protected:
    void RegisterVariables();
    void TranslateToMoos(mavlink_message_t* msg);
    void TranslateToMavlink(CMOOSMsg &moos_msg);
    double getCorrectedUnixTimestamp(uint32_t boot_time_ms);

private: // State variables
    unsigned char* m_buf;
    unsigned char* m_buf_tx;
    uint8_t        m_system_id;
    uint8_t        m_component_id;
    uint8_t        m_target_system;
    uint8_t        m_target_component;
    uint8_t        m_coordinate_frame;
    float m_desired_altitude;
    CMOOSMsg       rc_msg;
    bool           send_rc= 0;
    // System time
    double m_boot_pxunix_offset_ms; // very stable, resets on connect to GPS so should not average/filter
    double m_moosunix_pxunix_offset_s;
    double m_moosunix_pxunix_offset_average_s;
    int m_number_moosunix_pxunix_offset_s_receive{0};
    int m_number_moosunix_pxunix_offset_s_to_ignore{10};
    bool initial_sys_time_received{false};

    // PX4 Data time of arrival
    double m_moosTimePX4DATA{MOOSTime()};

    // Ping sequencing
    uint64_t m_ping_average_us{0};
    int m_number_pings_received{0};
    int m_number_pings_to_ignore{10};
    uint64_t m_ping_max_us{0};
    uint64_t m_ping_min_us{0};

    // Retain index info for debugging
    int next_msg_idx{0};
    // Time average for debugging
    double m_timesync_delay_average_ms;
    int m_number_timesync_receive{0};
    int m_number_timesyncs_to_ignore{10};
    double m_largest_diff{0};
    double m_smallest_diff;
    float yaw, yaw_rate;
    float m_desired_speed;
    float m_desired_heading_indeg;
    float m_desired_heading_inrad;
    float vx, vy, vz;
};

#endif
