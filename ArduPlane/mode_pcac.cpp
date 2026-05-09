#include "mode.h"
#include "Plane.h"

#define AP_PCAC_BANK_UPDATE_RATE 2
#define AP_PCAC_BANK_PREVIEW_DELAY_DIST 0.0 //m
#define AP_PCAC_BANK_MAX_PREVIEW 30

bool ModePcac::_enter()
{
    plane.custom_control_reset_last = true;
    plane.custom_control_mission_active = true;
    
    //Must enter from auto mode to continue mission
    plane.heading = 0.0;
    plane.flightPathAngle = 0.0;
    plane.pathBankAngle = 0.0;
    plane.navCmdDelta = 0.0;
    plane.navCmdStep = -1;
    return true;
}
void ModePcac::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING && plane.custom_control_mission_active) {
        plane.mission.stop();
        plane.custom_control_mission_active = false;

        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
}

void ModePcac::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in pcac without a running mission");
    }
    else{
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
    }
    
    update_pathBankAngle();
    
    Location p0;
    Vector2f r_cmd_pk;
    AP_Mission::Mission_Command prev_cmd;
    int32_t desired_heading;
    int32_t desired_heading_next;
    int32_t stepsToGo;
    int32_t nav_cmd_step = -1;
    uint32_t prev_cmd_id;
    float nav_cmd_delta = 0.0;
    Vector2f v_proj_to_r;
    Vector2f v_p_wrt_0_in_NED = ahrs.groundspeed_vector();
    
    //Read next waypoint, if it exists, for command preview
    const AP_Mission::Mission_Command& cmd = plane.mission.get_current_nav_cmd();

    // Get current position and velocity
    if (ahrs.get_location(p0) == true) {
        // update _target_bearing_cd
        // _target_bearing_cd = p0.get_bearing_to(next_WP);
        prev_cmd_id = plane.mission.get_prev_nav_cmd_with_wp_index();
        if (prev_cmd_id != AP_MISSION_CMD_INDEX_NONE)
        {
            plane.mission.get_next_nav_cmd(prev_cmd_id, prev_cmd);
            //Determine distance to waypoint
            r_cmd_pk = p0.get_distance_NE(cmd.content.location);
            //r_cmd_pk_hat = normalize(r_cmd_pk)
            //Use ceiling so that first step is always zero command
            v_proj_to_r = v_p_wrt_0_in_NED.projected(r_cmd_pk);
            //Check for correct sign
            if (    (r_cmd_pk + v_proj_to_r).length() >= r_cmd_pk.length()
                &&  (v_proj_to_r.length() > 1)
                &&  (r_cmd_pk.length() + AP_PCAC_BANK_PREVIEW_DELAY_DIST < std::numeric_limits<int32_t>::max()/AP_PCAC_BANK_UPDATE_RATE)
                )
            {
                stepsToGo = ceil((r_cmd_pk.length() + AP_PCAC_BANK_PREVIEW_DELAY_DIST)/v_proj_to_r.length()*AP_PCAC_BANK_UPDATE_RATE);
                if(stepsToGo > 0 && stepsToGo < AP_PCAC_BANK_MAX_PREVIEW){
                    desired_heading = prev_cmd.content.location.get_bearing_to(cmd.content.location);
                    desired_heading_next = plane.mission.get_next_ground_course_cd(-1);
                    if(desired_heading_next != -1){
                        //Get delta angle in radians, wrapped
                        nav_cmd_delta = wrap_PI(radians((float)(desired_heading_next - desired_heading)*0.01));
                        nav_cmd_step = AP_PCAC_BANK_MAX_PREVIEW-stepsToGo;
                    }
                }
            }
        }
    }
    plane.navCmdDelta = nav_cmd_delta;
    plane.navCmdStep = nav_cmd_step;
    gcs().send_message(MSG_PCAC_MEASUREMENTS);
}

void ModePcac::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}
bool ModePcac::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

void ModePcac::run()
{
    plane.custom_control_reset_last = false;
}

void ModePcac::update_pathBankAngle(){
    Vector3f ihat_G;
    if(ahrs.get_velocity_NED(ihat_G) && ihat_G.length() > 0.001){
        ihat_G.normalize();
        
        Matrix3f O_WG;
        float angleOfAttack = radians(ahrs.getAOA());
        float sideSlipAngle = radians(ahrs.getSSA());
        
        plane.flightPathAngle = safe_asin(-ihat_G[UNIT_K]);
        if(abs(cos(plane.flightPathAngle))>1e-10){
            plane.heading = atan2f(ihat_G[UNIT_J]/cos(plane.flightPathAngle),ihat_G[UNIT_I]/cos(plane.flightPathAngle));
        }
        
        Matrix3f O_BE = ahrs.get_rotation_body_to_ned();
        O_BE.transpose();
        Matrix3f O_EF = eulerOrientationMatrix(UNIT_K,-plane.heading);
        Matrix3f O_FG = eulerOrientationMatrix(UNIT_J,-plane.flightPathAngle);
        Matrix3f O_WS = eulerOrientationMatrix(UNIT_K,sideSlipAngle);
        Matrix3f O_SB = eulerOrientationMatrix(UNIT_J,-angleOfAttack);
        O_WG = O_WS*O_SB*O_BE*O_EF*O_FG;
        plane.pathBankAngle = atan2f(O_WG[0][2],O_WG[0][0]);
    }
    
}
Matrix3f ModePcac::eulerOrientationMatrix(unitVector v,float theta){
    Matrix3f O;
    O.identity();
    switch(v){
        case UNIT_K:
            O[0][0] = cos(theta);
            O[0][1] = sin(theta);
            O[1][0] = -sin(theta);
            O[1][1] = cos(theta);
            break;
        case UNIT_J:
            O[0][0] = cos(theta);
            O[0][2] = -sin(theta);
            O[2][0] = sin(theta);
            O[2][2] = cos(theta);
            break;
        
        case UNIT_I:
            O[1][1] = cos(theta);
            O[1][2] = sin(theta);
            O[2][1] = -sin(theta);
            O[2][2] = cos(theta);
            break;
        
        default:
            //Do nothing
            break;
    }
    return O;
}
