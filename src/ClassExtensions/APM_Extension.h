#ifndef __APM_EXTENSION_H__
#define __APM_EXTENSION_H__

#include <EquationsOfMotion/FixedWingEOM.h>

class APM : public caelus_fdm::APM {
    friend std::istream &operator>>(std::istream &i, caelus_fdm::APM& aero_data) {
        i >> 
            aero_data.m_CL_0 >>
            aero_data.m_CL_alpha >>
            aero_data.m_CL_delta_e >>
            aero_data.m_CL_q >>
            aero_data.m_CD_0 >>
            aero_data.m_CD_alpha >>
            aero_data.m_CD_alpha2 >>
            aero_data.m_CD_delta_e2 >>
            aero_data.m_CD_beta2 >>
            aero_data.m_CD_beta >>
            aero_data.m_CS_0 >>
            aero_data.m_CS_beta >>
            aero_data.m_CS_delta_a >>
            aero_data.m_CS_p >>
            aero_data.m_CS_r >>
            aero_data.m_Cm_0 >>
            aero_data.m_Cm_alpha >>
            aero_data.m_Cm_delta_e >>
            aero_data.m_Cm_q >>
            aero_data.m_Cl_0 >>
            aero_data.m_Cl_beta >>
            aero_data.m_Cl_delta_a >>
            aero_data.m_Cl_p >>
            aero_data.m_Cl_r >>
            aero_data.m_Cn_0 >>
            aero_data.m_Cn_beta >>
            aero_data.m_Cn_delta_a >>
            aero_data.m_Cn_p >>
            aero_data.m_Cn_r;
        return i;
    }
};

#endif // __APM_EXTENSION_H__