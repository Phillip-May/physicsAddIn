#pragma once

#include "../Common/RobotMotionCore.h"

namespace motion_fixtures {

inline RobotMotionCore::RobotModel makeAr4Model() {
    RobotMotionCore::RobotModel model{};
    const double dhm[24] = {
        0.0, -1.5707963267948966, 0.0, 0.0, 0.0, 3.141592653589793,
        0.0, 64.2, 305.0, 0.0, 0.0, 0.0,
        0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 1.5707963267948966, -1.5707963267948966,
        169.77, 0.0, 0.0, 222.63, 0.0, 41.0
    };
    const double qMin[6] = {
        -2.792526803190927, -0.7330382858376184, -1.5533430342749532,
        -3.141592653589793, -1.8325957145940461, -3.141592653589793
    };
    const double qMax[6] = {
        2.792526803190927, 1.5707963267948966, 0.9075712110370514,
        3.141592653589793, 1.8325957145940461, 3.141592653589793
    };
    for (int i = 0; i < 24; ++i) model.dhm[i] = dhm[i];
    for (int i = 0; i < 6; ++i) {
        model.qHome[i] = 0.0;
        model.qMin[i] = qMin[i];
        model.qMax[i] = qMax[i];
        model.dhmSigns[i] = 1.0;
    }
    model.toolBindPose = RobotMotionCore::identityTransform();
    model.valid = 1;
    return model;
}

} // namespace motion_fixtures
