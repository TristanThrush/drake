// GENERATED FILE DO NOT EDIT
//------------------------------------------------------------------------------
// File: MGKukaIIwaRobot.cc created Jul 31 2017 by MotionGenesis 5.9.
// Portions copyright (c) 2009-2017 Motion Genesis LLC. are licensed under
// the 3-clause BSD license.  https://opensource.org/licenses/BSD-3-Clause
// MotionGenesis Professional Licensee: Paul Mitiguy.
//------------------------------------------------------------------------------
#include <cmath>

//------------------------------------------------------------------------------
#ifndef MGKUKAIIWAROBOTAUTOGENERATED_
#define MGKUKAIIWAROBOTAUTOGENERATED_

//------------------------------------------------------------------------------
namespace MotionGenesis {


//------------------------------------------------------------------------------
class MGKukaIIwaRobotAutoGenerated {
 public:
  MGKukaIIwaRobotAutoGenerated() {}

  // Set member state variables from array.
  void  SetVariablesFromArray(const double VAR[21]);

  // Calculate quantities listed in MotionGenesis Output statements.
  void  CalculateOutput();

  // Class data.
  double g = 0.0;

  // Rigid body (link) mass properties in kg.
  double mA = 5.76, mB = 6.35, mC = 3.5, mD = 3.5, mE = 3.5, mF = 1.8, mG = 1.2;

  // Inertia of rigid bodies about their center of mass, in kg*m^2.
  double IAzz = 0.0123, IBxx = 0.0305, IByy = 0.0304, IBzz = 0.011,
      ICxx = 0.025, ICyy = 0.0238, ICzz = 0.0076, IDxx = 0.017, IDyy = 0.0164,
      IDzz = 0.006, IExx = 0.01, IEyy = 0.0087, IEzz = 0.00449, IFxx = 0.0049,
      IFyy = 0.0047, IFzz = 0.0036, IGxx = 0.001, IGyy = 0.001, IGzz = 0.001;

  double qA, qB, qC, qD, qE, qF, qG;
  double qAp, qBp, qCp, qDp, qEp, qFp, qGp;
  double qApp, qBpp, qCpp, qDpp, qEpp, qFpp, qGpp;
  double fAx, fAy, fAz, fBx, fBy, fBz, fCx, fCy, fCz, fDx, fDy, fDz, fEx, fEy,
         fEz, fFx, fFy, fFz, fGx, fGy, fGz, tAx, tAy, tAz, tBx, tBy, tBz, tCx,
         tCy, tCz, tDx, tDy, tDz, tEx, tEy, tEz, tFx, tFy, tFz, tGx, tGy, tGz;
  double z[251], R_NG[3][3], p_NoGo_N[3], w_NG_N[3], v_NGo_N[3],
         alpha_NG_N[3], a_NGo_N[3], fA[3], tA[3], fB[3], tB[3],
         fC[3], tC[3], fD[3], tD[3], fE[3], tE[3], fF[3], tF[3], fG[3], tG[3];
};


//------------------------------------------------------------------------------
}       // End of namespace MotionGenesis.

//------------------------------------------------------------------------------
#endif  //  MGKUKAIIWAROBOTAUTOGENERATED_

