using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using KSP;

namespace AJE
{

    public class AJEPropeller : PartModule
    {


        [KSPField(isPersistant = false, guiActive = false)]
        public float IspMultiplier = 1f;
        [KSPField(isPersistant = false, guiActive = false)]
        public bool useOxygen = true;
        [KSPField(isPersistant = false, guiActive = false)]
        public float idle = 0f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float r0;
        [KSPField(isPersistant = false, guiActive = false)]
        public float v0;
        [KSPField(isPersistant = false, guiActive = false)]
        public float omega0;
        [KSPField(isPersistant = false, guiActive = false)]
        public float rho0;
        [KSPField(isPersistant = false, guiActive = false)]
        public float power0;
        [KSPField(isPersistant = false, guiActive = false)]
        public float fine;
        [KSPField(isPersistant = false, guiActive = false)]
        public float coarse;
        [KSPField(isPersistant = false, guiActive = false)]
        public float omega;
        [KSPField(isPersistant = false, guiActive = false)]
        public float power;
        [KSPField(isPersistant = false, guiActive = false)]
        public float gearratio;
        [KSPField(isPersistant = false, guiActive = false)]
        public float turbo = 1f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float BSFC;
        [KSPField(isPersistant = false, guiActive = true)]
        public string ShaftPower;
        [KSPField(isPersistant = false, guiActive = false)]
        public float SpeedBuff = 1.3f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float maxThrust = 99999;

        public float density;
//        [KSPField(isPersistant = false, guiActive = true)]
        public float pressure;
//        [KSPField(isPersistant = false, guiActive = true)]
        public float temperature;
//        [KSPField(isPersistant = false, guiActive = true)]
        public float v;
//        [KSPField(isPersistant = false, guiActive = true)]
        public float targetTorque;
 //       [KSPField(isPersistant = false, guiActive = true)]
        public float propTorque;
//        [KSPField(isPersistant = false, guiActive = true)]
        public float thrust;
 //       [KSPField(isPersistant = false, guiActive = true)]
        public float isp;


 //       [KSPField(isPersistant = false, guiActive = true)]
 //       public string thrustvector;
 //       [KSPField(isPersistant = false, guiActive = true)]
//          public string velocityvector;

        public AJEPropellerSolver propeller;
        public PistonEngine pistonengine;

        public EngineWrapper engine;

        public override void OnStart(StartState state)
        {
            if (state == StartState.Editor)
                return;
            if (vessel == null)
                return;
            engine = new EngineWrapper(part);
            engine.IspMultiplier = IspMultiplier;
            engine.idle = idle;
            engine.useVelocityCurve = false;
            engine.ThrustUpperLimit = maxThrust;
   
    //        v0 *= 0.5144f;
    //        omega0 *= 0.1047f;
    //        power0 *= 745.7f;
     //       omega *= 0.1047f;
     //       power *= 745.7f;

            propeller = new AJEPropellerSolver(r0, v0 * 0.5144f, omega0 * 0.1047f, rho0, power0 * 745.7f);
            pistonengine = new PistonEngine(power * 745.7f, omega * 0.1047f / gearratio);
            pistonengine._hasSuper = true;
            pistonengine.setTurboParams(turbo, 101300f);
            pistonengine._mixture = 1f;
            propeller.setStops(fine, coarse);
            
        }


        public void FixedUpdate()
        {

            if (HighLogic.LoadedSceneIsEditor)
                return;
            if (engine.type == EngineWrapper.EngineType.NONE || !engine.EngineIgnited)
                return;
            if ((!vessel.mainBody.atmosphereContainsOxygen && useOxygen) || part.vessel.altitude > vessel.mainBody.maxAtmosphereAltitude)
            {
                engine.SetThrust(0);
                return;
            }

            density = (float)ferram4.FARAeroUtil.GetCurrentDensity(part.vessel.mainBody, (float)part.vessel.altitude);
            v = Vector3.Dot(vessel.srf_velocity,-part.FindModelTransform(engine.thrustVectorTransformName).forward.normalized);
            pressure = (float)FlightGlobals.getStaticPressure(vessel.altitude, vessel.mainBody) * 101300f;
            temperature = FlightGlobals.getExternalTemperature((float)vessel.altitude, vessel.mainBody) + 273.15f;

            propeller.calc(density, v / SpeedBuff, omega * 0.1047f);



            pistonengine.calc(pressure, temperature, omega * 0.1047f / gearratio);
            
            if (!useOxygen)
            {
                pistonengine._power = power * 745.7f;
                pistonengine._torque = power * 745.7f / (omega * 0.1047f);
            }

            ShaftPower = ((int)(pistonengine._power/745.7)).ToString()+"HP";

            float mod = 1;
            targetTorque = pistonengine._torque/gearratio;
            propTorque = propeller._torqueOut;
            float momt = 500f;

            mod = propTorque < targetTorque ? 1.04f : (1.0f / 1.04f);
            float diff = Mathf.Abs((propTorque - targetTorque) / momt);
            if (diff < 10)
                mod = 1 + (mod - 1) * (0.1f * diff);
            
            propeller.modPitch(mod);

            thrust = propeller._thrustOut / 1000f;
            engine.SetThrust(thrust);
            isp = propeller._thrustOut / 9.801f / BSFC / pistonengine._power;
            engine.SetIsp(isp);
            
//            Vector3d v1 = part.FindModelTransform("thrustTransform").forward;
//            v1 = vessel.ReferenceTransform.InverseTransformDirection(v1)*100;
 //           Vector3d v2 = vessel.srf_velocity;
 //           v2 = vessel.ReferenceTransform.InverseTransformDirection(v2);
  //          thrustvector = ((int)v1.x).ToString() + " " + ((int)v1.y).ToString() + " " + ((int)v1.z).ToString() + " " + ((int)v1.magnitude).ToString();
 //           velocityvector = ((int)v2.x).ToString() + " " + ((int)v2.y).ToString() + " " + ((int)v2.z).ToString() + " " + ((int)v2.magnitude).ToString();
            
        }



    }
    public class PistonEngine
    {

        public float _throttle = 1f;
        public bool _starter = false; // true=engaged, false=disengaged
        public int _magnetos = 0; // 0=off, 1=right, 2=left, 3=both
        public float _mixture = 1;
        public float _boost;
        public bool _fuel;
        public bool _running = true;
        public float _power0;   // reference power setting
        public float _omega0;   //   "       engine speed
        public float _rho0;     //   "       manifold air density
        public float _f0;       // "ideal" fuel flow at P0/omega0
        public float _mixCoeff; // fuel flow per omega at full mixture
        public float _turbo;    // (or super-)charger pressure multiplier
        public bool _hasSuper;  // true indicates gear-driven (not turbo)
        public float _turboLag; // turbo lag time in seconds
        public float _charge;   // current {turbo|super}charge multiplier
        public float _chargeTarget;  // eventual charge value
        public float _maxMP;    // static maximum pressure
        public float _wastegate;    // wastegate setting, [0:1]
        public float _displacement; // piston stroke volume
        public float _compression;  // compression ratio (>1)
        public float _minthrottle; // minimum throttle [0:1]

        // Runtime state/output:
        public float _mp;
        public float _torque;
        public float _fuelFlow;
        public float _egt;
        public float _boostPressure;
        public float _oilTemp;
        public float _oilTempTarget;
        public float _dOilTempdt;
        public float _power;

        const float HP2W = 745.7f;
        const float CIN2CM = 1.6387064e-5f;
        const float RPM2RADPS = 0.1047198f;

        public PistonEngine(float power, float speed)
        {
            _boost = 1;
            _running = false;
            _fuel = true;
            _boostPressure = 0;
            _hasSuper = false;

            _oilTemp = 298f;
            _oilTempTarget = _oilTemp;
            _dOilTempdt = 0;

            // Presume a BSFC (in lb/hour per HP) of 0.45.  In SI that becomes
            // (2.2 lb/kg, 745.7 W/hp, 3600 sec/hour) 7.62e-08 kg/Ws.
            _f0 = power * 7.62e-08f;

            _power0 = power;
            _omega0 = speed;

            // We must be at sea level under standard conditions
            _rho0 = 1.27f;

            // Further presume that takeoff is (duh) full throttle and
            // peak-power, that means that by our efficiency function, we are
            // at 11/8 of "ideal" fuel flow.
            float realFlow = _f0 * (11.0f / 8.0f);
            _mixCoeff = realFlow * 1.1f / _omega0;

            _turbo = 1;
            _minthrottle = 0.1f;
            _maxMP = 1e6f; // No waste gate on non-turbo engines.
            _wastegate = 1;
            _charge = 1;
            _chargeTarget = 1;
            _turboLag = 2;

            // Guess at reasonable values for these guys.  Displacements run
            // at about 2 cubic inches per horsepower or so, at least for
            // non-turbocharged engines.
            _compression = 8;
            _displacement = power * (2 * CIN2CM / HP2W);
        }
        public void setTurboParams(float turbo, float maxMP)
        {
            _turbo = turbo;
            _maxMP = maxMP;

            // This changes the "sea level" manifold air density
            float P0 = 101000;
            float P = P0 * (1 + _boost * (_turbo - 1));
            if (P > _maxMP) P = _maxMP;
            float T = 298f * Mathf.Pow(P / P0, 2f / 7f);
            _rho0 = P / (287.1f * T);
        }
        public void calc(float pressure, float temp, float speed)
        {
            _running = true; //_magnetos && _fuel && (speed > 60*RPM2RADPS);

            // Calculate the factor required to modify supercharger output for
            // rpm. Assume that the normalized supercharger output ~= 1 when
            // the engine is at the nominal peak-power rpm.  A power equation
            // of the form (A * B^x * x^C) has been derived empirically from
            // some representative supercharger data.  This provides
            // near-linear output over the normal operating range, with
            // fall-off in the over-speed situation.
            float rpm_norm = (speed / _omega0);
            float A = 1.795206541f;
            float B = 0.55620178f;
            float C = 1.246708471f;
            float rpm_factor = A * Mathf.Pow(B, rpm_norm) * Mathf.Pow(rpm_norm, C);
            _chargeTarget = 1 + (_boost * (_turbo - 1) * rpm_factor);

            if (_hasSuper)
            {
                // Superchargers have no lag
                _charge = _chargeTarget;
            } //else if(!_running) {
            // Turbochargers only work well when the engine is actually
            // running.  The 25% number is a guesstimate from Vivian.
            //   _chargeTarget = 1 + (_chargeTarget - 1) * 0.25;
            //  }

            // We need to adjust the minimum manifold pressure to get a
            // reasonable idle speed (a "closed" throttle doesn't suck a total
            // vacuum in real manifolds).  This is a hack.
            float _minMP = (-0.008f * _turbo) + _minthrottle;

            _mp = pressure * _charge;

            // Scale to throttle setting, clamp to wastegate
            if (_running)
                _mp *= _minMP + (1 - _minMP) * _throttle;

            // Scale the max MP according to the WASTEGATE control input.  Use
            // the un-supercharged MP as the bottom limit.
            float max = _wastegate * _maxMP;
            if (max < _mp / _charge) max = _mp / _charge;
            if (_mp > max) _mp = max;


            // The "boost" is the delta above ambient
            _boostPressure = _mp - pressure;

            // Air entering the manifold does so rapidly, and thus the
            // pressure change can be assumed to be adiabatic.  Calculate a
            // temperature change, and use that to get the density.
            // Note: need to model intercoolers here...
            float T = temp * Mathf.Pow((_mp * _mp) / (pressure * pressure), 1f / 7f);
            float rho = _mp / (287.1f * T);

            // The actual fuel flow is determined only by engine RPM and the
            // mixture setting.  Not all of this will burn with the same
            // efficiency.
            _fuelFlow = _mixture * speed * _mixCoeff;
            if (_fuel == false) _fuelFlow = 0;

            // How much fuel could be burned with ideal (i.e. uncorrected!)
            // combustion.
            float burnable = _f0 * (rho / _rho0) * (speed / _omega0);

            // Calculate the fuel that actually burns to produce work.  The
            // idea is that less than 5/8 of ideal, we get complete
            // combustion.  We use up all the oxygen at 1 3/8 of ideal (that
            // is, you need to waste fuel to use all your O2).  In between,
            // interpolate.  This vaguely matches a curve I copied out of a
            // book for a single engine.  Shrug.
            float burned;
            float r = _fuelFlow / burnable;
            if (burnable == 0) burned = 0;
            else if (r < .625) burned = _fuelFlow;
            else if (r > 1.375) burned = burnable;
            else
                burned = _fuelFlow + (burnable - _fuelFlow) * (r - 0.625f) * (4.0f / 3.0f);

            // Correct for engine control state
            if (!_running)
                burned = 0;
            if (_magnetos < 3)
                burned *= 0.9f;

            // And finally the power is just the reference power scaled by the
            // amount of fuel burned, and torque is that divided by RPM.
            float power = _power0 * burned / _f0;

            _power = power;
            _torque = power / speed;

            // Figure that the starter motor produces 15% of the engine's
            // cruise torque.  Assuming 60RPM starter speed vs. 1800RPM cruise
            // speed on a 160HP engine, that comes out to about 160*.15/30 ==
            // 0.8 HP starter motor.  Which sounds about right to me.  I think
            // I've finally got this tuned. :)
            if (_starter && !_running)
                _torque += 0.15f * _power0 / _omega0;

            // Also, add a negative torque of 8% of cruise, to represent
            // internal friction.  Propeller aerodynamic friction is too low
            // at low RPMs to provide a good deceleration.  Interpolate it
            // away as we approach cruise RPMs (full at 50%, zero at 100%),
            // though, to prevent interaction with the power computations.
            // Ugly.
            if (speed > 0 && speed < _omega0)
            {
                float interp = 2 - 2 * speed / _omega0;
                interp = (interp > 1) ? 1 : interp;
                _torque -= 0.08f * (_power0 / _omega0) * interp;
            }

            // Now EGT.  This one gets a little goofy.  We can calculate the
            // work done by an isentropically expanding exhaust gas as the
            // mass of the gas times the specific heat times the change in
            // temperature.  The mass is just the engine displacement times
            // the manifold density, plus the mass of the fuel, which we know.
            // The change in temperature can be calculated adiabatically as a
            // function of the exhaust gas temperature and the compression
            // ratio (which we know).  So just rearrange the equation to get
            // EGT as a function of engine power.  Cool.  I'm using a value of
            // 1300 J/(kg*K) for the exhaust gas specific heat.  I found this
            // on a web page somewhere; no idea if it's accurate.  Also,
            // remember that four stroke engines do one combustion cycle every
            // TWO revolutions, so the displacement per revolution is half of
            // what we'd expect.  And diddle the work done by the gas a bit to
            // account for non-thermodynamic losses like internal friction;
            // 10% should do it.
            float massFlow = _fuelFlow + (rho * 0.5f * _displacement * speed);
            float specHeat = 1300;
            float corr = 1.0f / (Mathf.Pow(_compression, 0.4f) - 1.0f);
            _egt = corr * (power * 1.1f) / (massFlow * specHeat);
            if (_egt < temp) _egt = temp;


            // Oil temperature.
            // Assume a linear variation between ~90degC at idle and ~120degC
            // at full power.  No attempt to correct for airflow over the
            // engine is made.  Make the time constant to attain target steady-
            // state oil temp greater at engine off than on to reflect no
            // circulation.  Nothing fancy, but populates the guage with a
            // plausible value.
            float tau;	// secs 
            if (_running)
            {
                _oilTempTarget = 363.0f + (30.0f * (power / _power0));
                tau = 600;
                // Reduce tau linearly to 300 at max power
                tau -= (power / _power0) * 300.0f;
            }
            else
            {
                _oilTempTarget = temp;
                tau = 1500;
            }
            _dOilTempdt = (_oilTempTarget - _oilTemp) / tau;
        }

    }


    public class AJEPropellerSolver
    {

        public float _r;           // characteristic radius
        public float _j0;          // zero-thrust advance ratio
        public float _baseJ0;      //  ... uncorrected for prop advance
        public float _f0;          // thrust coefficient
        public float _etaC;        // Peak efficiency
        public float _lambdaPeak;  // constant, ~0.759835;
        public float _beta;        // constant, ~1.48058;
        public float _tc0;         // thrust "coefficient" at takeoff
        public float _fine_stop;   // ratio for minimum pitch (high RPM)
        public float _coarse_stop; // ratio for maximum pitch (low RPM)
        public bool _matchTakeoff; // Does _tc0 mean anything?
        public bool _manual;      // manual pitch mode
        public float _proppitch;   // prop pitch control setting (0 ~ 1.0)
        public float _propfeather; // prop feather control setting (0 = norm, 1 = feather)

        public float _density;

        public float _thrustOut;
        public float _torqueOut;
        public bool _autopitch;
        public float _v0;


        public AJEPropellerSolver(float radius, float v, float omega, float rho, float power)
        {
            // Initialize numeric constants:
            _lambdaPeak = Mathf.Pow(1f / 9f, 1f / 8f);
            _beta = 1.0f / (_lambdaPeak - Mathf.Pow(_lambdaPeak, 9f));

            _r = radius;
            _etaC = 0.85f; // make this settable?
            _v0 = v;
            _j0 = v / (omega * _lambdaPeak);
            _baseJ0 = _j0;

            float V2 = v * v + (_r * omega) * (_r * omega);
            _f0 = 2 * _etaC * power / (rho * v * V2);

            _matchTakeoff = false;
            _manual = false;
            _proppitch = 0;
        }

        public void setTakeoff(float omega0, float power0)
        {
            // Takeoff thrust coefficient at lambda==0
            _matchTakeoff = true;
            float V2 = _r * omega0 * _r * omega0;
            float gamma = _etaC * _beta / _j0;
            float torque = power0 / omega0;

            _tc0 = (torque * gamma) / (0.5f * _density * V2 * _f0);
        }

        public void setStops(float fine_stop, float coarse_stop)
        {
            _fine_stop = fine_stop;
            _coarse_stop = coarse_stop;
        }

        public void modPitch(float mod)
        {
            _j0 *= mod;
            if (_j0 < _fine_stop * _baseJ0) _j0 = _fine_stop * _baseJ0;
            if (_j0 > _coarse_stop * _baseJ0) _j0 = _coarse_stop * _baseJ0;
        }

        public void setManualPitch()
        {
            _manual = true;
        }

        public void setPropPitch(float proppitch)
        {
            // makes only positive range of axis effective.
            if (proppitch < 0)
            {
                _proppitch = 0;
                return;
            }
            if (proppitch > 1)
            {
                _proppitch = 1;
                return;
            }
            _proppitch = proppitch;

        }

        public void setPropFeather(int state)
        {
            // 0 = normal, 1 = feathered
            if (state == 0)
            {
                _propfeather = 0;
            }
            else
            {
                _propfeather = 1;
            }
        }

        public void calc(float density, float v, float omega)
        {
            // For manual pitch, exponentially modulate the J0 value between
            // 0.25 and 4.  A prop pitch of 0.5 results in no change from the
            // base value.
            // TODO: integrate with _fine_stop and _coarse_stop variables
            if (_manual)
                _j0 = _baseJ0 * Mathf.Pow(2f, 2 - 4 * _proppitch);

            float tipspd = _r * omega;
            float V2 = v * v + tipspd * tipspd;

            // Sanify
            if (v < 0) v = 0;
            if (omega < 0.001f) omega = 0.001f;

            float J = v / omega;    // Advance ratio


            float lambda = J / _j0; // Unitless scalar advance ratio

            // There's an undefined point at lambda == 1.
            if (lambda == 1.0f) lambda = 0.9999f;

            float l4 = lambda * lambda; l4 = l4 * l4;   // lambda^4
            float gamma = (_etaC * _beta / _j0) * (1 - l4); // thrust/torque ratio

            // Compute a thrust coefficient, with clamping at very low
            // lambdas (fast propeller / slow aircraft).
            float tc = (1 - lambda) / (1 - _lambdaPeak);
            if (_matchTakeoff && tc > _tc0) tc = _tc0;

            float thrust = 0.5f * density * V2 * _f0 * tc;
            float torque = thrust / gamma;
            if (lambda > 1)
            {
                // This is the negative thrust / windmilling regime.  Throw
                // out the efficiency graph approach and instead simply
                // extrapolate the existing linear thrust coefficient and a
                // torque coefficient that crosses the axis at a preset
                // windmilling speed.  The tau0 value is an analytically
                // calculated (i.e. don't mess with it) value for a torque
                // coefficient at lamda==1.
                float tau0 = (0.25f * _j0) / (_etaC * _beta * (1 - _lambdaPeak));
                float lambdaWM = 1.2f; // lambda of zero torque (windmilling)
                torque = tau0 - tau0 * (lambda - 1) / (lambdaWM - 1);
                torque *= 0.5f * density * V2 * _f0;
            }

            _thrustOut = thrust;
            _torqueOut = torque;
        }

    }

}