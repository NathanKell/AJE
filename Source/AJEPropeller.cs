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
        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "v0", guiFormat = "000"), UI_FloatRange(minValue = 300f, maxValue = 1600f, stepIncrement = 5f)]
        public float v0;
        [KSPField(isPersistant = false, guiActive = false)]
        public float omega0;
        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "rho0", guiFormat = "0.###"), UI_FloatRange(minValue = 0.26f, maxValue = 1.26f, stepIncrement = 0.005f)]
        public float rho0;
        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = true, guiName = "power0", guiFormat = "000"), UI_FloatRange(minValue = 300.00f, maxValue = 4000.0f, stepIncrement = 10f)]
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
        public float BSFC = 7.62e-08f;
        [KSPField(isPersistant = false, guiActive = true)]
        public string ShaftPower;
        [KSPField(isPersistant = false, guiActive = false)]
        public float SpeedBuff = 1.0f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float maxThrust = 99999;

        [KSPField(isPersistant = false, guiActive = false)]
        public float wastegateMP = 29.921f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float compression = 7f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float displacement = -1f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float boost0 = -1f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float rated0 = -1f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float boost1 = -1f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float rated1 = -1f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float cost1 = 0f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float switchAlt = 180f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float exhaustThrust = 0f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float coolerEffic = 0f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float coolerMin = -200f;
        [KSPField(isPersistant = false, guiActive = false)]
        public float ramAir = 0.2f;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Manifold Pressure (inHG)")]
        public float manifoldPressure = 0.0f;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Fuel Flow (kg/s)")]
        public float fuelFlow = 0.0f;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Charge Air Temp")]
        public float chargeAirTemp = 15.0f;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Exhaust Thrust (kN)")]
        public float netExhaustThrust = 0.0f;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "Boost", guiFormat = "0.##"), UI_FloatRange(minValue = 0.0f, maxValue = 1.0f, stepIncrement = 0.01f)]
        public float boost = 1.0f;
        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "Mixture", guiFormat = "0.###"), UI_FloatRange(minValue = 0.0f, maxValue = 1.0f, stepIncrement = 0.005f)]
        public float mixture = 0.836481f; // optimal "auto rich"
        // ignore RPM for now

        //[KSPField(isPersistant = false, guiActive = true)]
        public float density = 1.225f;
        //[KSPField(isPersistant = false, guiActive = true)]
        public float pressure = 101325f;
        //[KSPField(isPersistant = false, guiActive = true)]
        public float temperature = 15f;
        //[KSPField(isPersistant = false, guiActive = true)]
        public float v;
        //[KSPField(isPersistant = false, guiActive = true)]
        public float targetTorque;
        //[KSPField(isPersistant = false, guiActive = true)]
        public float propTorque;
        //[KSPField(isPersistant = false, guiActive = true)]
        public float thrust;
        //[KSPField(isPersistant = false, guiActive = true)]
        public float isp;

        public const float INHG2PA = 101325.0f / 760f * 1000f * 0.0254f; // 1 inch of Mercury in Pascals


        //[KSPField(isPersistant = false, guiActive = true)]
        //public string thrustvector;
        //[KSPField(isPersistant = false, guiActive = true)]
        //public string velocityvector;

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

            //v0 *= 0.5144f;
            //omega0 *= 0.1047f;
            //power0 *= 745.7f;
            //omega *= 0.1047f;
            //power *= 745.7f;

            propeller = new AJEPropellerSolver(r0, v0 * 0.5144f, omega0 * PistonEngine.RPM2RADPS, rho0, power0 * PistonEngine.HP2W);
            pistonengine = new PistonEngine(power * PistonEngine.HP2W, omega * PistonEngine.RPM2RADPS / gearratio, BSFC);
            pistonengine._hasSuper = true;
            if (!pistonengine.setBoostParams(wastegateMP * INHG2PA, boost0 * INHG2PA, boost1 * INHG2PA, rated0, rated1, cost1 * PistonEngine.HP2W, switchAlt))
                pistonengine.setTurboParams(turbo, wastegateMP * INHG2PA);
            if (displacement > 0)
                pistonengine._displacement = displacement * PistonEngine.CIN2CM;
            pistonengine._compression = compression;
            pistonengine._coolerEffic = coolerEffic;
            pistonengine._coolerMin = coolerMin + 273.15f;
            pistonengine._ramAir = ramAir;

            pistonengine.ComputeVEMultiplier(); // given newly-set stats

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
            // for RPM handling - bool throttleCut = (object)vessel != null && vessel.ctrlState.mainThrottle <= 0;
            pistonengine._boost = boost;
            pistonengine._mixture = mixture;

            density = (float)ferram4.FARAeroUtil.GetCurrentDensity(part.vessel.mainBody, (float)part.vessel.altitude);
            v = Vector3.Dot(vessel.srf_velocity, -part.FindModelTransform(engine.thrustVectorTransformName).forward.normalized);
            pressure = (float)FlightGlobals.getStaticPressure(vessel.altitude, vessel.mainBody) * 101325f + 0.5f * density * v * v * ramAir; // include dynamic pressure
            temperature = FlightGlobals.getExternalTemperature((float)vessel.altitude, vessel.mainBody) + 273.15f;

            propeller.calc(density, v / SpeedBuff, omega * PistonEngine.RPM2RADPS);



            pistonengine.calc(pressure, temperature, omega * PistonEngine.RPM2RADPS / gearratio);

            if (!useOxygen)
            {
                pistonengine._power = power * PistonEngine.HP2W;
                pistonengine._torque = power * PistonEngine.HP2W / (omega * PistonEngine.RPM2RADPS);
            }

            ShaftPower = ((int)Math.Round(pistonengine._power / PistonEngine.HP2W)).ToString() + "HP";
            manifoldPressure = pistonengine._mp / INHG2PA;
            fuelFlow = pistonengine._fuelFlow;
            chargeAirTemp = pistonengine._chargeTemp - 273.15f;

            float mod = 1;
            targetTorque = pistonengine._torque / gearratio;
            propTorque = propeller._torqueOut;
            float momt = 500f;

            mod = propTorque < targetTorque ? 1.04f : (1.0f / 1.04f);
            float diff = Mathf.Abs((propTorque - targetTorque) / momt);
            if (diff < 10)
                mod = 1 + (mod - 1) * (0.1f * diff);

            propeller.modPitch(mod);

            thrust = propeller._thrustOut / 1000f;
            // exhaust thrust, normalized for 2200HP and 180m/s = 200lbs thrust
            netExhaustThrust = exhaustThrust * pistonengine._power / 2200f / 745.7f * 0.89f * (0.5f + v / 360f);
            thrust += netExhaustThrust;
            engine.SetThrust(thrust);
            isp = propeller._thrustOut / 9.80665f / BSFC / pistonengine._power;
            engine.SetIsp(isp);

            //Vector3d v1 = part.FindModelTransform("thrustTransform").forward;
            //v1 = vessel.ReferenceTransform.InverseTransformDirection(v1)*100;
            //Vector3d v2 = vessel.srf_velocity;
            //v2 = vessel.ReferenceTransform.InverseTransformDirection(v2);
            //thrustvector = ((int)v1.x).ToString() + " " + ((int)v1.y).ToString() + " " + ((int)v1.z).ToString() + " " + ((int)v1.magnitude).ToString();
            //velocityvector = ((int)v2.x).ToString() + " " + ((int)v2.y).ToString() + " " + ((int)v2.z).ToString() + " " + ((int)v2.magnitude).ToString();

        }



    }
    public class PistonEngine
    {

        public float _throttle = 1f;
        public bool _starter = false; // true=engaged, false=disengaged
        public int _magnetos = 0; // 0=off, 1=right, 2=left, 3=both
        public float _mixture = 0.836481f; //optimal
        public float _boost;
        public bool _fuel;
        public bool _running = true;
        public float _power0;   // reference power setting
        public float _omega0;   //   "       engine speed
        public float _mixCoeff; // fuel flow per omega at full mixture
        public bool _hasSuper;  // true indicates gear-driven (not turbo)
        public float _turboLag; // turbo lag time in seconds
        public float _charge;   // current {turbo|super}charge multiplier
        public float _chargeTarget;  // eventual charge value
        public float _maxMP;    // static maximum pressure
        public float _wastegate;    // wastegate setting, [0:1]
        public float _displacement; // piston stroke volume
        public float _compression;  // compression ratio (>1)
        public float _minthrottle; // minimum throttle [0:1]
        public float _voleffic; // volumetric efficiency
        public float[] _boostMults;
        public float[] _boostCosts;
        public int _boostMode;
        public float _boostSwitch;
        public float _bsfc;
        public float _coolerEffic;
        public float _coolerMin;
        public float _ramAir;

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
        public float _chargeTemp;

        public const float HP2W = 745.7f;
        public const float CIN2CM = 1.6387064e-5f;
        public const float RPM2RADPS = 0.1047198f;
        public const float RAIR = 287.3f;
        public FloatCurve mixtureEfficiency; //fuel:air -> efficiency

        const float T0 = 288.15f;
        const float P0 = 101325f;

        public PistonEngine(float power, float speed, float BSFC)
        {
            _boost = 1;
            _running = false;
            _fuel = true;
            _boostPressure = 0;
            _hasSuper = false;
            _boostMults = new float[2];
            _boostCosts = new float[2];
            _boostMults[0] = 1f;
            _boostMults[1] = 1f;
            _boostCosts[0] = 0f;
            _boostCosts[1] = 0f;
            _boostMode = 0;
            _boostSwitch = -1f; // auto
            _coolerEffic = 0f;
            _coolerMin = 0f;
            _ramAir = 0.2f;

            _oilTemp = 298f;
            _oilTempTarget = _oilTemp;
            _dOilTempdt = 0;

            _bsfc = BSFC;

            _minthrottle = 0.1f;
            _maxMP = 1e6f; // No waste gate on non-turbo engines.
            _wastegate = 1f;
            _charge = 1f;
            _chargeTarget = 1f;
            _turboLag = 2f;

            // set reference conditions
            _power0 = power;
            _omega0 = speed;

            // Guess at reasonable values for these guys.  Displacements run
            // at about 2 cubic inches per horsepower or so, at least for
            // non-turbocharged engines.
            // change to 1.4; supercharging...
            _compression = 8;
            _displacement = power * (1.4f * CIN2CM / HP2W);

            mixtureEfficiency = new FloatCurve();
            ConfigNode node = new ConfigNode("MixtureEfficiency");
            node.AddValue("key", "0.05	0.0");
            node.AddValue("key", "0.05137	0.00862");
            node.AddValue("key", "0.05179	0.21552");
            node.AddValue("key", "0.0543	0.48276");
            node.AddValue("key", "0.05842	0.7069");
            node.AddValue("key", "0.06312	0.83621");
            node.AddValue("key", "0.06942	0.93103");
            node.AddValue("key", "0.07786	1.0");
            node.AddValue("key", "0.08845	1.0");
            node.AddValue("key", "0.0927	0.98276");
            node.AddValue("key", "0.1012	0.93103");
            node.AddValue("key", "0.11455	0.72414");
            node.AddValue("key", "0.12158	0.4569");
            node.AddValue("key", "0.12435	0.23276");
            node.AddValue("key", "0.125 0.0");
            mixtureEfficiency.Load(node);


            ComputeVEMultiplier(); // compute volumetric efficiency of engine, based on rated power and BSFC.
        }

        // return the fuel:air ratio of the given mixture setting.
        // mix = range [0, 1]
        public float FuelAirRatio(float mix)
        {
            return 1f / (8.05f + 11.2f * (1f - mix)); // prevent an AFR too high or low
        }

        // return the relative volumetric efficiency of the engine, given its compression ratio
        // at the ambient pressure pAmb and the manifold absolute pressure MAP
        public float GetPressureVE(float pAmb, float MAP)
        {
            // JSBSim
            float gamma = 1.4f;
            return ((gamma - 1f) / gamma) + (_compression - (pAmb / MAP)) / (gamma * (_compression - 1f));
        }

        // return the charge air temperature after heating and cooling (if any)
        // at given manifold absolute pressure MAP, and given ambient pressure and temp
        // Very simple model: the aftercooler will cool the charge to (cooling efficiency) of
        //  ambient temperature, to a minimum temperature of (cooler min)
        public float GetCAT(float MAP, float pAmb, float tAmb)
        {
            // Air entering the manifold does so rapidly, and thus the
            // pressure change can be assumed to be adiabatic.  Calculate a
            // temperature change, and then apply aftercooling/intercooling (if any)
            float T = tAmb * Mathf.Pow((MAP * MAP) / (pAmb * pAmb), 1f / 7f);
            return Math.Max(_coolerMin, T - (T - tAmb) * _coolerEffic);
        }

        // return the mass airflow through the engine
        // running at given speed in radians/sec, given manifold absolute pressure MAP,
        // given ambient pressure and temperature. Depends on displacement and
        // the volumetric efficiency multiplier.
        public float GetAirflow(float pAmb, float tAmb, float speed, float MAP)
        {
            //from JSBSim
            // air flow
            float swept_volume = (_displacement * speed / RPM2RADPS / 60f) / 2f;
            float v_dot_air = swept_volume * GetPressureVE(pAmb, MAP) * _voleffic;

            float rho_air_manifold = MAP / (RAIR * GetCAT(MAP, pAmb, tAmb));
            return v_dot_air * rho_air_manifold;
        }

        // will compute the volumetric efficiency multiplier for the engine
        // given all reference engine stats.
        public void ComputeVEMultiplier()
        {
            _voleffic = 1f; // reset the volumetric efficiency multiplier

            // The idea: since HP will be proportional to fuel flow (* mixture efficiency)
            // we compute the multiplier to airflow needed to yield the desired HP.
            // Assume boost0 at takeoff, up to max MP. We use optimal mixture for power.
            float fuelAirRatio = FuelAirRatio(0.836481f);
            float MAP = Math.Min(_maxMP, P0 * _boostMults[0]);
            float m_dot_air = GetAirflow(P0, T0, _omega0, MAP);
            float m_dot_fuel = fuelAirRatio * m_dot_air;
            float power = m_dot_fuel * mixtureEfficiency.Evaluate(fuelAirRatio) / _bsfc;
            _voleffic = _power0 / power;
            float m_dot_air2 = GetAirflow(P0, T0, _omega0, MAP);
            float m_dot_fuel2 = fuelAirRatio * m_dot_air;
            power = m_dot_fuel2 * mixtureEfficiency.Evaluate(fuelAirRatio) / _bsfc;
            MonoBehaviour.print("*AJE* Setting volumetric efficiency. At SL with MAP " + MAP + ", power = " + power / HP2W + "HP, BSFC = " + _bsfc + ", mda/f = " + m_dot_air2 + "/" + m_dot_fuel2 + ", VE = " + _voleffic + ". Orig a/f: " + m_dot_air + "/" + m_dot_fuel);
        }

        // legacy support
        public void setTurboParams(float turbo, float maxMP)
        {
            _maxMP = maxMP;
            _boostMults[0] = turbo;
            _boostCosts[0] = 0f;
        }

        // set boost parameters:
        // maximum MAP, the two boost pressures to maintain, the two rated altitudes (km),
        // the cost for the second boost mode, and the switch altitude (in km), or -1f for auto
        public bool setBoostParams(float wastegate, float boost0, float boost1, float rated0, float rated1, float cost1, float switchAlt)
        {
            bool retVal = false;
            if (boost0 > 0)
            {
                _boostMults[0] = boost0 / ((float)FlightGlobals.getStaticPressure(rated0, FlightGlobals.Bodies[1]) * 101325f);
                _maxMP = wastegate;
                retVal = true;
            }
            if (boost1 > 0)
            {
                _boostMults[1] = boost1 / ((float)FlightGlobals.getStaticPressure(rated1, FlightGlobals.Bodies[1]) * 101325f);
                _boostCosts[1] = cost1;
            }
            else
            {
                _boostMults[1] = 0f;
                _boostCosts[1] = 0f;
            }
            if (switchAlt >= 0f)
                _boostSwitch = (float)FlightGlobals.getStaticPressure(switchAlt, FlightGlobals.Bodies[1]) * 101325f;
            else
                _boostSwitch = switchAlt;
            MonoBehaviour.print("*AJE* Setting boost params. MaxMP = " + wastegate + ", boosts = " + _boostMults[0] + "/" + _boostMults[1] + ", switch " + _boostSwitch + " from " + boost0 + "@" + rated0 + ", " + boost1 + "@" + rated1);

            return retVal;
        }

        // Gets the target for the [turbo]supercharger
        // takes engine speed, boost mode
        float GetChargeTarget(float speed, int boostMode)
        {
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
            return 1f + (_boost * (_boostMults[boostMode] - 1f) * rpm_factor);
        }

        // return the manifold absolute pressure in pascals
        // takes the ambient pressure and the boost mode
        // clamps to [ambient pressure.....wastegate]
        public float CalcMAP(float pAmb, int boostMode, float charge)
        {
            // We need to adjust the minimum manifold pressure to get a
            // reasonable idle speed (a "closed" throttle doesn't suck a total
            // vacuum in real manifolds).  This is a hack.
            float _minMP = (-0.008f * _boostMults[boostMode]) + _minthrottle;

            float MAP = pAmb * charge;

            // Scale to throttle setting, clamp to wastegate
            if (_running)
                MAP *= _minMP + (1 - _minMP) * _throttle;

            // Scale the max MP according to the WASTEGATE control input.  Use
            // the un-supercharged MP as the bottom limit.
            return (float)Math.Min(MAP, Math.Max(_wastegate * _maxMP, pAmb));
        }

        // Iteration method
        // Will calculate engine parameters.
        // Takes ambient pressure, temperature, and the engine revolutions in radians/sec
        public void calc(float pAmb, float tAmb, float speed)
        {
            _running = true; //_magnetos && _fuel && (speed > 60*RPM2RADPS);

            // check if we need to switch boost modes
            float power;
            float MAP;
            float fuelRatio = FuelAirRatio(_mixture);
            if (_boostSwitch > 0)
            {
                if (pAmb < _boostSwitch - 1000 && _boostMode < 1)
                    _boostMode++;
                if (pAmb > _boostSwitch + 1000 && _boostMode > 0)
                    _boostMode--;

                _chargeTarget = GetChargeTarget(speed, _boostMode);
                if (_hasSuper)
                {
                    // Superchargers have no lag
                    _charge = _chargeTarget;
                } //else if(!_running) {
                // Turbochargers only work well when the engine is actually
                // running.  The 25% number is a guesstimate from Vivian.
                //   _chargeTarget = 1 + (_chargeTarget - 1) * 0.25;
                //  }

                MAP = CalcMAP(pAmb, _boostMode, _charge);

                // Compute fuel flow
                _fuelFlow = GetAirflow(pAmb, tAmb, speed, MAP) * fuelRatio;
                power = _fuelFlow * mixtureEfficiency.Evaluate(fuelRatio) / _bsfc - _boostCosts[_boostMode];
            }
            else // auto switch
            {
                // assume supercharger for now, so charge = target
                float target0 = GetChargeTarget(speed, 0);
                float target1 = GetChargeTarget(speed, 1);
                float MAP0 = CalcMAP(pAmb, 0, target0);
                float MAP1 = CalcMAP(pAmb, 1, target1);

                float m_dot_fuel0 = GetAirflow(pAmb, tAmb, speed, MAP0) * fuelRatio;
                float power0 = m_dot_fuel0 * mixtureEfficiency.Evaluate(fuelRatio) / _bsfc - _boostCosts[0];

                float m_dot_fuel1 = GetAirflow(pAmb, tAmb, speed, MAP1) * fuelRatio;
                float power1 = m_dot_fuel1 * mixtureEfficiency.Evaluate(fuelRatio) / _bsfc - _boostCosts[1];

                if (power0 >= power1)
                {
                    MAP = MAP0;
                    _chargeTarget = _charge = target0;
                    power = power0;
                    _fuelFlow = m_dot_fuel0;
                }
                else
                {
                    MAP = MAP1;
                    _chargeTarget = _charge = target1;
                    power = power1;
                    _fuelFlow = m_dot_fuel0;
                }
            }

            _mp = MAP;
            _chargeTemp = GetCAT(MAP, pAmb, tAmb); // duplication of effort, but oh well

            // The "boost" is the delta above ambient
            _boostPressure = _mp - pAmb;

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


            // UNUSED
            /*
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
             */
        }

    }

    // largely a port of JSBSim's FGTable
    public class FGTable
    {
        double[,] Data;
        public uint nRows, nCols;
        uint lastRowIndex = 0;
        uint lastColumnIndex = 0;
        public FGTable(ConfigNode node)
        {
            nRows = (uint)node.values.Count;
            string[] tmp1 = node.values[0].value.Split(null);
            nCols = (uint)tmp1.Length; // first row has no first column
            if (node.values.Count > 1)
            {
                string[] tmp2 = node.values[1].value.Split(null);
                if (tmp2.Length > nCols)
                    nCols = (uint)tmp2.Length;
            }
            Data = new double[nRows, nCols];
            for (int i = 0; i < nRows; i++)
            {
                string[] curRow = node.values[i].value.Split(null);
                for (int j = 0; j < curRow.Length; j++)
                {
                    double dtmp;
                    if (double.TryParse(curRow[j], out dtmp))
                        Data[i,j] = dtmp;
                }
            }
        }
        public double GetValue(double rowKey, double colKey)
        {
            double rFactor, cFactor, col1temp, col2temp, Value;
            uint r = lastRowIndex;
            uint c = lastColumnIndex;

            while(r > 2     && Data[r-1,0] > rowKey) { r--; }
            while(r < nRows && Data[r,0] < rowKey) { r++; }

            while(c > 2     && Data[0,c-1] > colKey) { c--; }
            while(c < nCols && Data[0,c]   < colKey) { c++; }

            lastRowIndex=r;
            lastColumnIndex=c;

            rFactor = (rowKey - Data[r-1,0]) / (Data[r,0] - Data[r-1,0]);
            cFactor = (colKey - Data[0,c-1]) / (Data[0,c] - Data[0,c-1]);

            if (rFactor > 1.0) rFactor = 1.0;
            else if (rFactor < 0.0) rFactor = 0.0;

            if (cFactor > 1.0) cFactor = 1.0;
            else if (cFactor < 0.0) cFactor = 0.0;

            col1temp = rFactor*(Data[r,c-1] - Data[r-1,c-1]) + Data[r-1,c-1];
            col2temp = rFactor*(Data[r,c] - Data[r-1,c]) + Data[r-1,c];

            Value = col1temp + cFactor*(col2temp - col1temp);

            return Value;
        }
        double GetValue(double key)
        {
            double Factor, Value, Span;
            uint r = lastRowIndex;

            //if the key is off the end of the table, just return the
            //end-of-table value, do not extrapolate
            if( key <= Data[1,0] ) {
                lastRowIndex=2;
                //cout << "Key underneath table: " << key << endl;
                return Data[1,1];
            } else if ( key >= Data[nRows,0] ) {
                lastRowIndex=nRows;
                //cout << "Key over table: " << key << endl;
                return Data[nRows,1];
            }

            // the key is somewhere in the middle, search for the right breakpoint
            // The search is particularly efficient if 
            // the correct breakpoint has not changed since last frame or
            // has only changed very little

            while (r > 2     && Data[r-1,0] > key) { r--; }
            while (r < nRows && Data[r,0]   < key) { r++; }

            lastRowIndex=r;
            // make sure denominator below does not go to zero.

            Span = Data[r,0] - Data[r-1,0];
            if (Span != 0.0) {
                Factor = (key - Data[r-1,0]) / Span;
                if (Factor > 1.0) Factor = 1.0;
            } else {
                Factor = 1.0;
            }

            Value = Factor*(Data[r,1] - Data[r-1,1]) + Data[r-1,1];

            return Value;
        }
    }
    
    // taken from JSBSim
    public class AJEPropJSB
    {
        // FGThruster members
        double Thrust;
        double PowerRequired;
        double deltaT;
        double GearRatio;
        double ThrustCoeff;
        double ReverserAngle;

        //Vector3 ActingLocation;

        // FGPropeller members
        int numBlades;
        double J;
        double RPM;
        double Ixx;
        double Diameter;
        double MaxPitch;
        double MinPitch;
        double MinRPM;
        double MaxRPM;
        double Pitch;
        double P_Factor;
        double Sense;
        double Advance;
        double ExcessTorque;
        double D4;
        double D5;
        double HelicalTipMach;
        double Vinduced;
        Vector3 vTorque;
        FGTable cThrust;
        FGTable cPower;
        FloatCurve CtMach;
        FloatCurve CpMach;
        double CtFactor;
        double CpFactor;
        int ConstantSpeed;
        void Debug(int from);
        double ReversePitch; // Pitch, when fully reversed
        bool Reversed;     // true, when propeller is reversed
        double Reverse_coef; // 0 - 1 defines AdvancePitch (0=MIN_PITCH 1=REVERSE_PITCH)
        bool Feathered;    // true, if feather command

        /** Constructor for FGPropeller.
            @param exec a pointer to the main executive object
            @param el a pointer to the thruster config file XML element
            @param num the number of this propeller */
        public AJEPropJSB(ConfigNode node)
        {
            MaxPitch = MinPitch = P_Factor = Pitch = Advance = MinRPM = MaxRPM = 0.0;
            Sense = 1; // default clockwise rotation
            ReversePitch = 0.0;
            Reversed = false;
            Feathered = false;
            Reverse_coef = 0.0;
            GearRatio = 1.0;
            CtFactor = CpFactor = 1.0;
            ConstantSpeed = 0;
            cThrust = new FloatCurve();
            cPower = new FloatCurve();
            CtMach = new FloatCurve();
            CpMach = new FloatCurve();
            Vinduced = 0.0;
            if (node.HasValue("ixx"))
                Ixx = double.Parse(node.GetValue("ixx"));
            if (node.HasValue("diameter"))
                Diameter = double.Parse(node.GetValue("diameter"));
            if (node.HasValue("numblades"))
                numBlades = int.Parse(node.GetValue("numblades"));
            if (node.HasValue("gearratio"))
                GearRatio = double.Parse(node.GetValue("gearratio"));
            if (node.HasValue("minpitch"))
                MinPitch = double.Parse(node.GetValue("minpitch"));
            if (node.HasValue("maxpitch"))
                MaxPitch = double.Parse(node.GetValue("maxpitch"));
            if (node.HasValue("minrpm"))
                MinRPM = double.Parse(node.GetValue("minrpm"));
            if (node.HasValue("maxrpm"))
            {
                MaxRPM = double.Parse(node.GetValue("maxrpm"));
                ConstantSpeed = 1;
            }
            if (node.HasValue("constspeed"))
                ConstantSpeed = int.Parse(node.GetValue("constspeed"));
            if (node.HasValue("reversepitch"))
                ReversePitch = double.Parse(node.GetValue("reversepitch"));
            if (node.HasNode("cThrust"))
                cThrust.Load(node.GetNode("cThrust"));
            if (node.HasNode("cPower"))
                cThrust.Load(node.GetNode("cPower"));
            if (node.HasNode("CtMach"))
                cThrust.Load(node.GetNode("CtMach"));
            if (node.HasNode("CpMach"))
                cThrust.Load(node.GetNode("CpMach"));

            if(node.HasValue("sense"))
                Sense = double.Parse(node.GetValue("sense"));
            SetSense(Sense >= 0.0 ? 1.0 : -1.0);
            if(node.HasValue("P_Factor"))
                P_Factor = double.Parse(node.GetValue("P_Factor"));
            if(node.HasValue("ct_factor"))
                SetCtFactor(double.Parse(node.GetValue("ct_factor")));
            if(node.HasValue("cp_factor"))
                SetCpFactor(double.Parse(node.GetValue("cp_factor")));

            RPM = 0;
            vTorque = new Vector3(0f,0f,0f);
            D4 = Diameter * Diameter * Diameter * Diameter;
            D5 = D4 * Diameter;
            Pitch = MinPitch;
        }

        /** Sets the Revolutions Per Minute for the propeller. Normally the propeller
            instance will calculate its own rotational velocity, given the Torque
            produced by the engine and integrating over time using the standard
            equation for rotational acceleration "a": a = Q/I , where Q is Torque and
            I is moment of inertia for the propeller.
            @param rpm the rotational velocity of the propeller */
        public void SetRPM(double rpm) { RPM = rpm; }

        /** Sets the Revolutions Per Minute for the propeller using the engine gear ratio **/
        public void SetEngineRPM(double rpm) { RPM = rpm / GearRatio; }

        /// Returns true of this propeller is variable pitch
        public bool IsVPitch() { return MaxPitch != MinPitch; }

        /** This commands the pitch of the blade to change to the value supplied.
            This call is meant to be issued either from the cockpit or by the flight
            control system (perhaps to maintain constant RPM for a constant-speed
            propeller). This value will be limited to be within whatever is specified
            in the config file for Max and Min pitch. It is also one of the lookup
            indices to the power and thrust tables for variable-pitch propellers.
            @param pitch the pitch of the blade in degrees. */
        public void SetPitch(double pitch) { Pitch = pitch; }

        public void SetAdvance(double advance) { Advance = advance; }

        /// Sets the P-Factor constant
        public void SetPFactor(double pf) { P_Factor = pf; }

        /// Sets propeller into constant speed mode, or manual pitch mode
        public void SetConstantSpeed(int mode) { ConstantSpeed = mode; }

        /// Sets coefficient of thrust multiplier
        public void SetCtFactor(double ctf) { CtFactor = ctf; }

        /// Sets coefficient of power multiplier
        public void SetCpFactor(double cpf) { CpFactor = cpf; }

        /** Sets the rotation sense of the propeller.
            @param s this value should be +/- 1 ONLY. +1 indicates clockwise rotation as
                     viewed by someone standing behind the engine looking forward into
                     the direction of flight. */
        public void SetSense(double s) { Sense = s; }

        /// Retrieves the pitch of the propeller in degrees.
        public double GetPitch() { return Pitch; }

        /// Retrieves the RPMs of the propeller
        public double GetRPM() { return RPM; }

        /// Calculates the RPMs of the engine based on gear ratio
        public double GetEngineRPM() { return RPM * GearRatio; }

        /// Retrieves the propeller moment of inertia
        public double GetIxx() { return Ixx; }

        /// Retrieves the coefficient of thrust multiplier
        public double GetCtFactor() { return CtFactor; }

        /// Retrieves the coefficient of power multiplier
        public double GetCpFactor() { return CpFactor; }

        /// Retrieves the propeller diameter
        public double GetDiameter() { return Diameter; }

        /// Retrieves propeller thrust table
        public FloatCurve GetCThrustTable() { return cThrust; }
        /// Retrieves propeller power table
        public FloatCurve GetCPowerTable() { return cPower; }

        /// Retrieves propeller thrust Mach effects factor
        public FloatCurve GetCtMachTable() { return CtMach; }
        /// Retrieves propeller power Mach effects factor
        public FloatCurve GetCpMachTable() { return CpMach; }

        /// Retrieves the Torque in foot-pounds (Don't you love the English system?)
        public double GetTorque() { return vTorque.x; }

        public void SetReverseCoef(double c) { Reverse_coef = c; }
        public double GetReverseCoef() { return Reverse_coef; }
        public void SetReverse(bool r) { Reversed = r; }
        public bool GetReverse() { return Reversed; }
        public void SetFeather(bool f) { Feathered = f; }
        public bool GetFeather() { return Feathered; }
        public double GetThrustCoefficient() { return ThrustCoeff; }
        public double GetHelicalTipMach() { return HelicalTipMach; }
        public int GetConstantSpeed() { return ConstantSpeed; }
        public void SetInducedVelocity(double Vi) { Vinduced = Vi; }
        public double GetInducedVelocity() { return Vinduced; }

        /*public Vector3 GetPFactor()
        {
            double px = 0.0, py, pz;

            py = Thrust * Sense * (ActingLocation.y - GetLocationY()) / 12.0;
            pz = Thrust * Sense * (ActingLocation.z - GetLocationZ()) / 12.0;

            return Vector3(px, py, pz);
        }*/

        /** Retrieves the power required (or "absorbed") by the propeller -
            i.e. the power required to keep spinning the propeller at the current
            velocity, air density,  and rotational rate. */
        public double GetPowerRequired(double rho, double Vel)
        {
            double cPReq, J;
  double RPS = RPM / 60.0;

  if (RPS != 0.0) J = Vel / (Diameter * RPS);
  else            J = Vel / Diameter;

  if (MaxPitch == MinPitch) {   // Fixed pitch prop
    cPReq = cPower.Evaluate((float)J);

  } else {                      // Variable pitch prop

    if (ConstantSpeed != 0) {   // Constant Speed Mode

      // do normal calculation when propeller is neither feathered nor reversed
      // Note:  This method of feathering and reversing was added to support the
      //        turboprop model.  It's left here for backward compatablity, but
      //        now feathering and reversing should be done in Manual Pitch Mode.
      if (!Feathered) {
        if (!Reversed) {

          double rpmReq = MinRPM + (MaxRPM - MinRPM) * Advance;
          double dRPM = rpmReq - RPM;
          // The pitch of a variable propeller cannot be changed when the RPMs are
          // too low - the oil pump does not work.
          if (RPM > 200) Pitch -= dRPM * deltaT;
          if (Pitch < MinPitch)       Pitch = MinPitch;
          else if (Pitch > MaxPitch)  Pitch = MaxPitch;

        } else { // Reversed propeller

          // when reversed calculate propeller pitch depending on throttle lever position
          // (beta range for taxing full reverse for braking)
          double PitchReq = MinPitch - ( MinPitch - ReversePitch ) * Reverse_coef;
          // The pitch of a variable propeller cannot be changed when the RPMs are
          // too low - the oil pump does not work.
          if (RPM > 200) Pitch += (PitchReq - Pitch) / 200;
          if (RPM > MaxRPM) {
            Pitch += (MaxRPM - RPM) / 50;
            if (Pitch < ReversePitch) Pitch = ReversePitch;
            else if (Pitch > MaxPitch)  Pitch = MaxPitch;
          }
        }

      } else { // Feathered propeller
               // ToDo: Make feathered and reverse settings done via FGKinemat
        Pitch += (MaxPitch - Pitch) / 300; // just a guess (about 5 sec to fully feathered)
      }

    } else { // Manual Pitch Mode, pitch is controlled externally

    }

    cPReq = cPower.Evaluate(J, Pitch);
  }

  // Apply optional scaling factor to Cp (default value = 1)
  cPReq *= CpFactor;

  // Apply optional Mach effects from CP_MACH table
  if (CpMach) cPReq *= CpMach.Evaluate(HelicalTipMach);

  double local_RPS = RPS < 0.01 ? 0.01 : RPS; 

  PowerRequired = cPReq*local_RPS*RPS*local_RPS*D5*rho;
  vTorque(eX) = -Sense*PowerRequired / (local_RPS*2.0*M_PI);

  return PowerRequired;
        }

        /** Calculates and returns the thrust produced by this propeller.
            Given the excess power available from the engine (in foot-pounds), the thrust is
            calculated, as well as the current RPM. The RPM is calculated by integrating
            the torque provided by the engine over what the propeller "absorbs"
            (essentially the "drag" of the propeller).
            @param PowerAvailable this is the excess power provided by the engine to
            accelerate the prop. It could be negative, dictating that the propeller
            would be slowed.
            @return the thrust in pounds */
        double Calculate(double EnginePower)
        {
            FGColumnVector3 localAeroVel = Transform().Transposed() * in.AeroUVW;
  double omega, PowerAvailable;

  double Vel = localAeroVel(eU);
  double rho = in.Density;
  double RPS = RPM/60.0;

  // Calculate helical tip Mach
  double Area = 0.25*Diameter*Diameter*M_PI;
  double Vtip = RPS * Diameter * M_PI;
  HelicalTipMach = sqrt(Vtip*Vtip + Vel*Vel) / in.Soundspeed;

  PowerAvailable = EnginePower - GetPowerRequired();

  if (RPS > 0.0) J = Vel / (Diameter * RPS); // Calculate J normally
  else           J = Vel / Diameter;

  if (MaxPitch == MinPitch) {    // Fixed pitch prop
    ThrustCoeff = cThrust.Evaluate(J);
  } else {                       // Variable pitch prop
    ThrustCoeff = cThrust.Evaluate(J, Pitch);
  }

  // Apply optional scaling factor to Ct (default value = 1)
  ThrustCoeff *= CtFactor;

  // Apply optional Mach effects from CT_MACH table
  if (CtMach) ThrustCoeff *= CtMach.Evaluate(HelicalTipMach);

  Thrust = ThrustCoeff*RPS*RPS*D4*rho;

  // Induced velocity in the propeller disk area. This formula is obtained
  // from momentum theory - see B. W. McCormick, "Aerodynamics, Aeronautics,
  // and Flight Mechanics" 1st edition, eqn. 6.15 (propeller analysis chapter).
  // Since Thrust and Vel can both be negative we need to adjust this formula
  // To handle sign (direction) separately from magnitude.
  double Vel2sum = Vel*abs(Vel) + 2.0*Thrust/(rho*Area);
  
  if( Vel2sum > 0.0)
    Vinduced = 0.5 * (-Vel + sqrt(Vel2sum));
  else
    Vinduced = 0.5 * (-Vel - sqrt(-Vel2sum));

  // We need to drop the case where the downstream velocity is opposite in
  // direction to the aircraft velocity. For example, in such a case, the
  // direction of the airflow on the tail would be opposite to the airflow on
  // the wing tips. When such complicated airflows occur, the momentum theory
  // breaks down and the formulas above are no longer applicable
  // (see H. Glauert, "The Elements of Airfoil and Airscrew Theory",
  // 2nd edition, �16.3, pp. 219-221)

  if ((Vel+2.0*Vinduced)*Vel < 0.0) {
    // The momentum theory is no longer applicable so let's assume the induced
    // saturates to -0.5*Vel so that the total velocity Vel+2*Vinduced equals 0.
    Vinduced = -0.5*Vel;
  }
    
  // P-factor is simulated by a shift of the acting location of the thrust.
  // The shift is a multiple of the angle between the propeller shaft axis
  // and the relative wind that goes through the propeller disk.
  if (P_Factor > 0.0001) {
    double tangentialVel = localAeroVel.Magnitude(eV, eW);

    if (tangentialVel > 0.0001) {
      double angle = atan2(tangentialVel, localAeroVel(eU));
      double factor = Sense * P_Factor * angle / tangentialVel;
      SetActingLocationY( GetLocationY() + factor * localAeroVel(eW));
      SetActingLocationZ( GetLocationZ() + factor * localAeroVel(eV));
    }
  }

  omega = RPS*2.0*M_PI;

  vFn(eX) = Thrust;

  // The Ixx value and rotation speed given below are for rotation about the
  // natural axis of the engine. The transform takes place in the base class
  // FGForce::GetBodyForces() function.

  vH(eX) = Ixx*omega*Sense;
  vH(eY) = 0.0;
  vH(eZ) = 0.0;

  if (omega > 0.0) ExcessTorque = PowerAvailable / omega;
  else             ExcessTorque = PowerAvailable / 1.0;

  RPM = (RPS + ((ExcessTorque / Ixx) / (2.0 * M_PI)) * deltaT) * 60.0;

  if (RPM < 0.0) RPM = 0.0; // Engine won't turn backwards

  // Transform Torque and momentum first, as PQR is used in this
  // equation and cannot be transformed itself.
  vMn = in.PQR*(Transform()*vH) + Transform()*vTorque;

  return Thrust; // return thrust in pounds
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